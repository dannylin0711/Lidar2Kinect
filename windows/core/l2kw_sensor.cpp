/*
 * The virtual Kinect: the hub (newest data from the receiver), the INuiSensor object, its frame
 * textures, and the flat functions of Kinect10.dll. Behaviour follows docs/01 3-6:
 *  - stream and skeleton events are manual-reset; they are set when a new frame is ready and
 *    reset when it is taken (or found missing), so the game's thread does not spin;
 *  - GetNextFrame without a new frame fails with E_NUI_FRAME_NO_DATA (never with the two codes
 *    the game reads as "sensor missing");
 *  - the frame-end event is set after every frame, and at least every 100 ms while the sensor runs;
 *  - NuiStatus and the device status callback follow the sender's presence (windows/docs/no-sender.md).
 * One lock (g_cs) covers the hub and the sensor; events are set and reset under it, so a frame
 * published between "no data" and ResetEvent cannot be lost.
 */
#include "l2kw.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

namespace l2kw {

#define DEPTH_PIXELS (L2K_DEPTH_WIDTH * L2K_DEPTH_HEIGHT)
#define COLOR_BYTES (L2K_COLOR_WIDTH * L2K_COLOR_HEIGHT * 4)
#define TEXTURES_PER_STREAM 4

struct Lock {
    Lock();
    ~Lock();
};
static CRITICAL_SECTION g_cs;
static struct CsInit {
    CsInit() { InitializeCriticalSectionAndSpinCount(&g_cs, 1000); }
} g_cs_init;
Lock::Lock() { EnterCriticalSection(&g_cs); }
Lock::~Lock() { LeaveCriticalSection(&g_cs); }

/* ---- hub ---- */
static SenderInfo g_sender;
static l2k_skeleton_frame g_skel;            /* newest from the sender (unsmoothed) */
static uint32_t g_skel_serial;
static uint16_t *g_depth;                    /* newest depth image (sender or rasterised) */
static uint32_t g_depth_serial;
static uint64_t g_sender_depth_us;           /* when the sender last delivered a depth image */
static uint8_t *g_color;
static uint32_t g_color_serial;
static uint64_t g_last_frame_end_us, g_last_frame_us;
static LONG g_angle;                         /* from the newest frame's up vector */
static NuiStatusProc g_status_cb;
static void *g_status_user;

/* call counters for the periodic summary */
struct Calls {
    unsigned skel_ok, skel_nodata, depth_ok, depth_nodata, color_ok, color_nodata, status, angle_get, angle_set, smooth, map9, frame_end, tracked_set;
};
static Calls g_calls;
static uint64_t g_last_summary_us;

static void ensure_buffers() {
    if (!g_depth) g_depth = (uint16_t *)calloc(DEPTH_PIXELS, sizeof(uint16_t));
    if (!g_color) g_color = (uint8_t *)calloc(COLOR_BYTES, 1);
}

/* ---- textures ---- */
class Texture : public INuiFrameTexture {
public:
    int w = 0, h = 0, bpp = 0;
    BYTE *bits = nullptr;
    bool in_use = false;
    NUI_IMAGE_FRAME frame; /* the frame handed out with this texture (flat API returns a pointer to it) */

    void alloc(int width, int height, int bytes_per_pixel) {
        if (bits && w == width && h == height && bpp == bytes_per_pixel) return;
        free(bits);
        w = width;
        h = height;
        bpp = bytes_per_pixel;
        bits = (BYTE *)calloc((size_t)w * h * bpp, 1);
    }
    HRESULT STDMETHODCALLTYPE QueryInterface(REFIID riid, void **ppv) override {
        if (!ppv) return E_POINTER;
        if (IsEqualIID(riid, IID_IUnknown) || IsEqualIID(riid, L2KW_IID_INuiFrameTexture)) {
            *ppv = static_cast<INuiFrameTexture *>(this);
            return S_OK;
        }
        *ppv = nullptr;
        return E_NOINTERFACE;
    }
    /* textures belong to their stream and live as long as the process */
    ULONG STDMETHODCALLTYPE AddRef() override { return 2; }
    ULONG STDMETHODCALLTYPE Release() override { return 1; }
    int STDMETHODCALLTYPE BufferLen() override { return w * h * bpp; }
    int STDMETHODCALLTYPE Pitch() override { return w * bpp; }
    HRESULT STDMETHODCALLTYPE LockRect(UINT level, NUI_LOCKED_RECT *rect, RECT *, DWORD) override {
        if (level != 0) return E_INVALIDARG;
        if (!rect) return E_POINTER;
        rect->Pitch = w * bpp;
        rect->size = w * h * bpp;
        rect->pBits = bits;
        return S_OK;
    }
    HRESULT STDMETHODCALLTYPE GetLevelDesc(UINT level, NUI_SURFACE_DESC *desc) override {
        if (level != 0) return E_INVALIDARG;
        if (!desc) return E_POINTER;
        desc->Width = (UINT)w;
        desc->Height = (UINT)h;
        return S_OK;
    }
    HRESULT STDMETHODCALLTYPE UnlockRect(UINT level) override { return level == 0 ? S_OK : E_INVALIDARG; }
};

struct Stream {
    bool open = false;
    int type = 0, res = 0;
    DWORD flags = 0, limit = 0;
    HANDLE event = nullptr;
    uint32_t taken = 0; /* serial of the image last handed out */
    DWORD frame_number = 0;
    Texture tex[TEXTURES_PER_STREAM];
};

static int res_width(int res) {
    switch (res) {
    case NUI_IMAGE_RESOLUTION_80x60: return 80;
    case NUI_IMAGE_RESOLUTION_320x240: return 320;
    case NUI_IMAGE_RESOLUTION_640x480: return 640;
    case NUI_IMAGE_RESOLUTION_1280x960: return 1280;
    default: return 0;
    }
}

static bool sender_present() { return g_sender.present; }

static void set_event(HANDLE h) {
    if (h) SetEvent(h);
}
static void reset_event(HANDLE h) {
    if (h) ResetEvent(h);
}

/* ---- the sensor ---- */
class Sensor : public INuiSensor {
public:
    LONG refs = 0;
    DWORD init_flags = 0;
    bool initialized = false;
    HANDLE frame_end = nullptr;
    Stream depth, color;
    bool tracking = false;
    HANDLE skel_event = nullptr;
    DWORD skel_flags = 0;
    DWORD tracked_ids[NUI_SKELETON_MAX_TRACKED_COUNT] = {0, 0};
    uint32_t skel_taken = 0;
    LONG wanted_angle = 0;
    uint64_t init_us = 0;
    l2k_smoother smoother;

    Stream *stream(HANDLE h) {
        if (h == (HANDLE)&depth && depth.open) return &depth;
        if (h == (HANDLE)&color && color.open) return &color;
        return nullptr;
    }
    LONGLONG ms_now() const { return (LONGLONG)((now_us() - init_us) / 1000); }

    void reset_state() {
        initialized = false;
        init_flags = 0;
        frame_end = nullptr;
        depth.open = color.open = false;
        depth.event = color.event = nullptr;
        for (Texture &t : depth.tex) t.in_use = false;
        for (Texture &t : color.tex) t.in_use = false;
        tracking = false;
        skel_event = nullptr;
        skel_flags = 0;
        tracked_ids[0] = tracked_ids[1] = 0;
    }

    /* IUnknown */
    HRESULT STDMETHODCALLTYPE QueryInterface(REFIID riid, void **ppv) override {
        if (!ppv) return E_POINTER;
        if (IsEqualIID(riid, IID_IUnknown) || IsEqualIID(riid, L2KW_IID_INuiSensor)) {
            *ppv = static_cast<INuiSensor *>(this);
            AddRef();
            return S_OK;
        }
        *ppv = nullptr;
        return E_NOINTERFACE;
    }
    ULONG STDMETHODCALLTYPE AddRef() override { return (ULONG)InterlockedIncrement(&refs); }
    ULONG STDMETHODCALLTYPE Release() override {
        LONG r = InterlockedDecrement(&refs);
        if (r <= 0) {
            /* the object is static; the last release shuts it down */
            Lock l;
            if (initialized) log("sensor: released while initialised - shutting down");
            reset_state();
            refs = 0;
            r = 0;
        }
        log("sensor: Release -> %ld", r);
        return (ULONG)r;
    }

    HRESULT STDMETHODCALLTYPE NuiInitialize(DWORD flags) override {
        Lock l;
        HRESULT hr = S_OK;
        if (config().no_sender == NO_SENDER_MISSING && !sender_present()) hr = E_NUI_NOTCONNECTED;
        else {
            if (!initialized) {
                init_us = now_us();
                l2k_smoother_init(&smoother);
            }
            initialized = true;
            init_flags = flags;
            ensure_buffers();
        }
        log("game: NuiInitialize(0x%lx) -> 0x%08lx", flags, (unsigned long)hr);
        return hr;
    }
    void STDMETHODCALLTYPE NuiShutdown() override {
        Lock l;
        log("game: NuiShutdown()");
        reset_state();
    }
    HRESULT STDMETHODCALLTYPE NuiSetFrameEndEvent(HANDLE ev, DWORD flags) override {
        Lock l;
        log("game: NuiSetFrameEndEvent(%p, 0x%lx)", ev, flags);
        if (!initialized) return E_NUI_FEATURE_NOT_INITIALIZED;
        frame_end = ev;
        return S_OK;
    }
    HRESULT STDMETHODCALLTYPE NuiImageStreamOpen(int type, int res, DWORD flags, DWORD limit, HANDLE ev, HANDLE *out) override {
        Lock l;
        HRESULT hr = S_OK;
        Stream *s = nullptr;
        if (!out) hr = E_POINTER;
        else if (!initialized) hr = E_NUI_FEATURE_NOT_INITIALIZED;
        else if (type == NUI_IMAGE_TYPE_DEPTH_AND_PLAYER_INDEX && res == NUI_IMAGE_RESOLUTION_320x240) {
            if (!(init_flags & NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX)) hr = E_NUI_FEATURE_NOT_INITIALIZED;
            else s = &depth;
        } else if (type == NUI_IMAGE_TYPE_COLOR && res == NUI_IMAGE_RESOLUTION_640x480) {
            if (!(init_flags & NUI_INITIALIZE_FLAG_USES_COLOR)) hr = E_NUI_FEATURE_NOT_INITIALIZED;
            else s = &color;
        } else {
            hr = E_INVALIDARG; /* only what Dance Evolution opens (docs/01 3) is provided */
        }
        if (s) {
            s->open = true;
            s->type = type;
            s->res = res;
            s->flags = flags;
            s->limit = limit;
            s->event = ev;
            s->taken = s == &depth ? g_depth_serial : g_color_serial; /* only newer images */
            for (Texture &t : s->tex) {
                t.in_use = false;
                if (s == &depth) t.alloc(L2K_DEPTH_WIDTH, L2K_DEPTH_HEIGHT, 2);
                else t.alloc(L2K_COLOR_WIDTH, L2K_COLOR_HEIGHT, 4);
            }
            reset_event(ev);
            *out = (HANDLE)s;
        }
        log("game: NuiImageStreamOpen(type %d, res %d, flags 0x%lx, limit %lu, event %p) -> 0x%08lx, handle %p", type, res, flags, limit, ev,
            (unsigned long)hr, s);
        return hr;
    }
    HRESULT STDMETHODCALLTYPE NuiImageStreamSetImageFrameFlags(HANDLE h, DWORD flags) override {
        Lock l;
        Stream *s = stream(h);
        log("game: NuiImageStreamSetImageFrameFlags(%p, 0x%lx)", h, flags);
        if (!s) return E_INVALIDARG;
        s->flags = flags;
        return S_OK;
    }
    HRESULT STDMETHODCALLTYPE NuiImageStreamGetImageFrameFlags(HANDLE h, DWORD *flags) override {
        Lock l;
        Stream *s = stream(h);
        if (!s) return E_INVALIDARG;
        if (!flags) return E_POINTER;
        *flags = s->flags;
        return S_OK;
    }

    /* the core of both GetNextFrame variants; returns the texture used */
    HRESULT next_image(HANDLE h, DWORD wait_ms, Texture **used) {
        uint64_t deadline = now_us() + (uint64_t)wait_ms * 1000;
        for (;;) {
            {
                Lock l;
                Stream *s = stream(h);
                if (!s) return E_INVALIDARG;
                bool is_depth = s == &depth;
                uint32_t serial = is_depth ? g_depth_serial : g_color_serial;
                if (serial != s->taken) {
                    Texture *t = nullptr;
                    for (Texture &c : s->tex)
                        if (!c.in_use) {
                            t = &c;
                            break;
                        }
                    if (!t) {
                        reset_event(s->event);
                        return E_NUI_FRAME_LIMIT_EXCEEDED;
                    }
                    memcpy(t->bits, is_depth ? (const void *)g_depth : (const void *)g_color, is_depth ? DEPTH_PIXELS * 2 : COLOR_BYTES);
                    t->in_use = true;
                    s->taken = serial;
                    memset(&t->frame, 0, sizeof t->frame);
                    t->frame.liTimeStamp.QuadPart = ms_now();
                    t->frame.dwFrameNumber = ++s->frame_number;
                    t->frame.eImageType = s->type;
                    t->frame.eResolution = s->res;
                    t->frame.pFrameTexture = t;
                    t->frame.dwFrameFlags = 0;
                    reset_event(s->event);
                    (is_depth ? g_calls.depth_ok : g_calls.color_ok)++;
                    *used = t;
                    return S_OK;
                }
                if (wait_ms == 0 || now_us() >= deadline) {
                    reset_event(s->event);
                    (is_depth ? g_calls.depth_nodata : g_calls.color_nodata)++;
                    return E_NUI_FRAME_NO_DATA;
                }
            }
            Sleep(1);
        }
    }
    HRESULT release_image(HANDLE h, const NUI_IMAGE_FRAME *frame) {
        Lock l;
        Stream *s = stream(h);
        if (!s) return E_INVALIDARG;
        if (!frame) return E_POINTER;
        for (Texture &t : s->tex)
            if (frame->pFrameTexture == &t || frame == &t.frame) {
                t.in_use = false;
                return S_OK;
            }
        return E_INVALIDARG;
    }
    HRESULT STDMETHODCALLTYPE NuiImageStreamGetNextFrame(HANDLE h, DWORD wait_ms, NUI_IMAGE_FRAME *frame) override {
        Texture *t = nullptr;
        if (!frame) return E_POINTER;
        HRESULT hr = next_image(h, wait_ms, &t);
        if (SUCCEEDED(hr)) *frame = t->frame;
        if (config().verbose) log("game: NuiImageStreamGetNextFrame(%p, %lu) -> 0x%08lx", h, wait_ms, (unsigned long)hr);
        return hr;
    }
    HRESULT STDMETHODCALLTYPE NuiImageStreamReleaseFrame(HANDLE h, NUI_IMAGE_FRAME *frame) override {
        HRESULT hr = release_image(h, frame);
        if (config().verbose || FAILED(hr)) log("game: NuiImageStreamReleaseFrame(%p, %p) -> 0x%08lx", h, frame, (unsigned long)hr);
        return hr;
    }
    HRESULT STDMETHODCALLTYPE NuiImageGetColorPixelCoordinatesFromDepthPixel(int cres, const NUI_IMAGE_VIEW_AREA *va, LONG x, LONG y, USHORT d,
                                                                             LONG *cx, LONG *cy) override {
        return K_NuiImageGetColorPixelCoordinatesFromDepthPixel(cres, va, x, y, d, cx, cy);
    }
    HRESULT STDMETHODCALLTYPE NuiImageGetColorPixelCoordinatesFromDepthPixelAtResolution(int cres, int dres, const NUI_IMAGE_VIEW_AREA *va, LONG x,
                                                                                         LONG y, USHORT d, LONG *cx, LONG *cy) override {
        return K_NuiImageGetColorPixelCoordinatesFromDepthPixelAtResolution(cres, dres, va, x, y, d, cx, cy);
    }
    HRESULT STDMETHODCALLTYPE NuiImageGetColorPixelCoordinateFrameFromDepthPixelFrameAtResolution(int cres, int dres, DWORD count, USHORT *,
                                                                                                 DWORD ccount, LONG *coords) override {
        int cw = res_width(cres), dw = res_width(dres);
        if (!cw || !dw || !coords) return E_INVALIDARG;
        if (ccount < count * 2) return E_INVALIDARG;
        for (DWORD i = 0; i < count; i++) {
            LONG x = (LONG)(i % (DWORD)dw), y = (LONG)(i / (DWORD)dw);
            coords[i * 2] = x * cw / dw;
            coords[i * 2 + 1] = y * cw / dw;
        }
        return S_OK;
    }
    HRESULT STDMETHODCALLTYPE NuiCameraElevationSetAngle(LONG angle) override {
        Lock l;
        g_calls.angle_set++;
        log("game: NuiCameraElevationSetAngle(%ld) - accepted, ignored (the angle follows the device)", angle);
        wanted_angle = angle;
        return S_OK;
    }
    HRESULT STDMETHODCALLTYPE NuiCameraElevationGetAngle(LONG *angle) override { return K_NuiCameraElevationGetAngle(angle); }
    HRESULT STDMETHODCALLTYPE NuiSkeletonTrackingEnable(HANDLE ev, DWORD flags) override {
        Lock l;
        HRESULT hr = S_OK;
        if (!initialized || !(init_flags & NUI_INITIALIZE_FLAG_USES_SKELETON)) hr = E_NUI_FEATURE_NOT_INITIALIZED;
        else {
            tracking = true;
            skel_event = ev;
            skel_flags = flags;
            skel_taken = g_skel_serial;
            reset_event(ev);
        }
        log("game: NuiSkeletonTrackingEnable(%p, 0x%lx) -> 0x%08lx", ev, flags, (unsigned long)hr);
        return hr;
    }
    HRESULT STDMETHODCALLTYPE NuiSkeletonTrackingDisable() override {
        Lock l;
        log("game: NuiSkeletonTrackingDisable()");
        tracking = false;
        skel_event = nullptr;
        return S_OK;
    }
    HRESULT STDMETHODCALLTYPE NuiSkeletonSetTrackedSkeletons(DWORD *ids) override {
        Lock l;
        if (!ids) return E_POINTER;
        g_calls.tracked_set++;
        if (ids[0] != tracked_ids[0] || ids[1] != tracked_ids[1])
            log("game: NuiSkeletonSetTrackedSkeletons(%lu, %lu) (passed through, not enforced)", ids[0], ids[1]);
        tracked_ids[0] = ids[0];
        tracked_ids[1] = ids[1];
        return S_OK;
    }
    HRESULT STDMETHODCALLTYPE NuiSkeletonGetNextFrame(DWORD wait_ms, NUI_SKELETON_FRAME *out) override {
        uint64_t deadline = now_us() + (uint64_t)wait_ms * 1000;
        if (!out) return E_POINTER;
        for (;;) {
            {
                Lock l;
                if (!tracking) return E_NUI_STREAM_NOT_ENABLED;
                if (g_skel_serial != skel_taken) {
                    to_nui(g_skel, *out, ms_now());
                    skel_taken = g_skel_serial;
                    reset_event(skel_event);
                    g_calls.skel_ok++;
                    return S_OK;
                }
                if (wait_ms == 0 || now_us() >= deadline) {
                    reset_event(skel_event);
                    g_calls.skel_nodata++;
                    return E_NUI_FRAME_NO_DATA;
                }
            }
            Sleep(1);
        }
    }
    HRESULT STDMETHODCALLTYPE NuiTransformSmooth(NUI_SKELETON_FRAME *frame, const NUI_TRANSFORM_SMOOTH_PARAMETERS *params) override;
    HRESULT STDMETHODCALLTYPE NuiGetAudioSource(IUnknown **p) override {
        if (p) *p = nullptr;
        return E_NOTIMPL;
    }
    int STDMETHODCALLTYPE NuiInstanceIndex() override { return 0; }
    BSTR STDMETHODCALLTYPE NuiDeviceConnectionId() override { return SysAllocString(L"L2K\\Lidar2Kinect\\0"); }
    BSTR STDMETHODCALLTYPE NuiUniqueId() override { return SysAllocString(L"Lidar2Kinect"); }
    BSTR STDMETHODCALLTYPE NuiAudioArrayId() override { return nullptr; }
    HRESULT STDMETHODCALLTYPE NuiStatus() override {
        Lock l;
        g_calls.status++;
        if (config().no_sender == NO_SENDER_IDLE) return S_OK;
        return sender_present() ? S_OK : E_NUI_NOTCONNECTED;
    }
    DWORD STDMETHODCALLTYPE NuiInitializationFlags() override { return init_flags; }
    HRESULT STDMETHODCALLTYPE NuiGetCoordinateMapper(IUnknown **p) override {
        if (p) *p = nullptr;
        log("game: NuiGetCoordinateMapper - not implemented");
        return E_NOTIMPL;
    }
    HRESULT STDMETHODCALLTYPE NuiImageFrameGetDepthImagePixelFrameTexture(HANDLE, NUI_IMAGE_FRAME *, BOOL *, INuiFrameTexture **p) override {
        if (p) *p = nullptr;
        log("game: NuiImageFrameGetDepthImagePixelFrameTexture - not implemented");
        return E_NOTIMPL;
    }
    HRESULT STDMETHODCALLTYPE NuiGetColorCameraSettings(IUnknown **p) override {
        if (p) *p = nullptr;
        return E_NOTIMPL;
    }
    BOOL STDMETHODCALLTYPE NuiGetForceInfraredEmitterOff() override { return FALSE; }
    HRESULT STDMETHODCALLTYPE NuiSetForceInfraredEmitterOff(BOOL) override { return E_NOTIMPL; }
    HRESULT STDMETHODCALLTYPE NuiAccelerometerGetCurrentReading(L2KW_Vector4 *r) override {
        /* the accelerometer reads gravity: -up, in g */
        Lock l;
        if (!r) return E_POINTER;
        r->x = -g_skel.up.x;
        r->y = -g_skel.up.y;
        r->z = -g_skel.up.z;
        r->w = 0;
        return S_OK;
    }
};

static Sensor g_sensor;
static l2k_smoother g_flat_smoother; /* NuiTransformSmooth (flat) keeps its own history */
static bool g_flat_smoother_ready;

static HRESULT smooth(l2k_smoother &s, NUI_SKELETON_FRAME *frame, const NUI_TRANSFORM_SMOOTH_PARAMETERS *params) {
    l2k_skeleton_frame f;
    l2k_smooth_params p;
    if (!frame) return E_POINTER;
    if (params) {
        p.smoothing = params->fSmoothing;
        p.correction = params->fCorrection;
        p.prediction = params->fPrediction;
        p.jitter_radius = params->fJitterRadius;
        p.max_deviation_radius = params->fMaxDeviationRadius;
    } else {
        l2k_smooth_params_default(&p);
    }
    from_nui(*frame, f);
    l2k_smoother_apply(&s, &f, &p);
    for (int i = 0; i < NUI_SKELETON_COUNT; i++) {
        if (frame->SkeletonData[i].eTrackingState != L2K_SKELETON_TRACKED) continue;
        for (int j = 0; j < NUI_SKELETON_POSITION_COUNT; j++) {
            L2KW_Vector4 &v = frame->SkeletonData[i].SkeletonPositions[j];
            v.x = f.skeletons[i].joints[j].x;
            v.y = f.skeletons[i].joints[j].y;
            v.z = f.skeletons[i].joints[j].z;
        }
    }
    return S_OK;
}

HRESULT STDMETHODCALLTYPE Sensor::NuiTransformSmooth(NUI_SKELETON_FRAME *frame, const NUI_TRANSFORM_SMOOTH_PARAMETERS *params) {
    Lock l;
    g_calls.smooth++;
    return smooth(smoother, frame, params);
}

/* ---- hub, receiver side ---- */

static void notify_status(bool present) {
    NuiStatusProc cb;
    void *user;
    {
        Lock l;
        cb = g_status_cb;
        user = g_status_user;
    }
    HRESULT hr = present ? S_OK : E_NUI_NOTCONNECTED;
    log("status: sender %s -> device status callback %p(0x%08lx)", present ? "present" : "gone", cb, (unsigned long)hr);
    if (cb) cb(hr, L"L2K\\Lidar2Kinect\\0", L"Lidar2Kinect", user);
}

void hub_sender_present(bool present, const SenderInfo &info) {
    bool notify;
    {
        Lock l;
        notify = g_sender.present != present;
        g_sender = info;
        g_sender.present = present;
        if (!present) g_sender_depth_us = 0;
    }
    /* In "missing" mode the game reacts to the callback (it retries after S_OK, docs/01 5 and
     * windows/docs/no-sender.md). In "idle" mode the sensor never goes away, so stay quiet. */
    if (notify && config().no_sender == NO_SENDER_MISSING) notify_status(present);
    else if (notify) log("status: sender %s (idle mode: no callback)", present ? "present" : "gone");
}

static bool rasterize_depth_locked(uint64_t now) {
    if (!g_sensor.depth.open) return false;
    switch (config().depth_source) {
    case DEPTH_SENDER: return false;
    case DEPTH_SKELETON: return true;
    default:
        return !(g_sender.caps & L2K_STREAM_DEPTH) || !g_sender_depth_us || now - g_sender_depth_us > 500000;
    }
}

static void frame_end_locked(uint64_t now) {
    if (g_sensor.initialized && g_sensor.frame_end) {
        SetEvent(g_sensor.frame_end);
        g_calls.frame_end++;
    }
    g_last_frame_end_us = now;
}

void hub_skeleton(const l2k_skeleton_frame &frame) {
    uint64_t now = now_us();
    Lock l;
    ensure_buffers();
    g_skel = frame;
    g_skel_serial++;
    g_last_frame_us = now;
    {
        double a = l2k_elevation_degrees(frame.up);
        if (a > 27) a = 27;
        if (a < -27) a = -27;
        g_angle = (LONG)floor(a + 0.5);
    }
    if (g_sensor.tracking) set_event(g_sensor.skel_event);
    if (rasterize_depth_locked(now)) {
        l2k_depth_rasterize(&frame, g_depth, L2K_DEPTH_WIDTH, L2K_DEPTH_HEIGHT, 0);
        g_depth_serial++;
        set_event(g_sensor.depth.event);
    }
    if (g_sensor.color.open && !(g_sender.caps & L2K_STREAM_COLOR)) {
        /* a sender without a camera image: keep the colour stream ticking with a black image */
        g_color_serial++;
        set_event(g_sensor.color.event);
    }
    frame_end_locked(now);
}

void hub_depth(const uint16_t *pixels, uint32_t) {
    uint64_t now = now_us();
    Lock l;
    ensure_buffers();
    g_sender_depth_us = now;
    if (config().depth_source == DEPTH_SKELETON) return;
    memcpy(g_depth, pixels, DEPTH_PIXELS * 2);
    g_depth_serial++;
    if (g_sensor.depth.open) set_event(g_sensor.depth.event);
}

void hub_color(const uint8_t *bgrx, uint32_t) {
    Lock l;
    ensure_buffers();
    memcpy(g_color, bgrx, COLOR_BYTES);
    g_color_serial++;
    if (g_sensor.color.open) set_event(g_sensor.color.event);
}

uint32_t hub_wanted() {
    Lock l;
    uint32_t w = 0;
    if (!g_sensor.initialized) return 0;
    if (g_sensor.init_flags & NUI_INITIALIZE_FLAG_USES_SKELETON) w |= L2K_STREAM_SKELETON;
    if (g_sensor.depth.open) {
        w |= L2K_STREAM_SKELETON; /* also the source of the drawn depth image */
        if (config().depth_source != DEPTH_SKELETON) w |= L2K_STREAM_DEPTH;
    }
    if (g_sensor.color.open) w |= L2K_STREAM_COLOR;
    return w;
}

uint32_t hub_skeleton_flags() {
    Lock l;
    return g_sensor.tracking ? g_sensor.skel_flags : 0;
}

/* A Kinect delivers 30 skeleton frames a second and has no 30-joint body: ask for the default rate and
 * never for L2K_STREAM_BODY (hub_wanted), so this is never called. */
uint8_t hub_rate() { return 0; }
void hub_body(const l2k_body_frame &) {}

bool hub_sender_has_depth() {
    Lock l;
    return (g_sender.caps & L2K_STREAM_DEPTH) != 0;
}

void hub_tick() {
    uint64_t now = now_us();
    Calls c = {};
    bool summary = false;
    {
        Lock l;
        if (g_sensor.initialized) {
            bool idle_frames = config().no_sender == NO_SENDER_IDLE && !g_sender.present;
            if (idle_frames && now - g_last_frame_us >= 33333) {
                /* idle mode without a sender: a Kinect that sees nobody still delivers frames */
                l2k_skeleton_frame empty;
                memset(&empty, 0, sizeof empty);
                empty.frame_number = g_skel.frame_number + 1;
                empty.up.y = 1.0f;
                g_skel = empty;
                g_skel_serial++;
                g_last_frame_us = now;
                if (g_sensor.tracking) set_event(g_sensor.skel_event);
                if (g_sensor.depth.open) {
                    memset(g_depth, 0, DEPTH_PIXELS * 2);
                    g_depth_serial++;
                    set_event(g_sensor.depth.event);
                }
                if (g_sensor.color.open) {
                    g_color_serial++;
                    set_event(g_sensor.color.event);
                }
                frame_end_locked(now);
            } else if (now - g_last_frame_end_us >= 100000) {
                /* keep the game's 30 s frame-end watchdog fed even between frames */
                frame_end_locked(now);
            }
        }
        if (now - g_last_summary_us >= 5000000) {
            c = g_calls;
            memset(&g_calls, 0, sizeof g_calls);
            summary = g_sensor.initialized || c.status;
            g_last_summary_us = now;
        }
    }
    if (summary)
        log("game: last 5 s: skeleton %u ok / %u no data, depth %u / %u, colour %u / %u, NuiStatus %u, angle get %u set %u, smooth %u, "
            "ord 9 %u, tracked-set %u, frame-end %u",
            c.skel_ok, c.skel_nodata, c.depth_ok, c.depth_nodata, c.color_ok, c.color_nodata, c.status, c.angle_get, c.angle_set, c.smooth, c.map9,
            c.tracked_set, c.frame_end);
}

/* ---- flat functions ---- */
extern "C" {

HRESULT WINAPI K_NuiGetSensorCount(int *count) {
    start();
    if (!count) return E_POINTER;
    Lock l;
    *count = (config().no_sender == NO_SENDER_IDLE || sender_present()) ? 1 : 0;
    log("game: NuiGetSensorCount -> %d", *count);
    return S_OK;
}

HRESULT WINAPI K_NuiCreateSensorByIndex(int index, INuiSensor **out) {
    start();
    HRESULT hr = S_OK;
    if (!out) return E_POINTER;
    *out = nullptr;
    {
        Lock l;
        bool exists = config().no_sender == NO_SENDER_IDLE || sender_present();
        if (index != 0 || !exists) hr = E_INVALIDARG;
        else {
            g_sensor.AddRef();
            *out = &g_sensor;
        }
    }
    log("game: NuiCreateSensorByIndex(%d) -> 0x%08lx, %p", index, (unsigned long)hr, *out);
    return hr;
}

HRESULT WINAPI K_NuiCreateSensorById(const OLECHAR *id, INuiSensor **out) {
    start();
    log("game: NuiCreateSensorById(%ls)", id ? id : L"(null)");
    return K_NuiCreateSensorByIndex(0, out);
}

HRESULT WINAPI K_NuiImageGetColorPixelCoordinatesFromDepthPixelAtResolution(int cres, int dres, const NUI_IMAGE_VIEW_AREA *, LONG x, LONG y, USHORT,
                                                                            LONG *cx, LONG *cy) {
    /* depth and colour come from the same virtual camera: the mapping is a pure scale (docs/01 6) */
    int cw = res_width(cres), dw = res_width(dres);
    if (!cx || !cy) return E_POINTER;
    if (!cw || !dw) return E_INVALIDARG;
    *cx = x * cw / dw;
    *cy = y * cw / dw;
    return S_OK;
}

HRESULT WINAPI K_NuiImageGetColorPixelCoordinatesFromDepthPixel(int cres, const NUI_IMAGE_VIEW_AREA *va, LONG x, LONG y, USHORT d, LONG *cx,
                                                                LONG *cy) {
    /* without a resolution argument the depth pixel is in 320x240 */
    HRESULT hr = K_NuiImageGetColorPixelCoordinatesFromDepthPixelAtResolution(cres, NUI_IMAGE_RESOLUTION_320x240, va, x, y, d, cx, cy);
    {
        Lock l;
        g_calls.map9++;
    }
    if (config().verbose) log("game: ord 9 (res %d, %ld, %ld, depth %u) -> (%ld, %ld)", cres, x, y, d, cx ? *cx : 0, cy ? *cy : 0);
    return hr;
}

HRESULT WINAPI K_NuiTransformSmooth(NUI_SKELETON_FRAME *frame, const NUI_TRANSFORM_SMOOTH_PARAMETERS *params) {
    Lock l;
    if (!g_flat_smoother_ready) {
        l2k_smoother_init(&g_flat_smoother);
        g_flat_smoother_ready = true;
        if (params)
            log("game: NuiTransformSmooth parameters %.3f %.3f %.3f %.3f %.3f", params->fSmoothing, params->fCorrection, params->fPrediction,
                params->fJitterRadius, params->fMaxDeviationRadius);
    }
    g_calls.smooth++;
    return smooth(g_flat_smoother, frame, params);
}

void WINAPI K_NuiSetDeviceStatusCallback(NuiStatusProc cb, void *user) {
    start();
    {
        Lock l;
        g_status_cb = cb;
        g_status_user = user;
    }
    log("game: NuiSetDeviceStatusCallback(%p, %p)", cb, user);
}

HRESULT WINAPI K_NuiCameraElevationGetAngle(LONG *angle) {
    if (!angle) return E_POINTER;
    Lock l;
    g_calls.angle_get++;
    *angle = g_angle;
    return S_OK;
}

/* default-sensor flat API: sensor 0 */
HRESULT WINAPI K_NuiInitialize(DWORD flags) {
    start();
    {
        Lock l;
        if (g_sensor.refs == 0) g_sensor.AddRef();
    }
    return g_sensor.NuiInitialize(flags);
}
void WINAPI K_NuiShutdown() { g_sensor.NuiShutdown(); }
HRESULT WINAPI K_NuiSetFrameEndEvent(HANDLE ev, DWORD flags) { return g_sensor.NuiSetFrameEndEvent(ev, flags); }
HRESULT WINAPI K_NuiImageStreamOpen(int type, int res, DWORD flags, DWORD limit, HANDLE ev, HANDLE *out) {
    return g_sensor.NuiImageStreamOpen(type, res, flags, limit, ev, out);
}
HRESULT WINAPI K_NuiImageStreamSetImageFrameFlags(HANDLE h, DWORD flags) { return g_sensor.NuiImageStreamSetImageFrameFlags(h, flags); }
HRESULT WINAPI K_NuiImageStreamGetImageFrameFlags(HANDLE h, DWORD *flags) { return g_sensor.NuiImageStreamGetImageFrameFlags(h, flags); }
HRESULT WINAPI K_NuiImageStreamGetNextFrame(HANDLE h, DWORD wait_ms, const NUI_IMAGE_FRAME **out) {
    Texture *t = nullptr;
    if (!out) return E_POINTER;
    HRESULT hr = g_sensor.next_image(h, wait_ms, &t);
    *out = SUCCEEDED(hr) ? &t->frame : nullptr;
    return hr;
}
HRESULT WINAPI K_NuiImageStreamReleaseFrame(HANDLE h, const NUI_IMAGE_FRAME *frame) { return g_sensor.release_image(h, frame); }
HRESULT WINAPI K_NuiCameraElevationSetAngle(LONG angle) { return g_sensor.NuiCameraElevationSetAngle(angle); }
HRESULT WINAPI K_NuiSkeletonTrackingEnable(HANDLE ev, DWORD flags) { return g_sensor.NuiSkeletonTrackingEnable(ev, flags); }
HRESULT WINAPI K_NuiSkeletonTrackingDisable() { return g_sensor.NuiSkeletonTrackingDisable(); }
HRESULT WINAPI K_NuiSkeletonSetTrackedSkeletons(DWORD *ids) { return g_sensor.NuiSkeletonSetTrackedSkeletons(ids); }
HRESULT WINAPI K_NuiSkeletonGetNextFrame(DWORD wait_ms, NUI_SKELETON_FRAME *frame) { return g_sensor.NuiSkeletonGetNextFrame(wait_ms, frame); }

HRESULT WINAPI K_NuiGetAudioSource(IUnknown **p) {
    if (p) *p = nullptr;
    log("game: NuiGetAudioSource - not implemented");
    return E_NOTIMPL;
}
HRESULT WINAPI K_NuiSkeletonCalculateBoneOrientations(const NUI_SKELETON_DATA *, void *) {
    log("game: NuiSkeletonCalculateBoneOrientations - not implemented");
    return E_NOTIMPL;
}
HRESULT WINAPI K_NuiCreateCoordinateMapperFromParameters(ULONG, void *, IUnknown **p) {
    if (p) *p = nullptr;
    log("game: NuiCreateCoordinateMapperFromParameters - not implemented");
    return E_NOTIMPL;
}
HRESULT WINAPI K_NuiCreateDepthFilter(LPCWSTR, DWORD, IUnknown **p) {
    if (p) *p = nullptr;
    log("game: NuiCreateDepthFilter - not implemented");
    return E_NOTIMPL;
}
const void *K_NuiOrdinal1() {
    static const uint8_t zero[64] = {0};
    log("game: ordinal 1 called (unknown function of the real DLL) - returning a zeroed block");
    return zero;
}

} // extern "C"

} // namespace l2kw
