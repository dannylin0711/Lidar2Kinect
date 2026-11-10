/*
 * l2k_probe - drives Kinect10.dll the way Dance Evolution's gamekdm.dll does (docs/01 3-6):
 * the same five imports by ordinal, the same call order, the same five events and the same
 * WaitForMultipleObjects loop, the same status handling (thread states 1..7). It prints what it
 * receives once a second.
 *
 * It loads whatever Kinect10.dll the loader finds: the replacement when it sits next to the
 * probe, Microsoft's from SysWOW64 otherwise.
 *
 *   l2k_probe [-t seconds] [-o dir] [-v]
 *             [--check] [--sender exe] [--jpeg file] [--players n] [--reconnect]
 *
 *   --sender    start this l2k_fake_sender against 127.0.0.1 (port from L2K_PORT or 24810)
 *   --jpeg      the colour image the sender sends; --check then compares every received colour
 *               frame with this file decoded by WIC, byte for byte
 *   --check     exit 1 unless the streams meet the requirements (rates, events, contents)
 *   --reconnect after the first run the sender says BYE; then it is started again and later
 *               killed without BYE: the probe must see the sensor go and come back both times
 */
#include "../core/nui_abi.h"

#include <wincodec.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>
#include <string>

/* ---- the five imports, by ordinal (kinect10_game.def) ---- */
extern "C" {
HRESULT WINAPI NuiGetSensorCount(int *pCount);
HRESULT WINAPI NuiCreateSensorByIndex(int index, INuiSensor **ppNuiSensor);
HRESULT WINAPI NuiImageGetColorPixelCoordinatesFromDepthPixel(int eColorResolution, const NUI_IMAGE_VIEW_AREA *pcViewArea, LONG lDepthX,
                                                              LONG lDepthY, USHORT usDepthValue, LONG *plColorX, LONG *plColorY);
HRESULT WINAPI NuiTransformSmooth(NUI_SKELETON_FRAME *pSkeletonFrame, const NUI_TRANSFORM_SMOOTH_PARAMETERS *pSmoothingParams);
void WINAPI NuiSetDeviceStatusCallback(NuiStatusProc callback, void *pUserData);
}

enum { ST_IDLE = 0, ST_OPENING = 1, ST_STOPPING = 2, ST_ERROR = 3, ST_MISSING = 4, ST_NOT_READY = 5, ST_TIMEOUT = 6, ST_RUNNING = 7 };
static const char *state_name(int s) {
    static const char *n[] = {"idle", "opening", "stopping", "error", "sensor missing", "not ready", "timeout", "running"};
    return s >= 0 && s <= 7 ? n[s] : "?";
}

struct Counters {
    unsigned skel, skel_nodata, tracked_frames, depth, depth_nodata, color, color_nodata, frame_end, wake, callbacks_ok, callbacks_err;
    unsigned color_mismatch, bad_pitch, depth_player_px, depth_bad_px, tracked, color_mean;
    double head_z;
};

static CRITICAL_SECTION g_cs;
static Counters g_c;
static volatile LONG g_state;
static volatile LONG g_quit;
static HANDLE g_retry;   /* the game's global event: set at start and by the status callback on S_OK */
static HANDLE g_ev[5];   /* wake, depth, colour, skeleton, frame end - as gamekdm.dll */
static int g_verbose;
static const char *g_outdir;
static std::vector<uint8_t> g_expect_color; /* BGRX of --jpeg */
static LONG g_angle = -360;
static LONG g_map[4][2];
static HRESULT g_map_hr = E_FAIL;

static double now_s() {
    static LARGE_INTEGER f, t0;
    LARGE_INTEGER c;
    if (!f.QuadPart) {
        QueryPerformanceFrequency(&f);
        QueryPerformanceCounter(&t0);
    }
    QueryPerformanceCounter(&c);
    return (double)(c.QuadPart - t0.QuadPart) / (double)f.QuadPart;
}

static void CALLBACK status_cb(HRESULT hr, const OLECHAR *instance, const OLECHAR *unique, void *) {
    printf("[%7.3f] status callback 0x%08lx \"%ls\" \"%ls\"\n", now_s(), (unsigned long)hr, instance ? instance : L"", unique ? unique : L"");
    fflush(stdout);
    EnterCriticalSection(&g_cs);
    if (hr == S_OK) g_c.callbacks_ok++;
    else g_c.callbacks_err++;
    LeaveCriticalSection(&g_cs);
    /* exactly what gamekdm.dll's callback does (0x1000ebb0): only S_OK wakes the thread */
    if (hr == S_OK) SetEvent(g_retry);
}

static int map_status(HRESULT hr) {
    /* gamekdm.dll 0x1000ec4d */
    switch ((unsigned long)hr) {
    case 0x80070015: case 0x8301027F: return ST_NOT_READY;
    case 0x8007048F: case 0x83010014: case 0x83010015: return ST_MISSING;
    default: return ST_ERROR;
    }
}

static bool decode_jpeg(const char *path, std::vector<uint8_t> &out) {
    IWICImagingFactory *f = NULL;
    IWICBitmapDecoder *d = NULL;
    IWICBitmapFrameDecode *fr = NULL;
    IWICBitmapSource *s = NULL;
    wchar_t wpath[MAX_PATH];
    bool ok = false;
    UINT w, h;
    MultiByteToWideChar(CP_ACP, 0, path, -1, wpath, MAX_PATH);
    CoInitializeEx(NULL, COINIT_MULTITHREADED);
    if (FAILED(CoCreateInstance(CLSID_WICImagingFactory, NULL, CLSCTX_INPROC_SERVER, IID_PPV_ARGS(&f)))) return false;
    if (SUCCEEDED(f->CreateDecoderFromFilename(wpath, NULL, GENERIC_READ, WICDecodeMetadataCacheOnDemand, &d)) && SUCCEEDED(d->GetFrame(0, &fr)) &&
        SUCCEEDED(fr->GetSize(&w, &h)) && w == 640 && h == 480 && SUCCEEDED(WICConvertBitmapSource(GUID_WICPixelFormat32bppBGR, fr, &s))) {
        out.resize(640 * 480 * 4);
        ok = SUCCEEDED(s->CopyPixels(NULL, 640 * 4, (UINT)out.size(), out.data()));
    }
    if (s) s->Release();
    if (fr) fr->Release();
    if (d) d->Release();
    f->Release();
    return ok;
}

static void save(const char *name, const char *header, const void *data, size_t len) {
    char path[MAX_PATH];
    snprintf(path, sizeof path, "%s\\%s", g_outdir, name);
    FILE *f = fopen(path, "wb");
    if (!f) return;
    if (header) fputs(header, f);
    fwrite(data, 1, len, f);
    fclose(f);
}

/* ---- the Nui thread, after hid::Natal::NuiThread (gamekdm.dll 0x1000ebd0) ---- */
struct Natal {
    INuiSensor *sensor = nullptr;
    HANDLE stream[2] = {nullptr, nullptr};
    NUI_IMAGE_FRAME frame[2];
    bool held[2] = {false, false};
    double last_frame_end = 0;
    NUI_SKELETON_FRAME skel;

    void open() {
        int count = 0;
        g_state = ST_OPENING;
        HRESULT hr = NuiGetSensorCount(&count); /* ordinal 5 */
        if (FAILED(hr)) {
            g_state = ST_ERROR;
            return;
        }
        if (count == 0) {
            g_state = ST_MISSING;
            return;
        }
        if (FAILED(NuiCreateSensorByIndex(0, &sensor)) || !sensor) { /* ordinal 6 */
            sensor = nullptr;
            g_state = ST_ERROR;
            return;
        }
        if (FAILED(sensor->NuiInitialize(0x0B))) {
            g_state = ST_ERROR;
            return;
        }
        sensor->NuiSetFrameEndEvent(g_ev[4], 0);
        last_frame_end = now_s();
        g_state = ST_RUNNING;
        if (FAILED(sensor->NuiImageStreamOpen(NUI_IMAGE_TYPE_DEPTH_AND_PLAYER_INDEX, NUI_IMAGE_RESOLUTION_320x240, 0, 3, g_ev[1], &stream[0])))
            g_state = ST_ERROR;
        if (FAILED(sensor->NuiImageStreamOpen(NUI_IMAGE_TYPE_COLOR, NUI_IMAGE_RESOLUTION_640x480, 0, 3, g_ev[2], &stream[1]))) g_state = ST_ERROR;
        /* the game enables tracking from its settings (0x1000f250), flags 0 */
        if (FAILED(sensor->NuiSkeletonTrackingEnable(g_ev[3], 0))) g_state = ST_ERROR;
    }

    void close() {
        if (g_state == ST_RUNNING) g_state = ST_STOPPING;
        if (sensor) {
            for (int i = 0; i < 2; i++)
                if (held[i]) {
                    sensor->NuiImageStreamReleaseFrame(stream[i], &frame[i]);
                    held[i] = false;
                }
            sensor->NuiShutdown();
            sensor->Release();
            sensor = nullptr;
        }
        stream[0] = stream[1] = nullptr;
    }

    void image(int i) {
        /* 0x1000f400: release the frame held, take the next one, keep it until the next event */
        if (held[i]) {
            sensor->NuiImageStreamReleaseFrame(stream[i], &frame[i]);
            held[i] = false;
        }
        HRESULT hr = sensor->NuiImageStreamGetNextFrame(stream[i], 0, &frame[i]);
        if (FAILED(hr)) {
            if (hr == (HRESULT)0x80070015 || hr == (HRESULT)0x8007048F) g_state = ST_MISSING;
            EnterCriticalSection(&g_cs);
            (i == 0 ? g_c.depth_nodata : g_c.color_nodata)++;
            LeaveCriticalSection(&g_cs);
            return;
        }
        held[i] = true;
        g_state = ST_RUNNING;
        NUI_LOCKED_RECT r;
        INuiFrameTexture *t = frame[i].pFrameTexture;
        if (!t || FAILED(t->LockRect(0, &r, NULL, 0))) return;
        EnterCriticalSection(&g_cs);
        if (i == 0) {
            g_c.depth++;
            if (r.Pitch != 640) g_c.bad_pitch++;
            unsigned players = 0, bad = 0;
            for (int y = 0; y < 240; y++) {
                const USHORT *row = (const USHORT *)(r.pBits + y * r.Pitch);
                for (int x = 0; x < 320; x++) {
                    USHORT p = row[x];
                    if (p & 7) {
                        players++;
                        if ((p >> 3) < 800 || (p >> 3) > 4000) bad++;
                    }
                }
            }
            g_c.depth_player_px = players;
            g_c.depth_bad_px += bad;
            if (g_outdir) {
                static uint8_t pgm[320 * 240];
                for (int k = 0; k < 320 * 240; k++) {
                    USHORT p = ((const USHORT *)r.pBits)[k];
                    pgm[k] = (uint8_t)((p & 7) ? 255 - ((p >> 3) - 800) * 200 / 3200 : 0);
                }
                save("depth.pgm", "P5\n320 240\n255\n", pgm, sizeof pgm);
            }
        } else {
            g_c.color++;
            if (r.Pitch != 2560) g_c.bad_pitch++;
            if (!g_expect_color.empty()) {
                bool same = true;
                for (int k = 0; k < 640 * 480 && same; k++)
                    same = memcmp(r.pBits + k * 4, g_expect_color.data() + k * 4, 3) == 0;
                if (!same) g_c.color_mismatch++;
            }
            unsigned long long sum = 0;
            for (int k = 0; k < 640 * 480; k += 7) sum += r.pBits[k * 4] + r.pBits[k * 4 + 1] + r.pBits[k * 4 + 2];
            g_c.color_mean = (unsigned)(sum / (3ull * (640 * 480 / 7 + 1)));
            if (g_outdir) {
                /* bottom-up 32-bit BMP */
                static uint8_t bmp[54 + 640 * 480 * 4];
                memset(bmp, 0, 54);
                bmp[0] = 'B', bmp[1] = 'M';
                *(DWORD *)(bmp + 2) = sizeof bmp;
                *(DWORD *)(bmp + 10) = 54;
                *(DWORD *)(bmp + 14) = 40;
                *(LONG *)(bmp + 18) = 640;
                *(LONG *)(bmp + 22) = -480;
                *(WORD *)(bmp + 26) = 1;
                *(WORD *)(bmp + 28) = 32;
                memcpy(bmp + 54, r.pBits, 640 * 480 * 4);
                save("color.bmp", NULL, bmp, sizeof bmp);
            }
        }
        LeaveCriticalSection(&g_cs);
        t->UnlockRect(0);
    }

    void skeleton() {
        /* 0x1000f300 then 0x100100d0: take the frame, smooth it with ordinal 20 */
        HRESULT hr = sensor->NuiSkeletonGetNextFrame(0, &skel);
        if (FAILED(hr)) {
            if (hr == (HRESULT)0x80070015 || hr == (HRESULT)0x8007048F) g_state = ST_MISSING;
            EnterCriticalSection(&g_cs);
            g_c.skel_nodata++;
            LeaveCriticalSection(&g_cs);
            return;
        }
        g_state = ST_RUNNING;
        NUI_TRANSFORM_SMOOTH_PARAMETERS p = {0.5f, 0.5f, 0.5f, 0.05f, 0.04f};
        NuiTransformSmooth(&skel, &p);
        unsigned tracked = 0;
        double head_z = 0;
        for (int i = 0; i < NUI_SKELETON_COUNT; i++)
            if (skel.SkeletonData[i].eTrackingState == 2) {
                if (!tracked) head_z = skel.SkeletonData[i].SkeletonPositions[3].z;
                tracked++;
            }
        EnterCriticalSection(&g_cs);
        g_c.skel++;
        g_c.tracked = tracked;
        if (tracked) {
            g_c.tracked_frames++;
            g_c.head_z = head_z;
        }
        LeaveCriticalSection(&g_cs);
    }

    void wake() {
        /* 0x1000f042: settings, elevation angle (0x1000f510), frame-end watchdog */
        LONG a;
        if (SUCCEEDED(sensor->NuiCameraElevationGetAngle(&a))) g_angle = (g_angle == -360) ? a : (a + 2 * g_angle) / 3;
        if (now_s() - last_frame_end >= 30.0) g_state = ST_TIMEOUT;
        EnterCriticalSection(&g_cs);
        g_c.wake++;
        LeaveCriticalSection(&g_cs);
    }

    void step() {
        DWORD r = WaitForMultipleObjects(5, g_ev, FALSE, 30000);
        switch (r) {
        case WAIT_OBJECT_0: wake(); break;
        case WAIT_OBJECT_0 + 1: image(0); break;
        case WAIT_OBJECT_0 + 2: image(1); break;
        case WAIT_OBJECT_0 + 3: skeleton(); break;
        case WAIT_OBJECT_0 + 4:
            last_frame_end = now_s();
            EnterCriticalSection(&g_cs);
            g_c.frame_end++;
            LeaveCriticalSection(&g_cs);
            break;
        default: g_state = ST_TIMEOUT; break;
        }
    }

    void run() {
        NuiSetDeviceStatusCallback(status_cb, NULL); /* ordinal 22 */
        SetEvent(g_retry);
        while (!g_quit) {
            if (WaitForSingleObject(g_retry, 200) != WAIT_OBJECT_0) continue;
            if (g_quit) break;
            open();
            printf("[%7.3f] open -> %s\n", now_s(), state_name(g_state));
            fflush(stdout);
            while (sensor && !g_quit) {
                HRESULT hr = sensor->NuiStatus();
                if (FAILED(hr)) {
                    g_state = map_status(hr);
                    printf("[%7.3f] NuiStatus 0x%08lx -> %s\n", now_s(), (unsigned long)hr, state_name(g_state));
                    fflush(stdout);
                    break;
                }
                if (g_state != ST_RUNNING) break;
                step();
            }
            close();
            if (g_state == ST_STOPPING) g_state = ST_IDLE;
            printf("[%7.3f] closed (%s), waiting for the status callback\n", now_s(), state_name(g_state));
            fflush(stdout);
        }
        close();
    }
};

static DWORD WINAPI nui_thread(void *p) {
    ((Natal *)p)->run();
    return 0;
}

/* ---- a sender process ---- */
static PROCESS_INFORMATION g_sender_pi;
static bool start_sender(const char *exe, const char *jpeg, int players, double seconds) {
    char cmd[2048];
    const char *port = getenv("L2K_PORT");
    snprintf(cmd, sizeof cmd, "\"%s\" -n %d -t %.1f %s%s%s %s%s 127.0.0.1", exe, players, seconds, jpeg ? "-j \"" : "", jpeg ? jpeg : "",
             jpeg ? "\"" : "", port ? "-p " : "", port ? port : "");
    STARTUPINFOA si;
    memset(&si, 0, sizeof si);
    si.cb = sizeof si;
    if (!CreateProcessA(NULL, cmd, NULL, NULL, FALSE, 0, NULL, NULL, &si, &g_sender_pi)) {
        fprintf(stderr, "cannot start %s (%lu)\n", exe, GetLastError());
        return false;
    }
    printf("[%7.3f] started sender: %s\n", now_s(), cmd);
    fflush(stdout);
    return true;
}
static void stop_sender(bool kill) {
    if (!g_sender_pi.hProcess) return;
    if (kill) TerminateProcess(g_sender_pi.hProcess, 9);
    WaitForSingleObject(g_sender_pi.hProcess, 20000);
    CloseHandle(g_sender_pi.hProcess);
    CloseHandle(g_sender_pi.hThread);
    memset(&g_sender_pi, 0, sizeof g_sender_pi);
}

/* ---- measuring ---- */
struct Window {
    double seconds = 0;
    Counters sum;
    unsigned min_tracked = 99, last_player_px = 0;
    Window() { memset(&sum, 0, sizeof sum); }
};

static Counters take() {
    EnterCriticalSection(&g_cs);
    Counters c = g_c;
    g_c.skel = g_c.skel_nodata = g_c.tracked_frames = g_c.depth = g_c.depth_nodata = g_c.color = g_c.color_nodata = g_c.frame_end = g_c.wake = 0;
    g_c.color_mismatch = g_c.bad_pitch = g_c.depth_bad_px = 0;
    LeaveCriticalSection(&g_cs);
    return c;
}

static void print_line(const Counters &c) {
    printf("[%7.3f] %-14s skel %2u (+%u no data, %u tracked, head z %.2f)  depth %2u (+%u, %u player px)  colour %2u (+%u, mean %u%s)  "
           "frame-end %2u  wake %2u  angle %ld\n",
           now_s(), state_name(g_state), c.skel, c.skel_nodata, c.tracked, c.head_z, c.depth, c.depth_nodata, c.depth_player_px, c.color,
           c.color_nodata, c.color_mean, c.color_mismatch ? ", MISMATCH" : "", c.frame_end, c.wake, g_angle);
    fflush(stdout);
}

/* run for `seconds`, printing a line a second; measure seconds [skip, end) */
static Window run_for(double seconds, double skip) {
    Window w;
    double t0 = now_s(), next = t0 + 1;
    take();
    while (now_s() - t0 < seconds) {
        SetEvent(g_ev[0]); /* the game's main thread wakes the Nui thread every frame */
        Sleep(16);
        if (now_s() >= next) {
            Counters c = take();
            print_line(c);
            if (now_s() - t0 > skip) {
                w.seconds += 1;
                w.sum.skel += c.skel;
                w.sum.skel_nodata += c.skel_nodata;
                w.sum.depth += c.depth;
                w.sum.depth_nodata += c.depth_nodata;
                w.sum.color += c.color;
                w.sum.color_nodata += c.color_nodata;
                w.sum.frame_end += c.frame_end;
                w.sum.color_mismatch += c.color_mismatch;
                w.sum.bad_pitch += c.bad_pitch;
                w.sum.depth_bad_px += c.depth_bad_px;
                w.sum.tracked_frames += c.tracked_frames;
                if (c.tracked < w.min_tracked) w.min_tracked = c.tracked;
                w.last_player_px = c.depth_player_px;
            }
            next += 1;
        }
    }
    return w;
}

static int g_failures;
static void expect(bool ok, const char *what, double value) {
    printf("  %s %-58s %.2f\n", ok ? "ok  " : "FAIL", what, value);
    if (!ok) g_failures++;
}

static void check_streaming(const Window &w, int players, bool color_expected) {
    double s = w.seconds > 0 ? w.seconds : 1;
    printf("check over %.0f s:\n", w.seconds);
    expect(w.sum.skel / s >= 27, "skeleton frames per second >= 27", w.sum.skel / s);
    expect(w.sum.depth / s >= 27, "depth frames per second >= 27", w.sum.depth / s);
    if (color_expected) expect(w.sum.color / s >= 27, "colour frames per second >= 27", w.sum.color / s);
    expect(w.sum.frame_end / s >= 27, "frame-end events per second >= 27", w.sum.frame_end / s);
    expect((w.sum.skel_nodata + w.sum.depth_nodata + w.sum.color_nodata) / s <= 3, "wake-ups without a frame per second <= 3 (events reset)",
           (w.sum.skel_nodata + w.sum.depth_nodata + w.sum.color_nodata) / s);
    expect(w.min_tracked == (unsigned)players, "tracked skeletons in every second == players", w.min_tracked);
    expect(w.last_player_px > 1000, "player pixels in the depth image > 1000", w.last_player_px);
    expect(w.sum.depth_bad_px == 0, "player pixels outside 800..4000 mm == 0", w.sum.depth_bad_px);
    expect(w.sum.bad_pitch == 0, "frames with a wrong LockRect pitch == 0", w.sum.bad_pitch);
    if (color_expected && !g_expect_color.empty()) expect(w.sum.color_mismatch == 0, "colour frames differing from the JPEG == 0", w.sum.color_mismatch);
    expect(g_state == ST_RUNNING, "thread state is running (7)", g_state);
}

/* wait until the thread reaches `state`; returns the time it took or -1 */
static double wait_state(int state, double limit) {
    double t0 = now_s();
    while (now_s() - t0 < limit) {
        if (g_state == state) return now_s() - t0;
        SetEvent(g_ev[0]);
        Sleep(10);
    }
    return -1;
}

int main(int argc, char **argv) {
    double seconds = 10;
    bool check = false, reconnect = false;
    const char *sender = NULL, *jpeg = NULL;
    int players = 2;
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "-t") && i + 1 < argc) seconds = atof(argv[++i]);
        else if (!strcmp(argv[i], "-o") && i + 1 < argc) g_outdir = argv[++i];
        else if (!strcmp(argv[i], "-v")) g_verbose = 1;
        else if (!strcmp(argv[i], "--check")) check = true;
        else if (!strcmp(argv[i], "--reconnect")) reconnect = true;
        else if (!strcmp(argv[i], "--sender") && i + 1 < argc) sender = argv[++i];
        else if (!strcmp(argv[i], "--jpeg") && i + 1 < argc) jpeg = argv[++i];
        else if (!strcmp(argv[i], "--players") && i + 1 < argc) players = atoi(argv[++i]);
        else {
            fprintf(stderr, "usage: l2k_probe [-t seconds] [-o dir] [-v] [--check] [--sender exe] [--jpeg file] [--players n] [--reconnect]\n");
            return 2;
        }
    }
    InitializeCriticalSection(&g_cs);
    {
        char path[MAX_PATH];
        HMODULE k = GetModuleHandleA("Kinect10.dll");
        GetModuleFileNameA(k, path, sizeof path);
        printf("Kinect10.dll loaded from %s\n", path);
    }
    if (jpeg && !decode_jpeg(jpeg, g_expect_color)) {
        fprintf(stderr, "cannot decode %s as a 640x480 JPEG\n", jpeg);
        return 2;
    }
    g_retry = CreateEventW(NULL, FALSE, FALSE, NULL);
    g_ev[0] = CreateEventW(NULL, FALSE, FALSE, NULL); /* auto-reset */
    g_ev[1] = CreateEventW(NULL, TRUE, FALSE, NULL);  /* manual-reset, never reset by the game */
    g_ev[2] = CreateEventW(NULL, TRUE, FALSE, NULL);
    g_ev[3] = CreateEventW(NULL, TRUE, FALSE, NULL);
    g_ev[4] = CreateEventW(NULL, FALSE, FALSE, NULL); /* auto-reset */

    /* ordinal 9 the way 0x10019c30 calls it: the four corners at 2.0 m */
    {
        static const LONG corner[4][2] = {{0, 0}, {320, 0}, {0, 240}, {320, 240}};
        g_map_hr = S_OK;
        for (int i = 0; i < 4; i++) {
            HRESULT hr = NuiImageGetColorPixelCoordinatesFromDepthPixel(NUI_IMAGE_RESOLUTION_640x480, NULL, corner[i][0], corner[i][1],
                                                                        (USHORT)(2000 << 3), &g_map[i][0], &g_map[i][1]);
            if (FAILED(hr)) g_map_hr = hr;
        }
        printf("ordinal 9 -> 0x%08lx: (0,0)->(%ld,%ld) (320,0)->(%ld,%ld) (0,240)->(%ld,%ld) (320,240)->(%ld,%ld)\n", (unsigned long)g_map_hr,
               g_map[0][0], g_map[0][1], g_map[1][0], g_map[1][1], g_map[2][0], g_map[2][1], g_map[3][0], g_map[3][1]);
    }

    Natal natal;
    HANDLE thread = CreateThread(NULL, 0, nui_thread, &natal, 0, NULL);

    /* the sender outlives the measurement; with --reconnect it then ends on its own (BYE) */
    if (sender && !start_sender(sender, jpeg, players, reconnect ? seconds + 1.5 : seconds + 5)) return 1;
    Window w = run_for(seconds, 2.5);
    if (check) {
        check_streaming(w, players, jpeg != NULL);
        expect(g_map_hr == S_OK && g_map[0][0] == 0 && g_map[0][1] == 0 && g_map[3][0] == 640 && g_map[3][1] == 480,
               "ordinal 9: depth (x,y) -> colour (2x,2y)", (double)g_map[3][0]);
    }

    if (reconnect && sender) {
        /* the sender ends with BYE: the sensor must go at once */
        stop_sender(false);
        double t = wait_state(ST_MISSING, 5);
        printf("[%7.3f] after BYE: %s in %.2f s\n", now_s(), state_name(g_state), t);
        if (check) expect(t >= 0 && t < 1.0, "sensor missing within 1 s of BYE", t);
        run_for(2, 0);
        /* it comes back: the status callback must bring the thread back to running */
        unsigned ok_before = g_c.callbacks_ok;
        start_sender(sender, jpeg, players, 60);
        t = wait_state(ST_RUNNING, 5);
        printf("[%7.3f] sender back: %s in %.2f s\n", now_s(), state_name(g_state), t);
        if (check) expect(t >= 0 && t < 2.5 && g_c.callbacks_ok > ok_before, "running again within 2.5 s, through the status callback", t);
        Window w2 = run_for(6, 2.5);
        if (check) check_streaming(w2, players, jpeg != NULL);
        /* killed without BYE: the 3 s silence timeout must notice */
        stop_sender(true);
        t = wait_state(ST_MISSING, 8);
        printf("[%7.3f] after kill: %s in %.2f s\n", now_s(), state_name(g_state), t);
        if (check) expect(t >= 2.5 && t < 4.5, "sensor missing 3 s after the sender went silent", t);
    }

    g_quit = 1;
    WaitForSingleObject(thread, 5000);
    stop_sender(false);
    if (check) {
        printf("%s: %d failure(s)\n", g_failures ? "FAILED" : "PASSED", g_failures);
        return g_failures ? 1 : 0;
    }
    return 0;
}
