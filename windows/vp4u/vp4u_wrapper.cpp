/*
 * DanceAround's VisionPoseWrapper.dll, rebuilt on Lidar2Kinect: the vp4u API (version 210) fed by
 * the L2K1 receiver instead of two RealSense cameras + VisionPose.
 *
 * Threads:
 *  - receiver (core/l2kw_receiver.cpp): socket; hands every new skeleton frame to hub_skeleton().
 *  - dispatcher (here): calls the game's callbacks. A new skeleton frame -> one pose frame; no
 *    fresh frame for 250 ms (or no sender) -> an empty pose frame every 33 ms so the stream never
 *    stops; camera preview / calibration preview frames at 30 fps while open.
 *  - the game's threads: call the table functions.
 * Callbacks run under g_cb_cs, and the Stop / Close functions take g_cb_cs before clearing a
 * callback, so a callback is never called after its Stop returned (the C# delegate may be
 * collected then).
 */
#include "l2kw_base.h"
#include "vp4u_abi.h"
#include "vp4u_map.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

using namespace l2kw;

namespace {

/* ---- configuration ([vp4u] in l2k.ini) ---- */
struct Vp4uConfig {
    int no_sender;         /* NO_SENDER_IDLE: cameras always "active"; NO_SENDER_MISSING: active only with a sender */
    bool camera_height_auto;
    vp4u::MapParams map;
    int video_w, video_h;
    int verbose;
    char calibration_file[MAX_PATH]; /* written so the game sees the cameras as calibrated; empty = never */
    bool pin_d3d10warp;              /* keep D3D10Warp.dll loaded (crash workaround, docs/vp4u-interface.md §9) */
};
Vp4uConfig g_cfg;

float ini_float(const char *ini, const char *key, float fallback, bool *is_auto) {
    char buf[64];
    GetPrivateProfileStringA("vp4u", key, "", buf, sizeof buf, ini);
    if (is_auto) *is_auto = false;
    if (!buf[0]) return fallback;
    if (!_stricmp(buf, "auto")) {
        if (is_auto) *is_auto = true;
        return fallback;
    }
    char *end = NULL;
    double v = strtod(buf, &end);
    return end != buf ? (float)v : fallback;
}

void vp4u_config_load() {
    const char *ini = config().ini_path;
    char buf[32];
    vp4u::map_params_default(&g_cfg.map);
    GetPrivateProfileStringA("vp4u", "no_sender", "idle", buf, sizeof buf, ini);
    g_cfg.no_sender = !_stricmp(buf, "missing") ? NO_SENDER_MISSING : NO_SENDER_IDLE;
    bool a = false;
    g_cfg.map.camera_height = ini_float(ini, "camera_height", 1.16f, &a);
    {
        char probe[8];
        GetPrivateProfileStringA("vp4u", "camera_height", "auto", probe, sizeof probe, ini);
        g_cfg.camera_height_auto = !_stricmp(probe, "auto");
    }
    g_cfg.map.phone_x = ini_float(ini, "phone_x", 0.55f, NULL);
    g_cfg.map.phone_z = ini_float(ini, "phone_z", 0.0f, NULL);
    g_cfg.map.phone_height = ini_float(ini, "phone_height", 0.0f, &a);
    if (a) g_cfg.map.phone_height = 0.0f;
    g_cfg.map.default_phone_height = ini_float(ini, "default_phone_height", 1.0f, NULL);
    g_cfg.map.yaw_deg = ini_float(ini, "yaw", 0.0f, NULL);
    g_cfg.map.mirror_x = GetPrivateProfileIntA("vp4u", "mirror_x", 0, ini) != 0;
    g_cfg.video_w = GetPrivateProfileIntA("vp4u", "video_width", 480, ini);
    g_cfg.video_h = GetPrivateProfileIntA("vp4u", "video_height", 480, ini);
    if (g_cfg.video_w < 16 || g_cfg.video_w > 1920) g_cfg.video_w = 480;
    if (g_cfg.video_h < 16 || g_cfg.video_h > 1920) g_cfg.video_h = 480;
    g_cfg.verbose = config().verbose;
    /* The game's own check (StereoCameraCalibrationHelper.CalibrationIsRequired) is "the VFS file
     * dev/nvram/stereo_cam.yml exists"; written through kamunity.dll's avs functions
     * (docs/vp4u-interface.md §4). */
    GetPrivateProfileStringA("vp4u", "calibration_file", "dev/nvram/stereo_cam.yml", g_cfg.calibration_file, sizeof g_cfg.calibration_file, ini);
    if (!_stricmp(g_cfg.calibration_file, "none")) g_cfg.calibration_file[0] = 0;
    g_cfg.pin_d3d10warp = GetPrivateProfileIntA("vp4u", "pin_d3d10warp", 1, ini) != 0;
}

/* ---- compatibility: D3D10Warp.dll ----
 * The game's video playback (Media Foundation) creates and releases a WARP (software) D3D device
 * several times; on this Windows build D3D10Warp.dll can be unloaded while one of its thread-pool
 * callbacks is still queued, which then runs into the unloaded code (every crash dump of the
 * game: <Unloaded_D3D10Warp.dll>+0x1303a0 from a thread-pool thread). Keeping the DLL loaded
 * for the life of the process makes the late callback harmless. Nothing to do with poses; it lives
 * here because this is the one DLL of ours the game loads at boot. */
const char *pin_d3d10warp() {
    HMODULE m = NULL;
    if (!GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_PIN, L"d3d10warp.dll", &m)) {
        if (!LoadLibraryExW(L"d3d10warp.dll", NULL, LOAD_LIBRARY_SEARCH_SYSTEM32)) return "LOAD FAILED";
        if (!GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_PIN, L"d3d10warp.dll", &m)) return "PIN FAILED";
    }
    return "pinned";
}

/* ---- the calibration placeholder ---- */
const char kCalibrationPlaceholder[] =
    "%YAML:1.0\n"
    "---\n"
    "# Lidar2Kinect vp4u: placeholder. DanceAround only checks that this file exists; there are no\n"
    "# stereo cameras to calibrate. Put the original VisionPoseWrapper.dll back -> calibrate again in test mode.\n"
    "Memo: \"Lidar2Kinect vp4u placeholder\"\n";

bool file_exists(const char *path) {
    FILE *f = fopen(path, "rb");
    if (!f) return false;
    fclose(f);
    return true;
}

/* The game's VFS (avs) through kamunity.dll, the game's own native bridge to it: GameFs.IsFileExistVfs
 * is KAMUNITY.Avs.FsLstat(path) with stat.mode == 2 (a file). kamunity's FsOpen records the handle
 * that its FsWrite / FsClose then accept, so all three come from kamunity, not from avs directly.
 * Signatures from KAMUNITY's delegates (docs/vp4u-interface.md §4). */
struct AvsFs {
    int (*open)(const char *path, int flags, int mode); /* > 0: handle */
    int (*write)(int fd, const void *data, int len);
    int (*close)(int fd);
    uint8_t (*lstat)(const char *path, void *stat); /* nonzero: exists */
};
const int kAvsOpenWriteCreateTruncate = 50; /* what KAMUNITY.Avs.FsOpenW passes */
const int kAvsMode0644 = 420;

bool avs_fs(AvsFs *fs) {
    HMODULE k = GetModuleHandleA("kamunity.dll");
    if (!k) return false;
    fs->open = (int (*)(const char *, int, int))GetProcAddress(k, "FsOpen");
    fs->write = (int (*)(int, const void *, int))GetProcAddress(k, "FsWrite");
    fs->close = (int (*)(int))GetProcAddress(k, "FsClose");
    fs->lstat = (uint8_t(*)(const char *, void *))GetProcAddress(k, "FsLstat");
    return fs->open && fs->write && fs->close && fs->lstat;
}

bool avs_is_file(const AvsFs &fs, const char *path) {
    /* AVS_FS_STAT: u64 ctime, mtime, atime; u32 link_count, size, hi_size; u16 mode, permission */
    uint8_t st[64];
    memset(st, 0, sizeof st);
    if (!fs.lstat(path, st)) return false;
    uint16_t mode;
    memcpy(&mode, st + 36, sizeof mode);
    return mode == 2;
}

/* writes the placeholder if `overwrite` or if there is no file yet; returns a word for the log */
const char *write_calibration(bool overwrite) {
    const char *path = g_cfg.calibration_file;
    const int len = (int)(sizeof kCalibrationPlaceholder - 1);
    if (!path[0]) return "disabled";
    AvsFs fs;
    if (avs_fs(&fs)) {
        if (!overwrite && avs_is_file(fs, path)) return "present in the game's VFS";
        int fd = fs.open(path, kAvsOpenWriteCreateTruncate, kAvsMode0644);
        if (fd <= 0) return "VFS OPEN FAILED";
        int n = fs.write(fd, kCalibrationPlaceholder, len);
        fs.close(fd);
        if (n != len) return "VFS WRITE FAILED";
        return avs_is_file(fs, path) ? "written to the game's VFS" : "written to the VFS but not found back";
    }
    /* no kamunity (the probe, a test): a plain file relative to the working directory */
    if (!overwrite && file_exists(path)) return "present (plain file, no kamunity.dll)";
    FILE *f = fopen(path, "wb");
    if (!f) return "WRITE FAILED";
    size_t n = fwrite(kCalibrationPlaceholder, 1, (size_t)len, f);
    fclose(f);
    if (n != (size_t)len) return "WRITE FAILED";
    return file_exists(path) ? "written (plain file, no kamunity.dll)" : "written but not readable back";
}

/* ---- diagnostics: has the loader redirected this module's imports (EXECEXE::HookModule)? ---- */
void import_report(char *out, size_t cap) {
    static const char *const watch[] = {"CreateFileW", "CreateFileA", "GetFileAttributesW", "GetModuleFileNameA", "GetPrivateProfileStringA"};
    HMODULE self = NULL;
    GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT, (LPCSTR)&import_report, &self);
    BYTE *base = (BYTE *)self;
    out[0] = 0;
    if (!base) return;
    IMAGE_NT_HEADERS *nt = (IMAGE_NT_HEADERS *)(base + ((IMAGE_DOS_HEADER *)base)->e_lfanew);
    IMAGE_DATA_DIRECTORY dir = nt->OptionalHeader.DataDirectory[IMAGE_DIRECTORY_ENTRY_IMPORT];
    if (!dir.VirtualAddress) return;
    int total = 0, redirected = 0;
    char detail[256] = "";
    for (IMAGE_IMPORT_DESCRIPTOR *d = (IMAGE_IMPORT_DESCRIPTOR *)(base + dir.VirtualAddress); d->Name; d++) {
        HMODULE m = GetModuleHandleA((const char *)(base + d->Name));
        IMAGE_THUNK_DATA *names = (IMAGE_THUNK_DATA *)(base + (d->OriginalFirstThunk ? d->OriginalFirstThunk : d->FirstThunk));
        IMAGE_THUNK_DATA *addrs = (IMAGE_THUNK_DATA *)(base + d->FirstThunk);
        for (; names->u1.AddressOfData; names++, addrs++) {
            if (IMAGE_SNAP_BY_ORDINAL(names->u1.Ordinal)) continue;
            const char *fn = (const char *)((IMAGE_IMPORT_BY_NAME *)(base + names->u1.AddressOfData))->Name;
            FARPROC real = m ? GetProcAddress(m, fn) : NULL;
            bool moved = real && (ULONG_PTR)real != (ULONG_PTR)addrs->u1.Function;
            total++;
            if (moved) redirected++;
            for (const char *w : watch)
                if (!strcmp(fn, w)) {
                    size_t l = strlen(detail);
                    snprintf(detail + l, sizeof detail - l, "%s%s %s", l ? ", " : "", fn, moved ? "hooked" : "direct");
                }
        }
    }
    snprintf(out, cap, "%d of %d imports redirected by the loader (%s)", redirected, total, detail);
}

/* if the log could not be opened where l2k.ini says (an absolute path the loader's VFS does not
 * map, for one), try paths inside the game's file system */
void log_fallback() {
    if (log_is_open() || !config().log_path[0]) return;
    static const char *const candidates[] = {"dev/raw/log/l2k.log", "l2k.log"};
    for (const char *c : candidates)
        if (log_open_at(c)) return;
}

/* ---- game log callbacks ---- */
struct LogSink {
    vp4u_log_cb cb;
    void *ctx;
};
LogSink g_sinks[4];
CRITICAL_SECTION g_log_cs;

/* to l2k.log and to the game's log (VisionPoseLogger), prefixed so it can be found there */
void glog(const char *fmt, ...) {
    char msg[512], line[540];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(msg, sizeof msg, fmt, ap);
    va_end(ap);
    log("%s", msg);
    snprintf(line, sizeof line, "l2k: %s", msg);
    EnterCriticalSection(&g_log_cs);
    LogSink sinks[4];
    memcpy(sinks, g_sinks, sizeof sinks);
    LeaveCriticalSection(&g_log_cs);
    for (const LogSink &s : sinks)
        if (s.cb) s.cb(s.ctx, line);
}

void *add_sink(void *cb, void *ctx) {
    if (!cb) return NULL;
    void *handle = NULL;
    EnterCriticalSection(&g_log_cs);
    for (int i = 0; i < 4; i++)
        if (g_sinks[i].cb == (vp4u_log_cb)cb && g_sinks[i].ctx == ctx) handle = (void *)(intptr_t)(i + 1);
    for (int i = 0; i < 4 && !handle; i++)
        if (!g_sinks[i].cb) {
            g_sinks[i].cb = (vp4u_log_cb)cb;
            g_sinks[i].ctx = ctx;
            handle = (void *)(intptr_t)(i + 1);
        }
    LeaveCriticalSection(&g_log_cs);
    return handle;
}

/* ---- shared state ---- */
CRITICAL_SECTION g_state_cs; /* frame, presence */
CRITICAL_SECTION g_cb_cs;    /* the game's callbacks; held while calling them */
HANDLE g_wake;               /* auto-reset: a new frame arrived or a callback was set */
bool g_present;
char g_sender_name[L2K_NAME_SIZE + 1];
l2k_skeleton_frame g_frame;
uint32_t g_frame_seq;
uint64_t g_frame_time;
char g_config_path[MAX_PATH];

struct Callbacks {
    vp4u_pose_cb pose;
    void *pose_ctx;
    vp4u_video_cb prev_left, prev_right;
    void *prev_left_ctx, *prev_right_ctx;
    vp4u_calib_status_cb calib_status;
    vp4u_video_cb calib_preview;
    void *calib_ctx;
    int calib_step; /* 0 nothing sent, 1 "in progress" sent, 2 success sent */
    uint64_t calib_started;
    vp4u_pose_cb offline;
    void *offline_ctx;
    uint64_t offline_started;
} g_cb;

/* the fake camera image (static, shared by every frame: AddRef / Release do nothing) */
vp4u_image_desc g_image;
uint8_t *g_pixels;

void make_image() {
    int w = g_cfg.video_w, h = g_cfg.video_h;
    g_pixels = (uint8_t *)malloc((size_t)w * h * 3);
    if (!g_pixels) return;
    for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++) {
            bool border = x < 4 || y < 4 || x >= w - 4 || y >= h - 4;
            uint8_t v = border ? 96 : 24;
            uint8_t *p = g_pixels + ((size_t)y * w + x) * 3;
            p[0] = p[1] = p[2] = v;
        }
    g_image.pixels = g_pixels;
    g_image.width = w;
    g_image.height = h;
    g_image.bits_per_pixel = 24;
    g_image.bytes_per_pixel = 3;
    g_image.bytes_per_row = w * 3;
    g_image.bytes_per_frame = w * h * 3;
}

/* statistics for the 5 s summary */
struct Stats {
    unsigned frames, empty, bodies, preview, mapped_bad;
} g_st;

bool g_initialized;
volatile LONG g_quit;
INIT_ONCE g_once = INIT_ONCE_STATIC_INIT;
struct Instance {
    char magic[8];
} g_instance = {"vp4uL2K"};

bool cameras_active() {
    if (g_cfg.no_sender == NO_SENDER_IDLE) return true;
    EnterCriticalSection(&g_state_cs);
    bool p = g_present;
    LeaveCriticalSection(&g_state_cs);
    return p;
}

/* read LeftCameraPose.Y from the game's VisionPoseForUnityConfig.json (the main camera's height) */
bool read_camera_height(const char *path, float *out) {
    FILE *f = fopen(path, "rb");
    if (!f) return false;
    static char text[64 * 1024];
    size_t n = fread(text, 1, sizeof text - 1, f);
    fclose(f);
    text[n] = 0;
    const char *k = strstr(text, "\"LeftCameraPose\"");
    if (!k) return false;
    const char *open = strchr(k, '{'), *close = open ? strchr(open, '}') : NULL;
    if (!open || !close) return false;
    for (const char *p = open; p < close; p++) {
        if (p[0] == '"' && p[1] == 'Y' && p[2] == '"') {
            const char *colon = strchr(p, ':');
            if (!colon || colon > close) return false;
            char *end = NULL;
            double v = strtod(colon + 1, &end);
            if (end == colon + 1 || v < 0.2 || v > 3.0) return false;
            *out = (float)v;
            return true;
        }
    }
    return false;
}

/* ---- dispatcher ---- */
DWORD WINAPI dispatcher_main(void *) {
    uint32_t last_seq = 0;
    uint64_t last_pose = 0, last_preview = 0, last_report = now_us();
    vp4u_body bodies[VP4U_MAX_BODIES];
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_ABOVE_NORMAL);

    while (!g_quit) {
        WaitForSingleObject(g_wake, 10);
        uint64_t now = now_us();

        EnterCriticalSection(&g_state_cs);
        bool fresh = g_frame_seq != last_seq;
        l2k_skeleton_frame f;
        if (fresh) f = g_frame;
        uint64_t frame_time = g_frame_time;
        bool present = g_present;
        last_seq = g_frame_seq;
        LeaveCriticalSection(&g_state_cs);

        EnterCriticalSection(&g_cb_cs);
        if (g_cb.pose) {
            if (fresh) {
                int n = vp4u::map_frame(f, g_cfg.map, bodies, VP4U_MAX_BODIES);
                g_cb.pose(g_cb.pose_ctx, frame_time, bodies, n, bodies, 0, bodies, 0, bodies, 0, &g_image, &g_image);
                g_st.frames++;
                g_st.bodies += (unsigned)n;
                if (n == 0) g_st.empty++;
                last_pose = now;
            } else if (now - last_pose >= 33333 && (!present || now - frame_time > 250000)) {
                g_cb.pose(g_cb.pose_ctx, now, bodies, 0, bodies, 0, bodies, 0, bodies, 0, &g_image, &g_image);
                g_st.frames++;
                g_st.empty++;
                last_pose = now;
            }
        }
        if ((g_cb.prev_left || g_cb.prev_right || g_cb.calib_preview) && now - last_preview >= 33333) {
            if (g_cb.prev_left) g_cb.prev_left(g_cb.prev_left_ctx, now, &g_image);
            if (g_cb.prev_right) g_cb.prev_right(g_cb.prev_right_ctx, now, &g_image);
            if (g_cb.calib_preview) g_cb.calib_preview(g_cb.calib_ctx, now, &g_image);
            g_st.preview++;
            last_preview = now;
        }
        if (g_cb.calib_status) {
            if (g_cb.calib_step == 0) {
                g_cb.calib_status(g_cb.calib_ctx, VP4U_CALIB_IN_PROGRESS, 0, 1, "Lidar2Kinect: no stereo calibration needed");
                g_cb.calib_step = 1;
            } else if (g_cb.calib_step == 1 && now - g_cb.calib_started > 500000) {
                g_cb.calib_status(g_cb.calib_ctx, VP4U_CALIB_SUCCESS, 1, 1, "Lidar2Kinect: no stereo calibration needed");
                g_cb.calib_step = 2;
            }
        }
        if (g_cb.offline && now - g_cb.offline_started > 100000) {
            /* nothing was recorded to analyse: end the stream at once (main video NULL = end) */
            vp4u_pose_cb cb = g_cb.offline;
            void *ctx = g_cb.offline_ctx;
            g_cb.offline = NULL;
            cb(ctx, now, bodies, 0, bodies, 0, bodies, 0, bodies, 0, NULL, NULL);
        }
        LeaveCriticalSection(&g_cb_cs);

        if (now - last_report >= 5000000) {
            if (g_st.frames || g_st.preview) {
                double s = (double)(now - last_report) / 1e6;
                log("game: last %.0f s: pose frames %.1f/s (%u empty), bodies per frame %.2f, preview frames %.1f/s, sender %s", s,
                    g_st.frames / s, g_st.empty, g_st.frames ? (double)g_st.bodies / g_st.frames : 0.0, g_st.preview / s,
                    present ? "present" : "absent");
            }
            memset(&g_st, 0, sizeof g_st);
            last_report = now;
        }
    }
    return 0;
}

BOOL CALLBACK start_once(PINIT_ONCE, PVOID, PVOID *) {
    InitializeCriticalSection(&g_state_cs);
    InitializeCriticalSection(&g_cb_cs);
    g_wake = CreateEventA(NULL, FALSE, FALSE, NULL);
    start(); /* the receiver: config_load, log_open, socket, thread; pins the module */
    log_fallback();
    vp4u_config_load();
    if (g_cfg.pin_d3d10warp) log("vp4u: D3D10Warp.dll %s (pin_d3d10warp=1)", pin_d3d10warp());
    make_image();
    log("vp4u: no_sender %s, camera_height %s%.2f, phone_x %.2f, phone_z %.2f, phone_height %s%.2f, yaw %.1f, mirror_x %d, video %dx%d",
        g_cfg.no_sender == NO_SENDER_IDLE ? "idle" : "missing", g_cfg.camera_height_auto ? "auto/" : "", g_cfg.map.camera_height,
        g_cfg.map.phone_x, g_cfg.map.phone_z, g_cfg.map.phone_height > 0 ? "" : "auto/", g_cfg.map.phone_height > 0 ? g_cfg.map.phone_height : g_cfg.map.default_phone_height,
        g_cfg.map.yaw_deg, g_cfg.map.mirror_x, g_cfg.video_w, g_cfg.video_h);
    HANDLE t = CreateThread(NULL, 0, dispatcher_main, NULL, 0, NULL);
    if (t) CloseHandle(t);
    else log("vp4u: CreateThread (dispatcher) failed (%lu)", GetLastError());
    return TRUE;
}

void ensure_started() { InitOnceExecuteOnce(&g_once, start_once, NULL, NULL); }

bool own_image(const vp4u_image_desc *d) { return d == &g_image; }

/* ---- the table ---- */
uint64_t api_GetCurrentTimestamp() { return now_us(); }

void *api_RegisterLoggerCallback(void *cb, void *ctx) {
    ensure_started();
    return add_sink(cb, ctx);
}

void api_UnregisterLoggerCallback(void *handle) {
    intptr_t i = (intptr_t)handle - 1;
    if (i < 0 || i >= 4) return;
    EnterCriticalSection(&g_log_cs);
    g_sinks[i].cb = NULL;
    g_sinks[i].ctx = NULL;
    LeaveCriticalSection(&g_log_cs);
}

void api_AddRefImageDesc(vp4u_image_desc *d) {
    if (d && !own_image(d)) log("vp4u: AddRefImageDesc on an unknown image %p", (void *)d);
}
void api_ReleaseImageDesc(vp4u_image_desc *d) {
    if (d && !own_image(d)) log("vp4u: ReleaseImageDesc on an unknown image %p", (void *)d);
}

void *api_Initialize() {
    ensure_started();
    g_initialized = true;
    glog("Initialize (Lidar2Kinect vp4u wrapper, API %d, UDP %u)", VP4U_API_VERSION, config().port);
    char module[MAX_PATH] = "", imports[400];
    HMODULE self = NULL;
    GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT, (LPCSTR)&api_Initialize, &self);
    GetModuleFileNameA(self, module, sizeof module);
    import_report(imports, sizeof imports);
    glog("module %s; ini %s (%s); log %s (%s)", module, config().ini_path, file_exists(config().ini_path) ? "found" : "not found",
         config().log_path[0] ? config().log_path : "off", log_is_open() ? "open" : "NOT OPEN");
    glog("%s", imports);
    glog("D3D10Warp.dll %s", !g_cfg.pin_d3d10warp ? "not pinned (pin_d3d10warp=0)" : GetModuleHandleW(L"d3d10warp.dll") ? "pinned" : "NOT LOADED");
    return &g_instance;
}

void stop_all_callbacks() {
    EnterCriticalSection(&g_cb_cs);
    memset(&g_cb, 0, sizeof g_cb);
    LeaveCriticalSection(&g_cb_cs);
}

void api_Shutdown() {
    glog("Shutdown");
    stop_all_callbacks();
    g_initialized = false;
}

void api_LoadConfig(const char *path) {
    ensure_started();
    snprintf(g_config_path, sizeof g_config_path, "%s", path ? path : "");
    float h;
    if (g_cfg.camera_height_auto && read_camera_height(g_config_path, &h)) {
        g_cfg.map.camera_height = h;
        glog("LoadConfig %s: main camera height %.2f m (LeftCameraPose.Y)", g_config_path, h);
    } else {
        glog("LoadConfig %s: main camera height %.2f m%s", g_config_path, g_cfg.map.camera_height,
             g_cfg.camera_height_auto ? " (LeftCameraPose.Y not found, default)" : " (l2k.ini)");
    }
    glog("calibration file %s: %s", g_cfg.calibration_file[0] ? g_cfg.calibration_file : "(none)", write_calibration(false));
}

void api_ReloadConfig() {
    glog("ReloadConfig");
    float keep = g_cfg.map.camera_height;
    bool was_auto = g_cfg.camera_height_auto;
    EnterCriticalSection(&g_cb_cs); /* the dispatcher reads g_cfg.map under this lock */
    vp4u_config_load();
    if (was_auto && g_cfg.camera_height_auto) g_cfg.map.camera_height = keep;
    LeaveCriticalSection(&g_cb_cs);
}

int32_t api_OpenRealSense() {
    glog("OpenRealSense -> 1 (no cameras; the skeleton comes from the Lidar2Kinect app)");
    return 1;
}
int32_t api_RealSenseLeftIsActive() { return cameras_active() ? 1 : 0; }
int32_t api_RealSenseRightIsActive() { return cameras_active() ? 1 : 0; }
int32_t api_TryRebootRealSense(int32_t hard) {
    glog("TryRebootRealSense(%d) -> 1", hard);
    return 1;
}
int32_t api_TryRebuildRealSenseFilters() { return 1; }

int32_t api_InitVisionPoseRealtime(const char *) {
    glog("InitVisionPoseRealtime -> 1");
    return 1;
}
int32_t api_InitVisionPoseOffline(const char *) {
    glog("InitVisionPoseOffline -> 1");
    return 1;
}

int32_t api_StartRealtimeCameraPreview(void *lcb, void *lctx, void *rcb, void *rctx) {
    ensure_started();
    EnterCriticalSection(&g_cb_cs);
    g_cb.prev_left = (vp4u_video_cb)lcb;
    g_cb.prev_left_ctx = lctx;
    g_cb.prev_right = (vp4u_video_cb)rcb;
    g_cb.prev_right_ctx = rctx;
    LeaveCriticalSection(&g_cb_cs);
    SetEvent(g_wake);
    glog("StartRealtimeCameraPreview -> 1 (%dx%d placeholder image)", g_image.width, g_image.height);
    return 1;
}
void api_StopRealtimeCameraPreview() {
    EnterCriticalSection(&g_cb_cs);
    g_cb.prev_left = g_cb.prev_right = NULL;
    g_cb.prev_left_ctx = g_cb.prev_right_ctx = NULL;
    LeaveCriticalSection(&g_cb_cs);
    glog("StopRealtimeCameraPreview");
}

int32_t api_StartStereoCameraCalibration(void *status_cb, void *preview_cb, void *ctx) {
    ensure_started();
    EnterCriticalSection(&g_cb_cs);
    g_cb.calib_status = (vp4u_calib_status_cb)status_cb;
    g_cb.calib_preview = (vp4u_video_cb)preview_cb;
    g_cb.calib_ctx = ctx;
    g_cb.calib_step = 0;
    g_cb.calib_started = now_us();
    LeaveCriticalSection(&g_cb_cs);
    SetEvent(g_wake);
    glog("StartStereoCameraCalibration -> 1 (reports success; there is nothing to calibrate)");
    return 1;
}
int32_t api_CommitStereoCameraCalibration() {
    glog("CommitStereoCameraCalibration -> 1 (calibration file %s: %s)", g_cfg.calibration_file[0] ? g_cfg.calibration_file : "(none)",
         write_calibration(true));
    return 1;
}
void api_CloseStereoCameraCalibration() {
    EnterCriticalSection(&g_cb_cs);
    g_cb.calib_status = NULL;
    g_cb.calib_preview = NULL;
    g_cb.calib_ctx = NULL;
    LeaveCriticalSection(&g_cb_cs);
    glog("CloseStereoCameraCalibration");
}

int32_t api_StartRealtimeAnalysis(void *cb, void *ctx) {
    ensure_started();
    EnterCriticalSection(&g_cb_cs);
    g_cb.pose = (vp4u_pose_cb)cb;
    g_cb.pose_ctx = ctx;
    LeaveCriticalSection(&g_cb_cs);
    SetEvent(g_wake);
    glog("StartRealtimeAnalysis -> 1");
    return 1;
}
void api_StopRealtimeAnalysis() {
    EnterCriticalSection(&g_cb_cs);
    g_cb.pose = NULL;
    g_cb.pose_ctx = NULL;
    LeaveCriticalSection(&g_cb_cs);
    glog("StopRealtimeAnalysis");
}

/* Play-video recording: nothing is recorded (phase 1, docs/vp4u-interface.md open point 4). */
int32_t api_PrepareVideoRecording(const char *path) {
    glog("PrepareVideoRecording(%s) -> 1 (not recorded)", path ? path : "");
    return 1;
}
int32_t api_StartVideoRecording() {
    glog("StartVideoRecording -> 1");
    return 1;
}
int32_t api_FinalizeVideoRecording() {
    glog("FinalizeVideoRecording -> 1");
    return 1;
}
int32_t api_DeleteRecordedVideo(const char *path) {
    glog("DeleteRecordedVideo(%s) -> 1 (nothing to delete)", path ? path : "");
    return 1;
}
int32_t api_StartOfflineAnalysis(const char *path, void *cb, void *ctx) {
    ensure_started();
    EnterCriticalSection(&g_cb_cs);
    g_cb.offline = (vp4u_pose_cb)cb;
    g_cb.offline_ctx = ctx;
    g_cb.offline_started = now_us();
    LeaveCriticalSection(&g_cb_cs);
    SetEvent(g_wake);
    glog("StartOfflineAnalysis(%s) -> 1 (ends at once: nothing recorded)", path ? path : "");
    return 1;
}
void api_StopOfflineAnalysis() {
    EnterCriticalSection(&g_cb_cs);
    g_cb.offline = NULL;
    g_cb.offline_ctx = NULL;
    LeaveCriticalSection(&g_cb_cs);
}

} // namespace

/* ---- hub (called by the receiver) ---- */
namespace l2kw {

void hub_sender_present(bool present, const SenderInfo &info) {
    EnterCriticalSection(&g_state_cs);
    g_present = present;
    snprintf(g_sender_name, sizeof g_sender_name, "%s", info.name);
    LeaveCriticalSection(&g_state_cs);
    SetEvent(g_wake);
    glog("sender \"%s\" %s", info.name, present ? "connected" : "gone");
}

void hub_skeleton(const l2k_skeleton_frame &frame) {
    EnterCriticalSection(&g_state_cs);
    g_frame = frame;
    g_frame_seq++;
    g_frame_time = now_us();
    LeaveCriticalSection(&g_state_cs);
    SetEvent(g_wake);
}

void hub_depth(const uint16_t *, uint32_t) {}
void hub_color(const uint8_t *, uint32_t) {}
void hub_tick() {}
uint32_t hub_wanted() { return L2K_STREAM_SKELETON; }
uint32_t hub_skeleton_flags() { return 0; }

} // namespace l2kw

/* ---- exports ---- */
static void fill_table(vp4u_api_table *t) {
    t->GetCurrentTimestamp = api_GetCurrentTimestamp;
    t->RegisterLoggerCallback = api_RegisterLoggerCallback;
    t->UnregisterLoggerCallback = api_UnregisterLoggerCallback;
    t->AddRefImageDesc = api_AddRefImageDesc;
    t->ReleaseImageDesc = api_ReleaseImageDesc;
    t->Initialize = api_Initialize;
    t->Shutdown = api_Shutdown;
    t->LoadConfig = api_LoadConfig;
    t->ReloadConfig = api_ReloadConfig;
    t->OpenRealSense = api_OpenRealSense;
    t->RealSenseLeftIsActive = api_RealSenseLeftIsActive;
    t->RealSenseRightIsActive = api_RealSenseRightIsActive;
    t->TryRebootRealSense = api_TryRebootRealSense;
    t->TryRebuildRealSenseFilters = api_TryRebuildRealSenseFilters;
    t->InitVisionPoseRealtime = api_InitVisionPoseRealtime;
    t->InitVisionPoseOffline = api_InitVisionPoseOffline;
    t->StartRealtimeCameraPreview = api_StartRealtimeCameraPreview;
    t->StopRealtimeCameraPreview = api_StopRealtimeCameraPreview;
    t->StartStereoCameraCalibration = api_StartStereoCameraCalibration;
    t->CommitStereoCameraCalibration = api_CommitStereoCameraCalibration;
    t->CloseStereoCameraCalibration = api_CloseStereoCameraCalibration;
    t->StartRealtimeAnalysis = api_StartRealtimeAnalysis;
    t->StopRealtimeAnalysis = api_StopRealtimeAnalysis;
    t->PrepareVideoRecording = api_PrepareVideoRecording;
    t->StartVideoRecording = api_StartVideoRecording;
    t->FinalizeVideoRecording = api_FinalizeVideoRecording;
    t->DeleteRecordedVideo = api_DeleteRecordedVideo;
    t->StartOfflineAnalysis = api_StartOfflineAnalysis;
    t->StopOfflineAnalysis = api_StopOfflineAnalysis;
}

extern "C" {

__declspec(dllexport) int32_t vp4uGetVersion(void) { return VP4U_API_VERSION; }

__declspec(dllexport) int32_t vp4uGetApiTable(int32_t version, vp4u_api_table *table, void *log_cb, void *log_ctx) {
    ensure_started();
    if (log_cb) add_sink(log_cb, log_ctx);
    if (version != VP4U_API_VERSION) {
        glog("vp4uGetApiTable: version %d asked, this wrapper is %d -> 0", version, VP4U_API_VERSION);
        return 0;
    }
    if (!table) {
        glog("vp4uGetApiTable: no table -> 0");
        return 0;
    }
    fill_table(table);
    glog("vp4uGetApiTable(%d) -> 1", version);
    return 1;
}

/* Called by the boot loader (ee-config.xml <vp4u>) before the game starts. Under spice2x the first
 * argument is the text of VisionPoseForUnityConfig.json, not its path (seen in the game). */
__declspec(dllexport) int32_t vp4uPreboot(const char *config, void *, void *) {
    ensure_started();
    size_t n = config ? strlen(config) : 0;
    const char *first = config ? config : "";
    while (*first == ' ' || *first == '\t' || *first == '\r' || *first == '\n') first++;
    glog("vp4uPreboot(%s, %zu bytes) -> 1", *first == '{' ? "config text" : first, n);
    return 1;
}

__declspec(dllexport) void vp4uShutdown(void) { api_Shutdown(); }

BOOL WINAPI DllMain(HINSTANCE module, DWORD reason, LPVOID) {
    if (reason == DLL_PROCESS_ATTACH) {
        DisableThreadLibraryCalls(module);
        InitializeCriticalSection(&g_log_cs);
        init_module(module, L2K_CONSUMER_UNKNOWN, "vp4u visionposewrapper.dll (DanceAround)");
    }
    return TRUE;
}

} // extern "C"
