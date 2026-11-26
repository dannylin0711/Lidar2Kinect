/*
 * vp4u_probe - drives visionposewrapper.dll the way DanceAround's VisionPoseForUnity.dll does
 * (call order and arguments: windows/docs/vp4u-interface.md "Boot sequence") and checks what comes
 * back. With --sender it starts l2k_fake_sender on loopback and checks the skeleton stream too.
 *
 *   vp4u_probe [--check] [-t seconds] [--players n] [--sender path\l2k_fake_sender.exe] [--dll path]
 *
 * The DLL and the sender use the port in L2K_PORT (default 24810).
 */
#include "vp4u_abi.h"

#include <windows.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>

static int g_checks, g_failed;
static bool g_verbose = true;
#define CHECK(c)                                                              \
    do {                                                                      \
        g_checks++;                                                           \
        if (!(c)) {                                                           \
            g_failed++;                                                       \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c);               \
        }                                                                     \
    } while (0)

static vp4u_api_table T;

/* ---- what the callbacks saw ---- */
static CRITICAL_SECTION g_cs;
struct PoseStats {
    LONG frames, with_body, in_area, all_joints, bad_image, bad_time, back_time, end;
    uint64_t last_ts;
    vp4u_joint last_spine;
    int64_t last_id;
};
static PoseStats g_pose, g_off;
static LONG g_prev_left, g_prev_right, g_calib_preview, g_calib_calls, g_log_lines;
static int g_calib_last_status = -1, g_calib_last_cur = -1, g_calib_last_total = -1;
static std::string g_log_text;

static const float kAreaCenter[3] = {0.55f, 0.0f, 2.30f};
static const float kAreaHalf[3] = {0.9f, 1.1f, 0.505f};

static bool image_ok(const vp4u_image_desc *d) {
    return d && d->pixels && d->width > 0 && d->height > 0 && d->bytes_per_pixel == 3 && d->bits_per_pixel == 24 &&
           d->bytes_per_row == d->width * 3 && d->bytes_per_frame == d->width * d->height * 3;
}

static void on_log(void *, const char *msg) {
    EnterCriticalSection(&g_cs);
    g_log_lines++;
    g_log_text += msg;
    g_log_text += "\n";
    LeaveCriticalSection(&g_cs);
}

static void count_pose(PoseStats &s, uint64_t ts, const vp4u_body *p3d, int32_t n3d, const vp4u_image_desc *mv, const vp4u_image_desc *sv) {
    uint64_t now = T.GetCurrentTimestamp();
    EnterCriticalSection(&g_cs);
    if (!mv) {
        s.end++;
        LeaveCriticalSection(&g_cs);
        return;
    }
    s.frames++;
    if (!image_ok(mv) || !image_ok(sv)) s.bad_image++;
    if (ts > now + 1000 || now - ts > 100000) s.bad_time++;
    if (s.last_ts && ts < s.last_ts) s.back_time++;
    s.last_ts = ts;
    if (n3d > 0 && p3d) {
        s.with_body++;
        const vp4u_joint &sb = p3d[0].joints[VP4U_SPINE_BASE];
        if (fabsf(sb.x - kAreaCenter[0]) <= kAreaHalf[0] && fabsf(sb.y - kAreaCenter[1]) <= kAreaHalf[1] && fabsf(sb.z - kAreaCenter[2]) <= kAreaHalf[2])
            s.in_area++;
        bool all = p3d[0].state > 0;
        for (int j = 0; j < VP4U_JOINT_COUNT; j++) all = all && p3d[0].joints[j].state > 0;
        if (all) s.all_joints++;
        s.last_spine = sb;
        s.last_id = p3d[0].tracking_id;
    }
    LeaveCriticalSection(&g_cs);
    /* the C# side clones the frame's images and releases them later */
    T.AddRefImageDesc((vp4u_image_desc *)mv);
    T.ReleaseImageDesc((vp4u_image_desc *)mv);
}

static void on_pose(void *ctx, uint64_t ts, const vp4u_body *p3d, int32_t n3d, const vp4u_body *, int32_t, const vp4u_body *, int32_t,
                    const vp4u_body *, int32_t, const vp4u_image_desc *mv, const vp4u_image_desc *sv) {
    CHECK(ctx == (void *)0x1001);
    count_pose(g_pose, ts, p3d, n3d, mv, sv);
}
static void on_offline(void *ctx, uint64_t ts, const vp4u_body *p3d, int32_t n3d, const vp4u_body *, int32_t, const vp4u_body *, int32_t,
                       const vp4u_body *, int32_t, const vp4u_image_desc *mv, const vp4u_image_desc *sv) {
    CHECK(ctx == (void *)0x1002);
    count_pose(g_off, ts, p3d, n3d, mv, sv);
}
static void on_prev(void *ctx, uint64_t, const vp4u_image_desc *img) {
    CHECK(image_ok(img));
    if (ctx == (void *)0x2001) InterlockedIncrement(&g_prev_left);
    else if (ctx == (void *)0x2002) InterlockedIncrement(&g_prev_right);
    else CHECK(!"preview callback with a wrong ctx");
}
static void on_calib_prev(void *ctx, uint64_t, const vp4u_image_desc *img) {
    CHECK(ctx == (void *)0x3001);
    CHECK(image_ok(img));
    InterlockedIncrement(&g_calib_preview);
}
static void on_calib(void *ctx, int32_t status, int32_t cur, int32_t total, const char *msg) {
    CHECK(ctx == (void *)0x3001);
    CHECK(msg != NULL);
    EnterCriticalSection(&g_cs);
    g_calib_calls++;
    g_calib_last_status = status;
    g_calib_last_cur = cur;
    g_calib_last_total = total;
    LeaveCriticalSection(&g_cs);
}

static PoseStats snapshot(PoseStats &s) {
    EnterCriticalSection(&g_cs);
    PoseStats c = s;
    LeaveCriticalSection(&g_cs);
    return c;
}
static void reset(PoseStats &s) {
    EnterCriticalSection(&g_cs);
    memset(&s, 0, sizeof s);
    LeaveCriticalSection(&g_cs);
}

static std::string exe_dir() {
    char path[MAX_PATH];
    GetModuleFileNameA(NULL, path, MAX_PATH);
    char *slash = strrchr(path, '\\');
    if (slash) slash[1] = 0;
    return path;
}

int main(int argc, char **argv) {
    double seconds = 10;
    int players = 1;
    bool check = false;
    std::string sender, dll = exe_dir() + "visionposewrapper.dll";
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--check")) check = true;
        else if (!strcmp(argv[i], "-t") && i + 1 < argc) seconds = atof(argv[++i]);
        else if (!strcmp(argv[i], "--players") && i + 1 < argc) players = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--sender") && i + 1 < argc) sender = argv[++i];
        else if (!strcmp(argv[i], "--dll") && i + 1 < argc) dll = argv[++i];
        else {
            fprintf(stderr, "usage: vp4u_probe [--check] [-t seconds] [--players n] [--sender l2k_fake_sender.exe] [--dll visionposewrapper.dll]\n");
            return 2;
        }
    }
    const char *port_env = getenv("L2K_PORT");
    unsigned port = port_env && atoi(port_env) > 0 ? (unsigned)atoi(port_env) : 24810;
    InitializeCriticalSection(&g_cs);
    setvbuf(stdout, NULL, _IONBF, 0);

    /* ---- load, like VisionPoseManager.LoadPluginModule + Vpw4U.LoadPlugin ---- */
    HMODULE m = LoadLibraryA(dll.c_str());
    if (!m) {
        printf("FAIL cannot load %s (%lu)\n", dll.c_str(), GetLastError());
        return 1;
    }
    auto get_table = (vp4u_get_api_table_fn)GetProcAddress(m, "vp4uGetApiTable");
    auto get_version = (vp4u_get_version_fn)GetProcAddress(m, "vp4uGetVersion");
    auto preboot = (vp4u_preboot_fn)GetProcAddress(m, "vp4uPreboot");
    auto shutdown = (vp4u_shutdown_fn)GetProcAddress(m, "vp4uShutdown");
    CHECK(get_table && get_version && preboot && shutdown);
    if (!get_table || !get_version || !preboot || !shutdown) return 1;
    CHECK(GetProcAddress(m, MAKEINTRESOURCEA(1)) == (FARPROC)get_table);
    CHECK(GetProcAddress(m, MAKEINTRESOURCEA(4)) == (FARPROC)shutdown);
    CHECK(get_version() == VP4U_API_VERSION);

    /* the boot loader's call (ee-config.xml <vp4u>) */
    CHECK(preboot("/vpprops/visionposeforunityconfig.json", NULL, NULL) == 1);

    vp4u_api_table wrong;
    memset(&wrong, 0, sizeof wrong);
    CHECK(get_table(VP4U_API_VERSION - 1, &wrong, (void *)on_log, NULL) == 0);
    CHECK(get_table(VP4U_API_VERSION, NULL, (void *)on_log, NULL) == 0);
    memset(&T, 0, sizeof T);
    CHECK(get_table(VP4U_API_VERSION, &T, (void *)on_log, NULL) == 1);
    void *const *slots = (void *const *)&T;
    for (int i = 0; i < 29; i++) CHECK(slots[i] != NULL);
    if (!T.Initialize) return 1;
    void *log_handle = T.RegisterLoggerCallback((void *)on_log, NULL);
    CHECK(log_handle != NULL);
    void *instance = T.Initialize();
    CHECK(instance != NULL);

    /* The game checks that dev/nvram/stereo_cam.yml exists (in its VFS; here: under a scratch
     * working directory). LoadConfig creates it when missing, Commit rewrites it. */
    char old_cwd[MAX_PATH], work[MAX_PATH], calib[MAX_PATH];
    GetCurrentDirectoryA(MAX_PATH, old_cwd);
    GetTempPathA(MAX_PATH, work);
    strcat(work, "vp4u_probe_cwd");
    CreateDirectoryA(work, NULL);
    snprintf(calib, sizeof calib, "%s\\dev", work);
    CreateDirectoryA(calib, NULL);
    snprintf(calib, sizeof calib, "%s\\dev\\nvram", work);
    CreateDirectoryA(calib, NULL);
    snprintf(calib, sizeof calib, "%s\\dev\\nvram\\stereo_cam.yml", work);
    DeleteFileA(calib);
    SetCurrentDirectoryA(work);

    /* ---- VisionPoseManager.InitializeAsync ---- */
    {
        char tmp[MAX_PATH], cfg[MAX_PATH];
        GetTempPathA(MAX_PATH, tmp);
        snprintf(cfg, sizeof cfg, "%svp4u_probe_config.json", tmp);
        FILE *f = fopen(cfg, "w");
        if (f) {
            fputs("{\n  \"RealSenseInputSettings\": {\n    \"LeftCameraPose\": {\n      \"X\": 0.30,\n      \"Y\": 1.16,\n      \"Z\": 3.20,\n"
                  "      \"Yaw\": 180.00,\n    },\n  },\n}\n",
                  f);
            fclose(f);
        }
        T.LoadConfig(cfg);
        EnterCriticalSection(&g_cs);
        bool saw = g_log_text.find("main camera height 1.16 m (LeftCameraPose.Y)") != std::string::npos;
        LeaveCriticalSection(&g_cs);
        CHECK(saw);
        DeleteFileA(cfg);
        CHECK(GetFileAttributesA(calib) != INVALID_FILE_ATTRIBUTES); /* placeholder created */
    }
    SetCurrentDirectoryA(old_cwd);
    CHECK(T.InitVisionPoseRealtime("PRODUCTKEY") == 1);
    CHECK(T.InitVisionPoseOffline("PRODUCTKEY") == 1);
    CHECK(T.RealSenseLeftIsActive() == 1); /* no_sender = idle (default) */
    CHECK(T.RealSenseRightIsActive() == 1);
    CHECK(T.OpenRealSense() == 1);
    CHECK(T.TryRebootRealSense(0) == 1);
    CHECK(T.TryRebuildRealSenseFilters() == 1);
    uint64_t t0 = T.GetCurrentTimestamp();
    Sleep(20);
    CHECK(T.GetCurrentTimestamp() >= t0 + 15000);

    /* ---- realtime analysis without a sender: empty frames keep coming ---- */
    CHECK(T.StartRealtimeAnalysis((void *)on_pose, (void *)0x1001) == 1);
    Sleep(1000);
    PoseStats s = snapshot(g_pose);
    printf("no sender: %ld frames/s, %ld with a body\n", s.frames, s.with_body);
    CHECK(s.frames >= 20);
    CHECK(s.with_body == 0);
    CHECK(s.bad_image == 0 && s.bad_time == 0 && s.back_time == 0);

    /* ---- with the fake sender ---- */
    PROCESS_INFORMATION pi;
    memset(&pi, 0, sizeof pi);
    if (!sender.empty()) {
        char cmd[1024];
        snprintf(cmd, sizeof cmd, "\"%s\" -p %u -n %d -t %d 127.0.0.1", sender.c_str(), port, players, (int)seconds + 30);
        STARTUPINFOA si;
        memset(&si, 0, sizeof si);
        si.cb = sizeof si;
        si.dwFlags = STARTF_USESTDHANDLES;
        si.hStdOutput = si.hStdError = GetStdHandle(STD_ERROR_HANDLE);
        if (!CreateProcessA(NULL, cmd, NULL, NULL, TRUE, 0, NULL, NULL, &si, &pi)) {
            printf("FAIL cannot start %s (%lu)\n", cmd, GetLastError());
            g_failed++;
        } else {
            printf("sender: %s\n", cmd);
            /* wait for the first body */
            for (int i = 0; i < 60 && snapshot(g_pose).with_body == 0; i++) Sleep(100);
            CHECK(snapshot(g_pose).with_body > 0);
            Sleep(300);
            reset(g_pose);
            double done = 0;
            while (done < seconds) {
                double step = seconds - done < 1 ? seconds - done : 1;
                Sleep((DWORD)(step * 1000));
                done += step;
                s = snapshot(g_pose);
                if (g_verbose)
                    printf("  %.0f s: %ld frames, %ld with a body, %ld in the play area, %ld all 30 joints; SpineBase (%.2f, %.2f, %.2f) id %lld\n",
                           done, s.frames, s.with_body, s.in_area, s.all_joints, s.last_spine.x, s.last_spine.y, s.last_spine.z, (long long)s.last_id);
            }
            s = snapshot(g_pose);
            double rate = s.frames / seconds;
            printf("sender: %.1f pose frames/s, %.0f%% with a body, %.0f%% of those in the play area, %.0f%% with all 30 joints\n", rate,
                   s.frames ? 100.0 * s.with_body / s.frames : 0.0, s.with_body ? 100.0 * s.in_area / s.with_body : 0.0,
                   s.with_body ? 100.0 * s.all_joints / s.with_body : 0.0);
            CHECK(rate >= 27);
            CHECK(s.with_body >= s.frames * 95 / 100);
            CHECK(s.in_area == s.with_body);
            CHECK(s.all_joints == s.with_body);
            CHECK(s.bad_image == 0 && s.bad_time == 0 && s.back_time == 0);
        }
    }

    /* ---- camera preview (test mode) ---- */
    CHECK(T.StartRealtimeCameraPreview((void *)on_prev, (void *)0x2001, (void *)on_prev, (void *)0x2002) == 1);
    Sleep(1000);
    LONG l = g_prev_left, r = g_prev_right;
    printf("preview: %ld / %ld frames in 1 s\n", l, r);
    CHECK(l >= 20 && r >= 20);
    T.StopRealtimeCameraPreview();
    l = g_prev_left;
    r = g_prev_right;
    Sleep(200);
    CHECK(g_prev_left == l && g_prev_right == r);

    /* ---- stop: no frame after StopRealtimeAnalysis returns ---- */
    T.StopRealtimeAnalysis();
    LONG frames_at_stop = snapshot(g_pose).frames;
    Sleep(200);
    CHECK(snapshot(g_pose).frames == frames_at_stop);

    /* ---- stereo calibration (test mode) ---- */
    CHECK(T.StartStereoCameraCalibration((void *)on_calib, (void *)on_calib_prev, (void *)0x3001) == 1);
    Sleep(1500);
    EnterCriticalSection(&g_cs);
    int st = g_calib_last_status, cur = g_calib_last_cur, tot = g_calib_last_total;
    LeaveCriticalSection(&g_cs);
    printf("calibration: %ld status calls, last %d (%d/%d), %ld preview frames\n", g_calib_calls, st, cur, tot, g_calib_preview);
    CHECK(st == VP4U_CALIB_SUCCESS && cur == tot && tot > 0);
    CHECK(g_calib_preview >= 20);
    DeleteFileA(calib);
    SetCurrentDirectoryA(work);
    CHECK(T.CommitStereoCameraCalibration() == 1);
    SetCurrentDirectoryA(old_cwd);
    CHECK(GetFileAttributesA(calib) != INVALID_FILE_ATTRIBUTES); /* rewritten on commit */
    DeleteFileA(calib);
    RemoveDirectoryA((std::string(work) + "\\dev\\nvram").c_str());
    RemoveDirectoryA((std::string(work) + "\\dev").c_str());
    RemoveDirectoryA(work);
    T.CloseStereoCameraCalibration();
    LONG cp = g_calib_preview;
    Sleep(200);
    CHECK(g_calib_preview == cp);

    /* ---- play video recording and offline analysis ---- */
    CHECK(T.PrepareVideoRecording("probe.mp4") == 1);
    CHECK(T.StartVideoRecording() == 1);
    CHECK(T.FinalizeVideoRecording() == 1);
    CHECK(T.StartOfflineAnalysis("probe.mp4", (void *)on_offline, (void *)0x1002) == 1);
    for (int i = 0; i < 20 && snapshot(g_off).end == 0; i++) Sleep(50);
    CHECK(snapshot(g_off).end == 1);
    T.StopOfflineAnalysis();
    CHECK(T.DeleteRecordedVideo("probe.mp4") == 1);

    /* ---- the no-op parts and the end ---- */
    T.ReloadConfig();
    T.UnregisterLoggerCallback(log_handle);
    T.Shutdown();
    shutdown();
    printf("log lines from the DLL: %ld\n", g_log_lines);
    CHECK(g_log_lines > 10);

    if (pi.hProcess) {
        TerminateProcess(pi.hProcess, 0);
        CloseHandle(pi.hProcess);
        CloseHandle(pi.hThread);
    }
    printf("%d checks, %d failed\n", g_checks, g_failed);
    return (check && g_failed) ? 1 : 0;
}
