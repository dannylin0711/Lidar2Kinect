/* Configuration file, log, clock. */
#include "l2kw_base.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

namespace l2kw {

static Config g_config;
static char g_product[48];
static FILE *g_log;
static CRITICAL_SECTION g_log_cs;
static LARGE_INTEGER g_freq, g_t0;

uint64_t now_us() {
    LARGE_INTEGER c;
    if (!g_freq.QuadPart) QueryPerformanceFrequency(&g_freq);
    QueryPerformanceCounter(&c);
    return (uint64_t)(c.QuadPart / g_freq.QuadPart) * 1000000ull + (uint64_t)(c.QuadPart % g_freq.QuadPart) * 1000000ull / (uint64_t)g_freq.QuadPart;
}

double uptime_s() {
    LARGE_INTEGER c;
    QueryPerformanceCounter(&c);
    return (double)(c.QuadPart - g_t0.QuadPart) / (double)g_freq.QuadPart;
}

const Config &config() { return g_config; }

void set_product_name(const char *product) { snprintf(g_product, sizeof g_product, "%s", product ? product : ""); }

static void module_dir(HMODULE module, char *out, size_t cap) {
    char *slash;
    DWORD n = GetModuleFileNameA(module, out, (DWORD)cap);
    if (n == 0 || n >= cap) {
        out[0] = 0;
        return;
    }
    /* a loader that maps the module through a virtual file system may report it with '/' */
    slash = strrchr(out, '\\');
    char *fwd = strrchr(out, '/');
    if (fwd && (!slash || fwd > slash)) slash = fwd;
    if (slash) slash[1] = 0;
    else out[0] = 0;
}

/* relative paths are relative to the DLL's folder */
static void resolve(const char *dir, const char *path, char *out, size_t cap) {
    if (!path[0]) out[0] = 0;
    else if (path[1] == ':' || (path[0] == '\\' && path[1] == '\\')) snprintf(out, cap, "%s", path);
    else snprintf(out, cap, "%s%s", dir, path);
}

static int pick(const char *value, const char *const *names, int count, int fallback) {
    for (int i = 0; i < count; i++)
        if (!_stricmp(value, names[i])) return i;
    return fallback;
}

void config_load(HMODULE module, unsigned consumer_kind) {
    static const char *const no_sender_names[] = {"missing", "idle"};
    static const char *const depth_names[] = {"auto", "sender", "skeleton"};
    static const char *const mode_names[] = {"players", "full"};
    char dir[MAX_PATH], buf[MAX_PATH], log_rel[MAX_PATH];
    const char *env;
    Config &c = g_config;
    static bool loaded;

    if (loaded) return;
    loaded = true;
    QueryPerformanceFrequency(&g_freq);
    QueryPerformanceCounter(&g_t0);
    InitializeCriticalSection(&g_log_cs);

    module_dir(module, dir, sizeof dir);
    env = getenv("L2K_INI");
    if (env && env[0]) snprintf(c.ini_path, sizeof c.ini_path, "%s", env);
    else snprintf(c.ini_path, sizeof c.ini_path, "%sl2k.ini", dir);

    const char *ini = c.ini_path, *sec = "l2k";
    c.port = GetPrivateProfileIntA(sec, "port", L2K_DEFAULT_PORT, ini);
    GetPrivateProfileStringA(sec, "log", "l2k.log", log_rel, sizeof log_rel, ini);
    GetPrivateProfileStringA(sec, "no_sender", "missing", buf, sizeof buf, ini);
    c.no_sender = pick(buf, no_sender_names, 2, NO_SENDER_MISSING);
    GetPrivateProfileStringA(sec, "depth", "auto", buf, sizeof buf, ini);
    c.depth_source = pick(buf, depth_names, 3, DEPTH_AUTO);
    GetPrivateProfileStringA(sec, "depth_mode", "players", buf, sizeof buf, ini);
    c.depth_mode = pick(buf, mode_names, 2, L2K_DEPTH_MODE_PLAYERS);
    c.color_quality = GetPrivateProfileIntA(sec, "color_quality", 0, ini);
    if (c.color_quality < 0 || c.color_quality > 100) c.color_quality = 0;
    c.dnssd = GetPrivateProfileIntA(sec, "dnssd", 1, ini) != 0;
    c.verbose = GetPrivateProfileIntA(sec, "verbose", 0, ini) != 0;
    {
        char host[MAX_COMPUTERNAME_LENGTH + 1];
        DWORD n = sizeof host;
        if (!GetComputerNameA(host, &n)) strcpy(host, "PC");
        GetPrivateProfileStringA(sec, "name", host, c.name, sizeof c.name, ini);
    }
    c.consumer_kind = consumer_kind;

    env = getenv("L2K_PORT");
    if (env && atoi(env) > 0 && atoi(env) < 65536) c.port = (unsigned)atoi(env);
    env = getenv("L2K_LOG");
    if (env) snprintf(log_rel, sizeof log_rel, "%s", env);
    if (!_stricmp(log_rel, "none") || !_stricmp(log_rel, "off")) log_rel[0] = 0;
    resolve(dir, log_rel, c.log_path, sizeof c.log_path);
}

void log_open() {
    char exe[MAX_PATH];
    if (!g_config.log_path[0] || g_log) return;
    g_log = fopen(g_config.log_path, "w");
    if (!g_log) return;
    GetModuleFileNameA(NULL, exe, sizeof exe);
    log("Lidar2Kinect %s, consumer kind %u, process %s (pid %lu)",
        g_product[0] ? g_product : g_config.consumer_kind == L2K_CONSUMER_SPICE_HOOK ? "spice2x hook" : "Kinect10.dll", g_config.consumer_kind,
        exe, GetCurrentProcessId());
    log("config %s: port %u, no_sender %s, depth %s, depth_mode %d, color_quality %d, dnssd %d, verbose %d, name \"%s\"", g_config.ini_path,
        g_config.port, g_config.no_sender == NO_SENDER_IDLE ? "idle" : "missing",
        g_config.depth_source == DEPTH_SENDER ? "sender" : g_config.depth_source == DEPTH_SKELETON ? "skeleton" : "auto", g_config.depth_mode,
        g_config.color_quality, g_config.dnssd, g_config.verbose, g_config.name);
}

bool log_is_open() { return g_log != NULL; }

bool log_open_at(const char *path) {
    if (g_log || !path || !path[0]) return g_log != NULL;
    FILE *f = fopen(path, "w");
    if (!f) return false;
    snprintf(g_config.log_path, sizeof g_config.log_path, "%s", path);
    g_log = f;
    log("Lidar2Kinect %s: log opened at %s", g_product[0] ? g_product : "core", path);
    return true;
}

void log(const char *fmt, ...) {
    va_list ap;
    if (!g_log) return;
    EnterCriticalSection(&g_log_cs);
    fprintf(g_log, "[%9.3f %5lu] ", uptime_s(), GetCurrentThreadId());
    va_start(ap, fmt);
    vfprintf(g_log, fmt, ap);
    va_end(ap);
    fputc('\n', g_log);
    fflush(g_log);
    LeaveCriticalSection(&g_log_cs);
}

} // namespace l2kw
