/*
 * 2a: l2k_spicehook.dll - the same core, loaded by spice2x with `-k l2k_spicehook.dll`.
 *
 * spice2x loads gamekdm.dll itself before it loads -k hooks (windows/docs/spice2x.md), so by the
 * time this DllMain runs gamekdm.dll's imports from Kinect10.dll are bound to whatever
 * Kinect10.dll the loader found. This rewrites those import slots (by ordinal or by name) to
 * point at the core instead. The game's own code has not run yet at that point.
 *
 * It needs *a* Kinect10.dll to exist for gamekdm.dll to load at all (Microsoft's runtime, or the
 * replacement from 2b); it only takes over the calls.
 */
#include "l2kw.h"

#include <stdio.h>
#include <string.h>

using namespace l2kw;

struct Entry {
    WORD ordinal;
    const char *name;
    void *fn;
};

static const Entry k_entries[] = {
    {1, nullptr, (void *)&K_NuiOrdinal1},
    {5, "NuiGetSensorCount", (void *)&K_NuiGetSensorCount},
    {6, "NuiCreateSensorByIndex", (void *)&K_NuiCreateSensorByIndex},
    {7, "NuiCameraElevationGetAngle", (void *)&K_NuiCameraElevationGetAngle},
    {8, "NuiCameraElevationSetAngle", (void *)&K_NuiCameraElevationSetAngle},
    {9, "NuiImageGetColorPixelCoordinatesFromDepthPixel", (void *)&K_NuiImageGetColorPixelCoordinatesFromDepthPixel},
    {10, "NuiImageGetColorPixelCoordinatesFromDepthPixelAtResolution", (void *)&K_NuiImageGetColorPixelCoordinatesFromDepthPixelAtResolution},
    {11, "NuiImageStreamGetNextFrame", (void *)&K_NuiImageStreamGetNextFrame},
    {12, "NuiImageStreamOpen", (void *)&K_NuiImageStreamOpen},
    {13, "NuiImageStreamReleaseFrame", (void *)&K_NuiImageStreamReleaseFrame},
    {14, "NuiInitialize", (void *)&K_NuiInitialize},
    {15, "NuiSetFrameEndEvent", (void *)&K_NuiSetFrameEndEvent},
    {16, "NuiShutdown", (void *)&K_NuiShutdown},
    {17, "NuiSkeletonGetNextFrame", (void *)&K_NuiSkeletonGetNextFrame},
    {18, "NuiSkeletonTrackingDisable", (void *)&K_NuiSkeletonTrackingDisable},
    {19, "NuiSkeletonTrackingEnable", (void *)&K_NuiSkeletonTrackingEnable},
    {20, "NuiTransformSmooth", (void *)&K_NuiTransformSmooth},
    {21, "NuiCreateSensorById", (void *)&K_NuiCreateSensorById},
    {22, "NuiSetDeviceStatusCallback", (void *)&K_NuiSetDeviceStatusCallback},
    {23, "NuiImageStreamSetImageFrameFlags", (void *)&K_NuiImageStreamSetImageFrameFlags},
    {24, "NuiImageStreamGetImageFrameFlags", (void *)&K_NuiImageStreamGetImageFrameFlags},
    {25, "NuiGetAudioSource", (void *)&K_NuiGetAudioSource},
    {26, "NuiSkeletonSetTrackedSkeletons", (void *)&K_NuiSkeletonSetTrackedSkeletons},
    {27, "NuiSkeletonCalculateBoneOrientations", (void *)&K_NuiSkeletonCalculateBoneOrientations},
    {28, "NuiCreateCoordinateMapperFromParameters", (void *)&K_NuiCreateCoordinateMapperFromParameters},
    {29, "NuiCreateDepthFilter", (void *)&K_NuiCreateDepthFilter},
};

static void *lookup(WORD ordinal, const char *name) {
    for (const Entry &e : k_entries)
        if ((name && e.name && !strcmp(name, e.name)) || (!name && e.ordinal == ordinal)) return e.fn;
    return nullptr;
}

/* Rewrite `module`'s imports from Kinect10.dll. Returns the number of slots patched, -1 if the
 * module does not import Kinect10.dll. */
static int patch_imports(HMODULE module, char *report, size_t cap) {
    BYTE *base = (BYTE *)module;
    IMAGE_DOS_HEADER *dos = (IMAGE_DOS_HEADER *)base;
    IMAGE_NT_HEADERS32 *nt = (IMAGE_NT_HEADERS32 *)(base + dos->e_lfanew);
    IMAGE_DATA_DIRECTORY dir = nt->OptionalHeader.DataDirectory[IMAGE_DIRECTORY_ENTRY_IMPORT];
    int patched = -1;
    report[0] = 0;
    if (!dir.VirtualAddress) return -1;
    for (IMAGE_IMPORT_DESCRIPTOR *d = (IMAGE_IMPORT_DESCRIPTOR *)(base + dir.VirtualAddress); d->Name; d++) {
        if (_stricmp((const char *)(base + d->Name), "Kinect10.dll")) continue;
        patched = 0;
        IMAGE_THUNK_DATA32 *names = (IMAGE_THUNK_DATA32 *)(base + (d->OriginalFirstThunk ? d->OriginalFirstThunk : d->FirstThunk));
        IMAGE_THUNK_DATA32 *slots = (IMAGE_THUNK_DATA32 *)(base + d->FirstThunk);
        for (; names->u1.AddressOfData; names++, slots++) {
            WORD ordinal = 0;
            const char *name = nullptr;
            if (IMAGE_SNAP_BY_ORDINAL32(names->u1.Ordinal)) ordinal = (WORD)IMAGE_ORDINAL32(names->u1.Ordinal);
            else name = (const char *)((IMAGE_IMPORT_BY_NAME *)(base + names->u1.AddressOfData))->Name;
            void *fn = lookup(ordinal, name);
            size_t used = strlen(report);
            if (!fn) {
                snprintf(report + used, cap - used, " %s%u:unknown", name ? name : "#", name ? 0u : ordinal);
                continue;
            }
            DWORD old;
            if (VirtualProtect(&slots->u1.Function, sizeof(DWORD), PAGE_READWRITE, &old)) {
                slots->u1.Function = (DWORD)(uintptr_t)fn;
                VirtualProtect(&slots->u1.Function, sizeof(DWORD), old, &old);
                patched++;
                if (name) snprintf(report + used, cap - used, " %s", name);
                else snprintf(report + used, cap - used, " #%u", ordinal);
            }
        }
    }
    return patched;
}

BOOL WINAPI DllMain(HINSTANCE module, DWORD reason, LPVOID) {
    if (reason != DLL_PROCESS_ATTACH) return TRUE;
    DisableThreadLibraryCalls(module);
    init_module(module, L2K_CONSUMER_SPICE_HOOK);

    /* Open the log now (config + file only; the receiver thread starts at the game's first call). */
    config_load(module, L2K_CONSUMER_SPICE_HOOK);
    log_open();

    HMODULE game = GetModuleHandleA("gamekdm.dll");
    HMODULE real = GetModuleHandleA("Kinect10.dll");
    char where[MAX_PATH] = "(not loaded)";
    if (real) GetModuleFileNameA(real, where, sizeof where);
    log("spicehook: Kinect10.dll in the process: %s", where);
    if (!game) {
        log("spicehook: gamekdm.dll is not loaded - load this hook with -k (after the game DLL), not -z. Nothing redirected.");
        return TRUE;
    }
    char report[1024];
    int n = patch_imports(game, report, sizeof report);
    if (n < 0) log("spicehook: gamekdm.dll does not import Kinect10.dll - nothing redirected");
    else log("spicehook: redirected %d Kinect10.dll import(s) of gamekdm.dll:%s", n, report);
    return TRUE;
}

/* exported so spice2x's SDK loader and `dumpbin /exports` see what this is; not required */
extern "C" __declspec(dllexport) const char *l2k_spicehook_version() { return "Lidar2Kinect spice2x hook, protocol L2K1"; }
