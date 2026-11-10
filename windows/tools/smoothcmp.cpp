/*
 * l2k_smoothcmp - compares l2k_smoother_apply (shared/) with NuiTransformSmooth of Microsoft's
 * Kinect10.dll (ordinal 20) on the same synthetic skeleton frames. No sensor is needed for that
 * call (checked by this tool: it reports the HRESULT).
 *
 *   l2k_smoothcmp [path\to\Kinect10.dll] [-n frames] [-noise metres] [-csv file]
 *
 * Default DLL: %SystemRoot%\SysWOW64\Kinect10.dll (this is a 32-bit program).
 */
#include "../core/l2kw.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef HRESULT(WINAPI *SmoothFn)(NUI_SKELETON_FRAME *, const NUI_TRANSFORM_SMOOTH_PARAMETERS *);

/*
 * Without a sensor the flat NuiTransformSmooth returns E_NUI_NOTCONNECTED and leaves the frame
 * alone: it looks up the default sensor and calls its INuiSensor::NuiTransformSmooth, which only
 * forwards to an internal routine with the sensor's filter state (an assertion string in the DLL
 * names it "_NuiTransformSmooth(m_ppSkeletonMotionFilterState, pSkeletonFrame, pSmoothingParams)").
 * In Kinect10.dll 1.8.0.595 that routine is at RVA 0x3F9D0; it takes the state (6 pointers, one
 * 0x414-byte block per skeleton slot) and the frame on the stack, the parameters in ECX, and the
 * caller pops the stack. Calling it with our own state block gives the DLL's filter without a
 * sensor. Only for that exact file (checked by size and the export's RVA).
 */
static const DWORD k_internal_rva = 0x3F9D0;
static void *g_internal;
static void *g_state_ptrs[6];

static HRESULT internal_smooth(NUI_SKELETON_FRAME *frame, const NUI_TRANSFORM_SMOOTH_PARAMETERS *params) {
    HRESULT hr;
    void *fn = g_internal;
    void **state = g_state_ptrs;
    __asm {
        push frame
        push state
        mov ecx, params
        call fn
        add esp, 8
        mov hr, eax
    }
    return hr;
}

static void internal_reset() {
    for (void *p : g_state_ptrs) memset(p, 0, 0x414);
}

static uint32_t g_rng = 12345;
static float noise(float amp) {
    g_rng = g_rng * 1664525u + 1013904223u;
    return amp * (((float)(g_rng >> 8) / 16777216.0f) * 2.0f - 1.0f);
}

struct Case {
    const char *name;
    NUI_TRANSFORM_SMOOTH_PARAMETERS p;
    float noise;
    int jump_every; /* frames between 0.3 m jumps of the whole body (0 = none) */
};

int main(int argc, char **argv) {
    char dll[MAX_PATH];
    int frames = 300;
    float user_noise = -1;
    const char *csv_path = NULL;
    GetWindowsDirectoryA(dll, sizeof dll);
    strcat(dll, "\\SysWOW64\\Kinect10.dll");
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "-n") && i + 1 < argc) frames = atoi(argv[++i]);
        else if (!strcmp(argv[i], "-noise") && i + 1 < argc) user_noise = (float)atof(argv[++i]);
        else if (!strcmp(argv[i], "-csv") && i + 1 < argc) csv_path = argv[++i];
        else snprintf(dll, sizeof dll, "%s", argv[i]);
    }
    HMODULE k = LoadLibraryA(dll);
    if (!k) {
        fprintf(stderr, "cannot load %s (%lu)\n", dll, GetLastError());
        return 1;
    }
    SmoothFn real = (SmoothFn)GetProcAddress(k, MAKEINTRESOURCEA(20));
    if (!real) {
        fprintf(stderr, "%s has no ordinal 20\n", dll);
        return 1;
    }
    {
        char ver[MAX_PATH];
        GetModuleFileNameA(k, ver, sizeof ver);
        printf("reference: %s, ordinal 20 at %p\n", ver, (void *)real);
    }
    /* is the internal routine where we expect it? (1.8.0.595: ordinal 20 at RVA 0x40420) */
    bool use_internal = (BYTE *)real - (BYTE *)k == 0x40420;
    if (use_internal) {
        g_internal = (BYTE *)k + k_internal_rva;
        for (void *&p : g_state_ptrs) p = calloc(1, 0x414);
        printf("using the DLL's internal smoothing routine at %p (the flat function needs a sensor)\n", g_internal);
    } else {
        printf("unknown Kinect10.dll build: only the flat function is tried\n");
    }
    FILE *csv = csv_path ? fopen(csv_path, "w") : NULL;
    if (csv) fprintf(csv, "case,frame,joint,raw_x,raw_y,raw_z,ref_x,ref_y,ref_z,l2k_x,l2k_y,l2k_z\n");

    Case cases[] = {
        {"game (0.5 0.5 0.5 0.05 0.04), clean", {0.5f, 0.5f, 0.5f, 0.05f, 0.04f}, 0.0f, 0},
        {"game, 1 cm noise", {0.5f, 0.5f, 0.5f, 0.05f, 0.04f}, 0.01f, 0},
        {"game, 5 cm noise + 0.3 m jumps", {0.5f, 0.5f, 0.5f, 0.05f, 0.04f}, 0.05f, 45},
        {"smoothing 0 (pass-through?)", {0.0f, 0.0f, 0.0f, 0.0f, 0.0f}, 0.01f, 0},
        {"heavy (0.7 0.3 1.0 1.0 1.0)", {0.7f, 0.3f, 1.0f, 1.0f, 1.0f}, 0.02f, 0},
    };
    int worst = 0;
    for (const Case &c : cases) {
        l2k_mapper m;
        l2k_smoother ours;
        l2k_smooth_params p = {c.p.fSmoothing, c.p.fCorrection, c.p.fPrediction, c.p.fJitterRadius, c.p.fMaxDeviationRadius};
        double sum = 0, max = 0, sum_raw = 0;
        long n = 0;
        int max_frame = -1, max_joint = -1;
        HRESULT first_hr = S_OK;
        float amp = user_noise >= 0 ? user_noise : c.noise;
        l2k_mapper_init(&m, NULL);
        l2k_smoother_init(&ours);
        if (use_internal) internal_reset();
        g_rng = 12345;
        /* the real DLL keeps its history per skeleton slot / tracking id: give each case new ids */
        static uint32_t id_base = 100;
        id_base += 10;
        for (int f = 0; f < frames; f++) {
            l2k_src_body bodies[2];
            l2k_skeleton_frame sf, ours_frame;
            l2k_vec3 up = {0, 1, 0};
            for (int b = 0; b < 2; b++) {
                l2k_synth_body(f / 30.0, b, 2, &bodies[b]);
                bodies[b].tracking_id = id_base + (uint32_t)b;
                for (int j = 0; j < L2K_SRC_JOINT_COUNT; j++) {
                    bodies[b].p[j].x += noise(amp);
                    bodies[b].p[j].y += noise(amp);
                    bodies[b].p[j].z += noise(amp);
                    if (c.jump_every && (f / c.jump_every) % 2) bodies[b].p[j].x += 0.3f;
                }
            }
            l2k_mapper_map(&m, bodies, 2, up, 0, (uint64_t)f * 33333, L2K_SOURCE_SYNTHETIC, &sf);
            NUI_SKELETON_FRAME nf, raw;
            l2kw::to_nui(sf, nf, (LONGLONG)f * 33);
            raw = nf;
            HRESULT hr = real(&nf, &c.p);
            if (f == 0) first_hr = hr;
            if (FAILED(hr) && use_internal) {
                nf = raw;
                hr = internal_smooth(&nf, &c.p);
                if (f == 0) first_hr = hr;
            }
            ours_frame = sf;
            l2k_smoother_apply(&ours, &ours_frame, &p);
            for (int s = 0; s < NUI_SKELETON_COUNT; s++) {
                if (nf.SkeletonData[s].eTrackingState != 2) continue;
                for (int j = 0; j < NUI_SKELETON_POSITION_COUNT; j++) {
                    const L2KW_Vector4 &r = nf.SkeletonData[s].SkeletonPositions[j];
                    const L2KW_Vector4 &w = raw.SkeletonData[s].SkeletonPositions[j];
                    const l2k_vec3 &o = ours_frame.skeletons[s].joints[j];
                    double d = sqrt((r.x - o.x) * (r.x - o.x) + (r.y - o.y) * (r.y - o.y) + (r.z - o.z) * (r.z - o.z));
                    double dr = sqrt((r.x - w.x) * (r.x - w.x) + (r.y - w.y) * (r.y - w.y) + (r.z - w.z) * (r.z - w.z));
                    sum += d;
                    sum_raw += dr;
                    n++;
                    if (d > max) {
                        max = d;
                        max_frame = f;
                        max_joint = j;
                    }
                    if (csv && s == 0)
                        fprintf(csv, "%d,%d,%d,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f\n", (int)(&c - cases), f, j, w.x, w.y, w.z, r.x, r.y, r.z,
                                o.x, o.y, o.z);
                }
            }
        }
        printf("%-36s hr 0x%08lx  |ref - l2k| mean %.2f mm, max %.2f mm (frame %d joint %d)  |ref - raw| mean %.2f mm\n", c.name,
               (unsigned long)first_hr, n ? 1000 * sum / n : 0, 1000 * max, max_frame, max_joint, n ? 1000 * sum_raw / n : 0);
        if (max > 0.001) worst = 1;
    }
    if (csv) fclose(csv);
    return worst;
}
