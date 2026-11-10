/*
 * Unit test of the virtual sensor without the network: feeds the hub directly (as the receiver
 * would) and checks what the Kinect API gives back - events, frames, status, conversions.
 */
#include "../core/l2kw.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

using namespace l2kw;

static int g_checks, g_failed;
#define CHECK(c)                                                              \
    do {                                                                      \
        g_checks++;                                                           \
        if (!(c)) {                                                           \
            g_failed++;                                                       \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c);               \
        }                                                                     \
    } while (0)

static bool signaled(HANDLE h) { return WaitForSingleObject(h, 0) == WAIT_OBJECT_0; }

static int g_cb_ok, g_cb_err;
static void CALLBACK cb(HRESULT hr, const OLECHAR *, const OLECHAR *, void *user) {
    CHECK(user == (void *)0x1234);
    if (hr == S_OK) g_cb_ok++;
    else g_cb_err++;
}

static l2k_skeleton_frame synth_frame(uint32_t n, int players) {
    static l2k_mapper m;
    static bool init;
    if (!init) {
        l2k_mapper_init(&m, NULL);
        init = true;
    }
    l2k_src_body b[2];
    for (int i = 0; i < players; i++) l2k_synth_body(n / 30.0, i, players, &b[i]);
    l2k_vec3 up = {0, 1, 0};
    l2k_skeleton_frame f;
    l2k_mapper_map(&m, b, players, up, 0, n * 33333ull, L2K_SOURCE_SYNTHETIC, &f);
    return f;
}

int main() {
    /* the core opens a socket at the first API call: keep it off the default port, no log */
    _putenv("L2K_PORT=24899");
    _putenv("L2K_LOG=none");
    _putenv("L2K_INI=nonexistent.ini");
    init_module(GetModuleHandleA(NULL), L2K_CONSUMER_FAKE_KINECT);

    /* conversions */
    {
        l2k_skeleton_frame f = synth_frame(1, 2), back;
        NUI_SKELETON_FRAME n;
        to_nui(f, n, 1234);
        CHECK(n.liTimeStamp.QuadPart == 1234);
        CHECK(n.dwFrameNumber == f.frame_number);
        CHECK(n.SkeletonData[0].eTrackingState == 2 && n.SkeletonData[1].eTrackingState == 2);
        CHECK(n.SkeletonData[0].SkeletonPositions[3].w == 1.0f);
        CHECK(n.SkeletonData[0].SkeletonPositions[3].z == f.skeletons[0].joints[3].z);
        CHECK(n.vFloorClipPlane.w == f.floor_plane.w && n.vNormalToGravity.y == f.up.y);
        CHECK(n.SkeletonData[2].eTrackingState == 0 && n.SkeletonData[2].dwTrackingID == 0);
        from_nui(n, back);
        CHECK(back.skeletons[1].tracking_id == f.skeletons[1].tracking_id);
        CHECK(memcmp(back.skeletons[1].joints, f.skeletons[1].joints, sizeof f.skeletons[1].joints) == 0);
    }

    /* ordinal 9 and its AtResolution sibling */
    {
        LONG x, y;
        CHECK(K_NuiImageGetColorPixelCoordinatesFromDepthPixel(NUI_IMAGE_RESOLUTION_640x480, NULL, 320, 240, 2000 << 3, &x, &y) == S_OK);
        CHECK(x == 640 && y == 480);
        CHECK(K_NuiImageGetColorPixelCoordinatesFromDepthPixel(NUI_IMAGE_RESOLUTION_640x480, NULL, 17, 5, 0, &x, &y) == S_OK);
        CHECK(x == 34 && y == 10);
        CHECK(K_NuiImageGetColorPixelCoordinatesFromDepthPixelAtResolution(NUI_IMAGE_RESOLUTION_1280x960, NUI_IMAGE_RESOLUTION_640x480, NULL, 10,
                                                                            20, 0, &x, &y) == S_OK);
        CHECK(x == 20 && y == 40);
    }

    /* no sender: "missing" mode */
    K_NuiSetDeviceStatusCallback(cb, (void *)0x1234);
    int count = -1;
    CHECK(K_NuiGetSensorCount(&count) == S_OK && count == 0);
    INuiSensor *s = NULL;
    CHECK(FAILED(K_NuiCreateSensorByIndex(0, &s)) && !s);

    /* a sender appears: callback S_OK, one sensor */
    SenderInfo info;
    memset(&info, 0, sizeof info);
    info.present = true;
    info.caps = L2K_STREAM_SKELETON | L2K_STREAM_COLOR; /* no depth: the depth image is drawn from the skeleton */
    strcpy(info.name, "test");
    hub_sender_present(true, info);
    CHECK(g_cb_ok == 1);
    CHECK(K_NuiGetSensorCount(&count) == S_OK && count == 1);
    CHECK(K_NuiCreateSensorByIndex(0, &s) == S_OK && s);
    CHECK(hub_wanted() == 0);

    HANDLE ev_depth = CreateEventW(NULL, TRUE, FALSE, NULL), ev_color = CreateEventW(NULL, TRUE, FALSE, NULL);
    HANDLE ev_skel = CreateEventW(NULL, TRUE, FALSE, NULL), ev_end = CreateEventW(NULL, FALSE, FALSE, NULL);
    HANDLE hd = NULL, hc = NULL, hx = NULL;
    CHECK(s->NuiInitialize(0x0B) == S_OK);
    CHECK(s->NuiInitializationFlags() == 0x0B);
    CHECK(s->NuiSetFrameEndEvent(ev_end, 0) == S_OK);
    CHECK(s->NuiImageStreamOpen(NUI_IMAGE_TYPE_DEPTH_AND_PLAYER_INDEX, NUI_IMAGE_RESOLUTION_320x240, 0, 3, ev_depth, &hd) == S_OK && hd);
    CHECK(s->NuiImageStreamOpen(NUI_IMAGE_TYPE_COLOR, NUI_IMAGE_RESOLUTION_640x480, 0, 3, ev_color, &hc) == S_OK && hc);
    CHECK(s->NuiImageStreamOpen(NUI_IMAGE_TYPE_DEPTH, NUI_IMAGE_RESOLUTION_640x480, 0, 3, NULL, &hx) == E_INVALIDARG);
    CHECK(s->NuiSkeletonTrackingEnable(ev_skel, 0) == S_OK);
    CHECK(hub_wanted() == (L2K_STREAM_SKELETON | L2K_STREAM_DEPTH | L2K_STREAM_COLOR));
    CHECK(s->NuiStatus() == S_OK);

    /* nothing yet: no data, events stay reset */
    NUI_SKELETON_FRAME sk;
    NUI_IMAGE_FRAME fr;
    CHECK(s->NuiSkeletonGetNextFrame(0, &sk) == E_NUI_FRAME_NO_DATA);
    CHECK(s->NuiImageStreamGetNextFrame(hd, 0, &fr) == E_NUI_FRAME_NO_DATA);
    CHECK(!signaled(ev_skel) && !signaled(ev_depth));

    /* a skeleton frame: skeleton event, a drawn depth image, frame end */
    l2k_skeleton_frame f = synth_frame(10, 2);
    hub_skeleton(f);
    CHECK(signaled(ev_skel));
    CHECK(signaled(ev_depth));
    CHECK(signaled(ev_end));      /* auto-reset: consumed by this wait */
    CHECK(!signaled(ev_end));
    CHECK(!signaled(ev_color));   /* the sender announces colour: only its images set this event */
    CHECK(s->NuiSkeletonGetNextFrame(0, &sk) == S_OK);
    CHECK(sk.dwFrameNumber == f.frame_number && sk.SkeletonData[0].eTrackingState == 2);
    CHECK(!signaled(ev_skel)); /* reset when taken */
    CHECK(s->NuiSkeletonGetNextFrame(0, &sk) == E_NUI_FRAME_NO_DATA);

    CHECK(s->NuiImageStreamGetNextFrame(hd, 0, &fr) == S_OK);
    CHECK(!signaled(ev_depth));
    CHECK(fr.eImageType == 0 && fr.eResolution == 1 && fr.pFrameTexture);
    DWORD depth_frame1 = fr.dwFrameNumber;
    {
        NUI_LOCKED_RECT r;
        NUI_SURFACE_DESC d;
        CHECK(fr.pFrameTexture->LockRect(0, &r, NULL, 0) == S_OK);
        CHECK(r.Pitch == 640 && r.size == 640 * 240 && r.pBits);
        CHECK(fr.pFrameTexture->GetLevelDesc(0, &d) == S_OK && d.Width == 320 && d.Height == 240);
        unsigned p1 = 0, p2 = 0;
        for (int i = 0; i < 320 * 240; i++) {
            USHORT v = ((USHORT *)r.pBits)[i];
            p1 += (v & 7) == 1;
            p2 += (v & 7) == 2;
        }
        CHECK(p1 > 500 && p2 > 500); /* both players drawn, index = slot + 1 */
        CHECK(fr.pFrameTexture->UnlockRect(0) == S_OK);
    }
    /* frames are held until released; the next one comes in another texture */
    NUI_IMAGE_FRAME fr2;
    hub_skeleton(synth_frame(11, 2));
    CHECK(s->NuiImageStreamGetNextFrame(hd, 0, &fr2) == S_OK);
    CHECK(fr2.pFrameTexture != fr.pFrameTexture && fr2.dwFrameNumber == depth_frame1 + 1);
    CHECK(s->NuiImageStreamReleaseFrame(hd, &fr) == S_OK);
    CHECK(s->NuiImageStreamReleaseFrame(hd, &fr2) == S_OK);

    /* colour from the sender */
    {
        static uint8_t img[640 * 480 * 4];
        for (int i = 0; i < 640 * 480 * 4; i++) img[i] = (uint8_t)(i * 7);
        hub_color(img, 11);
        CHECK(signaled(ev_color));
        CHECK(s->NuiImageStreamGetNextFrame(hc, 0, &fr) == S_OK);
        NUI_LOCKED_RECT r;
        CHECK(fr.pFrameTexture->LockRect(0, &r, NULL, 0) == S_OK && r.Pitch == 2560);
        CHECK(memcmp(r.pBits, img, sizeof img) == 0);
        CHECK(s->NuiImageStreamReleaseFrame(hc, &fr) == S_OK);
        CHECK(!signaled(ev_color));
    }

    /* elevation from up: tilted 10 degrees upwards */
    {
        l2k_skeleton_frame t = synth_frame(12, 1);
        float a = 10.0f * 3.14159265f / 180.0f;
        t.up.x = 0;
        t.up.y = cosf(a);
        t.up.z = sinf(a);
        hub_skeleton(t);
        LONG angle = 0;
        CHECK(s->NuiCameraElevationGetAngle(&angle) == S_OK && angle == 10);
        CHECK(s->NuiCameraElevationSetAngle(-5) == S_OK);
        CHECK(s->NuiCameraElevationGetAngle(&angle) == S_OK && angle == 10);
    }

    /* smoothing: ordinal 20 does what l2k_smoother_apply does */
    {
        l2k_smoother ref;
        l2k_smooth_params p;
        NUI_TRANSFORM_SMOOTH_PARAMETERS np = {0.5f, 0.5f, 0.5f, 0.05f, 0.04f};
        l2k_smoother_init(&ref);
        l2k_smooth_params_default(&p);
        float maxd = 0;
        for (uint32_t n = 20; n < 60; n++) {
            l2k_skeleton_frame a = synth_frame(n, 2), b = a;
            NUI_SKELETON_FRAME nf;
            to_nui(a, nf, n);
            CHECK(K_NuiTransformSmooth(&nf, &np) == S_OK);
            l2k_smoother_apply(&ref, &b, &p);
            for (int j = 0; j < 20; j++) maxd = fmaxf(maxd, fabsf(nf.SkeletonData[0].SkeletonPositions[j].x - b.skeletons[0].joints[j].x));
        }
        CHECK(maxd == 0.0f);
    }

    /* the sender goes: status fails as "not connected", callback with an error */
    info.present = false;
    hub_sender_present(false, info);
    CHECK(g_cb_err == 1);
    CHECK(s->NuiStatus() == E_NUI_NOTCONNECTED);
    CHECK(K_NuiGetSensorCount(&count) == S_OK && count == 0);
    s->NuiShutdown();
    CHECK(hub_wanted() == 0);
    s->Release();
    /* and comes back */
    info.present = true;
    hub_sender_present(true, info);
    CHECK(g_cb_ok == 2);

    printf("%d checks, %d failed\n", g_checks, g_failed);
    return g_failed ? 1 : 0;
}
