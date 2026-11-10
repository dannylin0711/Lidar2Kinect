/*
 * Lidar2Kinect, Windows side - the core shared by the replacement Kinect10.dll (2b) and the
 * spice2x hook (2a): configuration, log, the L2K1 receiver and the virtual Kinect sensor.
 *
 * Threads: the receiver owns one thread (socket, decoding). The game's Nui thread calls the
 * sensor. Both meet in the hub (l2kw_sensor.cpp) under one lock.
 */
#ifndef L2KW_H
#define L2KW_H

#include "nui_abi.h"

extern "C" {
#include "l2k/l2k.h"
}

namespace l2kw {

/* What the game sees while no sender is connected (windows/docs/no-sender.md). */
enum NoSenderMode {
    NO_SENDER_MISSING = 0, /* like an unplugged Kinect: no sensor; the status callback announces it once a sender appears */
    NO_SENDER_IDLE = 1     /* a sensor is always there; it delivers empty frames until a sender appears */
};
/* Where the depth image comes from. */
enum DepthSource {
    DEPTH_AUTO = 0,     /* the sender's depth image while it sends one, else drawn from the skeleton */
    DEPTH_SENDER = 1,   /* only the sender's */
    DEPTH_SKELETON = 2  /* always drawn from the skeleton (l2k_depth_rasterize) */
};

struct Config {
    unsigned port;
    char log_path[MAX_PATH]; /* empty: no log */
    char ini_path[MAX_PATH];
    int no_sender;     /* NoSenderMode */
    int depth_source;  /* DepthSource */
    int depth_mode;    /* L2K_DEPTH_MODE_* asked of the sender */
    int color_quality; /* 0 = sender default */
    int dnssd;         /* advertise _l2k._udp */
    int verbose;       /* log every API call, not just summaries */
    char name[L2K_NAME_SIZE + 1];
    unsigned consumer_kind; /* L2K_CONSUMER_FAKE_KINECT or L2K_CONSUMER_SPICE_HOOK */
};

/* DllMain: remember which module we are (l2k.ini and the log live next to it). Stores only. */
void init_module(HMODULE module, unsigned consumer_kind);
/* Reads l2k.ini next to `module` (env overrides: L2K_INI, L2K_PORT, L2K_LOG). Called by start(). */
void config_load(HMODULE module, unsigned consumer_kind);
const Config &config();

void log_open();
void log(const char *fmt, ...);
double uptime_s();
uint64_t now_us();

/* Starts the receiver thread (idempotent, thread safe). Called from the first API call, never from DllMain. */
void start();

/* JPEG (baseline) -> BGRX via WIC. Returns false if it cannot be decoded or is not w x h. */
bool jpeg_decode_bgrx(const uint8_t *data, size_t len, uint8_t *out, int w, int h);

/* DNS-SD advertisement of _l2k._udp (dnsapi.dll, loaded at run time; absent = silently off). */
void dnssd_start(unsigned port, const char *name);
void dnssd_stop();

/* NUI <-> shared types */
void to_nui(const l2k_skeleton_frame &in, NUI_SKELETON_FRAME &out, LONGLONG timestamp_ms);
void from_nui(const NUI_SKELETON_FRAME &in, l2k_skeleton_frame &out);

/* ---- the hub: what the receiver gives the sensor ---- */
struct SenderInfo {
    bool present;
    char name[L2K_NAME_SIZE + 1];
    uint32_t caps;
    uint8_t source_kind;
};

/* receiver -> hub */
void hub_sender_present(bool present, const SenderInfo &info);
void hub_skeleton(const l2k_skeleton_frame &frame);
void hub_depth(const uint16_t *pixels, uint32_t frame_number); /* 320x240 */
void hub_color(const uint8_t *bgrx, uint32_t frame_number);    /* 640x480 x 4 */
void hub_tick();       /* called every few ms by the receiver: frame-end liveness, empty frames, summaries */
uint32_t hub_wanted(); /* L2K_STREAM_* the game currently needs */
uint32_t hub_skeleton_flags();
bool hub_sender_has_depth();

/* ---- the Kinect API (flat functions = Kinect10.dll exports; same signatures) ---- */
extern "C" {
HRESULT WINAPI K_NuiGetSensorCount(int *pCount);
HRESULT WINAPI K_NuiCreateSensorByIndex(int index, INuiSensor **ppNuiSensor);
HRESULT WINAPI K_NuiCreateSensorById(const OLECHAR *strInstanceId, INuiSensor **ppNuiSensor);
HRESULT WINAPI K_NuiImageGetColorPixelCoordinatesFromDepthPixel(int eColorResolution, const NUI_IMAGE_VIEW_AREA *pcViewArea, LONG lDepthX,
                                                                LONG lDepthY, USHORT usDepthValue, LONG *plColorX, LONG *plColorY);
HRESULT WINAPI K_NuiImageGetColorPixelCoordinatesFromDepthPixelAtResolution(int eColorResolution, int eDepthResolution,
                                                                            const NUI_IMAGE_VIEW_AREA *pcViewArea, LONG lDepthX, LONG lDepthY,
                                                                            USHORT usDepthValue, LONG *plColorX, LONG *plColorY);
HRESULT WINAPI K_NuiTransformSmooth(NUI_SKELETON_FRAME *pSkeletonFrame, const NUI_TRANSFORM_SMOOTH_PARAMETERS *pSmoothingParams);
void WINAPI K_NuiSetDeviceStatusCallback(NuiStatusProc callback, void *pUserData);

/* The "default sensor" flat API (sensor 0). The game does not use it; it works anyway. */
HRESULT WINAPI K_NuiInitialize(DWORD dwFlags);
void WINAPI K_NuiShutdown();
HRESULT WINAPI K_NuiSetFrameEndEvent(HANDLE hEvent, DWORD dwFrameEventFlag);
HRESULT WINAPI K_NuiImageStreamOpen(int eImageType, int eResolution, DWORD dwImageFrameFlags, DWORD dwFrameLimit, HANDLE hNextFrameEvent,
                                    HANDLE *phStreamHandle);
HRESULT WINAPI K_NuiImageStreamSetImageFrameFlags(HANDLE hStream, DWORD dwImageFrameFlags);
HRESULT WINAPI K_NuiImageStreamGetImageFrameFlags(HANDLE hStream, DWORD *pdwImageFrameFlags);
HRESULT WINAPI K_NuiImageStreamGetNextFrame(HANDLE hStream, DWORD dwMillisecondsToWait, const NUI_IMAGE_FRAME **ppcImageFrame);
HRESULT WINAPI K_NuiImageStreamReleaseFrame(HANDLE hStream, const NUI_IMAGE_FRAME *pImageFrame);
HRESULT WINAPI K_NuiCameraElevationSetAngle(LONG lAngleDegrees);
HRESULT WINAPI K_NuiCameraElevationGetAngle(LONG *plAngleDegrees);
HRESULT WINAPI K_NuiSkeletonTrackingEnable(HANDLE hNextFrameEvent, DWORD dwFlags);
HRESULT WINAPI K_NuiSkeletonTrackingDisable();
HRESULT WINAPI K_NuiSkeletonSetTrackedSkeletons(DWORD *TrackingIDs);
HRESULT WINAPI K_NuiSkeletonGetNextFrame(DWORD dwMillisecondsToWait, NUI_SKELETON_FRAME *pSkeletonFrame);

/* Not provided (E_NOTIMPL): audio, bone orientations, coordinate mapper, depth filter. */
HRESULT WINAPI K_NuiGetAudioSource(IUnknown **ppDmo);
HRESULT WINAPI K_NuiSkeletonCalculateBoneOrientations(const NUI_SKELETON_DATA *pSkeletonData, void *pBoneOrientations);
HRESULT WINAPI K_NuiCreateCoordinateMapperFromParameters(ULONG dataByteCount, void *pData, IUnknown **ppCoordinateMapper);
HRESULT WINAPI K_NuiCreateDepthFilter(LPCWSTR filename, DWORD factoryIndex, IUnknown **ppDepthFilter);
/* Ordinal 1 (no name) of the real DLL: takes no arguments and returns a pointer to static data. */
const void *K_NuiOrdinal1();
}

} // namespace l2kw

#endif /* L2KW_H */
