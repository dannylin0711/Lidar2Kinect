/*
 * The part of the Kinect for Windows SDK v1 ABI (Kinect10.dll) this project implements, declared
 * from the documented interface - not copied from Microsoft's headers.
 *
 * Sources for each item:
 *  - export ordinals and stack sizes: dumpbin of Kinect10.dll 1.8.0.595 (windows/docs/kinect10-exports.md)
 *  - vtable offsets, struct sizes and offsets the game uses: gamekdm.dll (docs/01)
 *  - everything else: the SDK documentation; checked where possible by the static_asserts below.
 *
 * 32-bit only. Interfaces are C++ classes with only virtual __stdcall methods and no virtual
 * destructor, which MSVC lays out exactly like a COM vtable.
 */
#ifndef L2KW_NUI_ABI_H
#define L2KW_NUI_ABI_H

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <unknwn.h>
#include <stddef.h>

#ifndef _M_IX86
#error "The Kinect ABI declared here is the 32-bit one: build with -A Win32"
#endif

/* ---- HRESULTs ---- */
#define L2KW_FACILITY_NUI 0x301
#define L2KW_NUI_E(code) ((HRESULT)(0x80000000u | (L2KW_FACILITY_NUI << 16) | (code)))

#define E_NUI_DEVICE_NOT_CONNECTED ((HRESULT)0x8007048Fu)  /* HRESULT_FROM_WIN32(ERROR_DEVICE_NOT_CONNECTED) */
#define E_NUI_DEVICE_NOT_READY ((HRESULT)0x80070015u)      /* HRESULT_FROM_WIN32(ERROR_NOT_READY) */
#define E_NUI_FRAME_NO_DATA L2KW_NUI_E(0x0001)
#define E_NUI_STREAM_NOT_ENABLED L2KW_NUI_E(0x0002)
#define E_NUI_IMAGE_STREAM_IN_USE L2KW_NUI_E(0x0003)
#define E_NUI_FRAME_LIMIT_EXCEEDED L2KW_NUI_E(0x0004)
#define E_NUI_FEATURE_NOT_INITIALIZED L2KW_NUI_E(0x0005)
#define E_NUI_NOTCONNECTED L2KW_NUI_E(0x0014)  /* the game maps this to "sensor missing" (docs/01 5) */
#define E_NUI_NOTREADY L2KW_NUI_E(0x0015)
#define E_NUI_NOTPOWERED L2KW_NUI_E(0x027F)

/* ---- NuiInitialize flags ---- */
#define NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX 0x00000001
#define NUI_INITIALIZE_FLAG_USES_COLOR 0x00000002
#define NUI_INITIALIZE_FLAG_USES_SKELETON 0x00000008
#define NUI_INITIALIZE_FLAG_USES_DEPTH 0x00000020

/* ---- images ---- */
enum {
    NUI_IMAGE_TYPE_DEPTH_AND_PLAYER_INDEX = 0,
    NUI_IMAGE_TYPE_COLOR = 1,
    NUI_IMAGE_TYPE_COLOR_YUV = 2,
    NUI_IMAGE_TYPE_COLOR_RAW_YUV = 3,
    NUI_IMAGE_TYPE_DEPTH = 4
};
enum {
    NUI_IMAGE_RESOLUTION_INVALID = -1,
    NUI_IMAGE_RESOLUTION_80x60 = 0,
    NUI_IMAGE_RESOLUTION_320x240 = 1,
    NUI_IMAGE_RESOLUTION_640x480 = 2,
    NUI_IMAGE_RESOLUTION_1280x960 = 3
};

typedef struct NUI_IMAGE_VIEW_AREA {
    int eDigitalZoom;
    LONG lCenterX;
    LONG lCenterY;
} NUI_IMAGE_VIEW_AREA;

typedef struct NUI_LOCKED_RECT {
    INT Pitch;
    INT size;
    BYTE *pBits;
} NUI_LOCKED_RECT;

typedef struct NUI_SURFACE_DESC {
    UINT Width;
    UINT Height;
} NUI_SURFACE_DESC;

class INuiFrameTexture : public IUnknown {
public:
    virtual int STDMETHODCALLTYPE BufferLen() = 0;                                                         /* 0x0c */
    virtual int STDMETHODCALLTYPE Pitch() = 0;                                                             /* 0x10 */
    virtual HRESULT STDMETHODCALLTYPE LockRect(UINT Level, NUI_LOCKED_RECT *pLockedRect, RECT *pRect, DWORD Flags) = 0; /* 0x14 */
    virtual HRESULT STDMETHODCALLTYPE GetLevelDesc(UINT Level, NUI_SURFACE_DESC *pDesc) = 0;               /* 0x18 */
    virtual HRESULT STDMETHODCALLTYPE UnlockRect(UINT Level) = 0;                                          /* 0x1c */
};

typedef struct NUI_IMAGE_FRAME {
    LARGE_INTEGER liTimeStamp;
    DWORD dwFrameNumber;
    int eImageType;
    int eResolution;
    INuiFrameTexture *pFrameTexture;
    DWORD dwFrameFlags;
    NUI_IMAGE_VIEW_AREA ViewArea;
} NUI_IMAGE_FRAME;

/* ---- skeleton ---- */
#define NUI_SKELETON_COUNT 6
#define NUI_SKELETON_POSITION_COUNT 20
#define NUI_SKELETON_MAX_TRACKED_COUNT 2
#define NUI_SKELETON_TRACKING_FLAG_SUPPRESS_NO_FRAME_DATA 0x1
#define NUI_SKELETON_TRACKING_FLAG_TITLE_SETS_TRACKED_SKELETONS 0x2
#define NUI_SKELETON_TRACKING_FLAG_ENABLE_SEATED_SUPPORT 0x4
#define NUI_SKELETON_TRACKING_FLAG_ENABLE_IN_NEAR_RANGE 0x8

typedef struct L2KW_Vector4 {
    FLOAT x, y, z, w;
} L2KW_Vector4;

typedef struct NUI_SKELETON_DATA {
    int eTrackingState;
    DWORD dwTrackingID;
    DWORD dwEnrollmentIndex;
    DWORD dwUserIndex;
    L2KW_Vector4 Position;
    L2KW_Vector4 SkeletonPositions[NUI_SKELETON_POSITION_COUNT];
    int eSkeletonPositionTrackingState[NUI_SKELETON_POSITION_COUNT];
    DWORD dwQualityFlags;
} NUI_SKELETON_DATA;

typedef struct NUI_SKELETON_FRAME {
    LARGE_INTEGER liTimeStamp;
    DWORD dwFrameNumber;
    DWORD dwFlags;
    L2KW_Vector4 vFloorClipPlane;
    L2KW_Vector4 vNormalToGravity;
    NUI_SKELETON_DATA SkeletonData[NUI_SKELETON_COUNT];
} NUI_SKELETON_FRAME;

typedef struct NUI_TRANSFORM_SMOOTH_PARAMETERS {
    FLOAT fSmoothing;
    FLOAT fCorrection;
    FLOAT fPrediction;
    FLOAT fJitterRadius;
    FLOAT fMaxDeviationRadius;
} NUI_TRANSFORM_SMOOTH_PARAMETERS;

typedef void(CALLBACK *NuiStatusProc)(HRESULT hrStatus, const OLECHAR *instanceName, const OLECHAR *uniqueDeviceName, void *pUserData);

/* ---- INuiSensor: the SDK 1.8 method order. The offsets the game calls are marked [DLL]. ---- */
class INuiSensor : public IUnknown {
public:
    virtual HRESULT STDMETHODCALLTYPE NuiInitialize(DWORD dwFlags) = 0;                    /* 0x0c [DLL] */
    virtual void STDMETHODCALLTYPE NuiShutdown() = 0;                                      /* 0x10 [DLL] */
    virtual HRESULT STDMETHODCALLTYPE NuiSetFrameEndEvent(HANDLE hEvent, DWORD dwFrameEventFlag) = 0; /* 0x14 [DLL] */
    virtual HRESULT STDMETHODCALLTYPE NuiImageStreamOpen(int eImageType, int eResolution, DWORD dwImageFrameFlags,
                                                         DWORD dwFrameLimit, HANDLE hNextFrameEvent, HANDLE *phStreamHandle) = 0; /* 0x18 [DLL] */
    virtual HRESULT STDMETHODCALLTYPE NuiImageStreamSetImageFrameFlags(HANDLE hStream, DWORD dwImageFrameFlags) = 0;     /* 0x1c */
    virtual HRESULT STDMETHODCALLTYPE NuiImageStreamGetImageFrameFlags(HANDLE hStream, DWORD *pdwImageFrameFlags) = 0;   /* 0x20 */
    virtual HRESULT STDMETHODCALLTYPE NuiImageStreamGetNextFrame(HANDLE hStream, DWORD dwMillisecondsToWait, NUI_IMAGE_FRAME *pImageFrame) = 0; /* 0x24 [DLL] */
    virtual HRESULT STDMETHODCALLTYPE NuiImageStreamReleaseFrame(HANDLE hStream, NUI_IMAGE_FRAME *pImageFrame) = 0;       /* 0x28 [DLL] */
    virtual HRESULT STDMETHODCALLTYPE NuiImageGetColorPixelCoordinatesFromDepthPixel(int eColorResolution, const NUI_IMAGE_VIEW_AREA *pcViewArea,
                                                                                     LONG lDepthX, LONG lDepthY, USHORT usDepthValue,
                                                                                     LONG *plColorX, LONG *plColorY) = 0; /* 0x2c */
    virtual HRESULT STDMETHODCALLTYPE NuiImageGetColorPixelCoordinatesFromDepthPixelAtResolution(int eColorResolution, int eDepthResolution,
                                                                                                 const NUI_IMAGE_VIEW_AREA *pcViewArea, LONG lDepthX,
                                                                                                 LONG lDepthY, USHORT usDepthValue,
                                                                                                 LONG *plColorX, LONG *plColorY) = 0; /* 0x30 */
    virtual HRESULT STDMETHODCALLTYPE NuiImageGetColorPixelCoordinateFrameFromDepthPixelFrameAtResolution(int eColorResolution, int eDepthResolution,
                                                                                                         DWORD cDepthValues, USHORT *pDepthValues,
                                                                                                         DWORD cColorCoordinates, LONG *pColorCoordinates) = 0; /* 0x34 */
    virtual HRESULT STDMETHODCALLTYPE NuiCameraElevationSetAngle(LONG lAngleDegrees) = 0;   /* 0x38 [DLL] */
    virtual HRESULT STDMETHODCALLTYPE NuiCameraElevationGetAngle(LONG *plAngleDegrees) = 0; /* 0x3c [DLL] */
    virtual HRESULT STDMETHODCALLTYPE NuiSkeletonTrackingEnable(HANDLE hNextFrameEvent, DWORD dwFlags) = 0; /* 0x40 [DLL] */
    virtual HRESULT STDMETHODCALLTYPE NuiSkeletonTrackingDisable() = 0;                    /* 0x44 [DLL] */
    virtual HRESULT STDMETHODCALLTYPE NuiSkeletonSetTrackedSkeletons(DWORD *TrackingIDs) = 0; /* 0x48 [DLL] */
    virtual HRESULT STDMETHODCALLTYPE NuiSkeletonGetNextFrame(DWORD dwMillisecondsToWait, NUI_SKELETON_FRAME *pSkeletonFrame) = 0; /* 0x4c [DLL] */
    virtual HRESULT STDMETHODCALLTYPE NuiTransformSmooth(NUI_SKELETON_FRAME *pSkeletonFrame, const NUI_TRANSFORM_SMOOTH_PARAMETERS *pSmoothingParams) = 0; /* 0x50 */
    virtual HRESULT STDMETHODCALLTYPE NuiGetAudioSource(IUnknown **ppDmo) = 0;              /* 0x54 */
    virtual int STDMETHODCALLTYPE NuiInstanceIndex() = 0;                                  /* 0x58 */
    virtual BSTR STDMETHODCALLTYPE NuiDeviceConnectionId() = 0;                            /* 0x5c */
    virtual BSTR STDMETHODCALLTYPE NuiUniqueId() = 0;                                      /* 0x60 */
    virtual BSTR STDMETHODCALLTYPE NuiAudioArrayId() = 0;                                  /* 0x64 */
    virtual HRESULT STDMETHODCALLTYPE NuiStatus() = 0;                                     /* 0x68 [DLL] */
    virtual DWORD STDMETHODCALLTYPE NuiInitializationFlags() = 0;                          /* 0x6c */
    virtual HRESULT STDMETHODCALLTYPE NuiGetCoordinateMapper(IUnknown **pMapping) = 0;      /* 0x70 */
    virtual HRESULT STDMETHODCALLTYPE NuiImageFrameGetDepthImagePixelFrameTexture(HANDLE hStream, NUI_IMAGE_FRAME *pImageFrame,
                                                                                  BOOL *pNearMode, INuiFrameTexture **ppFrameTexture) = 0; /* 0x74 */
    virtual HRESULT STDMETHODCALLTYPE NuiGetColorCameraSettings(IUnknown **pCameraSettings) = 0; /* 0x78 */
    virtual BOOL STDMETHODCALLTYPE NuiGetForceInfraredEmitterOff() = 0;                    /* 0x7c */
    virtual HRESULT STDMETHODCALLTYPE NuiSetForceInfraredEmitterOff(BOOL fForceInfraredEmitterOff) = 0; /* 0x80 */
    virtual HRESULT STDMETHODCALLTYPE NuiAccelerometerGetCurrentReading(L2KW_Vector4 *pReading) = 0; /* 0x84 */
};

/* IIDs as documented for SDK 1.x (used only to answer QueryInterface). */
/* {1f5e088c-a8c7-41d3-9957-209677a13e85} */
static const IID L2KW_IID_INuiSensor = {0x1f5e088c, 0xa8c7, 0x41d3, {0x99, 0x57, 0x20, 0x96, 0x77, 0xa1, 0x3e, 0x85}};
/* {13ea17f5-ff2e-4670-9ee5-1297a6e880d1} */
static const IID L2KW_IID_INuiFrameTexture = {0x13ea17f5, 0xff2e, 0x4670, {0x9e, 0xe5, 0x12, 0x97, 0xa6, 0xe8, 0x80, 0xd1}};

/* ---- the layout facts the game depends on (docs/01) ---- */
static_assert(sizeof(void *) == 4, "x86 only");
static_assert(sizeof(NUI_IMAGE_FRAME) == 40, "NUI_IMAGE_FRAME is 40 bytes (gamekdm copies 5 x 8 bytes)");
static_assert(offsetof(NUI_IMAGE_FRAME, dwFrameNumber) == 0x08, "game reads dwFrameNumber at +8");
static_assert(offsetof(NUI_IMAGE_FRAME, pFrameTexture) == 0x14, "game reads pFrameTexture at +0x14");
static_assert(sizeof(NUI_SKELETON_DATA) == 436, "NUI_SKELETON_DATA");
static_assert(sizeof(NUI_SKELETON_FRAME) == 0xA68, "NUI_SKELETON_FRAME is 2664 bytes (gamekdm copies 0x29A dwords)");
static_assert(offsetof(NUI_SKELETON_FRAME, vFloorClipPlane) == 0x10, "game reads the floor plane at +0x10");
static_assert(offsetof(NUI_SKELETON_FRAME, SkeletonData) == 0x30, "SkeletonData");
static_assert(sizeof(NUI_LOCKED_RECT) == 12, "NUI_LOCKED_RECT");
static_assert(sizeof(NUI_TRANSFORM_SMOOTH_PARAMETERS) == 20, "5 floats");

#endif /* L2KW_NUI_ABI_H */
