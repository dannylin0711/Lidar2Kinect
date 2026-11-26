/*
 * Lidar2Kinect, Windows side - the core shared by the replacement Kinect10.dll (2b) and the
 * spice2x hook (2a): configuration, log, the L2K1 receiver and the virtual Kinect sensor.
 *
 * Threads: the receiver owns one thread (socket, decoding). The game's Nui thread calls the
 * sensor. Both meet in the hub (l2kw_sensor.cpp) under one lock.
 *
 * The parts that do not depend on the Kinect ABI (config, log, receiver, hub interface) are in
 * l2kw_base.h, which the x64 vp4u wrapper uses on its own.
 */
#ifndef L2KW_H
#define L2KW_H

#include "nui_abi.h"
#include "l2kw_base.h"

namespace l2kw {

/* NUI <-> shared types */
void to_nui(const l2k_skeleton_frame &in, NUI_SKELETON_FRAME &out, LONGLONG timestamp_ms);
void from_nui(const NUI_SKELETON_FRAME &in, l2k_skeleton_frame &out);

/* hub, Kinect side only */
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
