/* Shared skeleton frame <-> NUI_SKELETON_FRAME. */
#include "l2kw.h"

#include <string.h>

namespace l2kw {

static L2KW_Vector4 point(l2k_vec3 v) {
    L2KW_Vector4 r = {v.x, v.y, v.z, 1.0f};
    return r;
}

void to_nui(const l2k_skeleton_frame &in, NUI_SKELETON_FRAME &out, LONGLONG timestamp_ms) {
    memset(&out, 0, sizeof out);
    out.liTimeStamp.QuadPart = timestamp_ms;
    out.dwFrameNumber = in.frame_number;
    out.dwFlags = 0;
    if (in.flags & L2K_FRAME_FLOOR_VALID) {
        out.vFloorClipPlane.x = in.floor_plane.x;
        out.vFloorClipPlane.y = in.floor_plane.y;
        out.vFloorClipPlane.z = in.floor_plane.z;
        out.vFloorClipPlane.w = in.floor_plane.w;
    }
    out.vNormalToGravity.x = in.up.x;
    out.vNormalToGravity.y = in.up.y;
    out.vNormalToGravity.z = in.up.z;
    out.vNormalToGravity.w = 0.0f;
    for (int i = 0; i < L2K_SKELETON_SLOTS; i++) {
        const l2k_skeleton &s = in.skeletons[i];
        NUI_SKELETON_DATA &d = out.SkeletonData[i];
        d.eTrackingState = s.state;
        if (s.state == L2K_SKELETON_NOT_TRACKED) continue;
        d.dwTrackingID = s.tracking_id;
        d.dwQualityFlags = s.quality;
        d.Position = point(s.position);
        if (s.state != L2K_SKELETON_TRACKED) continue;
        for (int j = 0; j < L2K_JOINT_COUNT; j++) {
            d.SkeletonPositions[j] = point(s.joints[j]);
            d.eSkeletonPositionTrackingState[j] = s.joint_state[j];
        }
    }
}

void from_nui(const NUI_SKELETON_FRAME &in, l2k_skeleton_frame &out) {
    memset(&out, 0, sizeof out);
    out.timestamp_us = (uint64_t)in.liTimeStamp.QuadPart * 1000u;
    out.frame_number = in.dwFrameNumber;
    out.floor_plane.x = in.vFloorClipPlane.x;
    out.floor_plane.y = in.vFloorClipPlane.y;
    out.floor_plane.z = in.vFloorClipPlane.z;
    out.floor_plane.w = in.vFloorClipPlane.w;
    if (in.vFloorClipPlane.x != 0 || in.vFloorClipPlane.y != 0 || in.vFloorClipPlane.z != 0) out.flags |= L2K_FRAME_FLOOR_VALID;
    out.up.x = in.vNormalToGravity.x;
    out.up.y = in.vNormalToGravity.y;
    out.up.z = in.vNormalToGravity.z;
    for (int i = 0; i < L2K_SKELETON_SLOTS; i++) {
        const NUI_SKELETON_DATA &d = in.SkeletonData[i];
        l2k_skeleton &s = out.skeletons[i];
        s.state = (uint8_t)d.eTrackingState;
        s.tracking_id = d.dwTrackingID;
        s.quality = (uint16_t)d.dwQualityFlags;
        s.position.x = d.Position.x;
        s.position.y = d.Position.y;
        s.position.z = d.Position.z;
        for (int j = 0; j < L2K_JOINT_COUNT; j++) {
            s.joints[j].x = d.SkeletonPositions[j].x;
            s.joints[j].y = d.SkeletonPositions[j].y;
            s.joints[j].z = d.SkeletonPositions[j].z;
            s.joint_state[j] = (uint8_t)d.eSkeletonPositionTrackingState[j];
        }
    }
}

} // namespace l2kw
