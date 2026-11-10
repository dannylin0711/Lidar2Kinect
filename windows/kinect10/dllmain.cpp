/* 2b: the replacement Kinect10.dll. All the work is in core/; this only says who we are. */
#include "l2kw.h"

BOOL WINAPI DllMain(HINSTANCE module, DWORD reason, LPVOID) {
    if (reason == DLL_PROCESS_ATTACH) {
        DisableThreadLibraryCalls(module);
        l2kw::init_module(module, L2K_CONSUMER_FAKE_KINECT);
    }
    return TRUE;
}
