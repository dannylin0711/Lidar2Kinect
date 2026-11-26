/*
 * Advertise the receiver as _l2k._udp with the DNS-SD API of dnsapi.dll (Windows 10 1809 and
 * later). The functions are looked up at run time: where they are missing, nothing happens.
 */
#include "l2kw_base.h"

#include <windns.h>
#include <stdio.h>

namespace l2kw {

typedef PDNS_SERVICE_INSTANCE(WINAPI *ConstructFn)(PCWSTR, PCWSTR, PIP4_ADDRESS, PIP6_ADDRESS, WORD, WORD, WORD, DWORD, PCWSTR *, PCWSTR *);
typedef DWORD(WINAPI *RegisterFn)(PDNS_SERVICE_REGISTER_REQUEST, PDNS_SERVICE_CANCEL);
typedef DWORD(WINAPI *DeRegisterFn)(PDNS_SERVICE_REGISTER_REQUEST, PDNS_SERVICE_CANCEL);
typedef VOID(WINAPI *FreeFn)(PDNS_SERVICE_INSTANCE);

static DNS_SERVICE_REGISTER_REQUEST g_request;
static PDNS_SERVICE_INSTANCE g_instance;
static DeRegisterFn g_deregister;
static FreeFn g_free;
static bool g_registered;

static VOID WINAPI on_registered(DWORD status, PVOID context, PDNS_SERVICE_INSTANCE instance) {
    (void)context;
    log("dns-sd: registration %s (%lu)", status == ERROR_SUCCESS ? "done" : "failed", status);
    if (instance && g_free) g_free(instance);
}

void dnssd_start(unsigned port, const char *name) {
    HMODULE dns = LoadLibraryW(L"dnsapi.dll");
    ConstructFn construct;
    RegisterFn reg;
    wchar_t service[256], host[MAX_COMPUTERNAME_LENGTH + 16], wname[L2K_NAME_SIZE + 1];
    char computer[MAX_COMPUTERNAME_LENGTH + 1];
    DWORD n = sizeof computer;

    if (!dns) return;
    construct = (ConstructFn)GetProcAddress(dns, "DnsServiceConstructInstance");
    reg = (RegisterFn)GetProcAddress(dns, "DnsServiceRegister");
    g_deregister = (DeRegisterFn)GetProcAddress(dns, "DnsServiceDeRegister");
    g_free = (FreeFn)GetProcAddress(dns, "DnsServiceFreeInstance");
    if (!construct || !reg || !g_deregister || !g_free) {
        log("dns-sd: not available in this Windows (dnsapi.dll has no DnsServiceRegister)");
        return;
    }
    if (!GetComputerNameA(computer, &n)) strcpy(computer, "PC");
    MultiByteToWideChar(CP_UTF8, 0, name, -1, wname, L2K_NAME_SIZE + 1);
    _snwprintf(service, 255, L"%s Lidar2Kinect._l2k._udp.local", wname);
    service[255] = 0;
    _snwprintf(host, MAX_COMPUTERNAME_LENGTH + 15, L"%S.local", computer);
    host[MAX_COMPUTERNAME_LENGTH + 15] = 0;
    g_instance = construct(service, host, NULL, NULL, (WORD)port, 0, 0, 0, NULL, NULL);
    if (!g_instance) {
        log("dns-sd: DnsServiceConstructInstance failed");
        return;
    }
    memset(&g_request, 0, sizeof g_request);
    g_request.Version = DNS_QUERY_REQUEST_VERSION1;
    g_request.InterfaceIndex = 0;
    g_request.pServiceInstance = g_instance;
    g_request.pRegisterCompletionCallback = on_registered;
    DWORD r = reg(&g_request, NULL);
    g_registered = r == DNS_REQUEST_PENDING;
    log("dns-sd: advertising \"%ls\" port %u: %s (%lu)", service, port, g_registered ? "pending" : "failed", r);
}

void dnssd_stop() {
    if (!g_registered) return;
    g_deregister(&g_request, NULL);
    g_registered = false;
}

} // namespace l2kw
