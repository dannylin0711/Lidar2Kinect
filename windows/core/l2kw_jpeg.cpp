/* JPEG -> BGRX with the Windows Imaging Component (part of every Windows since Vista). */
#include "l2kw.h"

#include <wincodec.h>

namespace l2kw {

bool jpeg_decode_bgrx(const uint8_t *data, size_t len, uint8_t *out, int w, int h) {
    static IWICImagingFactory *factory; /* receiver thread only */
    IWICStream *stream = NULL;
    IWICBitmapDecoder *decoder = NULL;
    IWICBitmapFrameDecode *frame = NULL;
    IWICBitmapSource *bgrx = NULL;
    UINT fw = 0, fh = 0;
    bool ok = false;

    if (!factory) {
        HRESULT hr = CoInitializeEx(NULL, COINIT_MULTITHREADED);
        (void)hr; /* S_FALSE / RPC_E_CHANGED_MODE: COM is usable either way on this thread */
        if (FAILED(CoCreateInstance(CLSID_WICImagingFactory, NULL, CLSCTX_INPROC_SERVER, IID_PPV_ARGS(&factory)))) {
            factory = NULL;
            return false;
        }
    }
    if (FAILED(factory->CreateStream(&stream))) goto done;
    if (FAILED(stream->InitializeFromMemory((BYTE *)data, (DWORD)len))) goto done;
    if (FAILED(factory->CreateDecoderFromStream(stream, NULL, WICDecodeMetadataCacheOnDemand, &decoder))) goto done;
    if (FAILED(decoder->GetFrame(0, &frame))) goto done;
    if (FAILED(frame->GetSize(&fw, &fh)) || (int)fw != w || (int)fh != h) goto done;
    if (FAILED(WICConvertBitmapSource(GUID_WICPixelFormat32bppBGR, frame, &bgrx))) goto done;
    ok = SUCCEEDED(bgrx->CopyPixels(NULL, (UINT)w * 4, (UINT)(w * h * 4), out));
done:
    if (bgrx) bgrx->Release();
    if (frame) frame->Release();
    if (decoder) decoder->Release();
    if (stream) stream->Release();
    return ok;
}

} // namespace l2kw
