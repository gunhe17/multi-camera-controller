#include <mfapi.h>
#include <mfidl.h>
#include <mfreadwrite.h>
#include <mfobjects.h>
#include <mferror.h>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <wrl/client.h>
#include <string>

using namespace Microsoft::WRL;
namespace fs = std::filesystem;

void PrintMediaTypeAttributes(IMFMediaType* mediaType, std::wofstream& out) {
    UINT32 width = 0, height = 0;
    if (SUCCEEDED(MFGetAttributeSize(mediaType, MF_MT_FRAME_SIZE, &width, &height))) {
        out << L"  Resolution: " << width << L"x" << height << std::endl;
    }

    UINT32 fr_num = 0, fr_den = 0;
    if (SUCCEEDED(MFGetAttributeRatio(mediaType, MF_MT_FRAME_RATE, &fr_num, &fr_den))) {
        out << L"  Frame Rate: " << fr_num << L"/" << fr_den << L" ("
            << (fr_den != 0 ? static_cast<float>(fr_num) / fr_den : 0) << L" fps)" << std::endl;
    }

    GUID subtype = {};
    if (SUCCEEDED(mediaType->GetGUID(MF_MT_SUBTYPE, &subtype))) {
        LPOLESTR guidStr = nullptr;
        StringFromCLSID(subtype, &guidStr);
        out << L"  Subtype: " << guidStr << std::endl;
        CoTaskMemFree(guidStr);
    }

    UINT32 interlace = 0;
    if (SUCCEEDED(mediaType->GetUINT32(MF_MT_INTERLACE_MODE, &interlace))) {
        out << L"  Interlace Mode: " << interlace << std::endl;
    }

    UINT32 sampleSize = 0;
    if (SUCCEEDED(mediaType->GetUINT32(MF_MT_SAMPLE_SIZE, &sampleSize))) {
        out << L"  Sample Size: " << sampleSize << L" bytes" << std::endl;
    }

    out << L"-----------------------------\n";
}

int main() {
    std::cout << "entered";

    HRESULT hr = MFStartup(MF_VERSION);
    if (FAILED(hr)) {
        std::cerr << "MFStartup failed" << std::endl;
        return -1;
    }
    
    fs::path output_path = fs::current_path() / "lab" / "lab2" /  "result" / "media_inspector.txt";
    fs::create_directories(output_path.parent_path());
    std::wofstream out(output_path);
    out.imbue(std::locale(""));  // 유니코드 출력 (윈도우용)

    ComPtr<IMFAttributes> attributes;
    hr = MFCreateAttributes(&attributes, 1);
    attributes->SetGUID(MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE, MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_GUID);

    IMFActivate** devices = nullptr;
    UINT32 deviceCount = 0;
    hr = MFEnumDeviceSources(attributes.Get(), &devices, &deviceCount);
    if (FAILED(hr) || deviceCount == 0) {
        out << L"No video capture devices found." << std::endl;
        MFShutdown();
        return -1;
    }

    for (UINT32 i = 0; i < deviceCount; ++i) {
        WCHAR* name = nullptr;
        devices[i]->GetAllocatedString(MF_DEVSOURCE_ATTRIBUTE_FRIENDLY_NAME, &name, nullptr);
        out << L"\n[" << i << L"] Device: " << name << std::endl;
        CoTaskMemFree(name);

        ComPtr<IMFMediaSource> mediaSource;
        hr = devices[i]->ActivateObject(IID_PPV_ARGS(&mediaSource));
        if (FAILED(hr)) continue;

        ComPtr<IMFPresentationDescriptor> presentationDescriptor;
        hr = mediaSource->CreatePresentationDescriptor(&presentationDescriptor);
        if (FAILED(hr)) continue;

        BOOL selected = FALSE;
        ComPtr<IMFStreamDescriptor> streamDescriptor;
        hr = presentationDescriptor->GetStreamDescriptorByIndex(0, &selected, &streamDescriptor);
        if (FAILED(hr)) continue;

        ComPtr<IMFMediaTypeHandler> handler;
        hr = streamDescriptor->GetMediaTypeHandler(&handler);
        if (FAILED(hr)) continue;

        DWORD typeCount = 0;
        handler->GetMediaTypeCount(&typeCount);

        for (DWORD j = 0; j < typeCount; ++j) {
            ComPtr<IMFMediaType> mediaType;
            hr = handler->GetMediaTypeByIndex(j, &mediaType);
            if (SUCCEEDED(hr)) {
                out << L"\n  Media Type #" << j << std::endl;
                PrintMediaTypeAttributes(mediaType.Get(), out);
            }
        }

        mediaSource->Shutdown();
    }

    for (UINT32 i = 0; i < deviceCount; ++i) {
        devices[i]->Release();
    }
    CoTaskMemFree(devices);

    out.close();
    MFShutdown();
    return 0;
}
