//
//  uvc_util.m
//  Cooperative macOS UVC + Insta360 Link 2/2C Pro Extension Unit (XU) CLI
//
//  Talks solely over USB Endpoint 0 (VideoControl interface class-specific requests)
//  without seizing interfaces or opening AVCaptureSession, so a live 24/7 OBS
//  broadcast is never interrupted.
//
//  Supports:
//    - Standard UVC Camera Terminal & Processing Unit controls (uvc-util compatible)
//    - Insta360 Link / Link 2 / Link 2C / Link 2C Pro (0x2e1a) Extension Unit controls:
//        exposure-comp-ev      (-3.00 .. +3.00 EV, XU 9 selector 0x09)
//        exposure-comp-raw     (-300  .. +300,     XU 9 selector 0x09)
//        insta360-auto-exp     (1=manual, 2=auto,  XU 9 selector 0x1e)
//        insta360-iso          (0..65535,          XU 9 selector 0x19)
//        insta360-shutter-us   (0..65535 us,       XU 9 selector 0x1d)
//        insta360-hdr          (0 or 1,            XU 9 selector 0x1b bit 2)
//
//  Build universal macOS binary:
//    clang -O2 -Wall -arch arm64 -arch x86_64 -framework Foundation -framework IOKit uvc_util.m -o uvc-util
//    codesign -s - --force uvc-util
//

#import <Foundation/Foundation.h>
#import <IOKit/IOKitLib.h>
#import <IOKit/IOCFPlugIn.h>
#import <IOKit/usb/IOUSBLib.h>
#include <getopt.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if (MAC_OS_X_VERSION_MAX_ALLOWED < 120000)
#define kIOMainPortDefault kIOMasterPortDefault
#endif

// UVC descriptor constants
#define CS_INTERFACE          0x24
#define VC_HEADER             0x01
#define VC_INPUT_TERMINAL     0x02
#define VC_PROCESSING_UNIT    0x05
#define VC_EXTENSION_UNIT     0x06

// UVC request opcodes
#define UVC_SET_CUR           0x01
#define UVC_GET_CUR           0x81
#define UVC_GET_MIN           0x82
#define UVC_GET_MAX           0x83
#define UVC_GET_RES           0x84
#define UVC_GET_LEN           0x85
#define UVC_GET_INFO          0x86
#define UVC_GET_DEF           0x87

// Insta360 Vendor ID & Extension Unit selectors (Link / Link 2 / Link 2C / Link 2C Pro)
#define INSTA360_VENDOR_ID    0x2e1a
#define INSTA_XU_DEFAULT_UNIT 9
#define INSTA_XU_EXP_COMP     0x09
#define INSTA_XU_ISO          0x19
#define INSTA_XU_FEATURE_MASK 0x1b
#define INSTA_XU_SHUTTER_US   0x1d
#define INSTA_XU_AUTO_EXP     0x1e
#define INSTA_FEATURE_HDR_BIT 2

typedef struct __attribute__((packed)) {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint8_t bDescriptorSubType;
} uvc_desc_prefix_t;

typedef struct __attribute__((packed)) {
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bDescriptorSubType;
    uint16_t bcdUVC;
    uint16_t wTotalLength;
    uint32_t dwClockFrequency;
    uint8_t  bInCollection;
} uvc_vc_header_t;

typedef struct {
    uint8_t unitId;
    uint8_t guid[16];
    uint8_t numControls;
} xu_info_t;

typedef struct {
    io_service_t            service;
    IOUSBDeviceInterface    **devIntf;
    IOUSBInterfaceInterface **vcIntf;
    BOOL                    vcOpenedByUs;
    uint16_t                vendorId;
    uint16_t                productId;
    uint32_t                locationId;
    uint8_t                 vcInterfaceNum;
    uint8_t                 ctUnitId;
    uint8_t                 puUnitId;
    uint8_t                 instaXuUnitId;
    int                     xuCount;
    xu_info_t               xus[16];
    char                    name[128];
} uvc_cam_t;

typedef enum {
    UNIT_CT = 1,
    UNIT_PU = 2,
    UNIT_INSTA_XU = 3
} unit_kind_t;

typedef enum {
    FMT_U8,
    FMT_U16,
    FMT_S16,
    FMT_U32,
    FMT_EV_CENTI,    // Insta360 signed int16 where -180 == -1.80 EV
    FMT_HDR_BIT      // Insta360 XU 0x1B bit 2
} val_fmt_t;

typedef struct {
    const char *name;
    unit_kind_t unitKind;
    uint8_t     selector;
    uint8_t     byteLen;
    val_fmt_t   fmt;
    const char *desc;
} ctrl_spec_t;

static const ctrl_spec_t kControls[] = {
    // Insta360 Link / Link 2 / Link 2C Pro Extension Unit controls
    {"exposure-comp-ev",        UNIT_INSTA_XU, INSTA_XU_EXP_COMP,     2, FMT_EV_CENTI, "Insta360 EV exposure compensation (-3.00 to +3.00 EV)"},
    {"exposure-comp-raw",       UNIT_INSTA_XU, INSTA_XU_EXP_COMP,     2, FMT_S16,      "Insta360 raw exposure compensation (-300 to +300)"},
    {"insta360-auto-exposure",  UNIT_INSTA_XU, INSTA_XU_AUTO_EXP,     1, FMT_U8,       "Insta360 auto exposure (2=Auto, 1=Manual)"},
    {"insta360-iso",            UNIT_INSTA_XU, INSTA_XU_ISO,          2, FMT_U16,      "Insta360 manual ISO (requires insta360-auto-exposure=1)"},
    {"insta360-shutter-us",     UNIT_INSTA_XU, INSTA_XU_SHUTTER_US,   2, FMT_U16,      "Insta360 manual shutter in microseconds"},
    {"insta360-hdr",            UNIT_INSTA_XU, INSTA_XU_FEATURE_MASK, 2, FMT_HDR_BIT,  "Insta360 HDR toggle (0=Off, 1=On; non-4K modes)"},

    // Standard UVC Camera Terminal controls
    {"auto-exposure-mode",      UNIT_CT, 0x02, 1, FMT_U8,  "Standard UVC AE mode (1=Manual, 2=Auto, 4=Shutter, 8=Aperture)"},
    {"auto-exposure-priority",  UNIT_CT, 0x03, 1, FMT_U8,  "Standard UVC AE priority"},
    {"exposure-time-abs",       UNIT_CT, 0x04, 4, FMT_U32, "Standard UVC exposure time (100us units)"},
    {"focus-abs",               UNIT_CT, 0x06, 2, FMT_U16, "Standard UVC absolute focus"},
    {"auto-focus",              UNIT_CT, 0x08, 1, FMT_U8,  "Standard UVC autofocus (0/1)"},
    {"zoom-abs",                UNIT_CT, 0x0b, 2, FMT_U16, "Standard UVC zoom (100..400)"},

    // Standard UVC Processing Unit controls
    {"backlight-compensation",  UNIT_PU, 0x01, 2, FMT_U16, "Standard UVC backlight compensation"},
    {"brightness",              UNIT_PU, 0x02, 2, FMT_S16, "Standard UVC brightness"},
    {"contrast",                UNIT_PU, 0x03, 2, FMT_U16, "Standard UVC contrast"},
    {"gain",                    UNIT_PU, 0x04, 2, FMT_U16, "Standard UVC gain"},
    {"power-line-frequency",    UNIT_PU, 0x05, 1, FMT_U8,  "Standard UVC anti-flicker (0=Off, 1=50Hz, 2=60Hz, 3=Auto)"},
    {"hue",                     UNIT_PU, 0x06, 2, FMT_S16, "Standard UVC hue"},
    {"saturation",              UNIT_PU, 0x07, 2, FMT_U16, "Standard UVC saturation"},
    {"sharpness",               UNIT_PU, 0x08, 2, FMT_U16, "Standard UVC sharpness"},
    {"gamma",                   UNIT_PU, 0x09, 2, FMT_U16, "Standard UVC gamma"},
    {"white-balance-temp",      UNIT_PU, 0x0a, 2, FMT_U16, "Standard UVC white balance Kelvin"},
    {"auto-white-balance-temp", UNIT_PU, 0x0b, 1, FMT_U8,  "Standard UVC auto white balance (0/1)"},
};
static const size_t kNumControls = sizeof(kControls) / sizeof(kControls[0]);

static BOOL get_int_prop(io_service_t svc, CFStringRef key, int32_t *outVal) {
    CFTypeRef ref = IORegistryEntryCreateCFProperty(svc, key, kCFAllocatorDefault, 0);
    if (!ref) {
        ref = IORegistryEntrySearchCFProperty(svc, kIOUSBPlane, key, kCFAllocatorDefault, 0);
    }
    if (!ref) {
        ref = IORegistryEntrySearchCFProperty(svc, kIOServicePlane, key, kCFAllocatorDefault, 0);
    }
    if (!ref) return NO;
    BOOL ok = NO;
    if (CFGetTypeID(ref) == CFNumberGetTypeID()) {
        ok = CFNumberGetValue((CFNumberRef)ref, kCFNumberSInt32Type, outVal);
    }
    CFRelease(ref);
    return ok;
}

static void get_str_prop(io_service_t svc, char *buf, size_t bufLen) {
    CFStringRef keys[] = {
        CFSTR("USB Product Name"),
        CFSTR("kUSBProductString"),
        CFSTR("Product Name")
    };
    for (size_t i = 0; i < 3; i++) {
        CFTypeRef ref = IORegistryEntryCreateCFProperty(svc, keys[i], kCFAllocatorDefault, 0);
        if (ref && CFGetTypeID(ref) == CFStringGetTypeID()) {
            if (CFStringGetCString((CFStringRef)ref, buf, (CFIndex)bufLen, kCFStringEncodingUTF8) && buf[0]) {
                CFRelease(ref);
                return;
            }
        }
        if (ref) CFRelease(ref);
    }
    io_name_t ioName;
    if (IORegistryEntryGetName(svc, ioName) == KERN_SUCCESS && ioName[0]) {
        snprintf(buf, bufLen, "%s", ioName);
    } else {
        snprintf(buf, bufLen, "UVC Camera");
    }
}

static BOOL cam_ensure_interface_ready(uvc_cam_t *cam) {
    if (!cam->vcIntf) return NO;
    if (cam->vcOpenedByUs) return YES;
    IOReturn rc = (*cam->vcIntf)->USBInterfaceOpen(cam->vcIntf);
    if (rc == kIOReturnSuccess) {
        cam->vcOpenedByUs = YES;
        return YES;
    }
    if (rc == kIOReturnExclusiveAccess) {
        // UVCAssistant / AppleUVC owns the interface for OBS; Pipe 0 ControlRequest still works!
        return YES;
    }
    return NO;
}

static BOOL cam_ctrl_transfer(uvc_cam_t *cam, BOOL isIn, uint8_t bRequest, uint8_t unitId, uint8_t selector, void *data, uint16_t wLength, IOReturn *outRc) {
    IOUSBDevRequest req;
    memset(&req, 0, sizeof(req));
    req.bmRequestType = USBmakebmRequestType(isIn ? kUSBIn : kUSBOut, kUSBClass, kUSBInterface);
    req.bRequest      = bRequest;
    req.wValue        = (uint16_t)(((uint16_t)selector) << 8);
    req.wIndex        = (uint16_t)((((uint16_t)unitId) << 8) | ((uint16_t)cam->vcInterfaceNum));
    req.wLength       = wLength;
    req.wLenDone      = 0;
    req.pData         = data;

    IOReturn rc = kIOReturnError;
    // Primary cooperative path: class-specific Endpoint 0 DeviceRequest directly on
    // IOUSBDeviceInterface requires NO USBDeviceOpen and NO USBInterfaceOpen, so it
    // never touches UVCAssistant / AppleUVC or OBS's active VideoStreaming pipe.
    if (cam->devIntf) {
        rc = (*cam->devIntf)->DeviceRequest(cam->devIntf, &req);
        if (rc == kIOReturnSuccess) {
            if (outRc) *outRc = rc;
            return YES;
        }
    }
    // Fallback path: Pipe 0 ControlRequest on VideoControl interface
    if (cam->vcIntf && rc != kIOUSBPipeStalled) {
        cam_ensure_interface_ready(cam);
        IOReturn ifRc = (*cam->vcIntf)->ControlRequest(cam->vcIntf, 0, &req);
        if (ifRc == kIOReturnSuccess) {
            if (outRc) *outRc = ifRc;
            return YES;
        }
        if (rc == kIOReturnError) rc = ifRc;
    }
    if (outRc) *outRc = rc;
    return NO;
}

static uint16_t cam_query_xu_len(uvc_cam_t *cam, uint8_t unitId, uint8_t selector) {
    uint8_t raw[2] = {0, 0};
    if (!cam_ctrl_transfer(cam, YES, UVC_GET_LEN, unitId, selector, raw, 2, NULL)) {
        return 0;
    }
    return (uint16_t)(raw[0] | ((uint16_t)raw[1] << 8));
}

static void cam_parse_vc_descriptors(uvc_cam_t *cam) {
    if (!cam->vcIntf) return;
    IOUSBDescriptorHeader *hdr = (*cam->vcIntf)->FindNextAssociatedDescriptor(cam->vcIntf, NULL, CS_INTERFACE);
    if (!hdr) return;

    const uvc_desc_prefix_t *pfx = (const uvc_desc_prefix_t *)hdr;
    if (pfx->bDescriptorSubType != VC_HEADER || pfx->bLength < sizeof(uvc_vc_header_t)) {
        return;
    }
    const uvc_vc_header_t *vcHdr = (const uvc_vc_header_t *)hdr;
    uint16_t totalLen = OSSwapLittleToHostInt16(vcHdr->wTotalLength);
    if (totalLen < vcHdr->bLength || totalLen > 4096) return;

    const uint8_t *ptr = (const uint8_t *)vcHdr + vcHdr->bLength;
    const uint8_t *end = (const uint8_t *)vcHdr + totalLen;

    uint8_t bestXuUnit = 0;
    uint8_t bestXuControls = 0;

    while (ptr + 3 <= end) {
        const uvc_desc_prefix_t *d = (const uvc_desc_prefix_t *)ptr;
        if (d->bLength < 3 || ptr + d->bLength > end) break;

        if (d->bDescriptorType == CS_INTERFACE) {
            if (d->bDescriptorSubType == VC_INPUT_TERMINAL && d->bLength >= 8) {
                uint16_t termType = (uint16_t)(ptr[4] | ((uint16_t)ptr[5] << 8));
                if (termType == 0x0201) { // ITT_CAMERA
                    cam->ctUnitId = ptr[3];
                }
            } else if (d->bDescriptorSubType == VC_PROCESSING_UNIT && d->bLength >= 5) {
                cam->puUnitId = ptr[3];
            } else if (d->bDescriptorSubType == VC_EXTENSION_UNIT && d->bLength >= 22) {
                uint8_t unitId = ptr[3];
                uint8_t numControls = ptr[20];
                if (cam->xuCount < 16) {
                    xu_info_t *xi = &cam->xus[cam->xuCount++];
                    xi->unitId = unitId;
                    memcpy(xi->guid, &ptr[4], 16);
                    xi->numControls = numControls;
                }
                if (unitId == INSTA_XU_DEFAULT_UNIT) {
                    cam->instaXuUnitId = unitId;
                }
                if (numControls > bestXuControls) {
                    bestXuControls = numControls;
                    bestXuUnit = unitId;
                }
            }
        }
        ptr += d->bLength;
    }

    // Verify Insta360 XU unit ID against live GET_LEN(selector 0x09) == 2 if possible
    if (cam->vendorId == INSTA360_VENDOR_ID) {
        if (cam->instaXuUnitId != 0 && cam_query_xu_len(cam, cam->instaXuUnitId, INSTA_XU_EXP_COMP) == 2) {
            return;
        }
        for (int i = 0; i < cam->xuCount; i++) {
            uint8_t uid = cam->xus[i].unitId;
            if (cam_query_xu_len(cam, uid, INSTA_XU_EXP_COMP) == 2) {
                cam->instaXuUnitId = uid;
                return;
            }
        }
        if (cam->instaXuUnitId == 0 && bestXuUnit != 0) {
            cam->instaXuUnitId = bestXuUnit;
        }
        if (cam->instaXuUnitId == 0) {
            cam->instaXuUnitId = INSTA_XU_DEFAULT_UNIT;
        }
    }
}

static void cam_close(uvc_cam_t *cam) {
    if (!cam) return;
    if (cam->vcIntf) {
        if (cam->vcOpenedByUs) {
            (*cam->vcIntf)->USBInterfaceClose(cam->vcIntf);
            cam->vcOpenedByUs = NO;
        }
        (*cam->vcIntf)->Release(cam->vcIntf);
        cam->vcIntf = NULL;
    }
    if (cam->devIntf) {
        (*cam->devIntf)->Release(cam->devIntf);
        cam->devIntf = NULL;
    }
    if (cam->service) {
        IOObjectRelease(cam->service);
        cam->service = 0;
    }
}

static BOOL cam_open_from_service(io_service_t service, uvc_cam_t *outCam) {
    memset(outCam, 0, sizeof(*outCam));
    int32_t vid = 0, pid = 0, loc = 0;
    if (!get_int_prop(service, CFSTR(kUSBVendorID), &vid)) return NO;
    if (!get_int_prop(service, CFSTR(kUSBProductID), &pid)) return NO;
    get_int_prop(service, CFSTR(kUSBDevicePropertyLocationID), &loc);

    IOCFPlugInInterface **plugIn = NULL;
    SInt32 score = 0;
    kern_return_t krc = IOCreatePlugInInterfaceForService(
        service, kIOUSBDeviceUserClientTypeID, kIOCFPlugInInterfaceID, &plugIn, &score
    );
    if (krc != kIOReturnSuccess || !plugIn) return NO;

    IOUSBDeviceInterface **devIntf = NULL;
    HRESULT hrc = (*plugIn)->QueryInterface(
        plugIn, CFUUIDGetUUIDBytes(kIOUSBDeviceInterfaceID), (LPVOID *)&devIntf
    );
    IODestroyPlugInInterface(plugIn);
    if (hrc != 0 || !devIntf) return NO;

    IOUSBFindInterfaceRequest ifReq = {
        .bInterfaceClass    = kUSBVideoInterfaceClass,
        .bInterfaceSubClass = kUSBVideoControlSubClass,
        .bInterfaceProtocol = kIOUSBFindInterfaceDontCare,
        .bAlternateSetting  = kIOUSBFindInterfaceDontCare
    };
    io_iterator_t ifIter = 0;
    if ((*devIntf)->CreateInterfaceIterator(devIntf, &ifReq, &ifIter) != kIOReturnSuccess || !ifIter) {
        (*devIntf)->Release(devIntf);
        return NO;
    }

    io_service_t vcService = IOIteratorNext(ifIter);
    IOObjectRelease(ifIter);
    if (!vcService) {
        (*devIntf)->Release(devIntf);
        return NO;
    }

    IOCFPlugInInterface **ifPlugIn = NULL;
    krc = IOCreatePlugInInterfaceForService(
        vcService, kIOUSBInterfaceUserClientTypeID, kIOCFPlugInInterfaceID, &ifPlugIn, &score
    );
    IOObjectRelease(vcService);
    if (krc != kIOReturnSuccess || !ifPlugIn) {
        (*devIntf)->Release(devIntf);
        return NO;
    }

    IOUSBInterfaceInterface **vcIntf = NULL;
    hrc = (*ifPlugIn)->QueryInterface(
        ifPlugIn, CFUUIDGetUUIDBytes(kIOUSBInterfaceInterfaceID), (LPVOID *)&vcIntf
    );
    IODestroyPlugInInterface(ifPlugIn);
    if (hrc != 0 || !vcIntf) {
        (*devIntf)->Release(devIntf);
        return NO;
    }

    UInt8 ifNum = 0;
    (*vcIntf)->GetInterfaceNumber(vcIntf, &ifNum);

    IOObjectRetain(service);
    outCam->service        = service;
    outCam->devIntf        = devIntf;
    outCam->vcIntf         = vcIntf;
    outCam->vendorId       = (uint16_t)vid;
    outCam->productId      = (uint16_t)pid;
    outCam->locationId     = (uint32_t)loc;
    outCam->vcInterfaceNum = ifNum;
    outCam->ctUnitId       = 1;
    outCam->puUnitId       = 2;
    outCam->instaXuUnitId  = (vid == INSTA360_VENDOR_ID) ? INSTA_XU_DEFAULT_UNIT : 0;
    get_str_prop(service, outCam->name, sizeof(outCam->name));
    cam_parse_vc_descriptors(outCam);
    return YES;
}

static int discover_uvc_cameras(uvc_cam_t *cams, int maxCams) {
    int count = 0;
    uint64_t seenIds[64];
    int seenCount = 0;

    const char *classNames[] = { "IOUSBHostDevice", kIOUSBDeviceClassName };
    for (size_t c = 0; c < 2; c++) {
        CFMutableDictionaryRef match = IOServiceMatching(classNames[c]);
        if (!match) continue;
        io_iterator_t iter = 0;
        if (IOServiceGetMatchingServices(kIOMainPortDefault, match, &iter) != KERN_SUCCESS || !iter) {
            continue;
        }
        io_service_t svc;
        while ((svc = IOIteratorNext(iter)) != 0) {
            uint64_t entryId = 0;
            IORegistryEntryGetRegistryEntryID(svc, &entryId);
            BOOL duplicate = NO;
            for (int i = 0; i < seenCount; i++) {
                if (seenIds[i] == entryId && entryId != 0) {
                    duplicate = YES;
                    break;
                }
            }
            if (!duplicate && count < maxCams) {
                if (seenCount < 64) seenIds[seenCount++] = entryId;
                if (cam_open_from_service(svc, &cams[count])) {
                    count++;
                }
            }
            IOObjectRelease(svc);
        }
        IOObjectRelease(iter);
    }
    return count;
}

static uint8_t unit_id_for_spec(const uvc_cam_t *cam, const ctrl_spec_t *spec) {
    switch (spec->unitKind) {
        case UNIT_CT:       return cam->ctUnitId;
        case UNIT_PU:       return cam->puUnitId;
        case UNIT_INSTA_XU: return cam->instaXuUnitId;
    }
    return 0;
}

static const ctrl_spec_t *find_ctrl_spec(const char *name) {
    for (size_t i = 0; i < kNumControls; i++) {
        if (strcasecmp(kControls[i].name, name) == 0) {
            return &kControls[i];
        }
    }
    return NULL;
}

static double decode_raw_bytes(const uint8_t *buf, const ctrl_spec_t *spec) {
    switch (spec->fmt) {
        case FMT_U8:
            return (double)buf[0];
        case FMT_U16: {
            uint16_t v = (uint16_t)(buf[0] | ((uint16_t)buf[1] << 8));
            return (double)v;
        }
        case FMT_S16: {
            int16_t v = (int16_t)(buf[0] | ((uint16_t)buf[1] << 8));
            return (double)v;
        }
        case FMT_U32: {
            uint32_t v = (uint32_t)buf[0]
                       | ((uint32_t)buf[1] << 8)
                       | ((uint32_t)buf[2] << 16)
                       | ((uint32_t)buf[3] << 24);
            return (double)v;
        }
        case FMT_EV_CENTI: {
            int16_t v = (int16_t)(buf[0] | ((uint16_t)buf[1] << 8));
            return ((double)v) / 100.0;
        }
        case FMT_HDR_BIT: {
            uint16_t mask = (uint16_t)(buf[0] | ((uint16_t)buf[1] << 8));
            return (mask & (1u << INSTA_FEATURE_HDR_BIT)) ? 1.0 : 0.0;
        }
    }
    return 0.0;
}

static void format_val(char *out, size_t outLen, double val, const ctrl_spec_t *spec) {
    if (spec->fmt == FMT_EV_CENTI) {
        snprintf(out, outLen, "%+.2f", val);
    } else {
        snprintf(out, outLen, "%.0f", val);
    }
}

static BOOL read_ctrl_value(uvc_cam_t *cam, const ctrl_spec_t *spec, uint8_t reqType, double *outVal, IOReturn *outRc) {
    uint8_t unitId = unit_id_for_spec(cam, spec);
    if (unitId == 0) return NO;
    uint8_t buf[8] = {0};
    if (!cam_ctrl_transfer(cam, YES, reqType, unitId, spec->selector, buf, spec->byteLen, outRc)) {
        return NO;
    }
    *outVal = decode_raw_bytes(buf, spec);
    return YES;
}

static BOOL write_ctrl_value(uvc_cam_t *cam, const ctrl_spec_t *spec, const char *valStr, IOReturn *outRc) {
    uint8_t unitId = unit_id_for_spec(cam, spec);
    if (unitId == 0) return NO;

    char *endPtr = NULL;
    double num = strtod(valStr, &endPtr);
    if (endPtr == valStr) return NO;

    uint8_t buf[8] = {0};
    switch (spec->fmt) {
        case FMT_U8: {
            int v = (int)llround(num);
            if (v < 0) v = 0;
            if (v > 255) v = 255;
            buf[0] = (uint8_t)v;
            break;
        }
        case FMT_U16: {
            long v = llround(num);
            if (v < 0) v = 0;
            if (v > 65535) v = 65535;
            uint16_t u = (uint16_t)v;
            buf[0] = (uint8_t)(u & 0xff);
            buf[1] = (uint8_t)((u >> 8) & 0xff);
            break;
        }
        case FMT_S16: {
            long v = llround(num);
            if (v < -32768) v = -32768;
            if (v > 32767) v = 32767;
            uint16_t u = (uint16_t)((int16_t)v);
            buf[0] = (uint8_t)(u & 0xff);
            buf[1] = (uint8_t)((u >> 8) & 0xff);
            break;
        }
        case FMT_U32: {
            long long v = llround(num);
            if (v < 0) v = 0;
            if (v > 0xffffffffLL) v = 0xffffffffLL;
            uint32_t u = (uint32_t)v;
            buf[0] = (uint8_t)(u & 0xff);
            buf[1] = (uint8_t)((u >> 8) & 0xff);
            buf[2] = (uint8_t)((u >> 16) & 0xff);
            buf[3] = (uint8_t)((u >> 24) & 0xff);
            break;
        }
        case FMT_EV_CENTI: {
            if (num < -3.0) num = -3.0;
            if (num > 3.0) num = 3.0;
            // Ensure Insta360 Auto Exposure (XU 0x1E = 2) is enabled so EV compensation is active
            uint8_t aeCur = 0;
            if (cam_ctrl_transfer(cam, YES, UVC_GET_CUR, unitId, INSTA_XU_AUTO_EXP, &aeCur, 1, NULL)) {
                if (aeCur != 2) {
                    uint8_t aeAuto = 2;
                    cam_ctrl_transfer(cam, NO, UVC_SET_CUR, unitId, INSTA_XU_AUTO_EXP, &aeAuto, 1, NULL);
                }
            }
            // Query firmware step size (GET_RES) and snap cleanly (Link 2 uses step=6 on [-300..+300])
            int step = 6;
            uint8_t resBuf[2] = {0};
            if (cam_ctrl_transfer(cam, YES, UVC_GET_RES, unitId, spec->selector, resBuf, 2, NULL)) {
                int16_t fwStep = (int16_t)(resBuf[0] | ((uint16_t)resBuf[1] << 8));
                if (fwStep >= 1 && fwStep <= 50) {
                    step = fwStep;
                }
            }
            int raw = (int)llround((num * 100.0) / (double)step) * step;
            if (raw < -300) raw = -300;
            if (raw > 300) raw = 300;
            uint16_t u = (uint16_t)((int16_t)raw);
            buf[0] = (uint8_t)(u & 0xff);
            buf[1] = (uint8_t)((u >> 8) & 0xff);
            break;
        }
        case FMT_HDR_BIT: {
            uint8_t cur[2] = {0, 0};
            if (!cam_ctrl_transfer(cam, YES, UVC_GET_CUR, unitId, spec->selector, cur, 2, outRc)) {
                return NO;
            }
            uint16_t mask = (uint16_t)(cur[0] | ((uint16_t)cur[1] << 8));
            if (llround(num) != 0) {
                mask |= (uint16_t)(1u << INSTA_FEATURE_HDR_BIT);
            } else {
                mask &= (uint16_t)~(1u << INSTA_FEATURE_HDR_BIT);
            }
            buf[0] = (uint8_t)(mask & 0xff);
            buf[1] = (uint8_t)((mask >> 8) & 0xff);
            break;
        }
    }

    return cam_ctrl_transfer(cam, NO, UVC_SET_CUR, unitId, spec->selector, buf, spec->byteLen, outRc);
}

static void print_usage(const char *prog) {
    printf("Usage: %s [device-selection] [actions]\n\n", prog);
    printf("Device selection (defaults to Insta360 0x2e1a if connected, else index 0):\n");
    printf("  -d, --list-devices          List connected UVC cameras\n");
    printf("  -I, --index <N>             Select UVC camera by index\n");
    printf("  -V, --vendor <vid[:pid]>    Select UVC camera by hex vendor[:product] (e.g. 0x2e1a:0x4c05)\n\n");
    printf("Actions:\n");
    printf("  -c, --list-controls         List readable controls and current values\n");
    printf("  -p, --probe                 Detailed UVC + Insta360 Extension Unit diagnostic dump\n");
    printf("  -g, --get <control>         Read current control value\n");
    printf("  -s, --set <control>=<value> Write control value cooperatively over USB Endpoint 0\n");
    printf("      --ev <float>            Shorthand for -s exposure-comp-ev=<float> (-3.0 to +3.0)\n");
    printf("  -R, --usb-reset             Re-enumerate USB device via IOKit (emulates physical unplug/replug)\n\n");
    printf("Examples:\n");
    printf("  %s --probe\n", prog);
    printf("  %s -V 0x2e1a -s exposure-comp-ev=-1.8\n", prog);
    printf("  %s -V 0x2e1a -s exposure-comp-ev=0.0\n", prog);
    printf("  %s -V 0x2e1a:0x4c05 --usb-reset\n", prog);
}

static int usb_reenumerate_matching_device(int matchVid, int matchPid) {
    uint64_t seenIds[64];
    int seenCount = 0;
    const char *classNames[] = { "IOUSBHostDevice", kIOUSBDeviceClassName };

    for (size_t c = 0; c < 2; c++) {
        CFMutableDictionaryRef match = IOServiceMatching(classNames[c]);
        if (!match) continue;
        io_iterator_t iter = 0;
        if (IOServiceGetMatchingServices(kIOMainPortDefault, match, &iter) != KERN_SUCCESS || !iter) {
            continue;
        }
        io_service_t svc;
        while ((svc = IOIteratorNext(iter)) != 0) {
            uint64_t entryId = 0;
            IORegistryEntryGetRegistryEntryID(svc, &entryId);
            BOOL duplicate = NO;
            for (int i = 0; i < seenCount; i++) {
                if (seenIds[i] == entryId) {
                    duplicate = YES;
                    break;
                }
            }
            if (duplicate) {
                IOObjectRelease(svc);
                continue;
            }
            if (seenCount < 64) seenIds[seenCount++] = entryId;

            int32_t vid = 0, pid = 0, loc = 0;
            if (!get_int_prop(svc, CFSTR(kUSBVendorID), &vid) ||
                !get_int_prop(svc, CFSTR(kUSBProductID), &pid)) {
                IOObjectRelease(svc);
                continue;
            }
            if (vid != matchVid || (matchPid > 0 && pid != matchPid)) {
                IOObjectRelease(svc);
                continue;
            }
            get_int_prop(svc, CFSTR(kUSBDevicePropertyLocationID), &loc);
            char devName[128] = {0};
            get_str_prop(svc, devName, sizeof(devName));

            IOCFPlugInInterface **plugIn = NULL;
            SInt32 score = 0;
            kern_return_t krc = IOCreatePlugInInterfaceForService(
                svc, kIOUSBDeviceUserClientTypeID, kIOCFPlugInInterfaceID, &plugIn, &score
            );
            IOObjectRelease(svc);
            if (krc != kIOReturnSuccess || !plugIn) {
                fprintf(stderr, "ERROR: IOCreatePlugInInterfaceForService failed (0x%08x)\n", krc);
                return 1;
            }

            IOUSBDeviceInterface187 **dev187 = NULL;
            HRESULT hrc = (*plugIn)->QueryInterface(
                plugIn, CFUUIDGetUUIDBytes(kIOUSBDeviceInterfaceID187), (LPVOID *)&dev187
            );
            IODestroyPlugInInterface(plugIn);
            if (hrc != 0 || !dev187) {
                fprintf(stderr, "ERROR: QueryInterface(IOUSBDeviceInterface187) failed (0x%08x)\n", (unsigned)hrc);
                return 1;
            }

            IOReturn openRc = kIOReturnError;
            for (int attempt = 0; attempt < 6; attempt++) {
                openRc = (*dev187)->USBDeviceOpen(dev187);
                if (openRc == kIOReturnSuccess) break;
                openRc = (*dev187)->USBDeviceOpenSeize(dev187);
                if (openRc == kIOReturnSuccess) break;
                usleep(200000);
            }
            if (openRc != kIOReturnSuccess) {
                fprintf(stderr, "ERROR: USBDeviceOpenSeize failed on %s 0x%04x:0x%04x (0x%08x)\n",
                        devName, vid, pid, openRc);
                (*dev187)->Release(dev187);
                return 1;
            }

            // Issue bus reset + full USB re-enumeration (emulates physical unplug/replug)
            IOReturn resetRc = (*dev187)->ResetDevice(dev187);
            IOReturn reenumRc = (*dev187)->USBDeviceReEnumerate(dev187, 0);
            if (reenumRc != kIOReturnSuccess) {
                (*dev187)->USBDeviceClose(dev187);
            }
            (*dev187)->Release(dev187);

            if (reenumRc == kIOReturnSuccess || resetRc == kIOReturnSuccess) {
                printf("USB_REENUMERATED name=\"%s\" vendor=0x%04x product=0x%04x location=0x%08x resetRc=0x%08x reenumRc=0x%08x\n",
                       devName, vid, pid, (uint32_t)loc, resetRc, reenumRc);
                return 0;
            }
            fprintf(stderr, "ERROR: USBDeviceReEnumerate failed on %s (resetRc=0x%08x, reenumRc=0x%08x)\n",
                    devName, resetRc, reenumRc);
            return 1;
        }
        IOObjectRelease(iter);
    }
    fprintf(stderr, "ERROR: No USB device matching vendor=0x%04x%s found in IORegistry.\n",
            matchVid, (matchPid > 0) ? " (with specified product)" : "");
    return 1;
}

static BOOL parse_vid_pid(const char *arg, int *outVid, int *outPid) {
    *outVid = -1;
    *outPid = -1;
    const char *colon = strchr(arg, ':');
    if (!colon) {
        *outVid = (int)strtol(arg, NULL, 16);
        return (*outVid > 0);
    }
    char vidBuf[32] = {0};
    size_t len = (size_t)(colon - arg);
    if (len >= sizeof(vidBuf)) return NO;
    memcpy(vidBuf, arg, len);
    *outVid = (int)strtol(vidBuf, NULL, 16);
    *outPid = (int)strtol(colon + 1, NULL, 16);
    return (*outVid > 0);
}

int main(int argc, char * const argv[]) {
    @autoreleasepool {
        BOOL optListDevices  = NO;
        BOOL optListControls = NO;
        BOOL optProbe        = NO;
        BOOL optUsbReset     = NO;
        int  targetIndex     = -1;
        int  targetVid       = -1;
        int  targetPid       = -1;

        NSMutableArray<NSString *> *gets = [NSMutableArray array];
        NSMutableArray<NSString *> *sets = [NSMutableArray array];

        static struct option longOpts[] = {
            {"list-devices",  no_argument,       NULL, 'd'},
            {"list-controls", no_argument,       NULL, 'c'},
            {"probe",         no_argument,       NULL, 'p'},
            {"usb-reset",     no_argument,       NULL, 'R'},
            {"index",         required_argument, NULL, 'I'},
            {"vendor",        required_argument, NULL, 'V'},
            {"get",           required_argument, NULL, 'g'},
            {"set",           required_argument, NULL, 's'},
            {"ev",            required_argument, NULL, 'e'},
            {"help",          no_argument,       NULL, 'h'},
            {NULL, 0, NULL, 0}
        };

        int ch;
        while ((ch = getopt_long(argc, argv, "dcpRI:V:g:s:e:h", longOpts, NULL)) != -1) {
            switch (ch) {
                case 'd': optListDevices = YES; break;
                case 'c': optListControls = YES; break;
                case 'p': optProbe = YES; break;
                case 'R': optUsbReset = YES; break;
                case 'I': targetIndex = atoi(optarg); break;
                case 'V':
                    if (!parse_vid_pid(optarg, &targetVid, &targetPid)) {
                        fprintf(stderr, "Invalid vendor[:product] spec: %s\n", optarg);
                        return 2;
                    }
                    break;
                case 'g':
                    [gets addObject:[NSString stringWithUTF8String:optarg]];
                    break;
                case 's':
                    [sets addObject:[NSString stringWithUTF8String:optarg]];
                    break;
                case 'e':
                    [sets addObject:[NSString stringWithFormat:@"exposure-comp-ev=%s", optarg]];
                    break;
                case 'h':
                default:
                    print_usage(argv[0]);
                    return (ch == 'h') ? 0 : 2;
            }
        }

        if (optUsbReset) {
            int matchVid = (targetVid > 0) ? targetVid : INSTA360_VENDOR_ID;
            return usb_reenumerate_matching_device(matchVid, targetPid);
        }

        if (!optListDevices && !optListControls && !optProbe && gets.count == 0 && sets.count == 0) {
            print_usage(argv[0]);
            return 0;
        }

        uvc_cam_t cams[16];
        int camCount = discover_uvc_cameras(cams, 16);

        if (optListDevices) {
            if (camCount == 0) {
                printf("No external UVC USB cameras found.\n");
            } else {
                for (int i = 0; i < camCount; i++) {
                    printf("[%d] %s (vendor=0x%04x, product=0x%04x, location=0x%08x, vcIf=%u, ct=%u, pu=%u, instaXU=%u)\n",
                           i, cams[i].name, cams[i].vendorId, cams[i].productId,
                           cams[i].locationId, cams[i].vcInterfaceNum,
                           cams[i].ctUnitId, cams[i].puUnitId, cams[i].instaXuUnitId);
                }
            }
            if (!optListControls && !optProbe && gets.count == 0 && sets.count == 0) {
                for (int i = 0; i < camCount; i++) cam_close(&cams[i]);
                return 0;
            }
        }

        if (camCount == 0) {
            fprintf(stderr, "ERROR: No external UVC USB cameras detected on this Mac.\n");
            return 1;
        }

        int sel = -1;
        if (targetIndex >= 0) {
            if (targetIndex < camCount) sel = targetIndex;
        } else if (targetVid > 0) {
            for (int i = 0; i < camCount; i++) {
                if (cams[i].vendorId == (uint16_t)targetVid &&
                    (targetPid < 0 || cams[i].productId == (uint16_t)targetPid)) {
                    sel = i;
                    break;
                }
            }
        } else {
            // Prefer Insta360 (0x2e1a) automatically if plugged in, otherwise index 0
            for (int i = 0; i < camCount; i++) {
                if (cams[i].vendorId == INSTA360_VENDOR_ID) {
                    sel = i;
                    break;
                }
            }
            if (sel < 0) sel = 0;
        }

        if (sel < 0) {
            fprintf(stderr, "ERROR: Requested UVC camera not found among %d connected UVC device(s).\n", camCount);
            for (int i = 0; i < camCount; i++) cam_close(&cams[i]);
            return 1;
        }

        uvc_cam_t *cam = &cams[sel];
        int exitCode = 0;

        if (optProbe) {
            printf("Camera [%d]: %s\n", sel, cam->name);
            printf("  USB ID:        0x%04x:0x%04x (location 0x%08x)\n", cam->vendorId, cam->productId, cam->locationId);
            printf("  VC Interface:  %u (CT=%u, PU=%u, Insta360_XU=%u)\n",
                   cam->vcInterfaceNum, cam->ctUnitId, cam->puUnitId, cam->instaXuUnitId);
            for (int x = 0; x < cam->xuCount; x++) {
                xu_info_t *xi = &cam->xus[x];
                printf("  ExtensionUnit: id=%u controls=%u guid=%02x%02x%02x%02x-%02x%02x-%02x%02x-%02x%02x-%02x%02x%02x%02x%02x%02x\n",
                       xi->unitId, xi->numControls,
                       xi->guid[3], xi->guid[2], xi->guid[1], xi->guid[0],
                       xi->guid[5], xi->guid[4], xi->guid[7], xi->guid[6],
                       xi->guid[8], xi->guid[9], xi->guid[10], xi->guid[11],
                       xi->guid[12], xi->guid[13], xi->guid[14], xi->guid[15]);
            }
            printf("\nSupported Controls:\n");
        }

        if (optProbe || optListControls) {
            for (size_t i = 0; i < kNumControls; i++) {
                const ctrl_spec_t *sp = &kControls[i];
                if (sp->unitKind == UNIT_INSTA_XU && cam->vendorId != INSTA360_VENDOR_ID) continue;
                double cur = 0, mn = 0, mx = 0, st = 0, df = 0;
                if (read_ctrl_value(cam, sp, UVC_GET_CUR, &cur, NULL)) {
                    char curS[32], minS[32], maxS[32], stS[32], defS[32];
                    format_val(curS, sizeof(curS), cur, sp);
                    BOOL hasMin = read_ctrl_value(cam, sp, UVC_GET_MIN, &mn, NULL);
                    BOOL hasMax = read_ctrl_value(cam, sp, UVC_GET_MAX, &mx, NULL);
                    BOOL hasRes = read_ctrl_value(cam, sp, UVC_GET_RES, &st, NULL);
                    BOOL hasDef = read_ctrl_value(cam, sp, UVC_GET_DEF, &df, NULL);
                    printf("  %-24s = %-8s", sp->name, curS);
                    if (hasMin && hasMax) {
                        format_val(minS, sizeof(minS), mn, sp);
                        format_val(maxS, sizeof(maxS), mx, sp);
                        printf(" [range %s .. %s", minS, maxS);
                        if (hasRes) {
                            format_val(stS, sizeof(stS), st, sp);
                            printf(", step %s", stS);
                        }
                        if (hasDef) {
                            format_val(defS, sizeof(defS), df, sp);
                            printf(", default %s", defS);
                        }
                        printf("]");
                    }
                    if (optProbe) {
                        printf("  -- %s", sp->desc);
                    }
                    printf("\n");
                }
            }
        }

        for (NSString *setExpr in sets) {
            const char *cstr = [setExpr UTF8String];
            const char *eq = strchr(cstr, '=');
            if (!eq) {
                fprintf(stderr, "ERROR: Invalid --set syntax '%s' (expected name=value)\n", cstr);
                exitCode = 2;
                continue;
            }
            char nameBuf[64] = {0};
            size_t nlen = (size_t)(eq - cstr);
            if (nlen == 0 || nlen >= sizeof(nameBuf)) {
                fprintf(stderr, "ERROR: Invalid control name in '%s'\n", cstr);
                exitCode = 2;
                continue;
            }
            memcpy(nameBuf, cstr, nlen);
            const char *valStr = eq + 1;

            const ctrl_spec_t *sp = find_ctrl_spec(nameBuf);
            if (!sp) {
                fprintf(stderr, "ERROR: Unknown control '%s'. Run with --list-controls.\n", nameBuf);
                exitCode = 2;
                continue;
            }

            IOReturn rc = kIOReturnSuccess;
            if (!write_ctrl_value(cam, sp, valStr, &rc)) {
                fprintf(stderr, "ERROR: Failed writing %s=%s (IOReturn 0x%08x)\n", sp->name, valStr, rc);
                exitCode = 1;
                continue;
            }

            double readBack = 0;
            if (read_ctrl_value(cam, sp, UVC_GET_CUR, &readBack, NULL)) {
                char rbS[32];
                format_val(rbS, sizeof(rbS), readBack, sp);
                printf("%s=%s\n", sp->name, rbS);
            } else {
                printf("%s=%s\n", sp->name, valStr);
            }
        }

        for (NSString *getName in gets) {
            const char *nameStr = [getName UTF8String];
            const ctrl_spec_t *sp = find_ctrl_spec(nameStr);
            if (!sp) {
                fprintf(stderr, "ERROR: Unknown control '%s'.\n", nameStr);
                exitCode = 2;
                continue;
            }
            double cur = 0;
            IOReturn rc = kIOReturnSuccess;
            if (!read_ctrl_value(cam, sp, UVC_GET_CUR, &cur, &rc)) {
                fprintf(stderr, "ERROR: Failed reading %s (IOReturn 0x%08x)\n", sp->name, rc);
                exitCode = 1;
                continue;
            }
            char curS[32];
            format_val(curS, sizeof(curS), cur, sp);
            printf("%s=%s\n", sp->name, curS);
        }

        for (int i = 0; i < camCount; i++) {
            cam_close(&cams[i]);
        }
        return exitCode;
    }
}
