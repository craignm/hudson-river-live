//
//  ptz_tracker.m
//  Real-Time Apple Neural Engine Visual Object Detector & Tracker for PTZ Target Framing
//
//  Combines three Apple Silicon hardware-accelerated vision stages in a persistent
//  zero-overhead stdin/stdout JSON-RPC process (~6 ms/frame on M-series Neural Engine):
//    1. CoreML YOLOv11s (`yolo11s.mlpackage` -> cached `.mlmodelc`) full-frame + automatic
//       2.2x zoomed sub-crop pass for small/distant aircraft & helicopters.
//    2. Apple Vision `VNGenerateObjectnessBasedSaliencyImageRequest` + `VNClassifyImageRequest`
//       (1,303-class taxonomy including `helicopter`, `aircraft`, `airplane`, `boat`, `sailboat`,
//       `cruise_ship`, `speedboat`, `warship`) so Hudson River helicopters and night silhouettes
//       lock reliably even when COCO `airplane` confidence is low.
//    3. Stateful `VNSequenceRequestHandler` + `VNTrackObjectRequest` (`VNRequestTrackingLevelAccurate`)
//       to maintain continuous inter-frame identity & smooth sub-pixel centroids across frames
//       without jumping onto passing vessels or water wake.
//
//  Coordinate convention (matches `server.py`):
//    Normalized `(u, v, w, h)` in `[0..1]` with `(0, 0)` at TOP-LEFT and `(1, 1)` at BOTTOM-RIGHT.
//
//  Build universal macOS binary:
//    clang -O3 -fobjc-arc -Wall -Wno-deprecated-declarations -arch arm64 -arch x86_64 \
//      -framework Foundation -framework CoreGraphics -framework ImageIO \
//      -framework Vision -framework CoreML \
//      ptz_tracker.m -o ptz-tracker && codesign -s - --force ptz-tracker
//

#import <Foundation/Foundation.h>
#import <CoreGraphics/CoreGraphics.h>
#import <ImageIO/ImageIO.h>
#import <Vision/Vision.h>
#import <CoreML/CoreML.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
    BOOL found;
    double u;      // 0..1 left-to-right
    double v;      // 0..1 top-to-bottom
    double w;      // 0..1 width
    double h;      // 0..1 height
    double conf;
    char source[24];
    char label[32];
} track_hit_t;

static CGImageRef create_cgimage_from_jpeg_data(NSData *jpegData) {
    if (!jpegData || jpegData.length < 64) return NULL;
    CGImageSourceRef src = CGImageSourceCreateWithData((__bridge CFDataRef)jpegData, NULL);
    if (!src) return NULL;
    CGImageRef cg = CGImageSourceCreateImageAtIndex(src, 0, NULL);
    CFRelease(src);
    return cg;
}

static CGImageRef create_cgimage_from_path(NSString *path) {
    if (!path.length) return NULL;
    NSURL *url = [NSURL fileURLWithPath:path];
    CGImageSourceRef src = CGImageSourceCreateWithURL((__bridge CFURLRef)url, NULL);
    if (!src) return NULL;
    CGImageRef cg = CGImageSourceCreateImageAtIndex(src, 0, NULL);
    CFRelease(src);
    return cg;
}

// Convert Vision bottom-left CGRect -> top-left center `(u, v, w, h)`
static inline void vision_rect_to_uvwh(CGRect r, double *u, double *v, double *w, double *h) {
    *w = fmax(0.001, fmin(1.0, (double)r.size.width));
    *h = fmax(0.001, fmin(1.0, (double)r.size.height));
    *u = fmax(0.0, fmin(1.0, (double)(r.origin.x + 0.5 * r.size.width)));
    *v = fmax(0.0, fmin(1.0, 1.0 - (double)(r.origin.y + 0.5 * r.size.height)));
}

static inline CGRect uvwh_to_vision_rect(double u, double v, double w, double h) {
    double cw = fmax(0.012, fmin(0.92, w));
    double ch = fmax(0.012, fmin(0.92, h));
    double ox = fmax(0.0, fmin(1.0 - cw, u - 0.5 * cw));
    double oy = fmax(0.0, fmin(1.0 - ch, (1.0 - v) - 0.5 * ch));
    return CGRectMake(ox, oy, cw, ch);
}

static double match_yolo_class_conf(VNRecognizedObjectObservation *obs, BOOL wantAir, char *outLabel, size_t outCap) {
    double best = 0.0;
    const char *bestName = "";
    NSUInteger limit = MIN((NSUInteger)8, obs.labels.count);
    for (NSUInteger i = 0; i < limit; i++) {
        VNClassificationObservation *cl = obs.labels[i];
        NSString *ident = cl.identifier.lowercaseString;
        double c = (double)cl.confidence;
        if (c < 0.08) break;
        if (wantAir) {
            if ([ident isEqualToString:@"airplane"]) {
                if (c > best) { best = c; bestName = "airplane"; }
            }
        } else {
            if ([ident isEqualToString:@"boat"]) {
                if (c > best) { best = c; bestName = "boat"; }
            } else if ([ident isEqualToString:@"surfboard"] && c * 0.75 > best) {
                best = c * 0.75;
                bestName = "watercraft";
            }
        }
    }
    if (best > 0.0 && outLabel && outCap > 0) {
        snprintf(outLabel, outCap, "%s", bestName);
    }
    return best;
}

static VNCoreMLModel *load_or_compile_yolo(NSString *pkgPath, NSString *cachePath) {
    NSFileManager *fm = [NSFileManager defaultManager];
    NSError *err = nil;
    MLModelConfiguration *cfg = [[MLModelConfiguration alloc] init];
    cfg.computeUnits = MLComputeUnitsAll;

    if (cachePath.length && [fm fileExistsAtPath:cachePath]) {
        MLModel *cachedML = [MLModel modelWithContentsOfURL:[NSURL fileURLWithPath:cachePath] configuration:cfg error:&err];
        if (cachedML) {
            VNCoreMLModel *vnm = [VNCoreMLModel modelForMLModel:cachedML error:&err];
            if (vnm) return vnm;
        }
    }

    if (!pkgPath.length || ![fm fileExistsAtPath:pkgPath]) {
        return nil;
    }

    NSURL *tmpCompiled = [MLModel compileModelAtURL:[NSURL fileURLWithPath:pkgPath] error:&err];
    if (!tmpCompiled) {
        fprintf(stderr, "[ptz-tracker] CoreML compile error: %s\n", [[err localizedDescription] UTF8String]);
        return nil;
    }

    NSURL *loadURL = tmpCompiled;
    if (cachePath.length) {
        [fm removeItemAtPath:cachePath error:nil];
        NSString *parent = [cachePath stringByDeletingLastPathComponent];
        if (parent.length) {
            [fm createDirectoryAtPath:parent withIntermediateDirectories:YES attributes:nil error:nil];
        }
        if ([fm copyItemAtURL:tmpCompiled toURL:[NSURL fileURLWithPath:cachePath] error:nil]) {
            [fm removeItemAtURL:tmpCompiled error:nil];
            loadURL = [NSURL fileURLWithPath:cachePath];
        }
    }

    MLModel *ml = [MLModel modelWithContentsOfURL:loadURL configuration:cfg error:&err];
    if (!ml) {
        fprintf(stderr, "[ptz-tracker] CoreML load error: %s\n", [[err localizedDescription] UTF8String]);
        return nil;
    }
    return [VNCoreMLModel modelForMLModel:ml error:&err];
}

// Run YOLOv11s on `cgImg` (optionally inside normalized top-left ROI `[rx0, ry0, rw, rh]`)
static void collect_yolo_candidates(
    VNCoreMLModel *vnModel,
    CGImageRef cgImg,
    BOOL wantAir,
    double rx0, double ry0, double rw, double rh,
    NSMutableArray<NSDictionary *> *outCands
) {
    if (!vnModel || !cgImg) return;
    CGImageRef subImg = NULL;
    if (rw < 0.999 || rh < 0.999) {
        size_t iw = CGImageGetWidth(cgImg);
        size_t ih = CGImageGetHeight(cgImg);
        CGRect cropPx = CGRectMake(round(rx0 * iw), round(ry0 * ih), round(rw * iw), round(rh * ih));
        subImg = CGImageCreateWithImageInRect(cgImg, cropPx);
        if (!subImg) return;
    }
    CGImageRef targetImg = subImg ? subImg : cgImg;

    VNCoreMLRequest *req = [[VNCoreMLRequest alloc] initWithModel:vnModel];
    req.imageCropAndScaleOption = VNImageCropAndScaleOptionScaleFill;
    VNImageRequestHandler *handler = [[VNImageRequestHandler alloc] initWithCGImage:targetImg options:@{}];
    NSError *err = nil;
    if ([handler performRequests:@[req] error:&err]) {
        double minConf = wantAir ? 0.14 : 0.17;
        for (VNObservation *rawObs in req.results) {
            if (![rawObs isKindOfClass:[VNRecognizedObjectObservation class]]) continue;
            VNRecognizedObjectObservation *ro = (VNRecognizedObjectObservation *)rawObs;
            char lbl[32] = {0};
            double clsConf = match_yolo_class_conf(ro, wantAir, lbl, sizeof(lbl));
            if (clsConf < minConf) continue;

            double su, sv, sw, sh;
            vision_rect_to_uvwh(ro.boundingBox, &su, &sv, &sw, &sh);
            double u = rx0 + su * rw;
            double v = ry0 + sv * rh;
            double w = sw * rw;
            double h = sh * rh;
            if (w > 0.88 || h > 0.84) continue;
            if (w < 0.009 && h < 0.009) continue;

            [outCands addObject:@{
                @"u": @(u),
                @"v": @(v),
                @"w": @(w),
                @"h": @(h),
                @"conf": @(clsConf),
                @"label": [NSString stringWithUTF8String:lbl],
                @"source": (subImg ? @"yolo11s_crop" : @"yolo11s"),
            }];
        }
    }
    if (subImg) CGImageRelease(subImg);
}

// Apple Vision Saliency + 1,303-class Classifier fallback (excels on Hudson helicopters & lit night vessels)
static void collect_saliency_classified_candidates(
    CGImageRef cgImg,
    BOOL wantAir,
    double expectU,
    double expectV,
    NSMutableArray<NSDictionary *> *outCands
) {
    if (!cgImg) return;
    VNGenerateObjectnessBasedSaliencyImageRequest *salReq = [[VNGenerateObjectnessBasedSaliencyImageRequest alloc] init];
    VNImageRequestHandler *handler = [[VNImageRequestHandler alloc] initWithCGImage:cgImg options:@{}];
    NSError *err = nil;
    if (![handler performRequests:@[salReq] error:&err] || !salReq.results.count) return;

    VNSaliencyImageObservation *salObs = salReq.results.firstObject;
    NSArray<VNRectangleObservation *> *rects = salObs.salientObjects;
    if (!rects.count) return;

    size_t iw = CGImageGetWidth(cgImg);
    size_t ih = CGImageGetHeight(cgImg);

    for (VNRectangleObservation *r in rects) {
        if (r.confidence < 0.52f) continue;
        double u, v, w, h;
        vision_rect_to_uvwh(r.boundingBox, &u, &v, &w, &h);
        if (w > 0.72 || h > 0.68 || (w < 0.015 && h < 0.015)) continue;
        if (hypot(u - expectU, v - expectV) > 0.36) continue;

        // Classify padded crop with Apple's Neural Engine 1,303-class taxonomy
        double padW = fmin(1.0, w * 1.6 + 0.04);
        double padH = fmin(1.0, h * 1.6 + 0.04);
        double cx0 = fmax(0.0, fmin(1.0 - padW, u - 0.5 * padW));
        double cy0 = fmax(0.0, fmin(1.0 - padH, v - 0.5 * padH));
        CGRect pxRect = CGRectMake(round(cx0 * iw), round(cy0 * ih), round(padW * iw), round(padH * ih));
        CGImageRef crop = CGImageCreateWithImageInRect(cgImg, pxRect);
        if (!crop) continue;

        VNClassifyImageRequest *clsReq = [[VNClassifyImageRequest alloc] init];
        VNImageRequestHandler *ch = [[VNImageRequestHandler alloc] initWithCGImage:crop options:@{}];
        BOOL okCls = [ch performRequests:@[clsReq] error:nil];
        CGImageRelease(crop);
        if (!okCls) continue;

        double bestCls = 0.0;
        NSString *bestIdent = nil;
        for (VNClassificationObservation *co in clsReq.results) {
            if (co.confidence < 0.08f) break;
            NSString *idLow = co.identifier.lowercaseString;
            if (wantAir) {
                if ([idLow isEqualToString:@"helicopter"] || [idLow isEqualToString:@"aircraft"] ||
                    [idLow isEqualToString:@"airplane"] || [idLow isEqualToString:@"airshow"]) {
                    if (co.confidence > bestCls) {
                        bestCls = co.confidence;
                        bestIdent = idLow;
                    }
                }
            } else {
                if ([idLow isEqualToString:@"boat"] || [idLow isEqualToString:@"cruise_ship"] ||
                    [idLow isEqualToString:@"sailboat"] || [idLow isEqualToString:@"speedboat"] ||
                    [idLow isEqualToString:@"houseboat"] || [idLow isEqualToString:@"rowboat"] ||
                    [idLow isEqualToString:@"warship"]) {
                    if (co.confidence > bestCls) {
                        bestCls = co.confidence;
                        bestIdent = idLow;
                    }
                }
            }
        }
        if (bestCls >= 0.16 && bestIdent) {
            [outCands addObject:@{
                @"u": @(u),
                @"v": @(v),
                @"w": @(w),
                @"h": @(h),
                @"conf": @(fmin(0.92, 0.45 * r.confidence + 0.65 * bestCls)),
                @"label": bestIdent,
                @"source": @"vision_saliency",
            }];
        }
    }
}

int main(int argc, const char *argv[]) {
    @autoreleasepool {
        NSString *pkgPath = (argc >= 2) ? [NSString stringWithUTF8String:argv[1]] : @"";
        NSString *cachePath = (argc >= 3) ? [NSString stringWithUTF8String:argv[2]] : @"";
        VNCoreMLModel *yoloModel = load_or_compile_yolo(pkgPath, cachePath);

        VNSequenceRequestHandler *seqHandler = [[VNSequenceRequestHandler alloc] init];
        VNDetectedObjectObservation *trackedObs = nil;
        NSString *activeTargetId = @"";
        NSString *lastLabel = @"target";
        int coastFrames = 0;

        char *lineBuf = NULL;
        size_t lineCap = 0;
        ssize_t lineLen = 0;

        while ((lineLen = getline(&lineBuf, &lineCap, stdin)) > 0) {
            @autoreleasepool {
                CFAbsoluteTime t0 = CFAbsoluteTimeGetCurrent();
                NSData *reqData = [NSData dataWithBytesNoCopy:lineBuf length:(NSUInteger)lineLen freeWhenDone:NO];
                NSDictionary *cmdObj = [NSJSONSerialization JSONObjectWithData:reqData options:0 error:nil];
                if (![cmdObj isKindOfClass:[NSDictionary class]]) {
                    printf("{\"ok\":false,\"error\":\"invalid_json\"}\n");
                    fflush(stdout);
                    continue;
                }

                NSString *cmd = cmdObj[@"cmd"] ?: @"";
                if ([cmd isEqualToString:@"ping"]) {
                    printf("{\"ok\":true,\"has_yolo\":%s}\n", yoloModel ? "true" : "false");
                    fflush(stdout);
                    continue;
                }
                if ([cmd isEqualToString:@"reset"]) {
                    seqHandler = [[VNSequenceRequestHandler alloc] init];
                    trackedObs = nil;
                    activeTargetId = cmdObj[@"target_id"] ?: @"";
                    coastFrames = 0;
                    printf("{\"ok\":true,\"reset\":true}\n");
                    fflush(stdout);
                    continue;
                }

                NSString *tid = cmdObj[@"target_id"] ?: @"";
                BOOL forceReseed = [cmdObj[@"reseed"] boolValue];
                if (forceReseed || (tid.length && ![tid isEqualToString:activeTargetId])) {
                    seqHandler = [[VNSequenceRequestHandler alloc] init];
                    trackedObs = nil;
                    if (tid.length) activeTargetId = tid;
                    coastFrames = 0;
                }

                NSString *kind = [cmdObj[@"kind"] ?: @"vessel" lowercaseString];
                BOOL wantAir = ([kind isEqualToString:@"aircraft"] || [kind isEqualToString:@"helicopter"]);
                double expectU = cmdObj[@"expect_u"] ? [cmdObj[@"expect_u"] doubleValue] : 0.5;
                double expectV = cmdObj[@"expect_v"] ? [cmdObj[@"expect_v"] doubleValue] : 0.5;

                CGImageRef cgImg = NULL;
                NSString *b64 = cmdObj[@"jpeg_b64"];
                if ([b64 isKindOfClass:[NSString class]] && b64.length > 32) {
                    NSData *jpg = [[NSData alloc] initWithBase64EncodedString:b64 options:NSDataBase64DecodingIgnoreUnknownCharacters];
                    cgImg = create_cgimage_from_jpeg_data(jpg);
                } else if ([cmdObj[@"path"] isKindOfClass:[NSString class]]) {
                    cgImg = create_cgimage_from_path(cmdObj[@"path"]);
                }

                if (!cgImg) {
                    printf("{\"ok\":false,\"error\":\"bad_image\"}\n");
                    fflush(stdout);
                    continue;
                }

                // 1) Advance inter-frame optical-flow / Siamese object tracker if locked
                BOOL hasTrk = NO;
                double trkU = expectU, trkV = expectV, trkW = 0.08, trkH = 0.08, trkConf = 0.0;
                if (trackedObs != nil && coastFrames <= 10) {
                    VNTrackObjectRequest *trkReq = [[VNTrackObjectRequest alloc] initWithDetectedObjectObservation:trackedObs];
                    trkReq.trackingLevel = VNRequestTrackingLevelAccurate;
                    NSError *tErr = nil;
                    if ([seqHandler performRequests:@[trkReq] onCGImage:cgImg error:&tErr] && trkReq.results.count > 0) {
                        VNDetectedObjectObservation *nextObs = trkReq.results.firstObject;
                        if (nextObs.confidence >= 0.36f) {
                            vision_rect_to_uvwh(nextObs.boundingBox, &trkU, &trkV, &trkW, &trkH);
                            if (trkW <= 0.88 && trkH <= 0.84 && trkU > 0.01 && trkU < 0.99 && trkV > 0.01 && trkV < 0.99) {
                                hasTrk = YES;
                                trkConf = nextObs.confidence;
                                trackedObs = nextObs;
                            }
                        }
                    }
                    if (!hasTrk) {
                        trackedObs = nil;
                        seqHandler = [[VNSequenceRequestHandler alloc] init];
                    }
                }

                // 2) Run Neural Engine YOLOv11s full-frame detection
                NSMutableArray<NSDictionary *> *cands = [NSMutableArray array];
                if (yoloModel) {
                    collect_yolo_candidates(yoloModel, cgImg, wantAir, 0.0, 0.0, 1.0, 1.0, cands);
                    // Automatic 2.1x sub-crop pass if small aircraft/boat wasn't caught in full frame
                    if (cands.count == 0) {
                        double refU = hasTrk ? trkU : expectU;
                        double refV = hasTrk ? trkV : expectV;
                        double cw = 0.48, ch = 0.48;
                        double rx0 = fmax(0.0, fmin(1.0 - cw, refU - 0.5 * cw));
                        double ry0 = fmax(0.0, fmin(1.0 - ch, refV - 0.5 * ch));
                        collect_yolo_candidates(yoloModel, cgImg, wantAir, rx0, ry0, cw, ch, cands);
                    }
                }

                // 3) Saliency + 1,303-class ANE classifier fallback (helicopters, unusual angles, lit night boats)
                if (cands.count == 0) {
                    double refU = hasTrk ? trkU : expectU;
                    double refV = hasTrk ? trkV : expectV;
                    collect_saliency_classified_candidates(cgImg, wantAir, refU, refV, cands);
                }

                CGImageRelease(cgImg);

                // 4) Associate best candidate with active track (or expected RF position on initial lock)
                NSDictionary *bestDet = nil;
                double bestScore = -1.0;
                for (NSDictionary *c in cands) {
                    double cu = [c[@"u"] doubleValue];
                    double cv = [c[@"v"] doubleValue];
                    double cc = [c[@"conf"] doubleValue];
                    if (hasTrk) {
                        double dTrk = hypot(cu - trkU, cv - trkV);
                        // Prevent jumping to a different passing boat/plane once locked
                        if (dTrk > 0.24) continue;
                        double sc = cc / (1.0 + 3.2 * dTrk);
                        if (sc > bestScore) { bestScore = sc; bestDet = c; }
                    } else {
                        double dExp = hypot(cu - expectU, cv - expectV);
                        if (dExp > 0.56) continue;
                        double sc = cc / (1.0 + 2.0 * dExp);
                        if (sc > bestScore) { bestScore = sc; bestDet = c; }
                    }
                }

                double elapsedMs = (CFAbsoluteTimeGetCurrent() - t0) * 1000.0;
                if (bestDet != nil) {
                    double du = [bestDet[@"u"] doubleValue];
                    double dv = [bestDet[@"v"] doubleValue];
                    double dw = [bestDet[@"w"] doubleValue];
                    double dh = [bestDet[@"h"] doubleValue];
                    double dc = [bestDet[@"conf"] doubleValue];
                    lastLabel = bestDet[@"label"] ?: (wantAir ? @"aircraft" : @"boat");
                    NSString *src = bestDet[@"source"] ?: @"yolo11s";

                    // Fuse centroid subtly with optical-flow tracker if already locked for sub-pixel smoothness
                    if (hasTrk && hypot(du - trkU, dv - trkV) < 0.08) {
                        du = 0.72 * du + 0.28 * trkU;
                        dv = 0.72 * dv + 0.28 * trkV;
                        dw = 0.65 * dw + 0.35 * trkW;
                        dh = 0.65 * dh + 0.35 * trkH;
                    }
                    trackedObs = [VNDetectedObjectObservation observationWithBoundingBox:uvwh_to_vision_rect(du, dv, dw, dh)];
                    coastFrames = 0;

                    printf(
                        "{\"ok\":true,\"found\":true,\"u\":%.5f,\"v\":%.5f,\"w\":%.5f,\"h\":%.5f,"
                        "\"conf\":%.4f,\"source\":\"%s\",\"label\":\"%s\",\"ms\":%.2f,\"candidates\":%lu}\n",
                        du, dv, dw, dh, dc, [src UTF8String], [lastLabel UTF8String], elapsedMs, (unsigned long)cands.count
                    );
                } else if (hasTrk && trkConf >= 0.56 && coastFrames < 2) {
                    coastFrames++;
                    printf(
                        "{\"ok\":true,\"found\":true,\"u\":%.5f,\"v\":%.5f,\"w\":%.5f,\"h\":%.5f,"
                        "\"conf\":%.4f,\"source\":\"vn_track_coast\",\"label\":\"%s\",\"ms\":%.2f,\"candidates\":0}\n",
                        trkU, trkV, trkW, trkH, trkConf, [lastLabel UTF8String], elapsedMs
                    );
                } else {
                    trackedObs = nil;
                    seqHandler = [[VNSequenceRequestHandler alloc] init];
                    coastFrames = 0;
                    printf("{\"ok\":true,\"found\":false,\"ms\":%.2f,\"candidates\":%lu}\n", elapsedMs, (unsigned long)cands.count);
                }
                fflush(stdout);
            }
        }
        if (lineBuf) free(lineBuf);
    }
    return 0;
}
