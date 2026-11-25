//
//  ptz_vision.m
//  Fast Zero-Dependency macOS Vision Engine for Automated Synoptic <-> 30x PTZ Calibration
//
//  Uses Apple built-in frameworks only (Foundation, CoreGraphics, ImageIO, Accelerate/vDSP)
//  so Craigs-Mac-Mini needs zero pip packages (no numpy/opencv/pillow).
//
//  Subcommands (all output single-line JSON on stdout):
//    1. stats <img_path>
//       -> Luminance distribution, highlight clipping, and center-ROI Tenengrad focus score.
//    2. pick-anchor <syn1_path> <syn2_path> <u0> <v0> <du> <dv> <patch_deg> <hfov_deg> <k1>
//       -> Finds the strongest stationary 2D Shi-Tomasi architectural corner near (u0, v0),
//          rejecting moving water/vessels/clouds via temporal frame-diff |syn2 - syn1|.
//    3. match-syn <syn_path> <ptz_path> <pan_syn_deg> <tilt_syn_deg> <syn_hfov_deg> <syn_k1>
//                 <ptz_hfov_est_deg> <tpl_deg>
//       -> Rectifies the 4K Synoptic patch around (pan_syn, tilt_syn) (undoing radial k1 barrel
//          distortion) and runs 5-scale 3-channel (HighPass-L, Sobel-Gx, Sobel-Gy) ZNCC via vDSP
//          against the PTZ frame with sub-pixel parabolic peak fitting & sidelobe PSR confidence.
//    4. match-zoom <wide_ptz_path> <tight_ptz_path> <anchor_u> <anchor_v> <zoom_ratio_est>
//                  <tight_hfov_est_deg>
//       -> Tracks an exact sub-pixel landmark (anchor_u, anchor_v) from an intermediate PTZ zoom
//          frame (e.g. 10x-12x) into a tight 25x-30x PTZ frame across a 5-scale pyramid.
//
//  Build universal macOS binary:
//    clang -O3 -Wall -Wno-deprecated-declarations -arch arm64 -arch x86_64 \
//      -framework Foundation -framework CoreGraphics -framework ImageIO -framework Accelerate \
//      ptz_vision.m -o ptz-vision && codesign -s - --force ptz-vision
//

#import <Foundation/Foundation.h>
#import <CoreGraphics/CoreGraphics.h>
#import <ImageIO/ImageIO.h>
#import <Accelerate/Accelerate.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
    int width;
    int height;
    float *lum; // [width * height] in 0..255
} gray_img_t;

static void free_gray(gray_img_t *img) {
    if (img && img->lum) {
        free(img->lum);
        img->lum = NULL;
    }
}

static BOOL load_gray_image(const char *path, gray_img_t *outImg) {
    memset(outImg, 0, sizeof(*outImg));
    NSString *nsPath = [NSString stringWithUTF8String:path];
    if (!nsPath) return NO;
    NSURL *url = [NSURL fileURLWithPath:nsPath];
    CGImageSourceRef src = CGImageSourceCreateWithURL((__bridge CFURLRef)url, NULL);
    if (!src) return NO;
    CGImageRef cg = CGImageSourceCreateImageAtIndex(src, 0, NULL);
    CFRelease(src);
    if (!cg) return NO;

    size_t w = CGImageGetWidth(cg);
    size_t h = CGImageGetHeight(cg);
    if (w < 16 || h < 16 || w > 8192 || h > 8192) {
        CGImageRelease(cg);
        return NO;
    }

    uint8_t *rgba = (uint8_t *)calloc(w * h * 4, 1);
    if (!rgba) {
        CGImageRelease(cg);
        return NO;
    }
    CGColorSpaceRef cs = CGColorSpaceCreateDeviceRGB();
    CGContextRef ctx = CGBitmapContextCreate(
        rgba, w, h, 8, w * 4, cs,
        kCGImageAlphaPremultipliedLast | kCGBitmapByteOrder32Big
    );
    CGColorSpaceRelease(cs);
    if (!ctx) {
        free(rgba);
        CGImageRelease(cg);
        return NO;
    }
    CGContextDrawImage(ctx, CGRectMake(0, 0, (CGFloat)w, (CGFloat)h), cg);
    CGContextRelease(ctx);
    CGImageRelease(cg);

    float *lum = (float *)malloc(sizeof(float) * w * h);
    if (!lum) {
        free(rgba);
        return NO;
    }
    const size_t n = w * h;
    for (size_t i = 0; i < n; i++) {
        const uint8_t *p = &rgba[i * 4];
        lum[i] = 0.299f * (float)p[0] + 0.587f * (float)p[1] + 0.114f * (float)p[2];
    }
    free(rgba);
    outImg->width = (int)w;
    outImg->height = (int)h;
    outImg->lum = lum;
    return YES;
}

static inline float sample_bilinear(const gray_img_t *img, float x, float y) {
    if (x < 0.0f) x = 0.0f;
    if (y < 0.0f) y = 0.0f;
    float maxX = (float)(img->width - 1);
    float maxY = (float)(img->height - 1);
    if (x > maxX) x = maxX;
    if (y > maxY) y = maxY;
    int x0 = (int)floorf(x);
    int y0 = (int)floorf(y);
    int x1 = (x0 + 1 < img->width) ? (x0 + 1) : x0;
    int y1 = (y0 + 1 < img->height) ? (y0 + 1) : y0;
    float fx = x - (float)x0;
    float fy = y - (float)y0;
    const float *row0 = &img->lum[y0 * img->width];
    const float *row1 = &img->lum[y1 * img->width];
    float v0 = row0[x0] * (1.0f - fx) + row0[x1] * fx;
    float v1 = row1[x0] * (1.0f - fx) + row1[x1] * fx;
    return v0 * (1.0f - fy) + v1 * fy;
}

// Area-averaging downsample for clean anti-aliased pyramid matching
static BOOL resize_gray_box(const gray_img_t *src, int dstW, int dstH, gray_img_t *dst) {
    memset(dst, 0, sizeof(*dst));
    float *out = (float *)malloc(sizeof(float) * (size_t)dstW * (size_t)dstH);
    if (!out) return NO;
    float sx = (float)src->width / (float)dstW;
    float sy = (float)src->height / (float)dstH;
    if (sx <= 1.15f && sy <= 1.15f) {
        for (int y = 0; y < dstH; y++) {
            float fy = ((float)y + 0.5f) * sy - 0.5f;
            for (int x = 0; x < dstW; x++) {
                float fx = ((float)x + 0.5f) * sx - 0.5f;
                out[y * dstW + x] = sample_bilinear(src, fx, fy);
            }
        }
    } else {
        for (int y = 0; y < dstH; y++) {
            float yStart = (float)y * sy;
            float yEnd = (float)(y + 1) * sy;
            int iy0 = (int)floorf(yStart);
            int iy1 = (int)ceilf(yEnd);
            if (iy0 < 0) iy0 = 0;
            if (iy1 > src->height) iy1 = src->height;
            for (int x = 0; x < dstW; x++) {
                float xStart = (float)x * sx;
                float xEnd = (float)(x + 1) * sx;
                int ix0 = (int)floorf(xStart);
                int ix1 = (int)ceilf(xEnd);
                if (ix0 < 0) ix0 = 0;
                if (ix1 > src->width) ix1 = src->width;
                float sum = 0.0f;
                int cnt = 0;
                for (int yy = iy0; yy < iy1; yy++) {
                    const float *row = &src->lum[yy * src->width];
                    for (int xx = ix0; xx < ix1; xx++) {
                        sum += row[xx];
                        cnt++;
                    }
                }
                out[y * dstW + x] = (cnt > 0) ? (sum / (float)cnt) : 0.0f;
            }
        }
    }
    dst->width = dstW;
    dst->height = dstH;
    dst->lum = out;
    return YES;
}

// Fast integral-image box blur to extract illumination-invariant local high-pass luminance
static void compute_feature_planes(const float *lum, int w, int h, float *hpOut, float *gxOut, float *gyOut) {
    double *integ = (double *)calloc((size_t)(w + 1) * (size_t)(h + 1), sizeof(double));
    for (int y = 0; y < h; y++) {
        double rowSum = 0.0;
        for (int x = 0; x < w; x++) {
            rowSum += (double)lum[y * w + x];
            integ[(y + 1) * (w + 1) + (x + 1)] = integ[y * (w + 1) + (x + 1)] + rowSum;
        }
    }
    const int rad = 7;
    for (int y = 0; y < h; y++) {
        int y0 = (y - rad < 0) ? 0 : (y - rad);
        int y1 = (y + rad + 1 > h) ? h : (y + rad + 1);
        for (int x = 0; x < w; x++) {
            int x0 = (x - rad < 0) ? 0 : (x - rad);
            int x1 = (x + rad + 1 > w) ? w : (x + rad + 1);
            double area = (double)((y1 - y0) * (x1 - x0));
            double sum = integ[y1 * (w + 1) + x1] - integ[y0 * (w + 1) + x1]
                       - integ[y1 * (w + 1) + x0] + integ[y0 * (w + 1) + x0];
            hpOut[y * w + x] = lum[y * w + x] - (float)(sum / area);
        }
    }
    free(integ);

    memset(gxOut, 0, sizeof(float) * (size_t)w * (size_t)h);
    memset(gyOut, 0, sizeof(float) * (size_t)w * (size_t)h);
    for (int y = 1; y < h - 1; y++) {
        for (int x = 1; x < w - 1; x++) {
            float tl = lum[(y - 1) * w + (x - 1)];
            float tc = lum[(y - 1) * w + x];
            float tr = lum[(y - 1) * w + (x + 1)];
            float ml = lum[y * w + (x - 1)];
            float mr = lum[y * w + (x + 1)];
            float bl = lum[(y + 1) * w + (x - 1)];
            float bc = lum[(y + 1) * w + x];
            float br = lum[(y + 1) * w + (x + 1)];
            gxOut[y * w + x] = (tr + 2.0f * mr + br) - (tl + 2.0f * ml + bl);
            gyOut[y * w + x] = (bl + 2.0f * bc + br) - (tl + 2.0f * tc + tr);
        }
    }
}

static void normalize_vec_zero_mean_unit(float *v, int n) {
    float mean = 0.0f;
    vDSP_meanv(v, 1, &mean, (vDSP_Length)n);
    float negMean = -mean;
    vDSP_vsadd(v, 1, &negMean, v, 1, (vDSP_Length)n);
    float dot = 0.0f;
    vDSP_dotpr(v, 1, v, 1, &dot, (vDSP_Length)n);
    float invNorm = (dot > 1e-6f) ? (1.0f / sqrtf(dot)) : 0.0f;
    vDSP_vsmul(v, 1, &invNorm, v, 1, (vDSP_Length)n);
}

static void normalize_grad_pair(float *gx, float *gy, int n) {
    float d1 = 0.0f, d2 = 0.0f;
    vDSP_dotpr(gx, 1, gx, 1, &d1, (vDSP_Length)n);
    vDSP_dotpr(gy, 1, gy, 1, &d2, (vDSP_Length)n);
    float tot = d1 + d2;
    float invNorm = (tot > 1e-6f) ? (1.0f / sqrtf(tot)) : 0.0f;
    vDSP_vsmul(gx, 1, &invNorm, gx, 1, (vDSP_Length)n);
    vDSP_vsmul(gy, 1, &invNorm, gy, 1, (vDSP_Length)n);
}

typedef struct {
    BOOL ok;
    float bestX;      // Sub-pixel center X in search image [0..searchW]
    float bestY;      // Sub-pixel center Y in search image [0..searchH]
    float bestScale;
    float score;      // Combined ZNCC in [-1..+1]
    float secondScore;
    float margin;
    float psr;        // Peak-to-sidelobe ratio (Z-score)
} match_result_t;

// Evaluates 3-channel (0.35 * HighPassL + 0.65 * SignedSobelGrad) ZNCC across search image
static void match_template_single_scale(
    const float *sHp, const float *sGx, const float *sGy, int sW, int sH,
    const float *tLum, int tW, int tH,
    float scaleTag,
    float *scoreMapOut, // [(sW - tW + 1) * (sH - tH + 1)]
    match_result_t *outRes
) {
    memset(outRes, 0, sizeof(*outRes));
    if (tW < 12 || tH < 12 || tW >= sW - 4 || tH >= sH - 4) return;

    const int nT = tW * tH;
    float *tHp = (float *)malloc(sizeof(float) * (size_t)nT);
    float *tGx = (float *)malloc(sizeof(float) * (size_t)nT);
    float *tGy = (float *)malloc(sizeof(float) * (size_t)nT);
    compute_feature_planes(tLum, tW, tH, tHp, tGx, tGy);
    normalize_vec_zero_mean_unit(tHp, nT);
    normalize_grad_pair(tGx, tGy, nT);

    // Integral images of sHp, sHp^2, and (sGx^2 + sGy^2) for O(1) window normalization!
    const int iW = sW + 1;
    const int iH = sH + 1;
    double *intHp  = (double *)calloc((size_t)iW * (size_t)iH, sizeof(double));
    double *intHp2 = (double *)calloc((size_t)iW * (size_t)iH, sizeof(double));
    double *intG2  = (double *)calloc((size_t)iW * (size_t)iH, sizeof(double));
    for (int y = 0; y < sH; y++) {
        double rHp = 0.0, rHp2 = 0.0, rG2 = 0.0;
        for (int x = 0; x < sW; x++) {
            int idx = y * sW + x;
            double hp = (double)sHp[idx];
            double g2 = (double)sGx[idx] * (double)sGx[idx] + (double)sGy[idx] * (double)sGy[idx];
            rHp  += hp;
            rHp2 += hp * hp;
            rG2  += g2;
            int ii = (y + 1) * iW + (x + 1);
            intHp[ii]  = intHp[y * iW + (x + 1)]  + rHp;
            intHp2[ii] = intHp2[y * iW + (x + 1)] + rHp2;
            intG2[ii]  = intG2[y * iW + (x + 1)]  + rG2;
        }
    }

    const int outW = sW - tW + 1;
    const int outH = sH - tH + 1;
    float bestScore = -2.0f;
    int bestIx = 0, bestIy = 0;
    double sumScores = 0.0, sumScores2 = 0.0;
    int validCnt = 0;

    for (int oy = 0; oy < outH; oy++) {
        int y0 = oy, y1 = oy + tH;
        for (int ox = 0; ox < outW; ox++) {
            int x0 = ox, x1 = ox + tW;
            double sumH = intHp[y1 * iW + x1] - intHp[y0 * iW + x1] - intHp[y1 * iW + x0] + intHp[y0 * iW + x0];
            double sumH2 = intHp2[y1 * iW + x1] - intHp2[y0 * iW + x1] - intHp2[y1 * iW + x0] + intHp2[y0 * iW + x0];
            double varH = sumH2 - (sumH * sumH) / (double)nT;
            double sumGrad2 = intG2[y1 * iW + x1] - intG2[y0 * iW + x1] - intG2[y1 * iW + x0] + intG2[y0 * iW + x0];

            if (varH < 25.0 || sumGrad2 < 200.0) {
                scoreMapOut[oy * outW + ox] = -1.0f;
                continue;
            }

            float dotHp = 0.0f, dotGx = 0.0f, dotGy = 0.0f;
            for (int r = 0; r < tH; r++) {
                const int sRow = (oy + r) * sW + ox;
                const int tRow = r * tW;
                float dH = 0.0f, dX = 0.0f, dY = 0.0f;
                vDSP_dotpr(&sHp[sRow], 1, &tHp[tRow], 1, &dH, (vDSP_Length)tW);
                vDSP_dotpr(&sGx[sRow], 1, &tGx[tRow], 1, &dX, (vDSP_Length)tW);
                vDSP_dotpr(&sGy[sRow], 1, &tGy[tRow], 1, &dY, (vDSP_Length)tW);
                dotHp += dH;
                dotGx += dX;
                dotGy += dY;
            }
            // Since tHp has zero mean, dot(sHp - mean(sHp), tHp) == dot(sHp, tHp)!
            float nccHp = dotHp / (float)sqrt(varH);
            float nccGrad = (dotGx + dotGy) / (float)sqrt(sumGrad2);
            float fused = 0.35f * nccHp + 0.65f * nccGrad;
            scoreMapOut[oy * outW + ox] = fused;

            sumScores += (double)fused;
            sumScores2 += (double)fused * (double)fused;
            validCnt++;

            if (fused > bestScore) {
                bestScore = fused;
                bestIx = ox;
                bestIy = oy;
            }
        }
    }

    free(intHp);
    free(intHp2);
    free(intG2);
    free(tHp);
    free(tGx);
    free(tGy);

    if (validCnt < 16 || bestScore <= -0.5f) return;

    // Parabolic sub-pixel peak refinement
    float subDx = 0.0f, subDy = 0.0f;
    if (bestIx > 0 && bestIx < outW - 1) {
        float sL = scoreMapOut[bestIy * outW + (bestIx - 1)];
        float sC = scoreMapOut[bestIy * outW + bestIx];
        float sR = scoreMapOut[bestIy * outW + (bestIx + 1)];
        float denom = 2.0f * (sL - 2.0f * sC + sR);
        if (fabsf(denom) > 1e-5f && sL > -0.5f && sR > -0.5f) {
            float d = (sL - sR) / denom;
            if (d > -0.6f && d < 0.6f) subDx = d;
        }
    }
    if (bestIy > 0 && bestIy < outH - 1) {
        float sU = scoreMapOut[(bestIy - 1) * outW + bestIx];
        float sC = scoreMapOut[bestIy * outW + bestIx];
        float sD = scoreMapOut[(bestIy + 1) * outW + bestIx];
        float denom = 2.0f * (sU - 2.0f * sC + sD);
        if (fabsf(denom) > 1e-5f && sU > -0.5f && sD > -0.5f) {
            float d = (sU - sD) / denom;
            if (d > -0.6f && d < 0.6f) subDy = d;
        }
    }

    // Best non-overlapping secondary sidelobe outside exclusion radius
    int exclR = tW / 3;
    if (exclR < 12) exclR = 12;
    int exclR2 = exclR * exclR;
    float secondBest = -1.0f;
    for (int oy = 0; oy < outH; oy++) {
        int dy = oy - bestIy;
        for (int ox = 0; ox < outW; ox++) {
            int dx = ox - bestIx;
            if (dx * dx + dy * dy <= exclR2) continue;
            float sc = scoreMapOut[oy * outW + ox];
            if (sc > secondBest) secondBest = sc;
        }
    }

    double meanS = sumScores / (double)validCnt;
    double varS = (sumScores2 / (double)validCnt) - meanS * meanS;
    float stdS = (varS > 1e-8) ? (float)sqrt(varS) : 1e-4f;

    outRes->ok = YES;
    outRes->bestX = (float)bestIx + subDx + 0.5f * (float)tW;
    outRes->bestY = (float)bestIy + subDy + 0.5f * (float)tH;
    outRes->bestScale = scaleTag;
    outRes->score = bestScore;
    outRes->secondScore = secondBest;
    outRes->margin = (secondBest > -0.9f) ? (bestScore - secondBest) : bestScore;
    outRes->psr = (bestScore - (float)meanS) / stdS;
}

static void uv_to_pan_tilt(double u, double v, double hfovDeg, double k1, double *outPanDeg, double *outTiltDeg) {
    double halfTanX = tan((hfovDeg * 0.5) * M_PI / 180.0);
    double halfTanY = halfTanX * (9.0 / 16.0);
    double xd = (u - 0.5) * 2.0 * halfTanX;
    double yd = (0.5 - v) * 2.0 * halfTanY;
    double xn = xd, yn = yd;
    for (int i = 0; i < 4; i++) {
        double f = 1.0 + k1 * (xn * xn + yn * yn);
        if (f < 0.5) f = 0.5;
        xn = xd / f;
        yn = yd / f;
    }
    *outPanDeg = atan(xn) * 180.0 / M_PI;
    *outTiltDeg = atan2(yn, sqrt(1.0 + xn * xn)) * 180.0 / M_PI;
}

static void ray_to_syn_uv(double xc, double yc, double zc, double hfovDeg, double k1, double *outU, double *outV) {
    double halfTanX = tan((hfovDeg * 0.5) * M_PI / 180.0);
    double halfTanY = halfTanX * (9.0 / 16.0);
    double xn = xc / zc;
    double yn = yc / zc;
    double r2 = xn * xn + yn * yn;
    double df = 1.0 + k1 * r2;
    *outU = 0.5 + 0.5 * (xn * df) / halfTanX;
    *outV = 0.5 - 0.5 * (yn * df) / halfTanY;
}

static int cmd_stats(int argc, char * const argv[]) {
    if (argc < 3) {
        printf("{\"ok\":false,\"error\":\"usage: stats <img_path>\"}\n");
        return 2;
    }
    gray_img_t img;
    if (!load_gray_image(argv[2], &img)) {
        printf("{\"ok\":false,\"error\":\"failed_to_load\"}\n");
        return 1;
    }
    const int n = img.width * img.height;
    int hist[256] = {0};
    double sum = 0.0, sum2 = 0.0;
    int clipped = 0;
    for (int i = 0; i < n; i++) {
        float v = img.lum[i];
        sum += (double)v;
        sum2 += (double)v * (double)v;
        int bin = (int)lrintf(v);
        if (bin < 0) bin = 0;
        if (bin > 255) bin = 255;
        hist[bin]++;
        if (v >= 250.0f) clipped++;
    }
    double mean = sum / (double)n;
    double std = sqrt(fmax(0.0, sum2 / (double)n - mean * mean));

    int p10 = 0, p50 = 0, p90 = 0, acc = 0;
    for (int b = 0; b < 256; b++) {
        acc += hist[b];
        if (acc >= (int)(0.10 * n) && p10 == 0 && b > 0) p10 = b;
        if (acc >= (int)(0.50 * n) && p50 == 0 && b > 0) p50 = b;
        if (acc >= (int)(0.90 * n) && p90 == 0 && b > 0) p90 = b;
    }

    // Center ROI Tenengrad focus metric (normalized by ROI mean luminance^2)
    // Optional args: stats <img_path> [roi_u roi_v roi_half_span]
    double roiU = (argc >= 6) ? atof(argv[3]) : 0.50;
    double roiV = (argc >= 6) ? atof(argv[4]) : 0.50;
    double roiHalf = (argc >= 6) ? atof(argv[5]) : 0.24;
    if (roiHalf < 0.08) roiHalf = 0.08;
    if (roiHalf > 0.45) roiHalf = 0.45;
    int x0 = (int)(img.width * (roiU - roiHalf));
    int x1 = (int)(img.width * (roiU + roiHalf));
    int y0 = (int)(img.height * (roiV - roiHalf));
    int y1 = (int)(img.height * (roiV + roiHalf));
    if (x0 < 1) x0 = 1;
    if (y0 < 1) y0 = 1;
    if (x1 > img.width - 1) x1 = img.width - 1;
    if (y1 > img.height - 1) y1 = img.height - 1;

    double roiLumSum = 0.0, tenSum = 0.0, gradMagSum = 0.0;
    int roiCnt = 0;
    for (int y = y0; y < y1; y++) {
        for (int x = x0; x < x1; x++) {
            float tl = img.lum[(y - 1) * img.width + (x - 1)];
            float tc = img.lum[(y - 1) * img.width + x];
            float tr = img.lum[(y - 1) * img.width + (x + 1)];
            float ml = img.lum[y * img.width + (x - 1)];
            float mc = img.lum[y * img.width + x];
            float mr = img.lum[y * img.width + (x + 1)];
            float bl = img.lum[(y + 1) * img.width + (x - 1)];
            float bc = img.lum[(y + 1) * img.width + x];
            float br = img.lum[(y + 1) * img.width + (x + 1)];
            double gx = (double)((tr + 2.0f * mr + br) - (tl + 2.0f * ml + bl));
            double gy = (double)((bl + 2.0f * bc + br) - (tl + 2.0f * tc + tr));
            double g2 = gx * gx + gy * gy;
            tenSum += g2;
            gradMagSum += sqrt(g2);
            roiLumSum += (double)mc;
            roiCnt++;
        }
    }
    double roiMean = (roiCnt > 0) ? (roiLumSum / (double)roiCnt) : 1.0;
    double normDenom = (roiMean + 20.0) * (roiMean + 20.0);
    double tenengrad = (roiCnt > 0) ? ((tenSum / (double)roiCnt) / normDenom * 100.0) : 0.0;
    double gradMean = (roiCnt > 0) ? (gradMagSum / (double)roiCnt) : 0.0;

    printf("{\"ok\":true,\"width\":%d,\"height\":%d,\"mean\":%.2f,\"std\":%.2f,"
           "\"p10\":%d,\"p50\":%d,\"p90\":%d,\"clipped_frac\":%.4f,"
           "\"roi_mean\":%.2f,\"tenengrad\":%.4f,\"grad_mean\":%.3f}\n",
           img.width, img.height, mean, std, p10, p50, p90,
           (double)clipped / (double)n, roiMean, tenengrad, gradMean);
    free_gray(&img);
    return 0;
}

static int cmd_pick_anchor(int argc, char * const argv[]) {
    if (argc < 11) {
        printf("{\"ok\":false,\"error\":\"usage: pick-anchor <syn1> <syn2> <u0> <v0> <du> <dv> <patch_deg> <hfov_deg> <k1>\"}\n");
        return 2;
    }
    gray_img_t im1, im2;
    if (!load_gray_image(argv[2], &im1)) {
        printf("{\"ok\":false,\"error\":\"load_syn1_failed\"}\n");
        return 1;
    }
    if (!load_gray_image(argv[3], &im2) || im2.width != im1.width || im2.height != im1.height) {
        free_gray(&im1);
        free_gray(&im2);
        printf("{\"ok\":false,\"error\":\"load_syn2_failed\"}\n");
        return 1;
    }

    double u0 = atof(argv[4]);
    double v0 = atof(argv[5]);
    double du = atof(argv[6]);
    double dv = atof(argv[7]);
    double patchDeg = atof(argv[8]);
    double hfovDeg = atof(argv[9]);
    double k1 = atof(argv[10]);

    int W = im1.width, H = im1.height;
    int halfPatchPx = (int)lrint(0.5 * (patchDeg / hfovDeg) * (double)W);
    if (halfPatchPx < 18) halfPatchPx = 18;
    int coreRad = (int)lrint(halfPatchPx * 0.55);
    if (coreRad < 10) coreRad = 10;

    int xMin = (int)lrint((u0 - du) * W);
    int xMax = (int)lrint((u0 + du) * W);
    int yMin = (int)lrint((v0 - dv) * H);
    int yMax = (int)lrint((v0 + dv) * H);
    if (xMin < halfPatchPx + 2) xMin = halfPatchPx + 2;
    if (yMin < halfPatchPx + 2) yMin = halfPatchPx + 2;
    if (xMax > W - halfPatchPx - 3) xMax = W - halfPatchPx - 3;
    if (yMax > H - halfPatchPx - 3) yMax = H - halfPatchPx - 3;

    int stride = (W >= 2560) ? 8 : 4;
    double bestMetric = -1.0;
    int bestCx = -1, bestCy = -1;
    double bestMotion = 0.0, bestStd = 0.0, bestLambda = 0.0;

    for (int cy = yMin; cy <= yMax; cy += stride) {
        for (int cx = xMin; cx <= xMax; cx += stride) {
            double diffSum = 0.0, lumSum = 0.0, lumSum2 = 0.0;
            int clippedCnt = 0, pCnt = 0;
            for (int y = cy - halfPatchPx; y <= cy + halfPatchPx; y += 2) {
                for (int x = cx - halfPatchPx; x <= cx + halfPatchPx; x += 2) {
                    int idx = y * W + x;
                    float v1 = im1.lum[idx];
                    float v2 = im2.lum[idx];
                    diffSum += fabs((double)v1 - (double)v2);
                    lumSum += (double)v1;
                    lumSum2 += (double)v1 * (double)v1;
                    if (v1 >= 250.0f) clippedCnt++;
                    pCnt++;
                }
            }
            if (pCnt < 16) continue;
            double mad = diffSum / (double)pCnt;
            if (mad > 3.8) continue; // Reject moving water/vessels/clouds
            double mean = lumSum / (double)pCnt;
            double std = sqrt(fmax(0.0, lumSum2 / (double)pCnt - mean * mean));
            if (std < 11.0 || (double)clippedCnt / (double)pCnt > 0.35) continue;

            // Compute 2D structure tensor Shi-Tomasi minimum eigenvalue over core region
            double sxx = 0.0, syy = 0.0, sxy = 0.0;
            int cCnt = 0;
            for (int y = cy - coreRad; y <= cy + coreRad; y++) {
                for (int x = cx - coreRad; x <= cx + coreRad; x++) {
                    float gx = (im1.lum[y * W + (x + 1)] - im1.lum[y * W + (x - 1)]);
                    float gy = (im1.lum[(y + 1) * W + x] - im1.lum[(y - 1) * W + x]);
                    sxx += (double)gx * (double)gx;
                    syy += (double)gy * (double)gy;
                    sxy += (double)gx * (double)gy;
                    cCnt++;
                }
            }
            sxx /= (double)cCnt;
            syy /= (double)cCnt;
            sxy /= (double)cCnt;
            double disc = sqrt((sxx - syy) * (sxx - syy) + 4.0 * sxy * sxy);
            double lambdaMin = 0.5 * (sxx + syy - disc);
            if (lambdaMin < 18.0) continue;

            // Mild Gaussian preference toward nominal landmark seed (u0, v0)
            double nu = ((double)cx / (double)W - u0) / fmax(0.015, du);
            double nv = ((double)cy / (double)H - v0) / fmax(0.015, dv);
            double prox = exp(-0.20 * (nu * nu + nv * nv));
            double metric = lambdaMin * prox;
            if (metric > bestMetric) {
                bestMetric = metric;
                bestCx = cx;
                bestCy = cy;
                bestMotion = mad;
                bestStd = std;
                bestLambda = lambdaMin;
            }
        }
    }

    free_gray(&im1);
    free_gray(&im2);

    if (bestCx < 0) {
        printf("{\"ok\":false,\"error\":\"no_static_2d_corner_found\"}\n");
        return 0;
    }

    double uBest = (double)bestCx / (double)W;
    double vBest = (double)bestCy / (double)H;
    double panDeg = 0.0, tiltDeg = 0.0;
    uv_to_pan_tilt(uBest, vBest, hfovDeg, k1, &panDeg, &tiltDeg);
    printf("{\"ok\":true,\"u\":%.5f,\"v\":%.5f,\"pan_deg\":%.4f,\"tilt_deg\":%.4f,"
           "\"corner_lambda\":%.2f,\"motion_mad\":%.3f,\"std\":%.2f}\n",
           uBest, vBest, panDeg, tiltDeg, bestLambda, bestMotion, bestStd);
    return 0;
}

static int cmd_match_syn(int argc, char * const argv[]) {
    if (argc < 10) {
        printf("{\"ok\":false,\"error\":\"usage: match-syn <syn> <ptz> <pan_syn> <tilt_syn> <syn_hfov> <syn_k1> <ptz_hfov_est> <tpl_deg>\"}\n");
        return 2;
    }
    gray_img_t synImg, ptzRaw;
    if (!load_gray_image(argv[2], &synImg)) {
        printf("{\"ok\":false,\"error\":\"load_syn_failed\"}\n");
        return 1;
    }
    if (!load_gray_image(argv[3], &ptzRaw)) {
        free_gray(&synImg);
        printf("{\"ok\":false,\"error\":\"load_ptz_failed\"}\n");
        return 1;
    }

    double panSynDeg = atof(argv[4]);
    double tiltSynDeg = atof(argv[5]);
    double synHfovDeg = atof(argv[6]);
    double synK1 = atof(argv[7]);
    double ptzHfovEstDeg = atof(argv[8]);
    double tplDeg = atof(argv[9]);

    const int sW = 480;
    const int sH = 270;
    gray_img_t ptzS;
    resize_gray_box(&ptzRaw, sW, sH, &ptzS);
    free_gray(&ptzRaw);

    float *sHp = (float *)malloc(sizeof(float) * sW * sH);
    float *sGx = (float *)malloc(sizeof(float) * sW * sH);
    float *sGy = (float *)malloc(sizeof(float) * sW * sH);
    compute_feature_planes(ptzS.lum, sW, sH, sHp, sGx, sGy);

    // Build orthonormal tangent basis around the Synoptic ray (panSynDeg, tiltSynDeg)
    double pRad = panSynDeg * M_PI / 180.0;
    double tRad = tiltSynDeg * M_PI / 180.0;
    // Camera frame: +X right, +Y up, +Z forward
    double fx = sin(pRad) * cos(tRad);
    double fy = sin(tRad);
    double fz = cos(pRad) * cos(tRad);
    // Horizontal right tangent vector (perpendicular to Y-up and forward ray)
    double rx = cos(pRad), ry = 0.0, rz = -sin(pRad);
    // Up tangent vector = f x r -> wait: in right-handed (+X right, +Y up, -Z or +Z forward):
    // Here +X right, +Y up, +Z forward (left-handed image depth): at pan=0,tilt=0, f=(0,0,1), r=(1,0,0), u=(0,1,0)
    double ux = -sin(pRad) * sin(tRad);
    double uy = cos(tRad);
    double uz = -cos(pRad) * sin(tRad);

    const float scales[7] = {0.82f, 0.88f, 0.94f, 1.00f, 1.06f, 1.13f, 1.20f};
    float *scoreScratch = (float *)malloc(sizeof(float) * sW * sH);
    match_result_t best = {0};
    best.score = -2.0f;

    for (int si = 0; si < 7; si++) {
        double effPtzHfov = ptzHfovEstDeg * (double)scales[si];
        double degPerPx = effPtzHfov / (double)sW;
        int tSide = (int)lrint(tplDeg / degPerPx);
        if (tSide % 2 == 1) tSide++;
        if (tSide < 24 || tSide > sH - 20) continue;

        float *tpl = (float *)malloc(sizeof(float) * (size_t)tSide * (size_t)tSide);
        for (int ty = 0; ty < tSide; ty++) {
            // Image +y is DOWN, camera +u is UP
            double angUpRad = ((0.5 * (double)(tSide - 1) - (double)ty) * degPerPx) * M_PI / 180.0;
            double tanU = tan(angUpRad);
            for (int tx = 0; tx < tSide; tx++) {
                double angRightRad = (((double)tx - 0.5 * (double)(tSide - 1)) * degPerPx) * M_PI / 180.0;
                double tanR = tan(angRightRad);
                double dx = fx + tanR * rx + tanU * ux;
                double dy = fy + tanR * ry + tanU * uy;
                double dz = fz + tanR * rz + tanU * uz;
                double su = 0.5, sv = 0.5;
                ray_to_syn_uv(dx, dy, dz, synHfovDeg, synK1, &su, &sv);
                tpl[ty * tSide + tx] = sample_bilinear(
                    &synImg,
                    (float)(su * (double)(synImg.width - 1)),
                    (float)(sv * (double)(synImg.height - 1))
                );
            }
        }

        match_result_t cur;
        match_template_single_scale(sHp, sGx, sGy, sW, sH, tpl, tSide, tSide, scales[si], scoreScratch, &cur);
        free(tpl);
        if (cur.ok && cur.score > best.score) {
            best = cur;
        }
    }

    free(scoreScratch);
    free(sHp);
    free(sGx);
    free(sGy);
    free_gray(&ptzS);
    free_gray(&synImg);

    if (!best.ok) {
        printf("{\"ok\":false,\"error\":\"match_failed\"}\n");
        return 0;
    }

    double effHfov = ptzHfovEstDeg * (double)best.bestScale;
    double effVfov = 2.0 * atan(tan((effHfov * 0.5) * M_PI / 180.0) * ((double)sH / (double)sW)) * 180.0 / M_PI;
    double normU = (double)best.bestX / (double)sW;
    double normV = (double)best.bestY / (double)sH;
    // Positive delta_pan = pan RIGHT to center; Positive delta_tilt = tilt UP to center
    double halfTanX = tan((effHfov * 0.5) * M_PI / 180.0);
    double halfTanY = tan((effVfov * 0.5) * M_PI / 180.0);
    double deltaPanDeg = atan((normU - 0.5) * 2.0 * halfTanX) * 180.0 / M_PI;
    double deltaTiltDeg = atan((0.5 - normV) * 2.0 * halfTanY) * 180.0 / M_PI;

    printf("{\"ok\":true,\"ptz_u\":%.5f,\"ptz_v\":%.5f,\"delta_pan_deg\":%.4f,\"delta_tilt_deg\":%.4f,"
           "\"eff_hfov_deg\":%.3f,\"best_scale\":%.3f,\"score\":%.4f,\"second_score\":%.4f,"
           "\"margin\":%.4f,\"psr\":%.2f}\n",
           normU, normV, deltaPanDeg, deltaTiltDeg, effHfov,
           best.bestScale, best.score, best.secondScore, best.margin, best.psr);
    return 0;
}

static int cmd_match_zoom(int argc, char * const argv[]) {
    if (argc < 8) {
        printf("{\"ok\":false,\"error\":\"usage: match-zoom <wide_ptz> <tight_ptz> <anchor_u> <anchor_v> <zoom_ratio_est> <tight_hfov_est_deg>\"}\n");
        return 2;
    }
    gray_img_t wideImg, tightRaw;
    if (!load_gray_image(argv[2], &wideImg)) {
        printf("{\"ok\":false,\"error\":\"load_wide_failed\"}\n");
        return 1;
    }
    if (!load_gray_image(argv[3], &tightRaw)) {
        free_gray(&wideImg);
        printf("{\"ok\":false,\"error\":\"load_tight_failed\"}\n");
        return 1;
    }

    double anchorU = atof(argv[4]);
    double anchorV = atof(argv[5]);
    double zoomRatioEst = atof(argv[6]);
    double tightHfovEstDeg = atof(argv[7]);

    const int sW = 480;
    const int sH = 270;
    gray_img_t tightS;
    resize_gray_box(&tightRaw, sW, sH, &tightS);
    free_gray(&tightRaw);

    float *sHp = (float *)malloc(sizeof(float) * sW * sH);
    float *sGx = (float *)malloc(sizeof(float) * sW * sH);
    float *sGy = (float *)malloc(sizeof(float) * sW * sH);
    compute_feature_planes(tightS.lum, sW, sH, sHp, sGx, sGy);

    const float scales[9] = {0.76f, 0.82f, 0.88f, 0.94f, 1.00f, 1.06f, 1.13f, 1.20f, 1.28f};
    const int tW = 160;
    const int tH = 120;
    float *tpl = (float *)malloc(sizeof(float) * tW * tH);
    float *scoreScratch = (float *)malloc(sizeof(float) * sW * sH);

    match_result_t best = {0};
    best.score = -2.0f;

    for (int si = 0; si < 9; si++) {
        double effRatio = zoomRatioEst * (double)scales[si];
        // One pixel in tightS (480x270) spans (1 / effRatio) of that normalized fraction in wideImg!
        double wideSpanU = ((double)tW / (double)sW) / effRatio;
        double wideSpanV = ((double)tH / (double)sH) / effRatio;
        for (int ty = 0; ty < tH; ty++) {
            double fv = anchorV + (((double)ty - 0.5 * (double)(tH - 1)) / (double)tH) * wideSpanV;
            for (int tx = 0; tx < tW; tx++) {
                double fu = anchorU + (((double)tx - 0.5 * (double)(tW - 1)) / (double)tW) * wideSpanU;
                tpl[ty * tW + tx] = sample_bilinear(
                    &wideImg,
                    (float)(fu * (double)(wideImg.width - 1)),
                    (float)(fv * (double)(wideImg.height - 1))
                );
            }
        }
        match_result_t cur;
        match_template_single_scale(sHp, sGx, sGy, sW, sH, tpl, tW, tH, scales[si], scoreScratch, &cur);
        if (cur.ok && cur.score > best.score) {
            best = cur;
        }
    }

    free(scoreScratch);
    free(tpl);
    free(sHp);
    free(sGx);
    free(sGy);
    free_gray(&tightS);
    free_gray(&wideImg);

    if (!best.ok) {
        printf("{\"ok\":false,\"error\":\"zoom_match_failed\"}\n");
        return 0;
    }

    double effHfov = tightHfovEstDeg / (double)best.bestScale;
    double effVfov = 2.0 * atan(tan((effHfov * 0.5) * M_PI / 180.0) * ((double)sH / (double)sW)) * 180.0 / M_PI;
    double normU = (double)best.bestX / (double)sW;
    double normV = (double)best.bestY / (double)sH;
    double halfTanX = tan((effHfov * 0.5) * M_PI / 180.0);
    double halfTanY = tan((effVfov * 0.5) * M_PI / 180.0);
    double deltaPanDeg = atan((normU - 0.5) * 2.0 * halfTanX) * 180.0 / M_PI;
    double deltaTiltDeg = atan((0.5 - normV) * 2.0 * halfTanY) * 180.0 / M_PI;

    printf("{\"ok\":true,\"ptz_u\":%.5f,\"ptz_v\":%.5f,\"delta_pan_deg\":%.4f,\"delta_tilt_deg\":%.4f,"
           "\"eff_hfov_deg\":%.3f,\"best_scale\":%.3f,\"score\":%.4f,\"second_score\":%.4f,"
           "\"margin\":%.4f,\"psr\":%.2f}\n",
           normU, normV, deltaPanDeg, deltaTiltDeg, effHfov,
           best.bestScale, best.score, best.secondScore, best.margin, best.psr);
    return 0;
}

int main(int argc, char * const argv[]) {
    @autoreleasepool {
        if (argc < 2) {
            printf("{\"ok\":false,\"error\":\"missing_subcommand\"}\n");
            return 2;
        }
        const char *cmd = argv[1];
        if (strcmp(cmd, "stats") == 0) return cmd_stats(argc, argv);
        if (strcmp(cmd, "pick-anchor") == 0) return cmd_pick_anchor(argc, argv);
        if (strcmp(cmd, "match-syn") == 0) return cmd_match_syn(argc, argv);
        if (strcmp(cmd, "match-zoom") == 0) return cmd_match_zoom(argc, argv);
        printf("{\"ok\":false,\"error\":\"unknown_subcommand\"}\n");
        return 2;
    }
}
