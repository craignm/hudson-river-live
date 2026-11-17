#!/usr/bin/env python3
"""
Optional Apple Silicon (CoreML / MPS) YOLOv11 Tiled Spotter for Mac mini.
Slices the 4K Insta360 Link 2C frame into overlapping high-res tiles along the
Hudson River horizon & helicopter corridor so 15-pixel distant boats/aircraft
are detected reliably, then posts bounding boxes to server.py (/api/cv_detections).

Setup (in a virtualenv when ready):
  python3 -m venv .venv && source .venv/bin/activate
  pip install ultralytics opencv-python
  python vision_yolo_mac.py --camera 0
"""

import argparse
import json
import time
import urllib.request


def post_detections(server_url: str, detections: list):
    body = json.dumps({"detections": detections}).encode("utf-8")
    req = urllib.request.Request(
        f"{server_url}/api/cv_detections",
        data=body,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    try:
        urllib.request.urlopen(req, timeout=0.5).read()
    except Exception:
        pass


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--camera", type=int, default=0, help="OpenCV camera index for Link 2C or OBS Virtual Cam")
    parser.add_argument("--server", type=str, default="http://127.0.0.1:8765")
    parser.add_argument("--model", type=str, default="yolo11s.pt")
    args = parser.parse_args()

    import cv2
    from ultralytics import YOLO

    # COCO classes of interest: 4 = airplane (includes helicopters), 8 = boat
    TARGET_CLASSES = {4: "AIRCRAFT", 8: "BOAT"}

    model = YOLO(args.model)
    cap = cv2.VideoCapture(args.camera)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 3840)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 2160)

    print(f"Running Apple Silicon MPS YOLOv11 tiled spotter -> {args.server}")
    prev_centers = {}

    while True:
        ok, frame = cap.read()
        if not ok:
            time.sleep(0.1)
            continue

        h, w = frame.shape[:2]
        now = time.time()

        # Horizontal band covering sky corridor + Hudson River surface (22%..84% height)
        y0, y1 = int(h * 0.20), int(h * 0.84)
        band = frame[y0:y1, :]

        # Split 4K width into 3 overlapping tiles so tiny aircraft/boats stay crisp at 640/960px
        tile_w = int(w * 0.42)
        strides = [0, int(w * 0.29), w - tile_w]
        tiles = [band[:, sx : sx + tile_w] for sx in strides]

        results = model.predict(tiles, imgsz=960, conf=0.24, classes=[4, 8], device="mps", verbose=False)

        raw_boxes = []
        for sx, res in zip(strides, results):
            for box in res.boxes:
                cls_id = int(box.cls[0])
                x1, by1, x2, by2 = box.xyxy[0].tolist()
                cx = (sx + 0.5 * (x1 + x2)) / w
                cy = (y0 + 0.5 * (by1 + by2)) / h
                bw = (x2 - x1) / w
                bh = (by2 - by1) / h
                raw_boxes.append((cx, cy, bw, bh, TARGET_CLASSES.get(cls_id, "OBJECT")))

        # Simple NMS + velocity tracking across tile overlaps
        merged = []
        for b in raw_boxes:
            if not any(abs(b[0] - m[0]) < 0.02 and abs(b[1] - m[1]) < 0.02 for m in merged):
                merged.append(b)

        detections = []
        new_prev = {}
        for idx, (cx, cy, bw, bh, label) in enumerate(merged[:12]):
            best_id, best_d = None, 0.04
            for pid, (px, py, pts) in prev_centers.items():
                d = ((cx - px) ** 2 + (cy - py) ** 2) ** 0.5
                if d < best_d:
                    best_d, best_id = d, pid
            tid = best_id if best_id is not None else f"yolo_{int(now * 100) % 10000}_{idx}"
            du_s, dv_s = 0.0, 0.0
            if best_id is not None:
                px, py, pts = prev_centers[best_id]
                dt = max(0.03, now - pts)
                du_s = (cx - px) / dt
                dv_s = (cy - py) / dt
            new_prev[tid] = (cx, cy, now)
            detections.append({
                "id": tid,
                "u": cx,
                "v": cy,
                "w": bw,
                "h": bh,
                "du_s": du_s,
                "dv_s": dv_s,
                "region": "sky" if label == "AIRCRAFT" else "water",
                "label": label,
            })

        prev_centers = new_prev
        post_detections(args.server, detections)
        time.sleep(0.12)


if __name__ == "__main__":
    main()
