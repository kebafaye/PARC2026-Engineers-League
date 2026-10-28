#!/usr/bin/env python3
"""Render a tag36h11 marker image with a proper white quiet zone, for the local
AprilTag test fixture in test_assets/models/apriltag36h11_00000/.

Not part of the submitted solution - this only produces a texture file used by a
Gazebo test world, so the detection pipeline can be validated against something
before any official AprilTag task world exists.
"""
import sys

import cv2
import numpy as np

TAG_ID = 0
CELLS_PER_SIDE = 10  # tag36h11 markers are rendered on a 10x10 grid (8x8 payload + border)
PX_PER_CELL = 40
QUIET_ZONE_CELLS = 1  # white margin required around the tag for reliable detection

tag_px = CELLS_PER_SIDE * PX_PER_CELL
margin_px = QUIET_ZONE_CELLS * PX_PER_CELL
canvas_px = tag_px + 2 * margin_px

dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
tag = cv2.aruco.drawMarker(dictionary, TAG_ID, tag_px)

canvas = np.full((canvas_px, canvas_px), 255, dtype=np.uint8)
canvas[margin_px : margin_px + tag_px, margin_px : margin_px + tag_px] = tag

out_path = sys.argv[1] if len(sys.argv) > 1 else "tag36h11_00000.png"
cv2.imwrite(out_path, canvas)
print(f"Wrote {canvas_px}x{canvas_px} {out_path} (tag36h11 id={TAG_ID})")
