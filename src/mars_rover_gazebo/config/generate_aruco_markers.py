#!/usr/bin/env python3
"""
Generate ArUco marker PNG images for use as Gazebo textures.
Run once before launching the simulation:
  python3 generate_aruco_markers.py

Outputs:
  ../models/aruco_marker/marker_<id>.png  for IDs 0-4
"""
import os
import cv2
import numpy as np

ARUCO_DICT = cv2.aruco.DICT_4X4_50
MARKER_SIZE_PX = 400
BORDER_BITS = 1
OUTPUT_DIR = os.path.join(os.path.dirname(__file__), '..', 'models', 'aruco_marker')

os.makedirs(OUTPUT_DIR, exist_ok=True)

aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)

for marker_id in range(5):
    img = np.zeros((MARKER_SIZE_PX, MARKER_SIZE_PX), dtype=np.uint8)
    img = cv2.aruco.drawMarker(aruco_dict, marker_id, MARKER_SIZE_PX, img, BORDER_BITS)
    # Add white border
    bordered = cv2.copyMakeBorder(img, 20, 20, 20, 20,
                                   cv2.BORDER_CONSTANT, value=255)
    out_path = os.path.join(OUTPUT_DIR, f'marker_{marker_id}.png')
    cv2.imwrite(out_path, bordered)
    print(f'Wrote {out_path}')

print('Done. Place marker textures on the board models in mars_terrain.sdf.')
