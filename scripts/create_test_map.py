#!/usr/bin/env python3
"""Create a simple office test map for NavSim."""

import numpy as np
from PIL import Image

def create_office_map():
    # 200x200 pixels = 10m x 10m at 5cm resolution
    width, height = 200, 200

    # Start with white (free space)
    img = np.ones((height, width), dtype=np.uint8) * 255

    # Add border walls (black = obstacle)
    wall_thickness = 3
    img[0:wall_thickness, :] = 0           # Top wall
    img[-wall_thickness:, :] = 0           # Bottom wall
    img[:, 0:wall_thickness] = 0           # Left wall
    img[:, -wall_thickness:] = 0           # Right wall

    # Add a single vertical wall in the middle with a gap
    # Wall from y=40 to y=80 and y=120 to y=160 (gap at y=80-120)
    img[40:80, 100:103] = 0    # Upper part of wall
    img[120:160, 100:103] = 0  # Lower part of wall

    # Create PIL image and save
    pil_img = Image.fromarray(img)
    pil_img.save('test_data/office.png')
    print("Created test_data/office.png (200x200, 10m x 10m)")
    print("Wall at x=5m with gap at y=4-6m")

if __name__ == "__main__":
    create_office_map()
