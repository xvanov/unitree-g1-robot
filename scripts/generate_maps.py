#!/usr/bin/env python3
"""Generate test maps for SLAM simulation."""

import numpy as np
import cv2
import os

def create_corridor_map(width=200, height=200):
    """Simple L-shaped corridor."""
    img = np.ones((height, width), dtype=np.uint8) * 255  # White = free

    # Outer walls (black = obstacle)
    img[0:10, :] = 0      # Top wall
    img[-10:, :] = 0      # Bottom wall
    img[:, 0:10] = 0      # Left wall
    img[:, -10:] = 0      # Right wall

    # Inner walls creating L-corridor
    img[10:120, 60:70] = 0    # Vertical wall
    img[110:120, 70:140] = 0  # Horizontal wall
    img[80:190, 130:140] = 0  # Another vertical wall

    return img

def create_rooms_map(width=200, height=200):
    """Four rooms with doorways."""
    img = np.ones((height, width), dtype=np.uint8) * 255

    # Outer walls
    img[0:8, :] = 0
    img[-8:, :] = 0
    img[:, 0:8] = 0
    img[:, -8:] = 0

    # Center cross walls
    img[96:104, 8:192] = 0   # Horizontal divider
    img[8:192, 96:104] = 0   # Vertical divider

    # Doorways (openings in walls)
    img[96:104, 40:60] = 255   # Top-left to top-right door
    img[96:104, 140:160] = 255 # Bottom-left to bottom-right door
    img[40:60, 96:104] = 255   # Top-left to bottom-left door
    img[140:160, 96:104] = 255 # Top-right to bottom-right door

    # Some furniture/obstacles in rooms
    img[30:50, 30:50] = 0      # Table in room 1
    img[140:160, 30:45] = 0    # Desk in room 2
    img[30:45, 150:170] = 0    # Cabinet in room 3
    img[150:170, 150:170] = 0  # Box in room 4

    return img

def create_maze_map(width=200, height=200):
    """Simple maze pattern."""
    img = np.ones((height, width), dtype=np.uint8) * 255

    # Outer walls
    img[0:8, :] = 0
    img[-8:, :] = 0
    img[:, 0:8] = 0
    img[:, -8:] = 0

    # Maze walls
    img[8:60, 50:58] = 0
    img[52:60, 50:120] = 0
    img[52:140, 112:120] = 0
    img[132:140, 40:120] = 0
    img[80:140, 40:48] = 0
    img[80:88, 48:90] = 0
    img[80:160, 150:158] = 0
    img[152:160, 70:158] = 0
    img[100:108, 70:110] = 0

    return img

def create_open_space_map(width=200, height=200):
    """Open area with scattered obstacles."""
    img = np.ones((height, width), dtype=np.uint8) * 255

    # Outer walls
    img[0:6, :] = 0
    img[-6:, :] = 0
    img[:, 0:6] = 0
    img[:, -6:] = 0

    # Random obstacles (pillars/objects)
    obstacles = [
        (40, 40, 15),
        (120, 50, 12),
        (60, 120, 18),
        (150, 100, 14),
        (100, 160, 16),
        (170, 160, 10),
        (30, 170, 12),
    ]

    for cx, cy, r in obstacles:
        cv2.circle(img, (cx, cy), r, 0, -1)

    return img

def main():
    # Create output directory
    output_dir = "test_data"
    os.makedirs(output_dir, exist_ok=True)

    maps = {
        "corridor": create_corridor_map(),
        "rooms": create_rooms_map(),
        "maze": create_maze_map(),
        "open_space": create_open_space_map(),
    }

    for name, img in maps.items():
        path = f"{output_dir}/{name}.png"
        cv2.imwrite(path, img)
        print(f"Created: {path} ({img.shape[1]}x{img.shape[0]})")

    print(f"\nGenerated {len(maps)} test maps in {output_dir}/")

if __name__ == "__main__":
    main()
