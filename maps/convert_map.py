import numpy as np
from imageio import imwrite

# Define map size (100x100 pixels)
map_size = (100, 100)

# Create a blank (white) occupancy grid (255 = free space)
empty_map = np.full(map_size, 255, dtype=np.uint8)

# Save it as PGM
pgm_path = "empty_world.pgm"
imwrite(pgm_path, empty_map)

print(f"Empty map saved to {pgm_path}")

