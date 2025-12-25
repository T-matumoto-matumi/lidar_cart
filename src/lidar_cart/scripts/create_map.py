#!/usr/bin/env python3
import numpy as np
from PIL import Image
import yaml
import os

def create_map():
    # Parameters for square_room.world
    resolution = 0.05 # m/pixel
    
    # helper to convert meters to pixels
    def m2px(m):
        return int(m / resolution)

    # Wall centers are at +/- 2.5m
    # Wall thickness is 0.1m
    # Wall boundaries:
    # Inner edge: 2.5 - 0.05 = 2.45
    # Outer edge: 2.5 + 0.05 = 2.55
    
    wall_center_m = 2.5
    wall_thickness_m = 0.1
    half_thickness = wall_thickness_m / 2.0
    
    inner_edge_m = wall_center_m - half_thickness # 2.45
    outer_edge_m = wall_center_m + half_thickness # 2.55
    
    # Map Dimensions
    # Let's make it 8x8m to having plenty of buffer
    map_size_meters = 8.0
    map_size_pixels = int(map_size_meters / resolution) # 160
    
    center_pixel = map_size_pixels // 2
    
    # Initialize with Unknown (205)
    image_data = np.full((map_size_pixels, map_size_pixels), 205, dtype=np.uint8)
    
    # Fill Interior with Free (254)
    # Interior is from -2.45 to 2.45
    px_inner = m2px(inner_edge_m) # 2.45 / 0.05 = 49
    
    # Slice limits
    # center - 49 to center + 49
    start = center_pixel - px_inner
    end = center_pixel + px_inner
    
    image_data[start:end, start:end] = 254
    
    # Draw Walls (Occupied = 0)
    # Walls from 2.45 to 2.55
    px_outer = m2px(outer_edge_m) # 2.55 / 0.05 = 51
    
    # Wall Limits
    w_inner = px_inner # 49
    w_outer = px_outer # 51
    
    # We need to draw the box outline
    # For a solid box outline of thickness (w_outer - w_inner)
    
    # Draw logic:
    # Fill the outer box with 0, then refill the inner box with existing content (which is 254)
    # OR explicitly draw 4 rectangles.
    
    # Let's use the outer/inner box method for simplicity of code
    t = center_pixel - w_outer
    b = center_pixel + w_outer
    l = center_pixel - w_outer
    r = center_pixel + w_outer
    
    # Set the whole outer box area to 0 temporarily? No, that overwrites the middle.
    # Better: explicit 4 rectangles.
    
    # Top wall (Y- in image coords if top-left origin? Let's just do symmetric)
    # Vertical range: [center - w_outer, center - w_inner]
    # Horizontal range: [center - w_outer, center + w_outer] (Full width of wall box)
    
    # Top band
    image_data[center_pixel - w_outer : center_pixel - w_inner, center_pixel - w_outer : center_pixel + w_outer] = 0
    # Bottom band
    image_data[center_pixel + w_inner : center_pixel + w_outer, center_pixel - w_outer : center_pixel + w_outer] = 0
    
    # Left band (between top and bottom bands)
    image_data[center_pixel - w_inner : center_pixel + w_inner, center_pixel - w_outer : center_pixel - w_inner] = 0
    # Right band
    image_data[center_pixel - w_inner : center_pixel + w_inner, center_pixel + w_inner : center_pixel + w_outer] = 0
    
    # Verification of corners
    # Top-Left intersection: 
    # Top band covers x: [-w_outer, +w_outer], y: [-w_outer, -w_inner]
    # Left band covers x: [-w_outer, -w_inner], y: [-w_inner, +w_inner]
    # They meet perfectly at y=-w_inner.
    
    # Save PGM
    output_dir = os.path.join(os.path.dirname(__file__), '../maps')
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
        
    pgm_path = os.path.join(output_dir, 'square_room.pgm')
    img = Image.fromarray(image_data)
    img.save(pgm_path)
    print(f"Generated {pgm_path}")
    
    # Create YAML
    yaml_path = os.path.join(output_dir, 'square_room.yaml')
    
    # Origin calculation
    origin_x = -(map_size_pixels * resolution) / 2.0
    origin_y = -(map_size_pixels * resolution) / 2.0
    
    metadata = {
        'image': 'square_room.pgm',
        'mode': 'trinary',
        'resolution': resolution,
        'origin': [origin_x, origin_y, 0.0],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.25
    }
    
    with open(yaml_path, 'w') as f:
        yaml.dump(metadata, f, sort_keys=False)
    print(f"Generated {yaml_path}")

if __name__ == '__main__':
    create_map()
