import numpy as np
from PIL import Image

def create_pgm_map(width, height, resolution):
    """
    Create a PGM map with walls as obstacles.

    Args:
    - width: Width of the map in meters.
    - height: Height of the map in meters.
    - resolution: Resolution of the map (meters per pixel).

    Returns:
    - None (saves the PGM and YAML files to the current directory).
    """
    # Calculate the map dimensions in pixels
    width_px = int(width / resolution)
    height_px = int(height / resolution)

    # Initialize the map with free space (255)
    map_data = np.ones((height_px, width_px), dtype=np.uint8) * 255

    # Define wall thickness in pixels (border thickness)
    wall_thickness_px = int(0.1 / resolution)  # 0.1 meter (10 cm) thick walls

    # Add walls (0 represents occupied space)
    map_data[:wall_thickness_px, :] = 0  # Top wall
    map_data[-wall_thickness_px:, :] = 0  # Bottom wall
    map_data[:, :wall_thickness_px] = 0  # Left wall
    map_data[:, -wall_thickness_px:] = 0  # Right wall

    # Create the PGM file
    img = Image.fromarray(map_data, mode='L')
    img.save('simple_map.pgm')

    # Create the YAML file
    yaml_content = f"""
image: simple_map.pgm
resolution: {resolution}
origin: [0.0, 0.0, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
"""
    with open('simple_map.yaml', 'w') as yaml_file:
        yaml_file.write(yaml_content)

# Example usage
create_pgm_map(
    width=10,   # 10 meters wide
    height=10,  # 10 meters high
    resolution=0.05  # 5 cm per pixel
)
