from PIL import Image
import numpy as np
import matplotlib.pyplot as plt
from RRT import RRTPlanner
from informed_RRTstar import Informed_RRTSTAR
import time

start_time = time.time()

def load_map(file_path, resolution_scale):
    # Load the image with grayscale
    img = Image.open(file_path).convert('L')

    # Rescale the image
    size_x, size_y = img.size
    new_x, new_y = int(size_x * resolution_scale), int(size_y * resolution_scale)
    img = img.resize((new_x, new_y), Image.BILINEAR)
    map_array = np.asarray(img, dtype='uint8')

    # Convert the grayscale image to binary image
    threshold = 150
    map_array = 1 * (map_array > threshold)

    # Return the 2D numpy array representation of the map
    return map_array        
        
if __name__ == "__main__":
    # Load the map
    start = (250, 30)
    goal = (20, 200)
    map_array = load_map("ASU_Map.jpg", 0.3)

    # Create RRTPlanner object with the loaded map
    rrt_planner = RRTPlanner(map_array, start, goal)

    # Search with RRT and RRT*
    rrt_planner.RRT_star(n_pts=1000)

    end_time = time.time()
    elapsed_time = end_time - start_time
    print(f"Elapsed time: {elapsed_time} seconds")
