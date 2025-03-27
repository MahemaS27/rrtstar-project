from PIL import Image
import numpy as np
import matplotlib.pyplot as plt
from RRT import RRTPlanner

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
    start = (1, 48)
    goal = (75, 151)
    map_array = load_map("test_images/multiple_hallway.jpg", 0.3)

    # Create RRTPlanner object with the loaded map
    rrt_planner = RRTPlanner(map_array, start, goal)

    # Search with RRT and RRT*
    # params num points, neighbor_size
    ## add in a bunch of these statements in a loop

    for i in range(1,6):
        rrt_planner.RRT_star(1000, 15, str(i))

    #rrt_planner.RRT_star(1000, 15, str(5)) 
    

