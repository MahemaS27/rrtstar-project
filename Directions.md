## Single Narrow

```
# Load the map
    start = (320, 0)
    goal = (13, 321)
    map_array = load_map("test_images/single_narrow.jpg", 0.3)
```

## Curve with dent (start from top)

```
    # Load the map
    start = (1, 240)
    goal = (70, 72)
    map_array = load_map("test_images/curve_with_dent.jpg", 0.3)

    # Create RRTPlanner object with the loaded map
    rrt_planner = RRTPlanner(map_array, start, goal)

    # Search with RRT and RRT*
    rrt_planner.RRT_star(2000)


```

## Curve with dent (start from bottom)

```


```
