# velarray
Simple ROS driver for a Velodyne [Velarray M1600](https://velodynelidar.com/products/velarray-m1600/).

It might work for Velodyne Velarray M800 too, but I have not tested it.

## Requirements
The package was developped for this system, I have not tested it anywhere else : 
- Ubuntu(=20.04)
- ROS(=Noetic)
- Python 3.8

# Usage
Before running the package, a catkin_make is needed.

```bash
    rosrun velarray main.py
```