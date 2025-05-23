# Lidar publish pointcloud msg format.

std_msgs/Header header    # ROS standard message header
uint64 timebase           # The time of first point
uint32 point_num          # Total number of pointclouds
uint8  lidar_id           # reserve
uint8[3]  rsvd            # reserve
CustomPoint[] points      # Pointcloud data

