# pigeon_converter

ROS topic converter

## scan_to_point_cloud2_node

Node that converts sensor_msgs::LaserScan to sensor_msgs::PointCloud2

### Run

```bash
roslaunch pigeon_converter scan_to_point_cloud2.launch
```

### Subscribed Topics

- scan ([sensor_msgs/LaserScan](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/LaserScan.html))


### Published Topics

- scan_to_point_cloud2 ([sensor_msgs::PointCloud2](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/PointCloud2.html))


### Parameters

- ~ TopicNameScan (string, default: scan)

  sensor_msgs/LaserScan message Topic Name to subscribe
  
- ~ TopicNameScanToPointCloud2 (string, default: scan_to_point_cloud2)

  The name of the topic to be published after being converted to form sensor_msgs::PointCloud2

  
  

