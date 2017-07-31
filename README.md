# cloud_tracker
ROS node for tracking a pointcloud


## Usage:
From command line:
```
rosrun cloud_tracker cloud_tracker /pc_filter
```
In launch file:
```
<node pkg="cloud_tracker" type="cloud_tracker" name="cloud_tracker" args="/pc_filter"/>
```

This will now listen to pointclouds on the pc_filter, and once a service request to track an object has been given, the system will track the object in this pointcloud. 
