# Example of PCL integration with ROS

This repository contains some simple C++ nodes for ROS which demonstrate
interoperation with the [PCL](http://pointclouds.org/).

All nodes listen for point cloud data as a sensor\_msgs::PointCloud2 message on
``/vamera/depth/points`` and publish to the ``/fused_points`` topic.

## pcl\_voxel

An example of the 'voxelisation' method of point cloud simplification. Choose
one point per 10cm x 10cm voxel grid point.

## pcl\_fuse

An example of fusing multiple point clouds. The input point clouds are all
transformed into the ``/odom`` frame and concatenated.
