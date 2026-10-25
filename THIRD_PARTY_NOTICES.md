# Third-party notices

The root MIT license applies to project-owned code. Existing third-party notices and licenses take precedence for their respective files.

## Bundled ground segmentation

`src/lidar/src/linefit_ground_segmentation` is third-party software by Lorenz Wellhausen and contributors, licensed under BSD 3-Clause. Its original [LICENSE](src/lidar/src/linefit_ground_segmentation/LICENSE) is retained. It implements ground segmentation described by Himmelsbach et al., “Fast segmentation of 3d point clouds for ground vehicles” (2010).

## External dependencies

catkin_simple, glog_catkin, gflags_catkin, ROS, Velodyne, NumPy, OpenCV, PyYAML, pyserial, ros_numpy, SciPy and scikit-learn remain separately licensed. Their source/binaries are not relicensed under this project's MIT license. Consult the exact versions installed on the target system for their notices.

The optional `dependencies.repos` file imports external source repositories; they are not part of this repository's authored code.
