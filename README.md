# gpd_vcloud
Remove dangerous grasp: VCloud(visible point cloud) filter implemented with "Grasp Pose Detection in Point Clouds"

* [Author's website](https://www.oit.ac.jp/elc/~matsunolab/index.html)
* [License](making)


Since the occlusion of sensor is considered, we want to remove the partly invisible grasp from the sensor.
We provide a filter/method that include incomplete point cloud information.
The method can be used for a lof of different scene such as complex background. 
To solve this problem, we introduce a novel technique named ‘visible point-cloud’ – generated using the point cloud and pose (position and orientation) information of the sensor(s) – that helps to eliminate unsafe grasp candidates quickly and efficiently.

Of crouse, the same affect can be achieved by using other filters. But I believe the method is more general and more efficient.

The method is implemented with "Grasp Pose Detection in Point Cloud"
* A standalone filter can be used as well for any grasp detection method ([making])

## 1) Requirements & Installation



## 2) Installation

The following instructions have been tested on **Ubuntu 18.04**.

1. Install ROS. the ros installs the requerements for you.
  [PCL 1.9 or newer]
  [Eigen 3.0 or newer]
  [OpenCV 3.3 or newer]

3. Install gpd:
   Since we use the old version of "Grasp Pose Detection in Point Clouds". please click [here] to install gpd

4. Build the package in ROS

## 3) References

If you like this package and use it in your own work, please cite our journal
paper. I attach the paper file in the package

Xixun Wang, S. Nisar, F. Matsuno, Robust grasp detection with incomplete point cloud and complex background.
