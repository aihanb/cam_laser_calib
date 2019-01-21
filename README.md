# [Hesai]camera-laser-calibration 
 Editor: aihanb
 
 Authentic authors: Nico

# overview  
Calibration of the LiDAR sensor with RGB camera finds its usage in many application fields from enhancing image classification to the environment perception and mapping. This package is used to calculate the translation and rotation matrix between camera and laser coordinate, especially for Hesai Pandar P40. There are two different important points between Hesai LiDAR and Velodyne: 
1)Timestamp. Because Pandar_P40 doesn't apply the Linux UTC time, you should add two sentences in /HesaiLidar-ros/src/main.cc
```
void lidarCallback(boost::shared_ptr<PPointCloud> cld, double timestamp)
{
	...
	output.header.frame_id = "/pandar_40"; 
    output.header.stamp = ros::Time::now();
    ...
}
```
In Panda40 User's Manual, it says "If the user wants to get absolute time, GPS module is required".
2)Different coordinate. Pay more attention on cv::solvePnPRansac.
I use the QR code board as the marker, detect the center point pair of the QR code as 2D point in image and 3D points in laser coordinates. Then use the PnP method to get the relation between the two coordinates. Actualy you can use any other marker. It will also give you good result.
Image shows the image and laser fuse result:
![](https://github.com/aihanb/camera-laser-calibration/raw/master/screenshots/1.png)

# prerequisites

## 1 ROS

We use [ros](http://wiki.ros.org/kinetic/Installation/Ubuntu) to get the image and laser message.
(Devices in here: [Camera]Point Grey BFLY-U3-23S6; [LiDAR]Hesai_Pandar40/40P)

## 2 Marker

Here we use the QR code as the marker. The size of the marker is 80*80 cm. Other sizes will also fit.

Marker

<img src="https://github.com/aihanb/camera-laser-calibration/raw/master/screenshots/2.png" width="50%" height="50%">

# Test data

You can download the test data in [here](https://pan.baidu.com/s/1fZiBqmOMXkmfprysrosVRg)(password: ppw0). This rosbag have two topic:
image topic:/camera/image_color 
laser topic: /pandar_points
Run the bag file:
```
$rosbag play ss8.bag -l
```
And follow the "Usage"step, you can test the package.

#Usage 

## 1 camera calibtation 

If you use the test data, you can ignore this step.
```
$rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.058 image:=/camera/image_color camera:=/camera
```
You shoude change the size/square/image/camera with your own parameters. Write the camera calibration result in the cam_laser_calib/src/solvepnp/param/calib.yml file. After this step, you can obtain the camera instrinsic matrix.

## 2 Get the points pair of QR code center

### 2.1 build and run

Create a ros workspaces named cam_laser_calib.
``` 
$cd cam_laser_calib/src
$git clone http://github.com/aihanb/camera-laser-calibration.git
$cd ..
$catkin_make
$source ./devel/setup.bash
$roslaunch cam_laser_calib calibration.launch
```
Here we shoud notice the parameter in launch file, change the "onlyDrawPointsColor" value to false. Replace the point cloud and image topic with your own topic name.
```
<launch>

	<node pkg="cam_laser_calib" type="cam_laser_calib_node" name="cam_laser_calib" args="$(find cam_laser_calib)/../solvepnp/imageCloudPoints.txt $(find cam_laser_calib)/../solvepnp/param/calib.yml" output="screen">

		    <param name="strSub_pc2" type="string" value="/pandar_points"/>

	   		<param name="strSub_img" type="string" value="/camera/image_color"/>

	  	 	<param name="onlyDrawPointsColor" type="bool" value="false"/>

		    <param name="DistanceThreshold" type="double" value="0.05"/>

	</node>

</launch>
```

### 2.2 Choose rectangle cut area of point cloud

We use rqt_reconfigure to dynamic config the rectangle cut area of point cloud.
```
$rosrun rqt_reconfigure rqt_reconfigure 
```
rqt_reconfigure

![rqt_reconfigure](https://github.com/aihanb/camera-laser-calibration/raw/master/screenshots/3.png)

Before cut the point cloud

<img src="https://github.com/aihanb/camera-laser-calibration/raw/master/screenshots/4.png" width="50%" height="50%">

After cut the point cloud

<img src="https://github.com/aihanb/camera-laser-calibration/raw/master/screenshots/5.png" width="50%" height="50%">

Estimated plane

<img src="https://github.com/aihanb/camera-laser-calibration/raw/master/screenshots/6.png" width="50%" height="50%">

After adjust the parameter, write it in the cam_laser_calib/src/camLaserCalib/cfg/cam_laser_calib.cfg file and shutdown the program. Recompile the package.
```
$cd cam_laser_calib/
$catkin_make 
```

### 2.3 Get the points pair of QR code center

```
$roslaunch  cam_laser_calib calibration.launch
```
After this step, we write the point pairs of QR code center in laser coordinate and image coordinate to cam_laser_calib/src/solvepnp/imageCloudPoints.txt file.

## 3 Calculate the calibration matrix

We use solvePnP method in openCV to get the calibration matrix. 
```
$cd cam_laser_calib/src/solvepnp/
$cmake . 
$make
$cd bin/
$./solvepnp
```
After this step, we can obtain the camera extrinsic matrix T. Please copy the T matrix in terminal to cam_laser_calib/src/solvepnp/param/calib.yml file. Replace the CameraExtrinsicMat.

## 4 Using calibration matrix to draw point cloud with image color

At this step we could change the onlyDrawPointsColor parameter in "calibtation.launch" file to true. Got the color point cloud as the first figure.



