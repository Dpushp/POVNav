# POVNav: A Pareto-Optimal Mapless Visual Navigator

POVNav is a planning and control framework that uses a novel image-based local representation of the environment using existing image segmentation methods, to navigate a robot to the specified goal point or direction using only a monocular camera without relying on a map. Apart from the collision-free motion, it also shows selective navigation behavior (such as ``Do not walk on the grass!'') which is not possible with the occupancy grid representation. 
The navigation task is formulated as a visual servoing problem where the robot is able to directly generate efficient motion according to the visual features representing the navigability. The navigability is represented as a binary image generated from existing segmentation techniques, makes it a handy plug-and-play suite for any segmentation methods.

![Dynamic Reconfigure Window](assets/POVNav_full.png)

## Requirements
- Any robot simulator that provides the control of it's linear and angular velocities (`v, w`), e.g. Jackal, Husky, Turtlebot e.t.c. 
Use the following instruction to setup Jackal simulator: `sudo apt-get install ros-<distro>-jackal-simulator ros-<distro>-jackal-desktop ros-<distro>-jackal-navigation`.
- Image Segmentation Method: Use any segmetation method of your choice, define the nevigabiliy vector and prodice binary segmentation image such that navigable segments gets a value `'255'` non-navigable gets `'0'`.

### Required ROS topics
| Subscribe            | Description                                                                                                   |        Example      |
| -------------------- | ------------------------------------------------------------------------------------------------------------- | :-----------------------: |
| ~odom_topic     | name of topic that provides the robot's state.                          | "/odom"            
| ~binary_segmented_image               | name of the topic that provides the binary segmeted image based on the defined navigability.                    |              "/ground_segmentation"            |

- It publishes linear and angular velocities on the "`/cmd_vel`" topic.

# Setup
## Install and run the dependencies
- Launch the jackal simulator (or any other UGV sumulator) with realsence camera on it. Verify that you are able to subscribe to the above mentioned topics.
```
export JACKAL_URDF_EXTRAS=/path_to_src/camera_urdf/realsense.urdf.xacro
roslaunch jackal_gazebo empty_world.launch 
```


- Setup segmentation module from [depth2surface_normals_seg](https://github.com/Dpushp/depth2surface_normals_seg). Alternatively, you can run any image segmentation algoritms and implement *Visual Horizon*. Launch the segmentation node.

## Setup `POVNav` repository
### How to Build
Clone and build the repository to your workspace. 
```
mkdir catkin_ws/src -p
cd catkin_ws/src
```
```
git clone https://github.com/Dpushp/POVNav.git
```
```
cd ..
catkin_make
```

### Lanuch the sim_pov_nav node
```
roslaunch roslaunch pov_nav sim_pov_nav.launch 
```
### Open rviz to visualise the topics and give goal to the robot
```
rosrun rviz rviz -d path_to_pov_nav_src/rviz/pov_nav_viz.rviz
```
![POVNav Planning Image with Gazebo](assets/POVNav_planning_image.png)

Robot will navigate to the goal given from rviz.

## Parameters Tuning
Run Dynamic Recongigure to change the parameters.
```
rosrun rqt_reconfigure rqt_reconfigure
```
![Dynamic Reconfigure Window](assets/rqt_params.png)

### Parameters Decription for  `POVNav`

| Parameter            | Description                                                                                                   |    Default Value  |
| -------------------- | ------------------------------------------------------------------------------------------------------------- | :---------------: |
| ~d_erosion_size      | The erosion operation is: `dst(x,y)=min(x???,y???):element(x???,y???)???0src(x+x???,y+y???)`. Used to denoize the depth image
| 2 |
| ~d_gaussian_blur_size    | Gaussian blur kernel size used to denoize the depth image.                                    |        11        |
| ~iLowH    | Hue lower limit in surface normals image to select the  navigable class. Used to generate the binary navigable image i.e., ground and non-ground segments.                                       |        30        |
| ~iLowS  | Saturation lower limit in surface normals image to select the  navigable class. Used to generate the binary navigable image i.e., ground and non-ground segments.      |        150        |
| ~iLowV  | Value lower limit in surface normals image to select the  navigable class. Used to generate the binary navigable image i.e., ground and non-ground segments.      |        60        |
| ~iHighH  | Hue upper limit.      |        110        |
| ~iHighS  | Saturation upper limit.      |        255        |
| ~iHighV  | Value upper limit.      |        255        |
| ~seg_erosion_size  | Erosion kernel size for ground segmentation.      |        5        |


