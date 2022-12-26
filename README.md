# POVNav: A Pareto-Optimal Mapless Visual Navigator

POVNav is a planning and control framework that uses a novel image-based local representation of the environment using existing image segmentation methods, to navigate a robot to the specified goal point or direction using only a monocular camera without relying on a map. Apart from the collision-free motion, it also shows selective navigation behavior (such as ``Do not walk on the grass!'') which is not possible with the occupancy grid representation. 
The navigation task is formulated as a visual servoing problem where the robot is able to directly generate efficient motion according to the visual features representing the navigability. The navigability is represented as a binary image generated from existing segmentation techniques, makes it a handy plug-and-play suite for any segmentation methods.

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


- Run any image segmentation algoritms. Goto the `depth2surface_normals_seg`. Install and launch the segmentation module.

## Setup `POVNav` repository
- 

