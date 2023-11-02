# Assignment 1

## Instructions

Install dependencies:

```bash
cd ~/noetic_ws
rosdep install --from-paths src --ignore-src -r -y
```

Build the workspace:

```bash
cd ~/noetic_ws
catkin_make
```

#### Connect to the local network

| Network name | Network password |
| ------------ | ---------------- |
| TP_LINK      | 03694008         |

#### Run the program from the robot (ROS Melodic)

Connect to the robot via SSH:

```bash
ssh husarion@192.168.1.10x
```

When asked, input the password: *husarion*

> *x* is the identifier number of the robot (written on the bot)

#### Run the program from your pc (ROS Noetic)

Write this in the terminal:

```bash
export ROS_MASTER_URI=https://192.168.1.11x:11311
```

> Remember to compress the images that you want to transmit!

## How to run

```bash
roslaunch rosbot_gazebo rosbot_world.launch
rosrun rosbot_navigation robot_camera_control.py
rosrun rosbot_navigation movement_controller.py
rosrun aruco_ros marker_publisher 		questa parte non serve più è gia implementata all'interno del file c++ -->  /image:=/camera/color/image_raw
rostopic pub /exp_rob/camera_position_controller/command std_msgs/Float64 "data: 0.73" 
rosrun image_view image_view image:=/camera/color/image_raw
```

## Troubleshoot

>If you have any problems with laser scan it probably means that you don't have a dedicated graphic card (or lack appropriate drivers). If that's the case then you'll have to change couple of things in `/rosbot_description/urdf/rosbot_gazebo` file: <br><br>
>Find:   `<!-- If you cant't use your GPU comment RpLidar using GPU and uncomment RpLidar using CPU gazebo plugin. -->`
next coment RpLidar using GPU using `<!-- -->` from `<gazebo>` to `</gazebo>` like below:
> ```xml
> <!-- gazebo reference="rplidar">
>   <sensor type="gpu_ray" name="head_rplidar_sensor">
>     <pose>0 0 0 0 0 0</pose>
>     <visualize>false</visualize>
>     <update_rate>40</update_rate>
>     <ray>
>       <scan>
>         <horizontal>
>           <samples>720</samples>
>           <resolution>1</resolution>
>           <min_angle>-3.14159265</min_angle>
>           <max_angle>3.14159265</max_angle>
>         </horizontal>
>       </scan>
>       <range>
>         <min>0.2</min>
>         <max>30.0</max>
>         <resolution>0.01</resolution>
>       </range>
>       <noise>
>         <type>gaussian</type>
>         <mean>0.0</mean>
>         <stddev>0.01</stddev>
>       </noise>
>     </ray>
>     <plugin name="gazebo_ros_head_rplidar_controller" 
>filename="libgazebo_ros_gpu_laser.so">
>      <topicName>/rosbot/laser/scan</topicName>
>       <frameName>rplidar</frameName>
>     </plugin>
>   </sensor>
> </gazebo -->
>```
>
>Now uncomment RpLidar using CPU plugin removing `<!-- -->`.
>
>If you want to make your laser scan visible just change:
>```xml
><visualize>false</visualize>
>```
>to:
>```xml
><visualize>true</visualize>
>```
>in the same plug in.
