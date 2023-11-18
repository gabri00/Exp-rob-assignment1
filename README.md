# Assignment 1

>Note: developed in ROS Noetic

## Group members

| Name Surname          | ID       |
| --------------------- | -------- |
| Gabriele Nicchiarelli | S4822677 |
| Ivan Terrile          | S4851947 |
| Miriam Anna Ruggero   | S4881702 |
| Davide Pisano         | S4363394 |

## Preliminary operations

Install dependencies:

```bash
cd ~/<your_workspace>
rosdep install --from-paths src --ignore-src -r -y
```

Build the workspace:

```bash
cd ~/<your_workspace>
catkin_make
```

## Run the Gazebo simulation

Run the simulation (with rotating camera):

```bash
roslaunch rosbot_gazebo assignment_gazebo.launch
```

Run the simulation (with fixed camera):

```bash
roslaunch rosbot_gazebo assignment_rosbot.launch
```

## Run on the Rosbot

#### Step 1: Connect to the local network

| Network name | Network password |
| ------------ | ---------------- |
| TP_LINK      | 03694008         |

#### Step 2: add the ROS master URI and user's IP address

Add the following lines at the bottom of the `~/.bashrc` file:

```bash
export ROS_MASTER_URI=https://192.168.1.10x:11311
export ROS_IP=<YOUR_IP_ADDRESS>
```

#### Step 3: Connect to the robot via SSH

```bash
ssh husarion@192.168.1.10x
# x is the identifier number of the robot (written on the bot)
```

**Password**: husarion

#### Step 4: Start the drivers

In the Rosbot terminal:

```bash
roslaunch tutorial_pkg all.launch
```

#### Step 5: Start the simulation

In the local terminal:

```bash
roslaunch rosbot_gazebo real_rosbot.launch
```

## Description of the packages

This project consists in making the Husarion Rosbot detect the ArUco markers in order.

The [marker publisher node](/src/aruco_ros/src/marker_publish.cpp) is responsible of detecting the markers and communicating the ID and center of the marker to the *controller node*. This node also communicates to the *controller node* when the desired marker has been reached.

The controller node is provided in two formats:
- The [gazebo simulation controller](/src/rosbot_navigation/scripts/sim_robot_controller.py) which controls the robot in a simulated environment with Gazebo. In this simulation the camera can rotate to detect the markers.
- The [real robot controller](/src/rosbot_navigation/scripts/real_robot_controller.py) which controls the real rosbot. In this case the camera is fixed and the whole robot has to rotate to detect the markers.

The functioning of the controllers is described in the [flowcharts below](#flowcharts).

To control the camera a *joint state controller* has been added in the rosbot_description package.

### Pub/sub architecture

#### Gazebo rqt graph

![Gazebo rqt](images/sim_rqt.png)

#### Rosbot rqt graph

![Rosbot rqt](images/rosbot_rqt.png)

Note: in the real Rosbot the topic `/camera/color/` is substituted by `/camera/rgb`.

### Flowcharts

![Flowchart](./media/flowchart.png)

### Video demo

[![Video demo](./media/video_demo.png)](./media/video_demo.mp4)

## References

- [Husarion ROSbot](https://husarion.com/manuals/rosbot/)

- [ROSbot GitHub repository](https://github.com/husarion/rosbot_ros/tree/noetic)

- [Aruco ROS](https://github.com/CarmineD8/aruco_ros)

## Troubleshoot

Here goes the description of the problems encountered and how they were solved.