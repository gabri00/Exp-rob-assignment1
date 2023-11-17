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

Here goes the description of the packages.

### Pub/sub architecture

##### Gazebo rqt graph

![Gazebo rqt](images/sim_rqt.png)

##### Rosbot rqt graph

![Rosbot rqt](images/rosbot_rqt.png)

Note: in the real Rosbot the topic `/camera/color/` is substituted by `/camera/rgb`.

### Flowchart

![Flowchart](./media/flowchart.png)

### Video demo

[![Video demo](./media/video_demo.png)](./media/video_demo.mp4)

## References

- [Husarion ROSbot](https://husarion.com/manuals/rosbot/)

- [ROSbot GitHub repository](https://github.com/husarion/rosbot_ros/tree/noetic)

- [Aruco ROS](https://github.com/CarmineD8/aruco_ros)

## Troubleshoot

Here goes the description of the problems encountered and how they were solved.