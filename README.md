# Robotic arm - Pick & Place project

Project description here

[//]: # (Image References)

[image1]: ./link_assignments.png
[image2]: ./DH_individual_transforms.png
[image2]: ./DH_individual_transforms_matrix.png
[image3]: ./misc_images/misc2.png

# Kinematic Analysis

### Forward kinematics

Solving the forward kinematics problem of manipulators involves calculating the location of the end-effector (gripper) given the joint variables. The solution procedure involves attaching a reference frame to each link of the manipulator and writing the homogenous transforms from the fixed base link to link 1, link 1 to link 2, and so forth, all the way to the end effector. 

Denavit-Hartenberg (DH) method of attaching reference frames to links of a manipulator simplifies the homogenous transforms and only requires 4 parameters to describe the position and orientation of neighboring reference frames (in general, each transform would require 6 parameters to describe frame i relative to i-1 - 3 for position, and 3 for orientation). 

The DH parameter assignment process for open kinematic chains with n degrees of freedom is the following:
 1. Label all joints from {1,2,...,n}
 2. Label all links from {0, 1,...,n} starting with the fixed base link
 3. Draw lines thorugh all joints, defining the joint axes
 4. Assign the Z-axis of each frame to point along its joint axis
 5. Identify the common normal between each frame Z{i-1} and frame Z{i}
 6. The endpoints of intermediate links (not the base link or end effector) are associated with 2 joint axes, {i} and {i+1}. For i from 1 to n-1, assign X{i} to be one of the following: (1) For Skew axes: along the normal between Z{i} and Z{i+1} and pointing from {i} to {i+1} (2) For intersecting axes: normal to the plane containing Z{i} and Z{i+1} (3) For parallel or coincident axes: the assignment is arbitrary, but look for ways to make other DH parameters equal to 0
 7. For the base link, choose frame {0} to be coincident with frame {1} when the first joint variable is equal to 0. This will guarantee that alpha{0} = a{0} = 0, and, if joint 1 is a revolute, d{1} = 0. If joint 1 is prismatic, then theta{1} = 0
 8. FOr the end effector frame, if joint n is revolute, chose X{n} to be in the direction of X{n-1} when theta{n} = 0 and th origin frame {n} such that d{n} = 0

Link assigments:

[Robot Arm Link Assignments][image1]

The 4 parameter values for the DH parameter table are (Note:Because the KR210 has 6 revolute joints, only the theta terms are time variable):
 - alpha = twist angle
 - a = link length
 - d = length offset
 - theta = joint angle (q)

The numerical values for the a's and d's comes from the URDF file. Importantly, the reference frame assignments in the URDF file are not coincident with the DH-convention, nor do they always have the same orientation. The URDf file defines each joint relative to its parent. 

DH parameter table:

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | q1
1->2 | - pi/2 | 0.35 | 0 | -pi/2 + q2
2->3 | 0 | 1.25 | 0 | q3
3->4 |  -pi/2 | -0.054 | 1.50 | q4
4->5 | pi/2 | 0 | 0 | q5
5->6 | -pi/2 | 0 | 0 | q6
6->EE | 0 | 0 | 0.303 | 0

Using DH parameters, create individual tranformation matrices about each joint. The DH convention uses 4 individual transforms:

[DH Individual Transforms][image2]

[DH Individual Transforms Matrix][image3]

Add code to showing how I generated the individual transforms with a function

Also generate a homogenous tranform between base_link and gripper_link using the end-effector pose. In order to compare the total homogeneuos transform between the base link and the gripper link, we need to account for the difference in orientation of the gripper link frame. To do this, apply a sequence of body fixed (intrinsic) rotations. To get the frames to align: (1) rotate the gripper frame about the z-axis by 180 degrees, and (2) about the y-axis by -90 degrees

Add code for total transform and the adjsuted made

### Inverse kinematics

 1. Calculate the location of the spherical wrist center (WC)
 2. use trigonometry to solve for the first 3 joint angles
 3. The orientation of the end-effector is known from ROS. Find joint angles 4, 5, and 6 as demonstrated in "Euler Angles from Rotation Matrix"
 
Break into position and orientation problems

Steps for deriving the equations for individual joint angles. (If any joint has multiple solutions, select the best solution and provide an explanation about your choice (hint: some joints have physical limits)

# Project Implementation

IK_server.py implementation details


# Running the project

Make sure you are using robo-nd VM or have Ubuntu+ROS installed locally.

### One time Gazebo setup step:
Check the version of gazebo installed on your system using a terminal:
```sh
$ gazebo --version
```
To run projects from this repository you need version 7.7.0+
If your gazebo version is not 7.7.0+, perform the update as follows:
```sh
$ sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
$ wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
$ sudo apt-get update
$ sudo apt-get install gazebo7
```

Once again check if the correct version was installed:
```sh
$ gazebo --version
```
### For the rest of this setup, catkin_ws is the name of active ROS Workspace, if your workspace name is different, change the commands accordingly

If you do not have an active ROS workspace, you can create one by:
```sh
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```

Now that you have a workspace, clone or download this repo into the **src** directory of your workspace:
```sh
$ cd ~/catkin_ws/src
$ git clone https://github.com/udacity/RoboND-Kinematics-Project.git
```

Now from a terminal window:

```sh
$ cd ~/catkin_ws
$ rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
$ cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts
$ sudo chmod +x target_spawn.py
$ sudo chmod +x IK_server.py
$ sudo chmod +x safe_spawner.sh
```
Build the project:
```sh
$ cd ~/catkin_ws
$ catkin_make
```

Add following to the .bashrc file
```
export GAZEBO_MODEL_PATH=~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/models

source ~/catkin_ws/devel/setup.bash
```

Launch the project by
```sh
$ cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts
$ ./safe_spawner.sh
```

Then run:
```sh
$ cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts
$ rosrun kuka_arm IK_server.py
```
