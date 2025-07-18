# Overview

A (Sim + Hardware Tested) Model Predictive Controller for Unitree quadruped robots based on the FSM framework from [unitree_guide](https://github.com/unitreerobotics/unitree_guide/tree/main/unitree_guide), using Quadprog++ as the solver.

## Tested Models

**Simulation:** Unitree A1, Unitree Go1  
**Hardware:** Unitree Go1 Air

## Environment

The develop environment is ROS Noetic and Ubuntu 20.04 but it should run just fine in ROS Melodic and Ubuntu 18.04.

## Dependencies

To run the controller in Gazebo and Hardware, you will need the following:

- [unitree_guide](https://github.com/unitreerobotics/unitree_guide/tree/main/unitree_guide)
- [unitree_ros](https://github.com/unitreerobotics/unitree_ros)
- [unitree_legged_msgs](https://github.com/unitreerobotics/unitree_ros_to_real) (Note: unitree_legged_real package should not be a part of dependencies)
- [free_dog_sdk_cpp](https://github.com/linzhuyue/free_dog_sdk_cpp)

Put these packages in the src folder of a ROS workspace. Paste my project at `~/NAME_OF_YOUR_PROJECT/src/unitree_guide/unitree_guide` and replace what's replicated.

## Demo
<div align="center">
  <img src="https://raw.githubusercontent.com/PMY9527/QUAD-MPC-SIM-HW/main/TrimmedMPCHW480.gif" alt="Trimmed MPC Hardware Demo" width="500">
</div>
<div align="center">
  A slightly longer video demo can be viewed at: <a href="https://www.youtube.com/watch?v=5x-IyCu0Nwc">Youtube</a> or <a href="https://www.bilibili.com/video/BV1YbuUzcEFp/?spm_id_from=333.1387.homepage.video_card.click&vd_source=926e11951d42d46224e97d067793de52">BiliBili</a>
</div>

## To Run

**[IMPORTANT]** You'll need to modify the CMakelist.txt under unitree_guide to switch between simulation and hardware.

### Simulation

1. Head to the project's folder:
   ```bash
   cd NAME_OF_YOUR_PROJECT
   ```

2. Build the workspace:
   ```bash
   catkin_make
   ```

3. Source the workspace:
   ```bash
   source devel/setup.bash
   ```

4. Open Gazebo:
   ```bash
   roslaunch unitree_guide gazeboSim.launch
   ```

5. New terminal, switch to root:
   ```bash
   sudo su
   ```

6. Source the workspace:
   ```bash
   source devel/setup.bash
   ```

7. Load the controller:
   ```bash
   rosrun unitree_guide junior_ctrl
   ```

### Hardware

Here I am showing an example of running on my Go1 Air.

1. Connect to robot via a network cable, and make sure the following pings correctly:
   ```bash
   sudo ifconfig eth0 down # eth0 is your PC Ethernet port
   sudo ifconfig eth0 192.168.123.162/24
   sudo ifconfig eth0 up
   ping 192.168.123.161
   ```

2. Do the following on your Remote Controller, for the robot to enter low-level mode:
   ```
   L2 + A
   L2 + A
   L2 + B
   L1 + L2 + START
   ```

3. Repeat steps 5, 6 and 7, just like in simulation.

After starting the controller, press '2' key on the keyboard to switch the robot's finite state machine (FSM) from Passive (initial state) to FixedStand, then press '6' to switch the FSM from FixedStand to MPC, now you can press the 'w' 'a' 's' 'd' key to control the translation of the robot, and press the 'j' 'l' key to control the rotation of the robot. Press the Spacebar, the robot will stop and stand on the ground. (If there is no response, you need to click on the terminal opened for starting the controller and then repeat the previous operation)

## Credits

A big thank you to [Unitree](https://github.com/unitreerobotics) and [Linzhuyue](https://github.com/linzhuyue) for their open-sourced projects!
