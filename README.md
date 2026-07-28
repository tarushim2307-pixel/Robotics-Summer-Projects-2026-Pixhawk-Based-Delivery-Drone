# Robotics Summer Project 2026-27 

# Project Overview

### Aim of the project:
We aim to build **Pixhawk Based Delivery Drone** that can carry a package from one place to another.

### Drone's Gripper:
The drone will use a smart robotic gripper inspired by a **fish's fin**. When it grabs a package, it bends perfectly around it so it never drops it.

### Flight Control:
* **First:** We fly it manually using a remote controller.
* **Next:** We fly it autonomously via ROS

### Computer Testing (Simulation):
Before building the real drone, we test it on **Gazebo** with **PX4** firmware. This helps us make sure the drone and the package stay 100% safe from crashes, wind, and accidents.

# Goals

* Assembly and hardware setup completion
* Simulate the default iris quadcoptor on Gazebo
* Working gripper attached to the drone with a suitable stand
* Autonomous flight of pixhawk based delivery drone

# Hardware Setup And Calibration

The softwares required:
* QGroundControl
* PX4 firmware

We follow Qgroundcontrol installation for installation of ground station.

Then we installed the PX4 firmware in pixhawk from firmware install section of Qgroundcontrol.

### Vehicle Configuration

We used almost all standard default PX4 parameters ensuring full operational awareness, robust failsafes, and GPS guided autonomous flight required for delivery operations.

### Flight Mode & Radio Configuration

In the Radio Configuration, we calibrated the Radio Controller as per the instructions given.

And in the Flight Modes Configuration,we mapped the modes to a physical 3 position switch on the RC transmitter assigned to **Channel 5** in the following manner:
* **Switch Position UP-** Stabilized Mode
* **Switch Position MIDDLE-** Altitude Hold Mode
* **Switch Position DOWN-** Land Mode

### Sensors Configuration

In the Sensors Configuration, we performe the following calibrations:
* **Compass**
* **Accelerometer**
* **Gyroscope**
* **Level Horizon**
* **Orientations**

### Power Settings

In the Power Tab, we connect to the USB Cable, and perform the ESC Caliberation by connecting and disconnecting the battery as per the instructions given.

### Actuators Settings

In the Actuators Settings, we connect to the USB Cable, and performed the following:
* Assigning up the motors with numbers along with their respective directions
* Assigning **RC AUX 1** to the respective channel for proper functioning of the attached Gripper

# Simulation

The softwares required:
* Gazebo Classic
* PX4 Autopilot

1. Installations
* Gazebo Classic

* sudo apt install aptitude

* sudo apt install gazebo libgazebo11-dev

* PX4-Autopilot

* git clone https://github.com/PX4/PX4-Autopilot.git --recursive

* bash ./PX4-Autopilot/Tools/setup/ubuntu.sh

2. Restart the computer.

3. Running Simulations

* go the px4 directory
* cd PX4-Autopilot
* terminal command for default model:
* make px4_sitl gazebo-classic
* terminal command for certain model:
* make px4_sitl gazebo-classic_custom

4. File Structure

* A custom model folder Tools/simulation/gazebo-classic/sitl/models/custom_model

* custom model should have
Meshes folder
custom_model.sdf
model.config

* Meshes folder should have

custom_model.stl
propeller cw.dae
propeller ccw.dae

**REMARKS :**

* It is suggested to modify the default models rather than making everything from scratch.

* sdf files cannot be easily generated from some tools => modify the parameters in a default .sdf

* Or use a .jinja template just in case you need to make a .sdf

* moving parts must be stored as dae.

* .stl for the visuals

* .sdf for the data required for the simulations

# ROS2 Humble and Teleop

1. Installing ROS2 from ROS documentation of debian packages(recommended)
* ROS2 Humble
* Test basic ros2
* ros2 run demo_nodes_cpp talker
2. Clean insatllation of Micro XRCE-DDS agent (Creates a Bridge between PX4 and ROS2)

## Install dependencies:

* sudo apt update
* sudo apt install git cmake g++ libasio-dev libtinyxml2-dev

## Create a Workspace Directory

* mkdir -p ~/micrortps_ws && cd ~/micrortps_ws

## Cloning from github

* git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
* cd Micro-XRCE-DDS-Agent
* git submodule update-init-recursive

## Build

* cd ~/micrortps_ws/Micro-XRCE-DDS-Agent
* mkdir build && cd build cmake .. -DUAGENT_USE_SYSTEM_FASTDDS=OFF -DUAGENT_USE_SYSTEM_FASTCDR=OFF
* make -j$(nproc)
* sudo make install

## Update Library Cache

* sudo ldconfig

## Run using udp port

* MicroXRCEAgent udp4 -p 8888

3. **Setting up our own teleop ros package**
   1. **Install teleop keyboard:** `sudo apt install ros-humble-teleop-twist-keyboard`
   2. **Create a working directory:** `mkdir -p ~/px4_ros2_ws/src` `cd ~/px4_ros2_ws/src`
   3. **Install px4 msgs and ros com:** `git clone https://github.com/PX4/px4_msgs.git` `git clone https://github.com/PX4/px4_ros_com.git`
   4. **Install dependencies:** `sudo apt update` `rosdep update` `rosdep install --from-paths src --ignore-src -r -y` (run this command after changing directory to `~/px4_ros2_ws`)
   5. **Build workspace:** `cd ~/px4_ros2_ws` `sudo apt install ros-humble-ament-lint-common` `colcon build`
   6. **Source the setup:** `source install/setup.bash`
   7. **Now create your custom teleop folder:** `cd ~/px4_ros2_ws/src` `ros2 pkg create teleop_px4 --build-type ament_python --dependencies rclpy geometry_msgs px4_msgs`
   8. **Create your own python script:**
      * `touch teleop_to_px4.py` (enter this command in `src/teleop_px4/teleop_px4` folder)
   9. **Download `teleop_to_px4.py`, `package.xml`, `setup.py`**
   10. **Replace the files in `px4_ros2_ws/src/teleop_px4`**
   11. **In the `src` directory `colcon build` in order to build the new teleop packages. All Done.**

---

4. **Running and controlling simulations**

   * In terminal **Run Gazebo Simulation as usual**
     * `cd PX4-Autopilot` `make px4_sitl gazebo-classic_iris`
   * In another terminal **run the bridge**
     * `MicroXRCEAgent udp4 -p 8888` the bridge must start giving info including client Id and in terminal 1, output would be like successfully created...
   * In another terminal **run teleop_twist_keyboard**
   * **Source the ros**
     * `source /opt/ros/humble/setup.bash`
     * `source ~/px4_ros2_ws/install/setup.bash`
     * `ros2 run teleop_twist_keyboard teleop_twist_keyboard`
   * In **yet another terminal**
   * **source again like last step**
     * `ros2 run teleop_px4 teleop_to_px4`

5. **Bugs and solutions :**

   * Don't forget to source any terminal in which you want to run ros2 commands
   * You can add `source /opt/ros/humble/setup.bash` in your `.bashrc` if you don't want to source everytime.
   * You can control the drone in the terminal 3, i.e. the terminal in which you opened the `teleop_twist_keyboard`
   * If the drone doesn't arms itself after few seconds : arm manually using `commander arm` in pxh shell in terminal 1
   * If you cannot control the drone : try `commander mode offboard` in the pxh shell in terminal 1.
   * **Bug : poll timeout =>** Restart the simulation :(
   * If you want to start another simulation by killing one, redo step 4 (Running and controlling ...)

# Gripper Designing & Working

### Working Principle:
The gripper is based on the **Fish Fin Ray Effect**.

### Structure:
* **Outer Beams-** The flexible,continuous beams form the V-shaped overall outline of the gripper's fins.
* **Internal Ribs-** The thin ribs inside the fins connect the front and back beams.
* **Intersection Joints-** The joints at the intersection of beams and ribs control and support bending of gripping upon contact with parcel.

### Concept:
The fins of the gripper are made up of a material called **Thermoplastic Polyurethane(TPU)** wrap around the parcel upon contact, by effective deformation, and hence accomodating fragile objects of various shapes and sizes.

