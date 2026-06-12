# Robotics Summer Project 2026-27 

# Project Overview

### Aim of the project:
We aim to build **Pixhawk Based Delivery Drone** that can carry a package from one place to another.

### Drone's Gripper:
The drone will use a smart robotic gripper inspired by a **fish's fin**. When it grabs a package, it bends perfectly around it so it never drops it.

### Flight Control:
* **First:** We fly it manually using a remote controller.
* **Next:** We fly it autonomously via ROS.

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

# Simulation

The softwares required:
* Gazebo Classic
* PX4 Autopilot

# Gripper Designing & Working

### Working Principle:
The gripper is based on the **Fish Fin Ray Effect**.

### Structure:
* **Outer Beams-** The flexible,continuous beams form the V-shaped overall outline of the gripper's fins.
* **Internal Ribs-** The thin ribs inside the fins connect the front and back beams.
* **Intersection Joints-** The joints at the intersection of beams and ribs control and support bending of gripping upon contact with parcel.

### Concept:
The fins of the gripper are made up of a material called **Thermoplastic Polyurethane(TPU)** wrap around the parcel upon contact, by effective deformation, and hence accomodating fragile objects of various shapes and sizes.

