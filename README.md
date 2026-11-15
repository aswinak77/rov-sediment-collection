# ROV Sediment Collection – ROS 2

## Project Overview

This project demonstrates a sediment collection system for an underwater Remotely Operated Vehicle (ROV) using ROS 2 and C++

The system controls the sediment collection process using a ROS 2 Action Server and Lifecycle Node. A simulated hardware node provides sensor data and allows the collection sequence to be tested without a physical ROV

## Main Components

- Controller: Manages the sediment collection sequence using a ROS 2 Lifecycle Node and Action Server
- Simulator: Simulates ROV hardware and publishes sensor data
- Action Interface: Defines the collection goal, feedback, and result
- Cancel Client: Demonstrates cancellation of an ongoing collection action

## Collection Sequence

1. Sensor and pressure checks
2. Deploy collection arm
3. Start pump and open valve
4. Monitor collected sediment volume
5. Stop pump and close valve
6. Retract collection arm
7. Return collection result

## Safety Features

- Pressure threshold monitoring.
- Sensor health monitoring.
- Collection timeout.
- Action cancellation.
- Safe-stop commands when a fault occurs.

## Run

1)Build the package

2)Start the simulator:

ros2 run rov_control simulator


3)Start the controller in another terminal:

ros2 run rov_control controller


4)Configure and activate the controller:

ros2 lifecycle set /controller configure
ros2 lifecycle set /controller activate

5)Send a collection request:

ros2 action send_goal /collect_sediment \
rov_interface/action/CollectSediment \
"{collection_duration: 10.0, target_volume: 50.0}" --feedback

