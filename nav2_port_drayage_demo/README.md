# `nav2_port_drayage_demo` package

## Introduction

This package integrates port drayage operations with a Navigation 2 autonomy stack. The software node listens for incoming port drayage messages, calculates a route, and sends the target path to the vehicle controller via an action call.

If you are new to these concepts, please review the [Key Terms](https://t3.chat/chat/28e1572d-0d00-4856-b695-66bdfaa548cc#key-terms) section at the bottom of this document.

## How It Works

Based on the underlying source code, the node operates through a continuous communication and execution loop.

1. **Receive Instruction:** It listens for incoming instructions on the mobility operation topic.
2. **Validate:** It verifies the message is intended for the specific vehicle by checking the configured commercial vehicle identification parameter. It will ignore messages intended for other vehicles.
3. **Plan:** It extracts the destination coordinates from the message payload and requests a route to the destination from the Navigation 2 system.
4. **Execute:** Once a path is generated, it sends the path to the Navigation 2 controller using a follow path action. This moves the physical or simulated vehicle.
5. **Confirm:** When the vehicle successfully reaches the destination, the node publishes an outgoing mobility operation message to confirm arrival and update its cargo status.

## Configuration

You can customize the behavior of the node using the following parameters.

- **`cmv_id`:** A string representing the unique commercial motor vehicle identification number. The node uses this to filter incoming messages.
- **`message_processing_delay`:** An integer defining the number of seconds to wait before executing a received instruction.

## Sending Mobility Operation Messages

The node listens for instructions using the `carma_v2x_msgs/msg/MobilityOperation` message type. For this demonstration, the only necessary fields to populate when sending instructions to the vehicle are the strategy and strategy parameters.

The strategy field must always be set to `carma/port_drayage`. The strategy parameters field must be set to a JSON string with the following format:

json

```
{
  "cmv_id": "DOT-80550",
  "operation": "ENTERING_STAGING_AREA",
  "cargo": false,
  "cargo_id": "SOME_CARGO",
  "location": {
    "longitude": 0,
    "latitude": 0
  },
  "destination": {
    "longitude": 0,
    "latitude": 0
  },
  "action_id": "SOMEUID"
}
```

To send an instruction for the vehicle to pick up cargo using the command line interface, run the following command in your terminal:

```
ros2 topic pub --once /incoming_mobility_operation carma_v2x_msgs/msg/MobilityOperation "m_header:
  sender_id: ''
  recipient_id: ''
  sender_bsm_id: ''
  plan_id: ''
  timestamp: 0
strategy: 'carma/port_drayage'
strategy_params: '{\"cmv_id\":\"turtlebot\",\"operation\":\"PICKUP\",\"cargo\":false,\"cargo_id\":\"CARGO_A\",\"destination\":{\"longitude\":3.8,\"latitude\":0.5},\"action_id\":\"PORT_DRAYAGE\"}'"
```

## Simulation Launch Instructions

To run the demonstration using the virtual vehicle in the Gazebo simulator, follow these steps.

1. **Install Prerequisites:** Follow the setup instructions in the `cda1tenth_bringup` repository to download and build the necessary software. Then, install the required simulator packages:

```
sudo apt install ros-humble-turtlebot3*
```

1. **Launch the Simulator:** Open a terminal and start the virtual world:

```
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

1. **Launch the Vehicle Software:** Open a second terminal and start the bringup package:

```
ros2 launch cda1tenth_bringup cda1tenth_bringup_launch.xml vehicle:=turtlebot
```

1. **Set the Initial Location:** Open the RViz visualizer and provide the vehicle with an initial location estimate using the 2D Pose Estimate tool. Wait for the vehicle sensor data to appear on the screen.
2. **Run the Demonstration Script:** Open a third terminal and execute the test script to start the scenario:

```
cd nav2_port_drayage_demo/
./test/turtlebot_port_drayage_test.sh
```

This script publishes a series of messages that instruct the vehicle to navigate to the top of the map to pick up cargo and then navigate to the bottom of the map to drop it off. After each successful operation, the vehicle will publish an outgoing message acknowledging that it completed the desired action.

## Key Terms

- **Port Drayage:** The transport of goods over a short distance, typically moving shipping containers between a port and a nearby logistical facility.
- **Mobility Operation:** A specific type of broadcast message used by connected vehicles to coordinate collaborative plans.
- **Waypoint Follower:** A Navigation 2 module that receives a list of specific coordinate locations and commands the vehicle to drive to each one in order.
