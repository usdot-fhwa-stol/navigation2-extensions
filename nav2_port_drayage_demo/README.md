# `nav2_port_drayage_demo` package

## Introduction

This package integrates port drayage operations with a Nav2 system. The software node listens for incoming port drayage messages, calculates a route, and sends the target path to the vehicle controller via an action call.

If you are new to these concepts, please review the [Key Terms](#key-terms) section at the bottom of this document.

## How It Works

The node operates through a continuous communication and execution loop.

1. **Receive Instruction:** It listens for incoming instructions on the mobility operation topic.
2. **Validate:** It verifies the message is intended for the specific vehicle by checking the configured commercial vehicle identification parameter.
3. **Plan:** It extracts the destination coordinates from the message payload and requests a route to the destination from the Nav2 stack.
4. **Execute:** Once a path is generated, it sends the path to the Nav2 controller using a follow path action. This moves the physical or simulated vehicle.
5. **Confirm:** When the vehicle successfully reaches the destination, the node publishes an outgoing mobility operation message to confirm arrival and update its cargo status.

## Configuration

You can customize the behavior of the node using the following parameters.

- **`cmv_id`:** A string representing the unique commercial motor vehicle identification number. The node uses this to filter incoming messages.
- **`message_processing_delay`:** An integer defining the number of seconds to wait before executing a received instruction.

## Sending Mobility Operation Messages

The node listens for instructions using the `carma_v2x_msgs/msg/MobilityOperation` message type. For this demonstration, the only necessary fields to populate when sending instructions to the vehicle are the strategy and strategy parameters.

The strategy field must always be set to `carma/port_drayage`. The strategy parameters field must be set to a JSON string with the following format:

```json
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

## Physical Vehicles Launch Instructions

To run the Port Drayage demo using the Turtlebot in the Gazebo simulator, first follow the cda1tenth-bringup repository physical vehicle instrucitons to download and build the necessary software.

Then, download and run the simulation environment using:
```bash
sudo apt install ros-humble-turtlebot3*
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

To launch the Port Drayage demo, run
```bash
ros2 launch cda1tenth_bringup cda1tenth_bringup_launch.xml vehicle:=turtlebot
```

Next, provide the Turtlebot with an initial pose estimate using the 2D Pose Estimate arrow in Rviz. Once the vehicle has localized, it is now ready to receive a Mobility Operation message to determine an action to complete. To start a simple demonstration of the Turtlebot picking up and dropping off cargo, run:
```bash
cd nav2_port_drayage_demo/
./test/turtlebot_port_drayage_test.sh
```

This will publish a series of Mobility Operation messages on /incoming_mobility_operation that instruct the Turtlebot to navigate to the top of the map to pickup cargo and then subsequently navigate to the bottom of the map to drop it off. After each successful operation, a Mobility Operation message acknowledging the Turtlebot completed the desired action will be published on /outgoing_mobility_operation.

## Simulation Launch Instructions

Assuming you have installed and setup the simulation in cda1tenth-brinup repository the below commands will launch it.

```bash
chmod +x launch.sh

# This should auto-detect your env settings
./launch.sh

# But you can manually change the gpu mode
./launch.sh --gpu
./launch.sh --no-gpu
```

1. **Set the Initial Location:** Open the RViz visualizer and provide the vehicle with an initial location estimate using the 2D Pose Estimate tool. Wait for the vehicle sensor data to appear on the screen.
2. **Run the Demonstration Script:** Open a third terminal and execute the test script to start the scenario:

```
docker exec -it cda_ws-cda1tenth-1 bash
cd ..
cd nav2_ext_ws/nav2_port_drayage_demo/
./test/turtlebot_port_drayage_test.sh
```

This script publishes a series of messages on ```/incoming_mobility_operation``` that instruct the vehicle to navigate to the top of the map to pick up cargo and then navigate to the bottom of the map to drop it off. After each successful operation, the vehicle will publish an outgoing message acknowledging on ```/outgoing_mobility_operation``` that it completed the desired action.

## Key Terms

- **Port Drayage:** The transport of goods over a short distance, typically moving shipping containers between a port and a nearby logistical facility.
- **Mobility Operation:** A specific type of broadcast message used by connected vehicles to coordinate collaborative plans.
- **Waypoint Follower:** A Navigation 2 module that receives a list of specific coordinate locations and commands the vehicle to drive to each one in order.
