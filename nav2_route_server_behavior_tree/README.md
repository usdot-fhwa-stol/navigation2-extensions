# `nav2_route_server_behavior_tree`

## Introduction

This package contains custom Behavior Tree nodes that supplement the standard Navigation 2 behavior tree package. Because the standard Navigation 2 package does not currently include a way to interface with the Route Server, these custom action nodes bridge that gap.

If you are new to these concepts, please review the [Key Terms](https://t3.chat/chat/28e1572d-0d00-4856-b695-66bdfaa548cc#key-terms) section at the bottom of this document.

For more detailed information on how Behavior Trees integrate with Navigation 2, you can review the official documentation on [Nav2 Behavior Trees](https://docs.nav2.org/behavior_trees/index.html) and [Writing a New Behavior Tree Plugin](https://docs.nav2.org/plugin_tutorials/docs/writing_new_bt_plugin.html).

## Action Nodes Included

This package provides two specific Action nodes written in C++ that allow the navigator to request and process routes.

### 1. ComputeRoute

This node sends a standard action request to the Route Server. It provides the starting location and destination goal. Once the Route Server finishes calculating the route, this node takes the resulting path and stores it in a Behavior Tree Blackboard variable. Other nodes in the system can then read this variable to know where the vehicle should drive.

### 2. ComputeAndTrackRoute

This node sends a continuous action request to the Route Server. Instead of waiting until the end of a calculation to output a path, it listens for live feedback messages from the Route Server. As the Route Server generates and tracks the dense path, this node continuously updates the path variable on the Behavior Tree Blackboard in real time.

## Key Terms

- **Action:** A type of ROS 2 communication used for long running tasks. It consists of a goal request, continuous feedback during the task, and a final result.
- **Behavior Tree:** A mathematical model used to describe how a robot plans and sequences its actions.
- **Blackboard:** A shared memory space used by the Behavior Tree. It allows different nodes to store and share data variables, such as a calculated driving path.
- **Route Server:** A custom module that plans routes along well structured graphs like road networks, rather than through open space.
