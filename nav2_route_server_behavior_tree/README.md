# `nav2_route_server_behavior_tree`

## Introduction

This package provides two specific Action nodes that interface with the Route Server and allow the navigator server to request and process routes. The navigator server emulates road networks with those routes. The road networks are defined as directed graphs and can be created based on a given map. 

If you are new to these concepts, please review the [Key Terms](#key-terms) section at the bottom of this document.

For more detailed information on how Behavior Trees integrate with Navigation 2, you can review the official documentation on [Nav2 Behavior Trees](https://docs.nav2.org/behavior_trees/index.html) and [Writing a New Behavior Tree Plugin](https://docs.nav2.org/plugin_tutorials/docs/writing_new_bt_plugin.html).

## Action Nodes

### ComputeRoute

#### How it Works

This node sends a standard action request to the Route Server. It provides the starting location and destination goal. Once the Route Server finishes calculating the route, this node takes the resulting path and stores it in a Behavior Tree Blackboard variable. Other nodes in the system can then read this variable to know where the vehicle should drive.

#### Configuration

`xml_tag_name`: The XML tag representing this node in the Behavior Tree.
`action_name`: The name of the target ROS 2 Action Server.
`conf`: BT configuration object managing Blackboard port connections.

### ComputeAndTrackRoute

#### How it Works

This node sends a continuous action request to the Route Server. Instead of waiting until the end of a calculation to output a path, it listens for live feedback messages from the Route Server. As the Route Server generates and tracks the dense path, this node continuously updates the path variable on the Behavior Tree Blackboard in real time.

#### Configuration

`xml_tag_name`: The XML tag representing this node in the Behavior Tree.
`action_name`: The name of the target ROS 2 Action Server.
`conf`: BT configuration object managing Blackboard port connections.

## Key Terms

- **Behavior Tree:** A mathematical model used to describe how a robot plans and sequences its actions.
- **Action Node:** A type of node on a behavior tree that executes a command. Upon completion, these action nodes will return SUCCESS if the action server believes the action has been completed correctly, RUNNING when still running, and will return FAILURE otherwise.
- **Blackboard:** A shared memory space used by the Behavior Tree. It allows different nodes to store and share data variables, such as a calculated driving path.
