## Introduction

This repository provides extensions and custom additions to the Navigation 2 framework for the CDA 1Tenth project. Navigation 2 is a motion planning and behavior planning framework for ROS 2. Because Navigation 2 is a complex system with several moving parts, this document provides an architectural overview to supplement the official documentation. If you are new to these concepts, please review the [Key Terms](#key-terms) section at the bottom of this document.

## How It Works

The fundamental process for navigators follows a three step loop.

1. **Generate a path:** The navigator requests a path from the active Planner Server plugin. This path goes from the current pose to the goal pose. The Planner Server independently maintains its understanding of the environment using a global costmap.
2. **Track the reference path:** The navigator sends the generated path to the active Controller Server plugin via a ROS 2 action. The Controller Server independently maintains a localized view of the immediate environment using a local costmap.
3. **React to operating changes:** The navigator monitors progress and can preemptively request a new path or cancel a control effort. If actions fail, the planner or controller plugins report back to the navigator, which then orchestrates the appropriate recovery steps.

## Behavior Trees

A behavior tree is a mathematical model used to describe how a robot plans its actions. In this model, leaf nodes represent distinct executable actions. Inner nodes provide the logic that controls which of these actions the robot will execute.

Navigation 2 relies on a lightweight C++ task orchestrator to decide which actions to take and dispatches the actual computation to separate threads. Navigation 2 extends this library with custom nodes that provide a ROS 2 compatible interface, allowing the behavior tree to dispatch work natively.

## Core System Servers

Navigation 2 is modular and divides its responsibilities across several specialized servers.

### 1. Behavior Tree Navigator Server

This server houses the navigators. Navigation 2 includes two default navigators that load their logic from customizable XML files. You can also provide custom navigators.

**XML Configuration Example:**

```
<root BTCPP_format="4" main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <PipelineSequence name="NavigateWithReplanning">
      <ControllerSelector selected_controller="{selected_controller}" default_controller="FollowPath" topic_name="controller_selector"/>
      <PlannerSelector selected_planner="{selected_planner}" default_planner="GridBased" topic_name="planner_selector"/>
      <DistanceController distance="1.0">
        <ComputePathToPose goal="{goal}" path="{path}" planner_id="{selected_planner}" error_code_id="{compute_path_error_code}"/>
      </DistanceController>
      <FollowPath path="{path}" controller_id="{selected_controller}" error_code_id="{follow_path_error_code}"/>
    </PipelineSequence>
  </BehaviorTree>
</root>
```

Based on this specification, the navigator will:

- **Activate the Controller:** Make a service call to the Controller Server to load the specified controller plugin.
- **Activate the Planner:** Make a service call to the Planner Server to load the specified planner plugin.
- **Generate a Path:** Make an action call to the Planner Server to compute a reference path. Because of the DistanceController node, it will request a new path whenever the robot moves more than one meter from the goal pose.
- **Follow the Path:** Make an action call to the Controller Server to track the generated reference path.

The navigator repeats this pipeline until the robot successfully reaches its goal. The navigator lives inside this server and only dictates when to send action requests. The actual planning and tracking math is handled by plugins inside the Planning and Control Servers.

### 2. Planner Server

This server manages path planner plugins. These plugins compute a free space path from the current pose to a goal pose. It exposes an action interface for path generation and relies on a global costmap to understand the broader environment.

### 3. Controller Server

This server manages controller plugins. These plugins command the hardware to follow the reference path provided by the Planner Server. It uses a local costmap to avoid immediate dynamic obstacles. Plugins output movement commands by publishing velocity messages.

Publishing standard velocity messages works perfectly for robots that can move in any direction instantly, but can cause issues for constrained platforms with Ackermann steering. There are two common workarounds.

- **Adapter Node:** Use an intermediary node to convert standard velocity messages into Ackermann drive messages. This cleanly documents the interface change.
- **Field Abuse:** Repurpose the existing fields inside the velocity message to carry the necessary vehicle specific data.

### 4. Behavior Server

This server houses various recovery behaviors. When path planning or tracking fails, navigators call upon this server to execute maneuvers like waiting, backing up, or calling for human assistance. The plugins here do not share a single unified interface because behaviors vary widely in function.

## Route Server

The Route Server is an experimental module that plans routes along well structured graphs like road networks rather than through open space. To use it, you provide a graph file that overlays your static map.

Open space planning from the Planner Server is still required for scenarios such as the last mile problem where the robot must navigate from its current off route location to the nearest starting point on the route graph. It is also required for maneuvering around temporary blockages on an otherwise defined route.

When utilizing the Route Server, a typical flow is:

1. **Generate Route:** Request a route from the Route Server.
2. **Bridge the Gap:** If the robot is too far from the start of the route, request a free space path from the Planner Server to reach the starting point.
3. **Track the Bridging Path:** Send the free space path to the Controller Server.
4. **Track the Main Route:** Send the main route path to the Controller Server.

The associated package has not yet been merged into the main Navigation 2 codebase. Its current integration is limited to demonstration scripts.

## Key Terms

- **Behavior Tree:** A mathematical model used to describe how a robot plans and sequences its actions.
- **Costmap:** A grid map that assigns risk values to specific areas to help the software understand where obstacles are located.
- **Navigator:** The primary decision maker that manages autonomy by calling on other specialized servers.
- **Pose:** The exact position and orientation of the robot in physical space.
- **Plugin:** A custom piece of software that can be loaded into a server to handle specific calculations or hardware commands.
- **Ackermann Steering:** A steering geometry used in cars where the front wheels turn to steer.
