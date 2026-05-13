# `nav2_emergency_stop` package

## Introduction

This packages provides a node that allows an operator to safely stop the physical vehicle and shut down the active navigation software using a remote controller.

If you are new to these concepts, please review the [Key Terms](#key-terms) section at the bottom of this document.

## How It Works

The emergency stop node listens for input from a game controller over the standard joystick message topic. By default, it is configured to trigger when the center button on a Logitech Gamepad F710 is pressed.

When the kill switch is activated, the node performs three immediate actions to ensure the vehicle stops safely:

1. It loops through a provided list of Navigation 2 lifecycle nodes and transitions them from an active state into a deactivated and shutdown state.
2. It publishes an empty Ackermann drive command to the vehicle hardware, which overrides current movement and commands the vehicle to stop physically.
3. It shuts itself down.

## Configuration

This node requires a parameter named `node_names` to function correctly. This parameter accepts an array of strings representing the exact names of the Navigation 2 lifecycle nodes that you want the emergency stop sequence to target and shut down.

## Key Terms

- **Ackermann Steering:** A steering geometry used in cars where the front wheels turn to steer.
- **Lifecycle Node:** A specialized software node that can be managed through different states such as active, inactive, or shut down. This allows the system to start up and tear down processes in a controlled order rather than crashing unexpectedly.
