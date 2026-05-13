# nav2_bsm_generator

## Introduction

This package provides a specialized node for creating and broadcasting Basic Safety Messages to connected vehicles and instructure in the CDA system. It collects various vehicle data topics in the CARMA System like position, velocity, and orientation, and formats it into a standardized message. The node is based off of the [bsm_generator](https://github.com/usdot-fhwa-stol/carma-platform/tree/develop/bsm_generator) node implementation in CARMA Platform.

If you are new to these concepts, please review the [Key Terms](#key-terms) section at the bottom of this document.

## How It Works

The generator node runs on a timer based on a configured frequency. During each cycle, it reads the latest data from the vehicle sensors and navigation software. Specifically, it gathers the vehicle position, speed, and turning rate. It then packages this data into a Basic Safety Message and publishes it for the communication hardware to broadcast.

To protect privacy, the node can optionally rotate the vehicle identification number at set intervals so the vehicle cannot be easily tracked by outside observers over long periods of time.

## Configuration

You can customize the behavior of the generator using several parameters.

- **`bsm_generation_frequency`:** How often the message is created and broadcast.
- **`bsm_id_rotation_enabled`:** A true or false setting that turns the privacy identification rotation on or off.
- **`bsm_id_change_period`:** How often the identification number changes if rotation is enabled.
- **`bsm_message_id`:** A static identification number used if rotation is disabled.
- **`vehicle_length`:** The physical length of the vehicle included in the broadcast.
- **`vehicle_width`:** The physical width of the vehicle included in the broadcast.

## Key Terms

- **CARMA Platform:** CARMA Platform provides the navigation and guidance functions for its host vehicle, as well as some of the control functions. It depends on low level controller hardware to provide the rest of the control function. The current version of CARMA Platform provides SAE level 3+ autonomy, with both speed and steering control.
- **BSM (Basic Safety Message):** A standardized broadcast message containing vehicle position, size, speed, and heading.
- **V2X:** Vehicle to Everything. A communication framework that allows vehicles to share information with other vehicles, roadside infrastructure, and pedestrians.
- **SAE J2735:** The industry standard that defines the specific format and data dictionary for V2X messages.
- **ID Rotation:** A privacy feature that periodically changes the unique identifier of the vehicle in broadcast messages. This prevents external observers from identifying or tracking a specific vehicle over a long duration.
- **SecMark:** A timestamp used in Basic Safety Messages that represents the current millisecond within the minute.
- **Lifecycle Node:** A software node that utilizes a state machine to provide greater control over the startup and shutdown sequences of the software.
