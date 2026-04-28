# nav2_bsm_generator

## Introduction

This package provides a specialized node for creating and broadcasting Basic Safety Messages. It collects internal vehicle data like position, velocity, and orientation, and formats it into a standardized message. This allows the vehicle to communicate its status to other connected vehicles and infrastructure in the Cooperative Driving Automation system.

If you are new to these concepts, please review the [Key Terms](https://t3.chat/chat/28e1572d-0d00-4856-b695-66bdfaa548cc#key-terms) section at the bottom of this document.

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

- **BSM (Basic Safety Message):** A standardized broadcast message containing vehicle position, size, speed, and heading. It is the primary data packet used in connected vehicle safety applications.
- **V2X:** Vehicle to Everything. A communication framework that allows vehicles to share information with other vehicles, roadside infrastructure, and pedestrians.
- **SAE J2735:** The industry standard that defines the specific format and data dictionary for V2X messages like the Basic Safety Message.
- **ID Rotation:** A privacy feature that periodically changes the unique identifier of the vehicle in broadcast messages. This prevents external observers from identifying or tracking a specific vehicle over a long duration.
- **SecMark:** A timestamp used in Basic Safety Messages that represents the current millisecond within the minute.
- **Lifecycle Node:** A software node that utilizes a state machine to provide greater control over the startup and shutdown sequences of the software.
