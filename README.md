# Navigation 2 (Nav2) Extensions

## Introduction

This repository integrates Cooperative Driving Automation (CDA) functionality with the open source Navigation 2 library.

This release targets automated port drayage where a semi truck communicates with an infrastructure computer within a port to coordinate the pickup and drop off of cargo.

If you are new to this project or concepts, please review the [Key Terms](#key-terms) section at the bottom of this document.

## Prerequisites and Setup

You must download and build this repository within your primary workspace alongside the core Navigation 2 packages.

1. Clone this repository into the source directory of your workspace.
2. Download the navigation2 [nav_route_server](https://github.com/usdot-fhwa-stol/navigation2/tree/nav2_route_server) branch and build `nav2_route` package, as this extension relies heavily on it for routing along defined road networks.
3. Build your workspace and source the setup file to ensure your system recognizes the new plugins and nodes.

## Related Repositories:

The [`cda1tenth_bringup`](https://github.com/usdot-fhwa-stol/cda1tenth-bringup/tree/develop) repository utilizes navigation 2 extensions for CDA in simulated and real environments. Instructions for building a physical vehicle can be found at [`cda1tenth_hardware`](https://github.com/usdot-fhwa-stol/cda1tenth-hardware/tree/develop).

## Key Terms
- **Cooperative Driving Automation:** Research focused on how automated vehicles can communicate with each other and infrastructure to improve safety and traffic flow.
- **Port Drayage:** The transport of goods over a short distance, typically moving shipping containers between a port and a nearby logistical facility.
- **Navigation2 (Nav2):** A navigation stack for mobile robots built on the ROS framework, [Nav2 Github](https://github.com/ros-navigation/navigation2).
- **Route Server:** A custom module that plans routes along well structured graphs like road networks, rather than through open space.
- **ROS 2:** Robot Operating System. The underlying framework that allows all the different parts of the vehicle to communicate. [Learn more about ROS here] (https://www.ros.org/)
- **Docker:** A platform that uses containerization to package software and its dependencies together so it runs reliably across different computing environments. [Learn more about Docker here](https://www.docker.com/)


## Contribution

Please read our [CARMA Contributing Guide](https://github.com/usdot-fhwa-stol/carma-platform/blob/develop/Contributing.md) to learn about our development process, how to propose pull requests and improvements, and how to build and test your changes to this project.

## Code of Conduct

Please read our [CARMA Code of Conduct](https://github.com/usdot-fhwa-stol/carma-platform/blob/develop/Code_of_Conduct.md) which outlines our expectations for participants within the CARMA community, as well as steps to reporting unacceptable behavior. We are committed to providing a welcoming and inspiring community for all and expect our code of conduct to be honored. Anyone who violates this code of conduct may be banned from the community.

## Attribution

The development team would like to acknowledge the people who have made direct contributions to the design and code in this repository. [CARMA Attribution](https://github.com/usdot-fhwa-stol/carma-platform/blob/develop/ATTRIBUTION.txt)

## License

By contributing to the Federal Highway Administration (FHWA) Connected Automated Research Mobility Applications (CARMA), you agree that your contributions will be licensed under its Apache License 2.0 license. [CARMA License](https://github.com/usdot-fhwa-stol/carma-platform/blob/develop/docs/License.md)

## Contact

Please click on the CARMA logo below to visit the Federal Highway Administration (FHWA) CARMA website. For technical support from the CARMA team, please contact the CARMA help desk at [CAVSupportServices@dot.gov](mailto:CAVSupportServices@dot.gov).

[](https://highways.dot.gov/research/research-programs/operations/CARMA)
