# Hydromea EXRAY™ SDK
[Aboute EXRAY™](#about-exray™) | [About the SDK](#about-the-sdk) | [Getting started](#getting-started) | [Sample code](#sample-code) | [FAQ](docs/FAQ.md) | [Reporting an issue](#reporting-an-issue) | [License](#license)

<p align="left">
  <a href="https://www.hydromea.com">
    <img alt="The EXRAY KIT" src="https://files.hydromea.com/exray/exray_kit_transparent.png" width="500"/>
  </a>
</p>

## About EXRAY™
**EXRAY™** is a Swiss-made, professional-grade robot designed specifically for detailed inspections of submerged assets. Slim, agile, and highly modular, it’s an excellent tool for tough underwater inspection missions.

The EXRAY™ KIT consists of two identical and therefore interchangeable EXRAY™ ROVs that are docked to each other with a docking system and are connected wirelessly using a proprietary LUMA™ optical wireless system.
The primary EXRAY™ ROV is always tethered to the ground station controller. The secondary EXRAY™ FLYOUT ROV does not have a tether. When the ROVs are docked, the kit acts as one single ROV, using the power of thrusters on both vehicles.When the ROVs are undocked, LUMA™ uses blue light ensuring smooth data transfer between the two vehicles.

All product specifications can be found at [this link](https://files.hydromea.com/exray/EXRAY_Technical_Specifications.pdf)

## About the SDK
The EXRAY™ SDK offers a streamlined interface for integrating and controlling the EXRAY™ vehicle using ROS 2. It enables users to access sensor data (pressure, accelerometer, gyroscope, magnetometer, DVL, and camera) and manage control targets with real-time feedback for closed loop high frequency control.

### Key Features

- Access real-time inertial sensor data (accelerometer, gyroscope, magnetometer).
- Retrieve pressure sensor readings
- Read DVL (Doppler Velocity Log) output for velocity estimation.
- Stream camera output for visual feedback.
- Publish control targets for attitude and velocity using twist commands.
- Receive feedback on attitude and velocity to monitor EXRAY™'s state.

## Getting started

### Prerequisites
Before starting, ensure you have the following:
- A working ROS 2 setup (tested with Galactic & Humble distributions).
- Python or C++ development environment.
- Access to the EXRAY™ ROS 2 topic namespace.
- Basic knowledge of ROS 2 concepts (e.g., publishers, subscribers, nodes).

### Networking
EXRAY™ equips a [NVIDIA® Jetson Xavier™ NX](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-xavier-nx/) with Jetpack 5.1.2 booted from SSD.

The NVIDIA® Jetson has a statically assigned IP on the ethernet interface which is `192.168.102.{100 + EXRAY ID}`. For example, if the EXRAY™ ID is 22, then the IP address is `192.168.102.122`. The default username is `jetson` and the default password is `exraynx`.

To access the network, there are two possibilities:
- via the direct ethernet cable, connected to the green port on EXRAY™ (100 Mbps)
- via the ethernet port available on the Pilot Station (100 Mbps), when the EXRAY™ and the Pilot Station are connected via tether.

The NVIDIA® Jetson has a WiFi dongle and a Bluetooth dongle that can be used for connection to your local network.

The following IPs are reserved:

| Component | IP address |
|---------------|--------------|
| Pilot Station | `192.168.102.2` |
| Inspector Station | `192.168.102.3` |
| LUMA™ | `192.168.102.<EXRAY ID>` |
| Onboard controller | `192.168.102.<50 + EXRAY ID>` |
| NVIDIA® Jetson | `192.168.102.<100 + EXRAY ID>` |
| DVL | `192.168.102.251` |
| Scanning sonar | `192.168.102.250` |

For your development and installing additional packages, we suggest using Docker containers to avoid risks of compromising the installed OS and ensuring compatibility with the EXRAY™ onboard controller. By default, the Docker installation requires sudo privileges (default password: `exraynx`)

### Using LUMA™
LUMA™ provides wireless communication between EXRAYs™. As blue light is used for communication, the communication range and bandwidth is affected by external lights (in particular the Sun and blue LED sources), as well as water turbidity. Therefore, it is suggested to use LUMA™ in low-light environments.

LUMA™ does not allow the transits of packets in the IP range `192.168.102.240-254`.

## ROS Message definitions
The message definitions for communicating with EXRAY™ are available in `exray_ros_messages`.

## Examples
Examples in Python are available in the `examples` folder of this repository.

## Contributing
Pull requests to improve this repository are welcome and will be taken care promptly.

## Reporting an issue
Please report any issue by sending an email to [support@hydromea.com](mailto:support@hydromea.com).
If you need access to the EXRAY™ user manual, please contact Hydromea.

Github issues are disabled.

## License
This SDK is released under GPL v3 License. See [LICENSE](LICENSE) for more details.
