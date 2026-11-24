# OmniMow Autonomous Lawn Mower

# Status & Pipelines
[![Status: ALPHA](https://img.shields.io/badge/Status-ALPHA-red.svg)](#) [![Build OmniMow Containers](https://github.com/Ragsie/OmniMow/actions/workflows/docker_build.yml/badge.svg)](https://github.com/Ragsie/OmniMow/actions/workflows/docker_build.yml) [![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy-blue?logo=ros)](https://docs.ros.org/en/jazzy/index.html) [![Wiki Docs](https://img.shields.io/badge/Docs-Wiki-blue?logo=github)](https://github.com/Ragsie/OmniMow/wiki) [![OmniMow Code Quality and Static Analysis](https://github.com/Ragsie/OmniMow/actions/workflows/code_check.yml/badge.svg)](https://github.com/Ragsie/OmniMow/actions/workflows/code_check.yml)

# Packaged Containers (GHCR)
[![GHCR Stereo Vision](https://img.shields.io/badge/GHCR-stereo__vision-blue?logo=github&logoColor=white)](https://github.com/Ragsie/OmniMow/pkgs/container/omnimow%2Fstereo_vision) [![GHCR Depth Fusion](https://img.shields.io/badge/GHCR-depth__fusion-blue?logo=github&logoColor=white)](https://github.com/Ragsie/OmniMow/pkgs/container/omnimow%2Fdepth_fusion) [![GHCR Docking Control](https://img.shields.io/badge/GHCR-docking__control-blue?logo=github&logoColor=white)](https://github.com/Ragsie/OmniMow/pkgs/container/omnimow%2Fdocking_control) [![GHCR App Backend](https://img.shields.io/badge/GHCR-app__backend-blue?logo=github&logoColor=white)](https://github.com/Ragsie/OmniMow/pkgs/container/omnimow%2Fapp_backend) [![GHCR ROSBridge](https://img.shields.io/badge/GHCR-rosbridge__websocket-blue?logo=github&logoColor=white)](https://github.com/Ragsie/OmniMow/pkgs/container/omnimow%2Frosbridge_websocket)

### 🚧 ALPHA software and hardware setup 🚧
* Damage to your equipment may occur.
* Always test with the blade removed, and keep a hand on the emergency stop.
* Do not rely on the AI to see people or pets: keep them away while it mows.

# OmniMow
Turning my old Landroid mower into a smart autonomous robot with OmniMow AI.

### The Companion App [![OmniMow Android & iOS Build](https://github.com/Ragsie/OmniMow_App/actions/workflows/main.yml/badge.svg)](https://github.com/Ragsie/OmniMow_App/actions/workflows/main.yml)
Control and monitor the mower in real time with the official Flutter companion app.
**[Get the OmniMow App here](https://github.com/Ragsie/OmniMow_App)**

### Why
This project started from frustration: my current Worx M600 Plus has a habit of digging holes in the lawn instead of cutting the grass. Rather than simply buying a new mower, I decided to build my own and learn a new tech stack in the process. While there are many open-source DIY mower projects available, my goal is to build this from the ground up so I can deeply understand the mechanics and software behind it. I want to learn by doing, not just by copying someone else's setup.

I’ll document the whole journey here. If you have a mower that is tearing up your lawn too, I hope this project can serve as inspiration for your own build.

### OmniMow Architecture
A lawn mower that drives itself, built on a Worx Landroid chassis. The original electronics are replaced by ROS 2 Jazzy (Ubuntu 24.04), VESC motor controllers and a stereo camera with a YOLO26n-seg model retrained on my own dataset.

#### Features
* **ROS 2 Nav2:** Dynamic path planning and obstacle avoidance.
* **AI Vision:** The segmentation model finds the lawn and marks dogs, toys, animal waste, trees and walls as obstacles for Nav2. When it sees a dog (or stops running), the blade stops, and while mowing the mower waits. It does not detect people yet.
* **Depth Vision & Visual SLAM:** A calibrated stereo camera (auto-calibration on first boot) provides depth, obstacle detection and RTAB-Map localization against landmarks in the garden - no GPS required.
* **Safety:** The bumper, a tilt check (more than 35 degrees) and a heartbeat watchdog (500 ms without a heartbeat from app_backend, once the first one has arrived) make the drive ESP32 cut the power to the drive motors with a relay. The relay does not switch off the blade yet: after a bumper hit or a tilt the mowing software stops the blade when the drive ESP32 reports it, up to about a second later. The blade ESP32 has its own heartbeat watchdog.
* **Collision monitor:** Nav2 slows down or stops the wheels for obstacles the stereo camera or the AI sees in front of the mower. It does not stop the blade, and docking and undocking drive without it.
* **VESC Motor Control:** The drive wheels and the blade run on VESCs over CAN bus.
* **Micro-ROS:** Connects the main computer to the two ESP32s (drive and blade) over USB serial.
* **100% Dockerized:** ROS 2, the AI and the sensor nodes all run in Docker containers, one per part.
* **AI Updates:** Regular AI model updates with automatic download and setup (see [wiki](https://github.com/Ragsie/OmniMow/wiki)).

---

## Documentation (Wiki)
To keep this repository clean and easy to navigate, all detailed documentation, installation guides, parts lists, and diagrams have been moved to the **Project Wiki**.

**[Click here to read the full Wiki Docs](https://github.com/Ragsie/OmniMow/wiki)**

---

# License, Credits & Inspiration

This project builds on a lot of great open-source work, but the hardware is done differently.

### Hardware Philosophy: 100% Off-The-Shelf (DIY)
While the original OpenMower project by Clemens Elflein is primarily designed to run on custom proprietary replacement motherboards, **this AI project is built 100% on standardized, off-the-shelf (OTS) components**.

The plan is to strip all the original electronics from a Worx lawn mower chassis and rebuild it completely from scratch using:
* **Radxa Dragon Q6A** (main computer with hardware NPU acceleration)
* **ESP32** & **Autoro VESC 6.7 ESCs** (drive and cutter motor control)
* **8MP IMX219 Binocular Camera** (AI vision, obstacle detection and visual SLAM localization - no GPS needed)

Everything can be bought off the shelf, so no special boards are needed.

### Acknowledgement of Software & Concepts
Although the hardware architecture uses off-the-shelf components, I would like to send a huge thank you to the projects that inspired us and provided the software building blocks we use under the hood:

* **[OpenMower](https://github.com/ClemensElflein/OpenMower) by Clemens Elflein:** For pioneering the concept of converting Worx chassis into RTK-guided ROS robots (licensed under GPL-3.0)
* **[YOLO26n-seg by Ultralytics](https://github.com/ultralytics/ultralytics):** The segmentation model that runs on the NPU (licensed under AGPL-3.0)
* **[ROS 2 (Robot Operating System)](https://www.ros.org/):** The framework all the nodes run on and talk through (licensed under Apache 2.0)
* **[micro-ROS](https://micro.ros.org/):** For running ROS 2 on the ESP32s (licensed under Apache 2.0)

---

## Support The Project
If this project helped you or inspired your own build, consider buying me a cup of coffee. It would mean a lot and would support me in developing more.

Please note that this project is, and will always remain, **100% free and open-source** under the **GNU GPLv3 License**, in accordance with the licenses of our upstream dependencies.
