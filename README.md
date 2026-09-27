# 🚜 OmniMow Autonomous Lawn Mower

# Status & Pipelines
[![Status: ALPHA](https://img.shields.io/badge/Status-ALPHA-red.svg)](#) [![Build OmniMow Containers](https://github.com/Ragsie/OmniMow/actions/workflows/docker_build.yml/badge.svg)](https://github.com/Ragsie/OmniMow/actions/workflows/docker_build.yml) [![ROS 2](https://img.shields.io/badge/ROS_2-Humble-blue?logo=ros)](https://docs.ros.org/en/humble/index.html) [![Wiki Docs](https://img.shields.io/badge/Docs-Wiki-blue?logo=github)](https://github.com/Ragsie/OmniMow/wiki) [![OmniMow Code Quality and Static Analysis](https://github.com/Ragsie/OmniMow/actions/workflows/code_check.yml/badge.svg)](https://github.com/Ragsie/OmniMow/actions/workflows/code_check.yml)

# Packaged Containers (GHCR)
[![GHCR Stereo Vision](https://img.shields.io/badge/GHCR-stereo__vision-blue?logo=github&logoColor=white)](https://github.com/Ragsie/OmniMow/pkgs/container/omnimow%2Fstereo_vision) [![GHCR Depth Fusion](https://img.shields.io/badge/GHCR-depth__fusion-blue?logo=github&logoColor=white)](https://github.com/Ragsie/OmniMow/pkgs/container/omnimow%2Fdepth_fusion) [![GHCR Docking Control](https://img.shields.io/badge/GHCR-docking__control-blue?logo=github&logoColor=white)](https://github.com/Ragsie/OmniMow/pkgs/container/omnimow%2Fdocking_control) [![GHCR App Backend](https://img.shields.io/badge/GHCR-app__backend-blue?logo=github&logoColor=white)](https://github.com/Ragsie/OmniMow/pkgs/container/omnimow%2Fapp_backend) [![GHCR ROSBridge](https://img.shields.io/badge/GHCR-rosbridge__websocket-blue?logo=github&logoColor=white)](https://github.com/Ragsie/OmniMow/pkgs/container/omnimow%2Frosbridge_websocket)

### 🚧 ALPHA software and hardware setup 🚧
* Damage to your equipment may occur.

# OmniMow
Turning my old Landroid mower into a smart autonomous robot with OmniMow AI.

### The Companion App 📱 [![Build and Release OmniMow AI](https://github.com/Ragsie/OmniMow_App/actions/workflows/build.yml/badge.svg)](https://github.com/Ragsie/OmniMow_App/actions/workflows/build.yml)
Control and monitor the mower in real time with the official Flutter companion app.
👉 **[Get the OmniMow App here](https://github.com/Ragsie/OmniMow_App)**

### Why
This project started from frustration: my current Worx M600 Plus has a habit of digging holes in the lawn instead of cutting the grass. Rather than simply buying a new mower, I decided to build my own and learn a new tech stack in the process. While there are many open-source DIY mower projects available, my goal is to build this from the ground up so I can deeply understand the mechanics and software behind it. I want to learn by doing, not just by copying someone else's setup.

I’ll document the whole journey here. If you have a mower that is tearing up your lawn too, I hope this project can serve as inspiration for your own build.

### 🤖 OmniMow Architecture
An advanced, fully autonomous lawn mower built on a Worx Landroid chassis. This project upgrades the original hardware with a modern ROS 2 (Humble/Jazzy) stack, VESC motor controllers, and real-time depth and AI vision using a YOLO26n-seg model retrained on my own dataset for maximum safety.

#### 🌟 Features
* **ROS 2 Nav2:** Dynamic path planning and obstacle avoidance.
* **AI Vision:** Real-time object detection for humans, pets, and custom-trained objects such as toys or animal waste, triggering immediate emergency stops.
* **Depth Vision:** A stereo camera providing visual navigation data to the mower, working in tandem with AI vision.
* **VESC Motor Control:** Smooth and powerful control of the drive wheels over CAN bus.
* **Micro-ROS:** Seamless communication between the main computer and the ESP32 drive controllers.
* **100% Dockerized:** The entire brain of the system—ROS 2, AI, and sensor nodes—runs in isolated Docker containers for reliability and fast startup.
* **AI Updates:** Regular AI model updates with automatic download and setup (see [wiki](https://github.com/Ragsie/OmniMow/wiki)). The mower AI is a YOLO26n-seg model, retrained on my own dataset and converted for deployment.

---

## 📚 Documentation (Wiki)
To keep this repository clean and easy to navigate, all detailed documentation, installation guides, parts lists, and diagrams have been moved to the **Project Wiki**.

**👉 [Click here to read the full Wiki Docs](https://github.com/Ragsie/OmniMow/wiki)**

---

# License, Credits & Inspiration 🛠️

This project is built on the inspiration of the fantastic work of the open-source community, but with a completely unique hardware philosophy.

### 🔌 Hardware Philosophy: 100% Off-The-Shelf (DIY)
While the original OpenMower project by Clemens Elflein is primarily designed to run on custom proprietary replacement motherboards, **this AI project is built 100% on standardized, off-the-shelf (OTS) components**.

The plan is to strip all the original electronics from a Worx lawn mower chassis and rebuild it completely from scratch using:
* **Radxa Dragon Q6A** (main computer with hardware NPU acceleration)
* **ESP32** & **Autoro VESC 6.7 ESCs** (drive and cutter motor control)
* **Quectel LC29H** (budget-friendly, millimeter-precise RTK-GNSS)
* **8MP IMX219 Binocular Camera** (real-time AI vision and emergency stop)

This makes our hardware platform extremely affordable, highly accessible, and fully independent of proprietary hardware manufacturers.

### 🤝 Acknowledgement of Software & Concepts
Although the hardware architecture uses off-the-shelf components, I would like to send a huge thank you to the projects that inspired us and provided the software building blocks we use under the hood:

* **[OpenMower](https://github.com/ClemensElflein/OpenMower) by Clemens Elflein:** For pioneering the concept of converting Worx chassis into RTK-guided ROS robots (licensed under GPL-3.0)
* **[YOLO26n-seg by Ultralytics](https://github.com/ultralytics/ultralytics):** For the lightning-fast AI segmentation system running locally on the NPU (licensed under AGPL-3.0)
* **[ROS 2 (Robot Operating System)](https://www.ros.org/):** The robust middleware framework driving our entire messaging and node architecture (licensed under Apache 2.0)
* **[micro-ROS](https://micro.ros.org/):** For bringing ROS 2 seamlessly onto our ESP32 microcontroller (licensed under Apache 2.0)

---

## ☕ Support The Project
If this project helped you or inspired your own build, consider buying me a cup of coffee. It would mean a lot and would support me in developing more.

Please note that this project is, and will always remain, **100% free and open-source** under the **GNU GPLv3 License**, in accordance with the licenses of our upstream dependencies.
