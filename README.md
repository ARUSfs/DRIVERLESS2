# DRIVERLESS2

![image](https://github.com/user-attachments/assets/e899bc0a-869d-4f2d-bb1c-81bb53f521bd)

## Table of Contents

1. [Introduction](#introduction)
2. [Manual installation](#manual-installation)
3. [Repository node structure](#repository-node-structure)
4. [Principal launch files](#principal-launch-files)

---

## Introduction

This guide provides comprehensive details on how to install and configure the DRIVERLESS2 enviroment, along with an overview of its repository node structure and launch file organization.

---

## Manual installation

### Prerequisites

- Ensure that you have **Ubuntu 22.04 LTS** correctly installed.
- Verify that git is installed in your environment. You can check this by running:

    ```bash
    git --version
    ```

### Step-by-step installation

1. **Create your workspace environment:**

    Create a workspace directory for your ROS packages. You can do this by running:

    ```bash
    mkdir -p /ws/src
    ```

2. **Clone the repository:**

    ```bash
    cd /ws/src
    git clone git@github.com:ARUSfs/DRIVERLESS2
    ```

    Also, clone the following auxiliary repository:

    ```bash
    git clone git@github.com:ARUSfs/ARUSSim
    ```

    This repository contains the simulation environment and additional tools required for the project.

3. **ROS2 Humble installation (optional):**

    Ensure you have ROS2 Humble installed by the following command:

    ```bash
    ls ~/opt/ros/
    ```

    If you don't see `humble` in the list, you can install it by selecting the first option from the `update_dependencies.sh` script:

    ```bash
    cd /ws/src/DRIVERLESS2
    # Select option 1
    ./update_dependencies.sh
    ```

    This script will automatically install the necessary for ROS2 Humble.

    If you already have ROS2 Humble installed, please make sure to have the following command line in your `~/.bashrc` file:

    ```bash
    source /opt/ros/humble/setup.bash
    ```

    If you don't have it, add it by running:

    ```bash
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    ```

4. **Install dependencies:**

    ```bash
    cd /ws/src/DRIVERLESS2
    # Select option 2
    ./update_dependencies.sh
    ```

    This script will automatically install all the dependencies required for the project.

5. **Setup and build the workspace:**

    ```bash
    # Setup the workspace
    echo "source ~/ws/install/setup.bash" >> ~/.bashrc
    source ~/.bashrc

    # Build the workspace
    cd /ws
    colcon build
    ```

    This command will compile all the packages in your workspace and create the necessary build files.

---

## Repository node structure

The repository is divided into several packages, each addressing a specific functionality to maintain modularity and scalability in the autonomous car system. The packages include:

- **car_state**: manages vehicle sensor data and state estimation.
- **common**: provides shared utilities and all launch files for the project.
- **control**: implements algorithms and controllers responsible for vehicle maneuvering.
- **interfaces**: facilitates communication between different system components, such as CAN, EPOS4 and simulation interfaces.
- **perception**: processes sensor inputs for LiDAR data.
- **planning**: contains path planning and decision-making algorithms for navigation.
- **slam**: offers simultaneous localization and mapping (SLAM) capabilities.
- **visualization**: provides tools for visualizing and debugging system states.

Additionally, each package follows a standard structure to maintain consistency:

- **config/**: contains configuration files and parameters, if needed (<package_name>_config.yaml).
- **include/<package_name>**: holds header files (.h and .hpp) used by the package.
- **launch/**: includes the launch file (<package_name>_launch.py) for starting the package nodes.
- **src/**: contains the core code files (<package_name>_node.cpp).
- **CMakeLists.txt**: the CMake build configuration file for the package.
- **package.xml**: the package manifest file that defines the package's metadata and dependencies.

### Principal launch files

The main launch files are located in the `common/common_meta/launch` directory. These files are responsible for starting the various nodes and components of the system. The main launch files include:

- **simulation_launch.py**: launches the simulation environment for testing and development.
- **inspection_launch.py**: launches the inspection controller for the inspection event.
- **acceleration_launch.py**: launches all necessary nodes for the acceleration event.
- **skidpad_launch.py**: launches all necessary nodes for the skidpad event.
- **autocross_launch.py**: launches all necessary nodes for the autocross event.
- **trackdrive_launch.py**: launches all necessary nodes for the trackdrive event.
- **record_launch.py**: launches the rslidar and can_interface nodes for recording data.
