---
name: "ros1-structure-guide"
description: "Standard project structure and refactoring guidelines for the ROS1 autonomous driving project. Invoke when creating packages, refactoring, or checking architectural compliance."
---

# ROS1 Project Structure Guide

This skill defines the standard architecture for the laboratory autonomous driving project.

## 1. Top-Level Architecture (Layered)

The workspace (`src/`) is organized into 5 layers:

1.  **Interface Layer (Stable)**
    *   `autodrive_msgs`: Unified messages/services.
    *   `autodrive_interfaces`: Pure C++ interfaces (no ROS).
        *   **MUST**: Contain only abstract interfaces / data structures / traits.
        *   **MAY**: Depend on `autodrive_common` or Eigen.
        *   **MUST NOT**: Depend on roscpp, message types (xxx_msgs), or param server.

2.  **Foundation Layer (Common)**
    *   `autodrive_common`: Utils, math, time, logging.

3.  **Capability Layer (Core Algorithms - No ROS)**
    *   `perception_core`, `localization_core`, `planning_core`, `control_core`.
    *   **MUST NOT**: Include `autodrive_msgs` or any ROS messages. Core only uses data structures from `autodrive_interfaces`.

4.  **Runtime Layer (ROS Adapters)**
    *   `perception_ros`, `localization_ros`, `planning_ros`, `control_ros`, `vehicle_interface_ros`.
    *   Responsible for converting ROS msgs <-> Core data structures.

5.  **System Layer (Bringup & Tools)**
    *   `autodrive_bringup`: Launch files, system config.
    *   `autodrive_description`: URDF, TF.
    *   `autodrive_viz`: RViz configs.
    *   `autodrive_sim`: Simulation environments (Gazebo/Carla wrappers) - keep separate from bringup.

## 2. Package Internal Structure Template

Every package must follow this layout:

*   `include/<package_name>/`: Public headers.
*   `src/`: Private implementation.
*   `src/nodelets/` (or `nodelets/`): Nodelet class implementations (Required for Nodelet packages).
*   `nodes/`: Lightweight ROS node entry points (main.cpp).
    *   **MAY**: Include a standalone debug node even for nodelet-based packages (optional).
*   `config/`: YAML parameters (no hardcoded params).
*   `launch/`: Launch files (argument passing only).
*   `test/`: 
    *   Unit tests (**gtest**) prioritize covering `*_core`.
    *   Integration tests (**rostest/launch**) for `*_ros`.
*   `plugins.xml`: **Required** when the package provides nodelets (pluginlib registration).

## 3. Engineering Conventions

*   **Topics**:
    *   Default to relative naming (not starting with `/`), managed by `ns` in launch files/groups.
    *   Prohibit global absolute topics like `/scan`, `/cmd_vel` without namespaces.
*   **Parameters**: Load from `config/*.yaml`. Use private node handles `~`. Core algorithms MUST NOT access param server directly; parameters are passed from ROS wrapper.
*   **Zero-copy (Nodelets)**:
    *   **MUST**: High-bandwidth modules (e.g., PointCloud, Image) with frequency >10Hz or large message size.
    *   **SHOULD**: Medium bandwidth modules where copy overhead is significant.
    *   **MAY**: Low frequency/small message modules (state machines, config) can use standard nodes.
*   **Build Targets**:
    *   `*_core`: Pure C++ library (0 ROS dependency).
    *   `*_ros`: ROS wrapper library / nodelet library (depends on roscpp).
    *   `*_node`: Executable entry point (argument parsing/glue only).
*   **Naming**:
    *   Packages: `snake_case`
    *   Nodes: `<pkg>_node`
    *   Classes: `PascalCase`
    *   Variables: `snake_case`

## 4. Refactoring & Development Workflow

*   **Refactoring**: Extract Core Logic -> Write ROS Wrapper -> Replace Old Node.
*   **New Module**: Define Interfaces/Msgs -> Implement Core -> Implement Wrapper -> Add Launch/Config.

Use this guide to validate or generate new components.
