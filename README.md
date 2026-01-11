Autonomous Retail Intelligence Robot (ARIR)

AI-Driven Inventory Management & Customer Assistance Using ROS and Computer Vision

Overview

Autonomous Retail Intelligence Robot (ARIR) is a research-grade AI-enabled robotic system developed as part of an MSc Robotics dissertation. The system autonomously navigates a retail environment, detects and tracks products on shelves using deep learning–based computer vision, updates inventory records in real time via ERP integration, and assists customers by guiding them to product locations on request.

The project focuses on system-level robotics engineering, combining perception, navigation, decision-making, and backend integration into a modular and scalable architecture suitable for real-world retail automation.

Key Capabilities

Autonomous navigation and SLAM using ROS

Real-time product detection with YOLOv4-Tiny and YOLOv5

Multi-object tracking with DeepSORT

Product debouncing and history buffering to prevent over-counting

Real-time inventory updates via ERP (Odoo XML-RPC)

Customer-initiated product search and robot-guided assistance

Modular ROS-based architecture designed for extensibility

System Architecture

The system is structured as loosely coupled ROS nodes to ensure robustness, fault isolation, and scalability:

Perception Layer

Camera input

YOLO-based object detection

DeepSORT multi-object tracking

Product debouncing & history buffer

Robotics Layer

ROS Noetic middleware

SLAM (gmapping)

Navigation stack (move_base)

Obstacle avoidance & path planning

Data & Integration Layer

Inventory state aggregation

CSV persistence

ERP integration via Odoo XML-RPC API

Customer Interaction Layer

Product request handling

Shortest-path computation

Robot-guided navigation to product location

Technology Stack

Languages: Python

Robotics: ROS Noetic, Gazebo

Computer Vision: OpenCV, YOLOv4-Tiny, YOLOv5

Tracking: DeepSORT

Backend / Integration: Odoo ERP (XML-RPC), Pandas

Platform: ROSbot Husarion 2 Pro (simulated)

OS: Linux (Ubuntu)

Why This Project Matters

This project goes beyond model training or simulation demos. It demonstrates:

End-to-end robotics system design

Real-time AI deployment constraints

Robust perception in cluttered environments

Practical integration with enterprise inventory systems

Engineering tradeoffs between accuracy, latency, and reliability

Limitations & Future Work

Evaluation conducted in a simulated retail environment

Limited dataset diversity

Cloud-dependent model training

Potential latency from perception and ERP synchronization

Planned extensions include real-world deployment, dataset expansion, demand forecasting integration, and hardware-optimized inference.

+--------------------+
|   RGB / Depth Cam  |
+---------+----------+
          |
          v
+-----------------------------+
|  Perception Node (ROS)      |
|  - YOLOv5 Detection         |
|  - DeepSORT Tracking        |
|  - Product Debouncing       |
+-------------+---------------+
              |
              v
+-----------------------------+
| Inventory Logic Node        |
| - Product Status            |
| - Location Aggregation      |
| - CSV Persistence           |
+-------------+---------------+
              |
              v
+-----------------------------+
| ERP Integration Node        |
| - Odoo XML-RPC API          |
| - Real-time Updates         |
+-----------------------------+

+-----------------------------+
| Navigation Stack (ROS)      |
| - SLAM (gmapping)           |
| - Path Planning             |
| - Obstacle Avoidance        |
+-------------+---------------+
              |
              v
+-----------------------------+
| Customer Assistance Node   |
| - Product Requests         |
| - Shortest Path Selection  |
| - Guided Navigation        |
+-----------------------------+
