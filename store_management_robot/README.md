
AI-Powered Retail Robot for Inventory Management and Customer Assistance

Project Summary

This project aims to revolutionize retail store operations by developing an innovative AI-powered robotic system for real-time inventory management and price label verification. This approach overcomes traditional inventory management system limitations, such as human error, lack of real-time data, and inefficiency. Furthermore, it enhances customer satisfaction by providing up-to-date pricing information and precise product location guidance.

The primary goal of this project is to build a comprehensive system that automates inventory management in a physical store and aids customers in finding products. Leveraging the power of the Robot Operating System (ROS), computer vision techniques, the robot will be programmed to move along store aisles, capture images of each shelf, and integrate with the existing inventory system via an API.

Introduction

The AI-Powered Retail Robot assists both store staff and customers, functioning as a tool for inventory management and customer service. It has two primary functionalities:

Product Status: The robot patrols aisles, detects products on shelves using computer vision, and logs their counts and locations to a database. This provides real-time inventory insights to the store staff.
Product Finder: For customers, the robot listens to product requests, fetches product locations from the database, calculates the shortest path, and navigates to the product location.
Dependencies

ROS (Robot Operating System)
ROS Packages: std_msgs, geometry_msgs, nav_msgs, actionlib_msgs
YOLOv5 for ROS: For computer vision-based product detection.
Pandas: To read from and write to CSV files.
Odoo Database: To store and fetch product details.
xmlrpc.client: For XML-RPC communication with Odoo.
Product Status

The robot uses computer vision techniques and YOLOv5 to detect and count products on the shelves:

Product Detection: Employs trained models to detect products in its camera's frame.
Debouncing: To ensure genuine product detection, it counts a product only when detected in multiple consecutive frames, filtering out momentary false positives.
Logging to Database: Detected products, their counts, and locations are recorded in the Odoo database.
Product Finder

Upon a customer's request:

The robot fetches product locations from the Odoo database.
Utilizes ROS services to determine the shortest path.
Navigates to the product location.
Additional Scripts

Image Saver (image_saver.py): This script saves images captured by the robot's camera, which can be used to create a dataset for model training.
Product Debounce (productdebounce.py): The ProductDebounce class is used to ensure that a product detection is consistent over several frames. This prevents erroneous triggers and ensures that a product is genuinely present before counting it.
Database Automation

A separate script allows for the automatic updating of the Odoo database with product statuses, removing the need for manual data entries.

Robot Navigation and Launch

The robot uses the store_management_robot ROS package, which includes two main Python scripts: product_status_location.py and product_finder.py, along with the additional scripts mentioned above. The launch file for this package is set up as follows:

Launch the rosbot_bringup package for robot setup.
Launch the yolov5_ros package for object detection.
Launch the product_status_location.py node for product detection and logging.
Launch the product_finder.py node for product location navigation.
Load parameters from a yaml configuration file.
Conclusion

This robot serves as a one-stop solution for both inventory management and customer service, ensuring efficiency and a better shopping experience. By reducing stockouts and overstocking incidents, improving product detection, and enhancing shelf status analysis, the AI-powered robotic system brings significant improvements to retail operations and customer experience.
