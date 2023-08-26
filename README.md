# AI-store-manager-project
Introduction
The AI-Powered Retail Robot is designed to assist store staff and customers, 
serving as a tool for inventory management and customer service. 
The robot has two primary functionalities:

Product Status: The robot patrols store aisles, uses computer vision to detect products on shelves, 
and logs their counts and locations in an Odoo database, 
offering real-time inventory insights to store staff.
Product Finder: Upon customer request, the robot listens to product queries, 
retrieves product locations from the database, calculates the shortest path, a
nd guides the customer to the product.

Dependencies

ROS (Robot Operating System)
ROS Packages: std_msgs, geometry_msgs, nav_msgs, actionlib_msgs
YOLOv5 for ROS: For computer vision-based product detection.
Pandas: To read from and write to CSV files.
Odoo Database: For product details storage and retrieval.
xmlrpc.client: For XML-RPC communication with Odoo.
DeepSort: For tracking and counting multiple objects.
History Buffer: For maintaining a record of recent detections to enhance accuracy.

Product Status
The robot uses computer vision techniques, YOLOv5, Deep MOT, and 
a history buffer to detect and count products on shelves:
Product Detection: Employs trained models to detect products in its camera frame.
Deep MOT: A multi-object tracking method that prevents overcounting.
History Buffer: Provides a record of recent detections to enhance accuracy.
Debouncing: To ensure genuine product detection, it counts a product only when detected in multiple consecutive frames, 
filtering out momentary false positives.
Logging to Database: Detected products, their counts, and locations are recorded in the Odoo database.

Product Finder
Upon a customer's request:
The robot fetches product locations from the Odoo database.
Utilizes ROS services to calculate the shortest path.
Navigates to the product location.
Additional Scripts

Image Saver (image_saver.py): This script saves images captured by the robot's camera,
which can be used to create a dataset for model training.
Product Debounce (productdebounce.py): The ProductDebounce class ensures that a product detection is consistent over multiple frames. 
This prevents erroneous triggers and confirms a product's presence before counting.
Database Automation
An additional script automates the updating of the Odoo database with product statuses, eliminating manual data entries.

Robot Navigation and Launch
The robot uses the store_management_robot ROS package, which includes two main Python scripts: 
product_status_location.py and product_finder.py, along with the additional scripts mentioned above.
The launch file for this package is set up as follows:

Launch the rosbot_bringup package for robot setup.
Launch the yolov5_ros package for object detection.
Launch the product_status_location.py node for product detection and logging.
Launch the product_finder.py node for product location navigation.
Load parameters from a yaml configuration file.

