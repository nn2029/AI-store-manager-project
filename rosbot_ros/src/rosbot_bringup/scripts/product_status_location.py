#!/usr/bin/env python3
import rospy
import csv
import tf
import tf2_ros
import tf2_geometry_msgs
import json
import numpy as np
import time
import threading
from collections import defaultdict, Counter
from cv_bridge import CvBridge
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from std_srvs.srv import Empty
from visualization_msgs.msg import Marker
from sensor_msgs.msg import Image, LaserScan, CameraInfo
from detection_msgs.msg import BoundingBox, BoundingBoxes
from geometry_msgs.msg import PoseWithCovarianceStamped, PointStamped, Point

# Initialize global data structures
# Shelf locations
shelves = rospy.get_param('shelves', {})

product_locations_publisher = None
detected_products = defaultdict(list)
camera_info = None
depth_image = None
tf_buffer = None
tf_listener = None
bridge = CvBridge()

last_seen = defaultdict(lambda: defaultdict(float))
product_confidences = defaultdict(lambda: defaultdict(list))
product_positions = defaultdict(lambda: defaultdict(list))
product_sizes = Counter()
segment_product_counts = defaultdict(lambda: Counter())
total_product_counts = Counter()

# Define your thresholds here
thresholds = rospy.get_param('thresholds', {"Out of Stock": 0, "Low Stock": 5, "In Stock": 10})
TIME_THRESHOLD = rospy.get_param('time_threshold', 2 * 60)

# Store robot position and orientation
robot_position = [0, 0]
robot_orientation = [0, 0, 0, 1]  # Quaternion (x, y, z, w), this represents 0 degree.

camera_info_lock = threading.Lock()
depth_image_lock = threading.Lock()


# Update robot's position and orientation based on amcl_pose topic
def update_robot_pose(msg):
    global robot_position, robot_orientation
    robot_position = [msg.pose.pose.position.x, msg.pose.pose.position.y]
    robot_orientation = [
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w,
    ]


def camera_info_cb(msg):
    global camera_info
    camera_info = msg
    with camera_info_lock:
        camera_info = msg


def depth_image_cb(msg):
    global depth_image
    with depth_image_lock:
        depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")


def bbox_callback(msg):
    global shelves, detected_products, camera_info, depth_image
    if not msg.bounding_boxes:
        return

    with camera_info_lock:
        fx = camera_info.K[0]
        fy = camera_info.K[4]
        cx = camera_info.K[2]
        cy = camera_info.K[5]

    with depth_image_lock:
        depth_image_copy = depth_image.copy()

    try:
        trans = tf_buffer.lookup_transform("map", "camera_depth_frame", rospy.Time())
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        return

    for bbox in msg.bounding_boxes:
        class_id = bbox.Class
        x = bbox.xmin
        y = bbox.ymin
        w = bbox.xmax - bbox.xmin
        h = bbox.ymax - bbox.ymin
        confidence = bbox.probability

        closest_shelf_id = get_closest_shelf(x + w / 2, y + h / 2)
        process_product_bbox(class_id, confidence, x, y, w, h, closest_shelf_id)

        center_x = (bbox.xmin + bbox.xmax) / 2
        center_y = (bbox.ymin + bbox.ymax) / 2
        D = depth_image_copy[int(center_y), int(center_x)]

        X = (center_x - cx) * D / fx
        Y = (center_y - cy) * D / fy
        Z = D

        point_cam = PointStamped()
        point_cam.header.frame_id = "camera_depth_frame"
        point_cam.point.x = X
        point_cam.point.y = Y
        point_cam.point.z = Z

        point_map = tf2_geometry_msgs.do_transform_point(point_cam, trans)
        detected_products[bbox.Class].append(
            (point_map.point.x, point_map.point.y, point_map.point.z)
        )

        # determine shelf statuses
        section_statuses = determine_section_statuses()

        # print product statuses
        for section_id, sections in section_statuses.items():
            for product_id, status in sections.items():
                rospy.loginfo(f"{section_id}/{product_id}: {status}")

    # publish detected products
    product_locations_publisher.publish(json.dumps(detected_products))


def get_closest_shelf(x, y):
    global shelves
    min_dist = float("inf")
    closest_shelf = None
    for shelf_id, marker in shelves.items():
        dist = ((marker["x"] - x) ** 2 + (marker["y"] - y) ** 2) ** 0.5
        if dist < min_dist:
            min_dist = dist
            closest_shelf = shelf_id
    return closest_shelf


def process_product_bbox(class_id, confidence, x, y, w, h, closest_shelf_id):
    global last_seen, product_sizes, product_confidences, segment_product_counts

    # Check if this is a new scan of the shelf
    current_time = time.time()
    if current_time - last_seen[closest_shelf_id].get(class_id, 0) > TIME_THRESHOLD:
        # If the shelf hasn't been seen recently (based on the TIME_THRESHOLD), reset the count for this shelf
        segment_product_counts[closest_shelf_id][class_id] = 0

    # update last seen timestamp
    last_seen[class_id] = time.time()

    # calculate area of bounding box
    area = round(w * h)

    # keep track of different product sizes
    product_sizes[area] += 1

    segment_product_counts[closest_shelf_id][
        class_id
    ] += 1  # Increment the count of this product type in the specific shelf

    total_product_counts[
        class_id
    ] += 1  # Increment the total count of this product type regardless of its type
    product_positions[closest_shelf_id][class_id].append(
        (x + w / 2, y + h / 2)
    )  # Store the location for each product
    product_confidences[closest_shelf_id][class_id].append(confidence)

    # rospy.loginfo(f"Product count: {segment_product_counts}")
    return segment_product_counts, product_positions, total_product_counts


def determine_section_statuses():
    global segment_product_counts, last_seen, product_confidences
    section_statuses = defaultdict(dict)

    for section_id, products in segment_product_counts.items():
        for product_id, product_count in products.items():
            last_seen[section_id][product_id] = time.time()
            if (
                time.time() - last_seen[section_id][product_id] > TIME_THRESHOLD
                
            ):
                status = "Out of Stock"
            elif product_count < thresholds["Low Stock"]:
                status = "Low Stock"
            elif product_count < thresholds["In Stock"]:
                status = "In Stock"
            else:
                status = "Fully Stocked"
            section_statuses[section_id][product_id] = status
    return section_statuses


def write_to_csv(filename, section_statuses, segment_product_counts):
    # combine both dictionaries into a list of dictionaries for writing to csv
    data = []
    for section_id, products in section_statuses.items():
        for product_id, status in products.items():
            row = {
                "product_id": product_id,
                "shelf_id": section_id,
                "status": status,
                "product_count": segment_product_counts[section_id].get(
                    product_id, 0
                ),
            }
            data.append(row)

    # field names
    fields = ["product_id", "shelf_id", "status", "product_count"]

    # writing to csv file
    with open(filename, mode="w") as csvfile:
        # csv writer object
        writer = csv.DictWriter(csvfile, fieldnames=fields)

        # writing headers (field names)
        writer.writeheader()

        # writing data rows
        writer.writerows(data)



def periodic_csv_write():
    while not rospy.is_shutdown():
        # Get the section statuses
        section_statuses = determine_section_statuses()

        # write segment statuses , product counts to csv
        write_to_csv(
            "product_status.csv",
            section_statuses,
            segment_product_counts,
            detected_products,
        )

        # Sleep for 2 minutes
        rospy.sleep(1 * 60)  # sleep for 2 minutes


def create_marker(x, y, z, shelf_id):
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "shelves"
    marker.id = int(shelf_id)
    marker.type = Marker.CUBE
    marker.action = Marker.ADD

    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = z

    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    marker.scale.x = 1
    marker.scale.y = 1
    marker.scale.z = 0.1

    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0

    return marker


def publish_markers(marker_pub):
    for shelf_id, coords in shelves.items():
        marker = create_marker(coords["x"], coords["y"], coords["z"], shelf_id)
        marker_pub.publish(marker)

def publish_product_locations(detected_products):
    data = []

    for products, locations in detected_products.items():
       row = {"location": locations, }
                data.append(row)
    product_locations_publisher.publish(json.dumps(data))    
       
def periodic_publish():
    while not rospy.is_shutdown():
        # Get the section statuses
        section_statuses = determine_section_statuses()

        # publish product locations to ROS topic
        publish_product_locations(
            detected_products
        )

        # Sleep for 2 minutes
        rospy.sleep(0.2 * 60)  # sleep for 2 minutes


def main():
    global product_locations_publisher, tf_buffer, tf_listener
    rospy.init_node("product_status")

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)

    csv_write_thread = threading.Thread(target=periodic_csv_write)
    csv_write_thread.start()

    publish_thread = threading.Thread(target=periodic_publish)
    publish_thread.start()

    product_locations_publisher = rospy.Publisher("/product_locations", String, queue_size=10)
    rospy.Subscriber("/yolov5/detections", BoundingBoxes, bbox_callback)
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, update_robot_pose)
    rospy.Subscriber("/camera/color/camera_info", CameraInfo, camera_info_cb)
    rospy.Subscriber("/camera/depth/image_raw", Image, depth_image_cb)

    rospy.sleep(1)

    publish_markers(marker_pub)

    rospy.spin()

if __name__ == "__main__":
    main()

