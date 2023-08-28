#!/usr/bin/env python3
import rospy
import csv
import cv2
import os
import tf
import tf2_ros
import tf2_geometry_msgs
import numpy as np
import time
import threading
from collections import defaultdict, Counter, deque
from cv_bridge import CvBridge
from visualization_msgs.msg import Marker
from sensor_msgs.msg import Image, LaserScan, CameraInfo
from geometry_msgs.msg import PoseWithCovarianceStamped, PointStamped
from detection_msgs.msg import BoundingBox, BoundingBoxes, ProductLocation
from deep_sort import *



# Initialize global data structures
shelves = rospy.get_param("shelves", {})  # Shelf locations
detected_products = ProductLocation()
camera_info = None
depth_image = None
tf_buffer = None
tf_listener = None
bridge = CvBridge()


# Define overlap threshold
OVERLAP_THRESHOLD = 0.5
# Define debounce time (in seconds)
DEBOUNCE_TIME = 3.0
publish_product_locations = rospy.Publisher(
    "/product_locations", ProductLocation, queue_size=10
)

last_seen = defaultdict(lambda: defaultdict(float))
product_confidences = defaultdict(lambda: defaultdict(list))
product_positions = defaultdict(lambda: defaultdict(list))
product_sizes = Counter()
segment_product_counts = defaultdict(lambda: defaultdict(int))
product_debounce = ProductDebounce()
total_product_counts = Counter()

# Initialize object tracker
max_cosine_distance = 0.2
nn_budget = None
metric = nn_matching.NearestNeighborDistanceMetric("cosine", max_cosine_distance, nn_budget)
tracker = Tracker(metric)

history_buffer = defaultdict(lambda: deque(maxlen=10))
tracked_products = set()

# Define your thresholds here
thresholds = rospy.get_param(
    "thresholds", {"Out of Stock": 0, "Low Stock": 5, "In Stock": 10}
)
TIME_THRESHOLD = rospy.get_param("time_threshold", 2 * 60)

# Store robot position and orientation
robot_position = [0, 0]
robot_orientation = [0, 0, 0, 1]  # Quaternion (x, y, z, w), this represents 0 degree.

camera_info_lock = threading.Lock()
depth_image_lock = threading.Lock()

class ProductDebounce:
    def __init__(self, size=10):
        self.size = size
        self.history = defaultdict(lambda: [])  
    def add_observation(self, product_id):
        if len(self.history[product_id]) >= self.size:
            self.history[product_id].pop(0)
        self.history[product_id].append(True)

    def should_count(self, product_id):
        return len(self.history[product_id]) == self.size and all(self.history[product_id])

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
    with camera_info_lock:
        camera_info = msg


def filter_invalid_depths(depth_image):
    return cv2.inRange(depth_image, 0.2, 10) * depth_image
    
def depth_image_cb(msg):
    global depth_image
    with depth_image_lock:
        depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        depth_image = cv2.medianBlur(depth_image, 5)
        depth_image = filter_invalid_depths(depth_image)
        
def is_new_product(tracking_id, x, y, w, h):
    """
    Check if the detected product is a new product based on the overlap ratio with previous detections.
    """
    global history_buffer
    if len(history_buffer[tracking_id]) == 0:
        history_buffer[tracking_id].append((x, y, w, h))
        return True
    prev_coords = history_buffer[tracking_id][-1]
    if len(prev_coords) != 4:
        rospy.logwarn(f"Incomplete coordinate history for tracking_id {tracking_id}: {prev_coords}")
        return False
    prev_x, prev_y, prev_w, prev_h = prev_coords
    overlap_area = max(0, min(x + w, prev_x + prev_w) - max(x, prev_x)) * max(0, min(y + h, prev_y + prev_h) - max(y, prev_y))
    current_area = w * h
    overlap_ratio = overlap_area / current_area
    
    # Update the history buffer with the new coordinates
    history_buffer[tracking_id].append((x, y, w, h))
    # Limit the history buffer to a certain size to prevent memory issues
    if len(history_buffer[tracking_id]) > HISTORY_BUFFER_SIZE:
        history_buffer[tracking_id].pop(0)
    
    return overlap_ratio < OVERLAP_THRESHOLD

    
def bbox_callback(msg):
    global shelves, detected_products, camera_info, depth_image, tracker, history_buffer, tracked_products

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

    detections = []
    for bbox in msg.bounding_boxes:
        class_id = bbox.Class
        confidence = bbox.probability
        detections.append(Detection(np.array([bbox.xmin, bbox.ymin, bbox.xmax, bbox.ymax]), confidence, None))
                
    # Update tracker
    tracker.predict()
    tracker.update(detections)

    for track in tracker.tracks:
        if not track.is_confirmed() or track.time_since_update > 1:
            continue

        x, y, w, h = map(int, track.to_tlwh())

        center_x = (x + w) / 2
        center_y = (y + h) / 2

        region_size = 10
        roi = depth_image_copy[max(0, int(center_y - region_size)):min(depth_image_copy.shape[0], int(center_y + region_size)),
                            max(0, int(center_x - region_size)):min(depth_image_copy.shape[1], int(center_x + region_size))]
        valid_depths = roi[roi > 0]
        if len(valid_depths) == 0:
            rospy.logwarn("Invalid depth value encountered.")
            continue
        D = np.mean(valid_depths)

        closest_shelf_id = get_closest_shelf(x + w / 2, y + h / 2)
        tracking_id = f"{class_id}_{closest_shelf_id}"

        # Process the bounding boxes
        process_product_bbox(class_id, confidence, x, y, w, h, closest_shelf_id, D)

        # Update history buffer
        history_buffer[tracking_id].append((x + w / 2, y + h / 2))

        # Check if this product is already being tracked
        if tracking_id not in tracked_products:
            tracker.init(depth_image_copy, (x, y, w, h))
            tracked_products.add(tracking_id)
        else:
            ok, new_bbox = tracker.update(depth_image_copy)
            if ok:
                x, y, w, h = map(int, new_bbox)
            else:
                tracked_products.remove(tracking_id)            
       
        point_cam = PointStamped()
        point_cam.header.frame_id = "camera_depth_frame"
        point_cam.point.x = X
        point_cam.point.y = Y
        point_cam.point.z = Z

        point_map = tf2_geometry_msgs.do_transform_point(point_cam, trans)

        detected_products = ProductLocation()
        detected_products.product_id = bbox.Class
        detected_products.location_x = point_map.point.x
        detected_products.location_y = point_map.point.y

        # determine shelf statuses
        section_statuses = determine_section_statuses()

        # print product statuses
        #for section_id, sections in section_statuses.items():
         #   for product_id, status in sections.items():
          #      rospy.loginfo(f"{section_id}/{product_id}: {status}")

    # publish detected products
    #product_locations_publisher.publish(detected_products)


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


def process_product_bbox(class_id, confidence, x, y, w, h, closest_shelf_id, D):
    global last_seen, product_sizes, product_confidences, segment_product_counts
    Z = D
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
    # Logging
    rospy.loginfo(f"Attempting to count product {class_id} in shelf {closest_shelf_id}")

    tracking_id = f"{class_id}_{closest_shelf_id}"
    if is_new_product(tracking_id, x, y, w, h):
        history_buffer[tracking_id].append((x, y, w, h))
        product_debounce.add_observation(class_id)
        if product_debounce.should_count(class_id, debounce_time=DEBOUNCE_TIME):
            segment_product_counts[closest_shelf_id][class_id] += 1
            rospy.loginfo(f"Incremented count for product {class_id} in shelf {closest_shelf_id}. New count: {segment_product_counts[closest_shelf_id][class_id]}")
    else:
        pass

    return segment_product_counts, product_positions, total_product_counts


def determine_section_statuses():
    global segment_product_counts, last_seen, product_confidences
    section_statuses = defaultdict(dict)

    for section_id, products in segment_product_counts.items():
        for product_id, product_count in products.items():
            last_seen[section_id][product_id] = time.time()
            if time.time() - last_seen[section_id][product_id] > TIME_THRESHOLD:
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
                "product_count": segment_product_counts[section_id].get(product_id, 0),
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

        csv_path = rospy.get_param(
            "csv_path", "/tmp"
        )  # Read the csv_path parameter or use /tmp as the default location
        csv_file = os.path.join(
            csv_path, "product_status.csv"
        )  # Join the folder path with the filename

        # write segment statuses , product counts to csv
        write_to_csv(
            csv_file,
            section_statuses,
            segment_product_counts,
        )

        # Sleep for 2 minutes
        rospy.sleep(2 * 60)  # sleep for 2 minutes


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


def periodic_publish():
    while not rospy.is_shutdown():
        # Get the section statuses
        section_statuses = determine_section_statuses()

        # publish product locations to ROS topic
        publish_product_locations.publish(detected_products)

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

    rospy.Subscriber("/yolov5/detections", BoundingBoxes, bbox_callback)
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, update_robot_pose)
    rospy.Subscriber("/camera/color/camera_info", CameraInfo, camera_info_cb)
    rospy.Subscriber("/camera/depth/image_raw", Image, depth_image_cb)

    rospy.sleep(1)

    publish_markers(marker_pub)

    rospy.spin()


if __name__ == "__main__":
    main()


