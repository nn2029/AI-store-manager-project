#!/usr/bin/env python3
import rospy
import csv
import time
import threading
from collections import defaultdict, Counter
from cv_bridge import CvBridge
from visualization_msgs.msg import Marker
from detection_msgs.msg import BoundingBox, BoundingBoxes
from geometry_msgs.msg import PoseWithCovarianceStamped

# Initialize global data structures
# Shelf locations
shelves = {
    "1": {"x": 3.790, "y": 2.56, "z": 0.0},
    "2": {"x": -1.0, "y": 4.82, "z": 0.0},
    "3": {"x": -3.56, "y": 1.14, "z": 0.0},
    "4": {"x": 1.48, "y": -3.5, "z": 0.0},
    "5": {"x": 5.210, "y": -0.468, "z": 0.0},
}

# product_counts = defaultdict(lambda: defaultdict(int))  # Count products by class_id/SKU
last_seen = defaultdict(lambda: defaultdict(float))
product_confidences = defaultdict(lambda: defaultdict(list))
product_positions = defaultdict(lambda: defaultdict(list))
product_sizes = Counter()
segment_product_counts = defaultdict(lambda: Counter())
total_product_counts = Counter()  # Keep track of all product counts



# Define your thresholds here
thresholds = {"Out of Stock": 1, "Low Stock": 5, "In Stock": 10}  # Modified this line
TIME_THRESHOLD = 0.5 * 60

# Store robot position and orientation
robot_position = [0, 0]
robot_orientation = [0, 0, 0, 1]  # Quaternion (x, y, z, w), this represents 0 degree.


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


# define object detection callback function
def bbox_callback(msg):
    global shelves

    for bbox in msg.bounding_boxes:
        class_id = bbox.Class
        x = bbox.xmin
        y = bbox.ymin
        w = bbox.xmax - bbox.xmin
        h = bbox.ymax - bbox.ymin
        confidence = bbox.probability

        closest_shelf_id = get_closest_shelf(x + w / 2, y + h / 2)
        process_product_bbox(class_id, confidence, x, y, w, h, closest_shelf_id)

    # determine shelf statuses
    section_statuses = determine_section_statuses()

    # print product statuses
    for section_id, sections in section_statuses.items():
        for product_id, status in sections.items():
            rospy.loginfo(f"{section_id}/{product_id}: {status}")


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

    #Check if this is a new scan of the shelf
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

    # if "sku" not in class_id.lower():
    #    segment_product_counts[closest_shelf_id][
    #        class_id
    #    ] += 1  # Counting individual products
    # else:
    #    segment_product_counts[closest_shelf_id][
    #        closest_shelf_id
    #    ] += 1  # Counting SKUs or sections

    segment_product_counts[closest_shelf_id][
        class_id
    ] += 1  # Increment the count of this product type in the specific shelf

    total_product_counts[
        class_id
    ] += 1  # Increment the total count of this product type regardless of its type
    product_positions[closest_shelf_id][class_id].append((x + w / 2, y + h / 2))
    product_confidences[closest_shelf_id][class_id].append(confidence)

    #rospy.loginfo(f"Product count: {segment_product_counts}")
    return segment_product_counts, product_positions, total_product_counts


def determine_section_statuses():
    global segment_product_counts, last_seen, product_confidences
    section_statuses = defaultdict(dict)

    for section_id, products in segment_product_counts.items():
        for product_id, product_count in products.items():
            last_seen[section_id][product_id] = time.time()
            # Determine status by count at a time 
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

        # write segment statuses and product counts to csv
        write_to_csv(
            "product_status.csv",
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
    marker.scale.z = 1

    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0

    return marker


def publish_markers(marker_pub):
    for shelf_id, coords in shelves.items():
        marker = create_marker(coords["x"], coords["y"], coords["z"], shelf_id)
        marker_pub.publish(marker)




def main():
    rospy.init_node("product_status")

    marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)

    # Start a thread to write csv periodically
    csv_write_thread = threading.Thread(target=periodic_csv_write)
    csv_write_thread.start()

    # Subscribe to the object detection topic
    rospy.Subscriber("/yolov5/detections", BoundingBoxes, bbox_callback)

    # Subscribe to the amcl_pose topic to get the current pose of the robot
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, update_robot_pose)

    rospy.sleep(1)

    publish_markers(marker_pub)

    rospy.spin()


if __name__ == "__main__":
    main()
