#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.srv import GetPlan
import pandas as pd
import math
from actionlib_msgs.msg import GoalID
from detection_msgs.msg import ProductLocation  



product_locations = []  # Store the product locations from the topic


# Robot's current position (initialize as PoseStamped)
robot_position = PoseStamped()

def update_robot_position(msg):
    global robot_position
    # Assuming msg is of type geometry_msgs/PoseWithCovarianceStamped
    robot_position.pose = msg.pose.pose
    robot_position.header = msg.header

def customer_request_callback(msg):
    rospy.loginfo(f"Received request for product: {msg.data}")
    # process customer request
    product_name = msg.data  
    
    # get the product locations from the Odoo database
    locations = get_product_locations(product_name)
    
    # calculate the nearest location based on path planning
    nearest_location = get_shortest_path_location(locations)
    
    # instruct the robot to move to the nearest location
    if nearest_location:
        navigate_to_location(nearest_location)

def update_product_locations(msg):
    global product_locations
    # Update product locations based on the messages from the topic
    product_locations.append((msg.product_id, (msg.location_x, msg.location_y)))

def get_product_locations(product_name):
    rospy.loginfo(f"Getting location for product: {product_name}")
    # Filter the locations based on the requested product
    locations = [loc[1] for loc in product_locations if loc[0].lower() == product_name.lower()]
    rospy.loginfo(f"Found locations for product {product_name}: {locations}")
    return locations


def get_shortest_path_location(locations):
    rospy.loginfo(f"Calculating shortest path from locations: {locations}")
    # This function calls the path planning service of the robot to calculate the shortest path
    rospy.wait_for_service('/move_base/GlobalPlanner/make_plan')
    get_plan = rospy.ServiceProxy('/move_base/GlobalPlanner/make_plan', GetPlan)

    # sort locations by euclidean distance, and take the top 5 closest
    locations = sorted(locations, key=lambda loc: euclidean_distance(loc, (robot_position.pose.position.x, robot_position.pose.position.y)))
    locations = locations[:5]

    min_length = float('inf')
    nearest_location = None
    for location in locations:
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.pose.position.x = location[0]
        goal.pose.position.y = location[1]
        goal.pose.orientation.w = 1.0  # assuming a simple 2D navigation task

        try:
            plan = get_plan(robot_position, goal, 0.5)  # tolerance of 0.5m
            path_length = sum(math.hypot(p1.pose.position.x - p2.pose.position.x, p1.pose.position.y - p2.pose.position.y)
                              for p1, p2 in zip(plan.plan.poses[:-1], plan.plan.poses[1:]))
            if path_length < min_length:
                min_length = path_length
                nearest_location = location
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
    if nearest_location:
        rospy.loginfo(f"Nearest location determined: {nearest_location}")
    else:
        rospy.logwarn("Unable to determine the nearest location.")
    return nearest_location

def euclidean_distance(loc1, loc2):
    return math.sqrt((loc1[0] - loc2[0]) ** 2 + (loc1[1] - loc2[1]) ** 2)


def navigate_to_location(location):
    rospy.loginfo(f"Attempting to navigate to location: {location}")
    
    # Cancel any active goals, interrupting exploration for example
    cancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)
    cancel_msg = GoalID()  # an empty goal ID means cancel all goals
    cancel_pub.publish(cancel_msg)
    
    rospy.sleep(1)  # Allow some time for cancellation to process
    
    # Now send the new goal
    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
    rospy.sleep(1)  # allowing time for the publisher to register

    goal = PoseStamped()
    goal.header.frame_id = "map"
    goal.pose.position.x = location[0]
    goal.pose.position.y = location[1]
    goal.pose.orientation.w = 1.0  # assuming a simple 2D navigation task

    pub.publish(goal)


if __name__ == '__main__':
    rospy.init_node('customer_request_handler', anonymous=True)
    rospy.Subscriber('/speech_to_text', String, customer_request_callback)
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, update_robot_position)
    rospy.Subscriber('/product_locations', ProductLocation, update_product_locations)  # Subscribe to the product locations topic
    rospy.spin()
