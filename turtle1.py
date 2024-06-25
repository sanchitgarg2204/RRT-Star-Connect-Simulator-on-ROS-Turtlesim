#!/usr/bin/env python

'''import rospy
import cv2
import numpy as np
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
#from std_srvs.srv import SetPen

class RRTStar:
    def __init__(self, start, end, img, step_size=12, max_iterations=100000, radius=10):
        self.start = start
        self.end = end
        self.img = img
        self.step_size = step_size
        self.max_iterations = max_iterations
        self.radius = radius
        self.nodes = [start]
        self.parent = {start: None}
        self.cost = {start: 0}

    def distance(self, point1, point2):
        x1, y1 = point1
        x2, y2 = point2
        return np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    def generate_rrt_star(self):
        goal_reached_iterations = 0  # Counter for iterations after reaching the goal
        best_cost = float('inf')
        best_path = None

        for iteration in range(self.max_iterations):
            rand_node = self.get_random_node()
            nearest_node = self.get_nearest_node(rand_node)
            new_node = self.steer(nearest_node, rand_node)
            if self.is_valid_node(new_node):
                neighbors = self.find_near_neighbors(new_node)
                best_node = nearest_node
                best_node_cost = self.cost[nearest_node] + self.distance(nearest_node, new_node)
                for neighbor in neighbors:
                    cost = self.cost[neighbor] + self.distance(neighbor, new_node)
                    if cost < best_node_cost:
                        best_node = neighbor
                        best_node_cost = cost

                self.nodes.append(new_node)
                self.parent[new_node] = best_node
                self.cost[new_node] = best_node_cost
                self.rewire_neighbors(new_node, neighbors)

                if self.is_goal_reached(new_node):
                    goal_reached_iterations += 1

                    if best_node_cost < best_cost:
                        best_cost = best_node_cost
                        best_path = self.construct_path(new_node)

                    if goal_reached_iterations >= 1000:
                        break

        return best_path

    def get_random_node(self):
        if np.random.randint(0, 100) > 10:
            return self.end
        else:
            rand_x = np.random.randint(0, self.img.shape[1] - 1)
            rand_y = np.random.randint(0, self.img.shape[0] - 1)
            return (rand_x, rand_y)

    def get_nearest_node(self, rand_node):
        nearest_node = self.nodes[0]
        min_dist = self.distance(nearest_node, rand_node)
        for node in self.nodes:
            dist = self.distance(node, rand_node)
            if dist < min_dist:
                nearest_node = node
                min_dist = dist
        return nearest_node

    def steer(self, from_node, to_node):
        dist = self.distance(from_node, to_node)
        if dist < self.step_size:
            return to_node
        else:
            angle = np.arctan2(to_node[1] - from_node[1], to_node[0] - from_node[0])
            new_x = int(from_node[0] + self.step_size * np.cos(angle))
            new_y = int(from_node[1] + self.step_size * np.sin(angle))
            return (new_x, new_y)

    def is_valid_node(self, node):
        x, y = node
        return (0 <= x < self.img.shape[1]) and (0 <= y < self.img.shape[0]) and (self.img[y, x] != 0).all()

    def find_near_neighbors(self, node):
        neighbors = []
        for n in self.nodes:
            if self.distance(node, n) < self.radius:
                neighbors.append(n)
        return neighbors

    def rewire_neighbors(self, node, neighbors):
        for neighbor in neighbors:
            new_cost = self.cost[node] + self.distance(node, neighbor)
            if new_cost < self.cost[neighbor]:
                self.parent[neighbor] = node
                self.cost[neighbor] = new_cost

    def is_goal_reached(self, node):
        return node == self.end

    def construct_path(self, end_node):
        path = [end_node]
        parent_node = self.parent[end_node]
        while parent_node:
            path.append(parent_node)
            parent_node = self.parent[parent_node]
        path.reverse()
        return path


class TurtleController:
    def __init__(self, path):
        self.path = path
        self.current_index = 0
        self.current_pose = None
        self.velocity_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.pose_sub = rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
        rospy.wait_for_service('/turtle1/set_pen')
        #self.set_pen = rospy.ServiceProxy('/turtle1/set_pen', SetPen)
        #self.set_pen(0, 0, 0, 2, 0)  # Set pen color to black
        rospy.spin()

    def pose_callback(self, data):
        self.current_pose = data
        if self.current_pose is not None:
            self.move_to_next()

    def move_to_next(self):
        if self.current_index >= len(self.path):
            return

        target_pose = self.path[self.current_index]
        current_x, current_y = self.current_pose.x, self.current_pose.y
        target_x, target_y = target_pose[0], target_pose[1]

        distance = np.sqrt((target_x - current_x) ** 2 + (target_y - current_y) ** 2)

        if distance < 0.05:
            self.current_index += 1
            self.move_to_next()
            return

        angle = np.arctan2(target_y - current_y, target_x - current_x)
        linear_vel = 1.0
        angular_vel = 1.5 * (angle - self.current_pose.theta)
        self.publish_velocity(linear_vel, angular_vel)

    def publish_velocity(self, linear, angular):
        velocity_msg = Twist()
        velocity_msg.linear.x = linear
        velocity_msg.angular.z = angular
        self.velocity_pub.publish(velocity_msg)


if __name__ == '__main__':
    rospy.init_node('rrt_star_turtle')

    # Load the image and define the start and end points
    maze= cv2.imread("src/Image1.png", 1)
    img=np.array(maze)
    start_point = (78, 526)
    end_point = (502, 56)

    # RRT* algorithm
    rrt_star = RRTStar(start_point, end_point, img)
    rrt_star_path = rrt_star.generate_rrt_star()

    if rrt_star_path is None:
        print("RRT*: Path not found.")
        rospy.loginfo("RRT*: Path not found.")
        exit()

    # Display the RRT* path
    rrt_star_image = img.copy()
    for i in range(len(rrt_star_path) - 1):
        cv2.line(rrt_star_image, rrt_star_path[i], rrt_star_path[i + 1], (0, 0, 255), 2)
    cv2.imshow("RRT* Path", rrt_star_image)
    cv2.waitKey(0)

    path=[(78, 526), (86, 517), (94, 508), (105, 511), (112, 501), (119, 491), (126, 481), (133, 472), (140, 463), (147, 454), (154, 445), (162, 436), (164, 428), (165, 424), (160, 412), (168, 403), (176, 394), (184, 385), (182, 373), (190, 364), (198, 355), (206, 346), (214, 337), (222, 328), (232, 333), (240, 324), (248, 315), (256, 306), (267, 305), (274, 314), (285, 314), (296, 315), (307, 316), (312, 326), (322, 320), (332, 325), (338, 314), (344, 303), (350, 292), (356, 281), (362, 270), (368, 259), (374, 248), (380, 238), (386, 228), (397, 232), (408, 235), (413, 224), (418, 213), (423, 202), (428, 191), (433, 180), (426, 169), (432, 159), (438, 149), (444, 139), (450, 129), (449, 117), (456, 107), (464, 98), (472, 89), (480, 80), (488, 71), (496, 62), (502, 56)]
    # Convert the path to float values
    rrt_star_path = [(float(point[0])*11/600, float(point[1])*11/600) for point in path]

    # Start the turtle control
    turtle_controller = TurtleController(rrt_star_path)

#!/usr/bin/env python'''

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import SetPen
from math import pow, atan2, sqrt

class TurtleController:
    def __init__(self):
        rospy.init_node('turtle_controller', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.update_pose)
        self.set_pen_service = rospy.ServiceProxy('/turtle1/set_pen', SetPen)
        self.pose = Pose()
        self.rate = rospy.Rate(10)  # 10Hz rate
        self.pen=True

    def update_pose(self, data):
        self.pose = data

    def get_distance(self, goal_x, goal_y):
        return sqrt(pow((goal_x - self.pose.x), 2) + pow((goal_y - self.pose.y), 2))

    def set_pen(self, r, g, b, width, off):
        rospy.wait_for_service('/turtle1/set_pen')
        try:
            set_pen = rospy.ServiceProxy('/turtle1/set_pen', SetPen)
            set_pen(r, g, b, width, off)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)

    def move_turtle(self, goal_x, goal_y):
        self.set_pen(255, 255, 255, 2, self.pen)

        vel_msg = Twist()

        while self.get_distance(goal_x, goal_y) > 0.1:
            # PID control
            # Proportional control
            vel_msg.linear.x = 1.5 * self.get_distance(goal_x, goal_y)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            # Angular control
            desired_angle = atan2(goal_y - self.pose.y, goal_x - self.pose.x)
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 4 * (desired_angle - self.pose.theta)

            # Publish the velocity message
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()

        # Stop the turtle when goal is reached
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
        self.pen=False
        

if __name__ == '__main__':
    try:
        controller = TurtleController()
        
        # List of points on the grid
        point_list = [(1, 1), (5, 5), (9, 3), (10, 10)]
        #path=[(78, 526), (86, 517), (94, 508), (105, 511), (112, 501), (119, 491), (126, 481), (133, 472), (140, 463), (147, 454), (154, 445), (162, 436), (164, 428), (165, 424), (160, 412), (168, 403), (176, 394), (184, 385), (182, 373), (190, 364), (198, 355), (206, 346), (214, 337), (222, 328), (232, 333), (240, 324), (248, 315), (256, 306), (267, 305), (274, 314), (285, 314), (296, 315), (307, 316), (312, 326), (322, 320), (332, 325), (338, 314), (344, 303), (350, 292), (356, 281), (362, 270), (368, 259), (374, 248), (380, 238), (386, 228), (397, 232), (408, 235), (413, 224), (418, 213), (423, 202), (428, 191), (433, 180), (426, 169), (432, 159), (438, 149), (444, 139), (450, 129), (449, 117), (456, 107), (464, 98), (472, 89), (480, 80), (488, 71), (496, 62), (502, 56)]
        path=[(78, 526), (86, 516), (94, 506), (102, 496), (110, 486), (118, 476), (126, 466), (134, 456), (142, 446), (150, 436), (158, 426), (166, 416), (171, 404), (179, 394), (187, 384), (185, 371), (189, 359), (197, 350), (205, 341), (214, 332), (223, 323), (232, 314), (241, 306), (252, 301), (262, 307), (273, 301), (283, 308), (290, 318), (300, 324), (304, 335), (309, 346), (318, 353), (328, 346), (336, 337), (342, 326), (348, 315), (354, 304), (360, 293), (366, 282), (372, 271), (378, 260), (384, 249), (389, 237), (401, 237), (410, 245), (419, 237), (424, 225), (429, 213), (434, 201), (439, 189), (444, 177), (449, 165), (454, 153), (443, 145), (449, 134), (448, 121), (455, 111), (463, 101), (471, 91), (479, 81), (487, 71), (495, 62), (502, 56)]
        #  Convert the path to float values
        rrt_star_path = [(float(point[0])*11/600, 11-float(point[1])*11/600) for point in path]

        for point in rrt_star_path:
            goal_x, goal_y = point
            controller.move_turtle(goal_x, goal_y)
            rospy.loginfo("Goal reached!")

        rospy.spin()

    except rospy.ROSInterruptException:
        pass

