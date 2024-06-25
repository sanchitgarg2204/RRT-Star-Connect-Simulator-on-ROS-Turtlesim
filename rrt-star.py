import numpy as np
import cv2
import random
import math

class RRTStar:
    def __init__(self, start, end, img, step_size=12.5, max_iterations=100000, radius=10):
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
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

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

                    if goal_reached_iterations >= 500:
                        break

        return best_path

    def get_random_node(self):
        if random.randint(0, 100) > 10:
            return self.end
        else:
            rand_x = random.randint(0, self.img.shape[1] - 1)
            rand_y = random.randint(0, self.img.shape[0] - 1)
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
            angle = math.atan2(to_node[1] - from_node[1], to_node[0] - from_node[0])
            new_x = int(from_node[0] + self.step_size * math.cos(angle))
            new_y = int(from_node[1] + self.step_size * math.sin(angle))
            return (new_x, new_y)

    def is_valid_node(self, node):
        x, y = node
        bool=True
        #return (0 <= x < self.img.shape[1]) and (0 <= y < self.img.shape[0]) and (self.img[y][x][0] != 0).all()
        if img[y][x][0] == 0 and img[y][x][1] == 0 and img[y][x][2] == 0:
            bool=False
        #print("x", x, "y", y,"is:",bool)

        return bool
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

# Your code for generating the maze image
white = [255, 255, 255]

maze2 = cv2.imread("Image1.png", 1)
img=np.array(maze2)
cv2.imshow("Initial Image", img)

cv2.waitKey(0)
start_point=(78,526)
end_point=(502,56)
print(img.shape)
# RRT* algorithm
rrt_star = RRTStar(start_point, end_point, img)
rrt_star_path = rrt_star.generate_rrt_star()

if rrt_star_path is None:
    print("RRT*: Path not found.")
else:
    rrt_star_image = img.copy()
    for i in range(len(rrt_star_path) - 1):
        cv2.line(rrt_star_image, rrt_star_path[i], rrt_star_path[i + 1], (0, 255, 0), 2)

    print(rrt_star_path)

    cv2.imshow("RRT* Path", rrt_star_image)
    cv2.waitKey(0)