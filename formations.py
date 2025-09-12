
import json
import numpy as np
import time
import math
import random
import pickle
import paho.mqtt.client as mqtt
from scipy.optimize import linear_sum_assignment
from dataclasses import dataclass, field
import matplotlib.pyplot as plt
from stuff import *

def do_line(robot_list: list[Robot], obstacles: list[Square], start: Pose = Pose(to_meters(36), to_meters(84)), end: Pose = Pose(13.0302 - to_meters(12), 13.3096 - to_meters(2))):
    # start = lowest globe size
    temp_list = sorted(robot_list, key=lambda r: r.globe_rad_m)
    x_dist = end.x - start.x
    y_dist = end.y - start.y
    num_robots = len(temp_list)
    
    colors = plt.cm.get_cmap('hsv', len(temp_list) + 1)
     
    for i in temp_list:
        index = temp_list.index(i)
        target_x = start.x + (x_dist * (index / (num_robots - 1)))
        target_y = start.y + (y_dist * (index / (num_robots - 1)))
        target_pose = Pose(target_x, target_y)
        path = astar(i.pose, target_pose, obstacles, 0.1)
        plt.plot([i.pose.x], [i.pose.y], 'o', color=colors(index))  # start point
        plt.plot([target_pose.x], [target_pose.y], 'x', color=colors(index))
        if path:            
            x_vals = [pose.x for pose in path]
            y_vals = [pose.y for pose in path]
            # make colors random and unique in plot
            
            plt.plot(x_vals, y_vals, '-', color=colors(index), linewidth=1)  # path is colored line
            # plt.plot(x_vals, y_vals, 'o', color=colors(index))
            
        i.path = path
        
    plt.xlim(0, 10)
    plt.ylim(0, 10)

    plt.grid()
    plt.savefig("line.png")
    
def move_line(robot_list: list[Robot], obstacles: list[Square], goals: list[Pose] = [Pose(11.811, 2.5908), Pose(1.2192, 12.0904)]):
    plt.clf()
    path = list[Pose]
    start = robot_list[0].pose
    for i in goals:
        path.extend(astar(start, i, obstacles, 0.25))
        start = i
    # path1 = astar(robot_list[0].pose, Pose(11.811, 1.2192), None, 0.25)
    # path2 = astar(Pose(11.811, 1.2192), Pose(1.2192, 12.0904), None, 0.25)
    for r in robot_list:
        r.path = astar(r.pose, robot_list[0].pose, obstacles, 0.25)
        # r.path.extend(path1)
        # r.path.extend(path2)
        r.path.extend(path)
        r.path_step = 0
        r.path_id = 5
    
    x_vals = [pose.x for pose in robot_list[len(robot_list)].path]
    y_vals = [pose.y for pose in robot_list[len(robot_list)].path]
    plt.plot(x_vals, y_vals, 'o', color = "blue")
    
    plt.grid()
    plt.savefig("move_line.png")
        
def random_path(robot: Robot, n: int, r: int, center: Pose = None):
    angle = robot.pose.heading
    next_target = robot.pose
    
    output: list[Pose] = []
    plt.plot([robot.pose.x], [robot.pose.y], 'x', color='red')  # start point
    for j in range(n):
        
        angle = random.uniform(angle - math.pi / 2, angle + math.pi / 2)
        if (center is not None):
            print(next_target.as_vector(), center.as_vector())
            angle = angle_nudge(next_target, center, angle)
            # print(angle*180/math.pi)
        next_target = Pose(next_target.x + (math.cos(angle) * r), next_target.y + (math.sin(angle) * r), angle)
        output.append(next_target)
        # make a plot of the angle range and points
        
        # plt.plot([i.pose.x], [i.pose.y], 'o', color = colors(robot_list.index(i)))  # start point
        # plot each target as orange arrow that points in the direction of angle
        plt.arrow(next_target.x, next_target.y, math.cos(angle)*0.2, math.sin(angle)*0.2, head_width=0.1, head_length=0.1, fc='orange', ec='orange')

        # plt.plot([next_target.x], [next_target.y], 'o', color = "orange")
        # plt.plot([next_target.x], [next_target.y], 'o', color= colors(robot_list.index(i)))  # end point    
        
    robot.path = output 
    return output

def do_random(robot_list: list[Robot], n: int = 25, r: float = 0.5):
    colors = plt.cm.get_cmap('hsv', len(robot_list) + 1)
    for i in robot_list:
        random_path(i, n, r)
        # angle = i.pose.heading
        # next_target = i.pose
        # output: list[Pose] = []
        # plt.plot([i.pose.x], [i.pose.y], 'x', color='red')  # start point
        # for j in range(n):
        #     angle = random.uniform(angle - math.pi / 2, angle + math.pi / 2)
        #     next_target = Pose(next_target.x + (math.cos(angle) * r_m), next_target.y + (math.sin(angle) * r_m), angle)
        #     output.append(next_target)
        #     # make a plot of the angle range and points
            
        #     # plt.plot([i.pose.x], [i.pose.y], 'o', color = colors(robot_list.index(i)))  # start point
        #     plt.plot([next_target.x], [next_target.y], 'o', color= colors(robot_list.index(i)))  # end point
        #     # plt.plot([i.pose.x, next_target.x], [i.pose.y, next_target.y], '-', color='gray', linewidth=0.5)  # path is colored line
        #     # if output:
        #     #     x_vals = [node.pose.x for node in output]
        #     #     y_vals = [node.pose.y for node in output]
        #     #     plt.plot(x_vals, y_vals, '-', color='green', linewidth=1)  # path is colored line
        #     #     # plt.plot(x_vals, y_vals, 'o', color='green')
    
    plt.xlim(-5, 15)
    plt.ylim(-5, 15)
        
    plt.grid()
    plt.savefig(f"random.png")
    # i.path = output
 
def graph_force(center: Pose = Pose(0, 0), n: float = 100):
    # plot a vector field with the equation r^2 / n^2, pointing towards center
    # make the vectors stronger further away from center
    
    x = np.linspace(-10, 10, 20)
    y = np.linspace(-10, 10, 20)
    X, Y = np.meshgrid(x, y)
    U = -(X - center.x)
    V = -(Y - center.y)
    R = np.sqrt(U**2 + V**2)
    U = U * (R) / n
    V = V * (R) / n
    print(V)
    # print(dict(X=X, Y=Y, U=U, V=V))
    plt.figure(figsize=(8, 8))
    plt.quiver(X, Y, U, V, color='blue')
    plt.plot(center.x, center.y, 'ro')  # center point
    plt.xlim(-10, 10)
    plt.ylim(-10, 10)
    plt.grid()
    plt.savefig("vector_field.png")
    
def calc_force(pose: Pose, center: Pose = Pose(0, 0), k: float = 0.01) -> Vector:
    delta = Vector(center.x - pose.x, center.y - pose.y)
    # distance = math.sqrt(direction.x**2 + direction.y**2)
    # if distance == 0:
    #     return Vector(0, 0)
    force_magnitude = (delta.mag()**2) * k
    # force = Vector((direction.x / distance) * force_magnitude, (direction.y / distance) * force_magnitude)
    return Vector.from_angle(math.atan2(delta.y, delta.x), force_magnitude)

def angle_nudge(pose: Pose, center: Pose, angle: float) -> float:
    # print(center, pose)
    if (center.as_vector() == pose.as_vector()): # if the center and pose are the same, do nothing

        return angle
    ideal = math.atan2(pose.y - center.y, pose.x - center.x)+math.pi
    delta = ideal - angle
    angle += delta * abs(delta) * 1 # 0.1(delta^2)
    if (angle > pose.heading + math.pi/2):
        angle = pose.heading + math.pi/2
    if (angle < pose.heading - math.pi/2):
        angle = pose.heading - math.pi/2
    return angle
  
def group_robots(group: list[Robot], obstacles: list[Square], center: Pose = Pose(0, 0), save_num = 0):
    plt.clf()
    if len(group) > 0:
        colors = plt.cm.get_cmap('hsv', len(group) + 1)
        
        # center.translate(-1 * calc_force(r.pose, center, 100))
        for r in group:
            r.path = astar(r.pose, center, obstacles, 0.25)
            # graph path
            if r.path:
                x_vals = [pose.x for pose in r.path]
                y_vals = [pose.y for pose in r.path]
                plt.plot(x_vals, y_vals, '-', color=colors(group.index(r)), linewidth=1)  # path is colored line
                plt.plot(x_vals, y_vals, 'o', color=colors(group.index(r)))
            plt.plot([r.pose.x], [r.pose.y], 'o', color='blue')  # start point
            print("Group ", save_num, ", Robot ", group.index(r))
        plt.plot([center.x], [center.y], 'x', color='blue')  # end point
        
        plt.grid()
    plt.savefig(f"group{save_num}.png")      
    
def form_groups(robot_list: list[Robot], obstacles: list[Square], capitals: list[Pose] = [Pose(3, 15), Pose(3, 5), Pose(7, 5), Pose(7, 15)]):
    group_0 = [r for r in robot_list if r.group_id == 0]
    group_1 = [r for r in robot_list if r.group_id == 1]
    group_2 = [r for r in robot_list if r.group_id == 2]
    group_3 = [r for r in robot_list if r.group_id == 3]
    group_4 = [r for r in robot_list if r.group_id == 4]
    
    group_robots(group_0, capitals[0], 0)
    group_robots(group_1, capitals[1], 1)
    group_robots(group_2, capitals[2], 2)
    group_robots(group_3, capitals[3], 3)
    
    # if len(group_0) > 0:
    #     colors = plt.cm.get_cmap('hsv', len(group_0) + 1)
        
    #     for r in group_0:
    #         r.path = astar(r.pose, capitals[0].translate(-1 * calc_force(r.pose, capitals[0], 100)), obstacles, 0.25)
    #         # graph path
    #         if r.path:
    #             x_vals = [node.pose.x for node in r.path]
    #             y_vals = [node.pose.y for node in r.path]
    #             plt.plot(x_vals, y_vals, '-', color=colors(group_0.index(r)), linewidth=1)  # path is colored line
    #             plt.plot(x_vals, y_vals, 'o', color=colors(group_0.index(r)))
    #         plt.plot([r.pose.x], [r.pose.y], 'o', color='blue')  # start point
    #     plt.plot([capitals[0].x], [capitals[0].y], 'x', color='blue')  # end point
        
    #     plt.grid()
    #     plt.savefig("group0.png")

def do_groups(robot_list: list[Robot], obstacles: list[Square], n: int = 25, r_m: float = 0.5):
    group_0 = [r for r in robot_list if r.group_id == 0]
    group_1 = [r for r in robot_list if r.group_id == 1]
    group_2 = [r for r in robot_list if r.group_id == 2]
    group_3 = [r for r in robot_list if r.group_id == 3]
    group_4 = [r for r in robot_list if r.group_id == 4]
    
    colors = plt.cm.get_cmap('hsv', len(robot_list) + 1)
    
    # plot all obstacles in matplotlib
    for obstacle in obstacles:
        square = plt.Rectangle((obstacle.center.x - obstacle.half_size, obstacle.center.y - obstacle.half_size), obstacle.half_size * 2, obstacle.half_size * 2, fc='gray')
        plt.gca().add_patch(square)
    
    path_1_vect: list[Vector] = []
    angle = group_0[0].pose.heading
    last_target = group_0[0].pose
    next_target = Vector(0.0, 0.0)
    plt.plot([group_0[0].pose.x], [group_0[0].pose.y], 'x', color='red')  # start point
    for i in range(n):
        last_target = last_target.translate(next_target)
        next_target = Vector(0, 0)
        while (test_line(last_target, last_target.translate(next_target), obstacles) or next_target == Vector(0, 0)):
            angle = random.uniform(angle - math.pi / 2, angle + math.pi / 2)
            print("Angle: ", angle)
            next_target = Vector(math.cos(angle) * r_m, math.sin(angle) * r_m)
        path_1_vect.append(next_target)
        plt.plot([last_target.x], [last_target.y], 'o', color="blue")  # end point
        
    
    
    plt.xlim(-12, 12)
    plt.ylim(-12, 12)
    
    plt.grid()
    plt.savefig(f"groups.png")
    return 

def do_6(robot_list: list[Robot], obstacles: list[Square]):
    # temp
    robot6 = robot_list[5]
    robot10 = robot_list[9]
    robot6.path = astar(robot6.pose, robot10.pose, obstacles, 0.25)
    robot6.path_step = 0
    robot6.path_id = 4
    random_path(robot10, 35, 0.5, Pose(robot10.pose.x, robot10.pose.y))
    
    plt.grid()
    plt.savefig("6test.png")      

# default pose needs to be changed
def form_circle(robot_list: list[Robot], obstacles: list[Square], center: Pose = Pose(6.7, 10.2 - to_meters(30)), rad: float = 3.175):
    circle_points = circle(len(robot_list), center, rad)
    A = np.array([[point.x, point.y] for point in circle_points])
    B = np.array([[robot.pose.x, robot.pose.y] for robot in robot_list])
    C = np.linalg.norm(A[:, None, :] - B[None, :, :], axis=2)

    rows, cols = linear_sum_assignment(C)
    pairs = list(zip(rows, cols))
    total_distance = C[rows, cols].sum()
    print(f"Optimal assignment pairs (circle point index, robot index): {pairs}")
    print(f"Total minimum distance: {total_distance}")

    # not necessary
    # output: list[list[Pose]] = []

    # make a list with each robot's circle point number
    circle_nums = [p[1] for p in pairs]
    print("Circle nums: ", circle_nums)

    colors = plt.cm.get_cmap('hsv', len(pairs) + 1)
    for index, i in enumerate(pairs):
        print(f"Robot {i[1]+1} to Circle Point {i[0]+1}")
        path = astar(robot_list[i[1]].pose, circle_points[i[0]], obstacles, 0.25)
        if path:
            x_vals = [node.x for node in path]
            y_vals = [node.y for node in path]
            plt.plot(x_vals, y_vals, '-', color=colors(index))  # path is colored line
            plt.plot(x_vals, y_vals, 'o', color=colors(index))
        robot_list[i[1]].path = path
        # output.append(path)
        plt.plot([robot_list[i[1]].pose.x], [robot_list[i[1]].pose.y], 'o', color=colors(index))  # start point
        plt.plot([circle_points[i[0]].x], [circle_points[i[0]].y], 'x', color=colors(index))  # end point
    # set axes [0,10] for x and y
    plt.xlim(0, 10)
    plt.ylim(0, 10)

    plt.grid()
    plt.savefig("circle.png")
    return circle_nums
    
    # return output
# default pose needs to be changed
def do_circle(robot_list: list[Robot], obstacles: list[Square], circle_nums: list[int], center: Pose = Pose(6.7, 10.2 - to_meters(30)), rad: float = 3.175, n: int = 1):
    
    # generate n * list length points in a circle around center
    circle_pts = circle(n * len(robot_list), center, rad)
    colors = plt.cm.get_cmap('hsv', len(robot_list) + 1)
    
    for r in robot_list:
        r.path = circle_pts
        r.path_step = circle_nums[robot_list.index(r)] * n
        plt.plot([r.pose.x], [r.pose.y], 'o', color=colors(robot_list.index(r)))  # start point
        plt.plot([circle_pts[r.path_step].x], [circle_pts[r.path_step].y], 'x', color=colors(robot_list.index(r)))  # end point
        if r.path:
            x_vals = [point.x for point in r.path]
            y_vals = [point.y for point in r.path]
            plt.plot(x_vals, y_vals, '-', color=colors(robot_list.index(r)), linewidth=1)  # path is colored line
            plt.plot(x_vals, y_vals, 'o', color=colors(robot_list.index(r)))
        
    
    plt.xlim(0, 10)
    plt.ylim(0, 10)
    
    plt.grid()
    plt.savefig(f"do_circle.png")