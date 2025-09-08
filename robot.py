import json
import numpy as np
import time
import math
import random
import paho.mqtt.client as mqtt
from scipy.optimize import linear_sum_assignment
from dataclasses import dataclass, field
import matplotlib.pyplot as plt
from stuff import *

BROKER = "midway.breckstampede.org"  # Change to your broker address
TOPIC = "robots/#"  # Subscribe to all subtopics
def on_connect(client, userdata, flags, reason_code, properties):
    if reason_code == 0:
        print("Connected to MQTT Broker!")
        client.subscribe(TOPIC)
    else:
        print(f"Failed to connect, return code {reason_code}")

def on_message(client, userdata, msg):
    topics = msg.topic.split('/')
    if topics[0] == "robots" and not topics[1] in robots:
        robots[topics[1]] = (Robot(name=topics[1]))
        print(f"Robot {topics[1]} connected")

    if topics[2] == "imu" and topics[3] == "euler":
     # read json from msg.payload
        data = json.loads(msg.payload.decode())
        heading = RobotHeading(
            x=data['x'],
            y=data['y'],
            z=data['z'],
            rad_accuracy=data['rad_accuracy'],
            accuracy=data['accuracy']
        )
        robots[topics[1]].heading = heading
        # print(f"Robot {topics[1]} heading: {heading}")

    if topics[2] == "led":
        # read json from msg.payload
        data = json.loads(msg.payload.decode())
        led = LED(
            color = Color(
                r=data['r'],
                g=data['g'],
                b=data['b']
            ),
            step=data['step']
        )
        robots[topics[1]].led = led

    if topics[3] == "telemetry":
        data = json.loads(msg.payload.decode())
        telemetry = Telemetry(
            speed=data['speed'],
            ticks=data['ticks'],
            output=data['output'],
            setpoint=data['setpoint'],
            hasPower=data['hasPower']
            # json in c++ might need to be changed to True and False (instead of true and false)
        )
        robots[topics[1]].telemetry = telemetry
    
    if topics[2] == "odometry":
        if topics[3] == "pose":
            data = json.loads(msg.payload.decode())
            pose = Pose(
                x=data['x'],
                y=data['y'],
                heading=data['heading']
            )
            robots[topics[1]].pose = pose

        elif topics[3] == "pose2":
            data = json.loads(msg.payload.decode())
            pose2 = Pose(
                x=data['x'],
                y=data['y'],
                heading=data['heading']
            )
            robots[topics[1]].pose2 = pose2

    if topics[2] == "wifi" and topics[3] == "rssi":
        data = json.loads(msg.payload.decode())
        rssi = data['rssi']
        robots[topics[1]].wifi_rssi = rssi  

    if topics[2] == "tcs":
        data = json.loads(msg.payload.decode())
        status = data['status']
        if status == "ok":
            tcs = TCS(
                r=data['r'],
                g=data['g'],
                b=data['b'],
                c=data['c'],
                lux=data['lux'],
                color_temperature=data['color_temperature'],
                status=data['status']
            )
            robots[topics[1]].tcs = tcs
        else:
            tcs = TCS(
                r=0,
                g=0,
                b=0,
                c=0,
                lux=0,
                color_temperature=0,
                status=status
            )
            robots[topics[1]].tcs = tcs

            robots[topics[1]].tcs.color_temperature = 0
            robots[topics[1]].tcs.status = status
            robots[topics[1]].tcs = tcs
        
    if topics[2] == "reroute":
        data = json.loads(msg.payload.decode())
        robots[topics[1]].reroute = data['reroute']

    if msg.topic.endswith("/ping") and msg.payload.decode() == "pong":
        robots[topics[1]].last_ping = time.time()


client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
# client.on_connect = on_connect
# client.on_message = on_message
# client.tls_set(ca_certs="ca.pem")
# client.username_pw_set("esp", "esp32")
# client.connect(BROKER, 8883, 60)
# client.loop_start()

obstacles = []

group0_obstacles = [
    Square(Pose(-20, 0, 0.0), 10),
    Square(Pose(0, 20, 0.0), 10),
    Square(Pose(20, 0, 0.0), 10),
    Square(Pose(0, -20, 0.0), 10)
]
#     # Square(Pose(5, 5, 0.0), 2),
#     Square(Pose(7, 8, 0.0), 2),
#     Square(Pose(6, 5, 0.0), 3),
#     Square(Pose(10, 6, 0.0), 3),
#     Square(Pose(15, 15, 0.0), 4)
# ]

def do_line(robot_list: list[Robot], start: Pose, end: Pose):
    # start = lowest globe size
    temp_list = sorted(robot_list, key=lambda r: r.globe_dia_cm)
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
            x_vals = [node.pose.x for node in path]
            y_vals = [node.pose.y for node in path]
            # make colors random and unique in plot
            
            plt.plot(x_vals, y_vals, '-', color=colors(index), linewidth=1)  # path is colored line
            # plt.plot(x_vals, y_vals, 'o', color=colors(index))
            
        i.path = path
        
    plt.xlim(0, 10)
    plt.ylim(0, 10)

    plt.grid()
    plt.savefig("line.png")
    
def do_random(robot_list: list[Robot], n: int = 25, r_m: float = 0.5):
    colors = plt.cm.get_cmap('hsv', len(robot_list) + 1)
    for i in robot_list:
        angle = i.pose.heading
        next_target = i.pose
        output: list[Pose] = []
        plt.plot([i.pose.x], [i.pose.y], 'x', color='red')  # start point
        for j in range(n):
            angle = random.uniform(angle - math.pi / 2, angle + math.pi / 2)
            next_target = Pose(next_target.x + (math.cos(angle) * r_m), next_target.y + (math.sin(angle) * r_m), angle)
            output.append(next_target)
            # make a plot of the angle range and points
            
            # plt.plot([i.pose.x], [i.pose.y], 'o', color = colors(robot_list.index(i)))  # start point
            plt.plot([next_target.x], [next_target.y], 'o', color= colors(robot_list.index(i)))  # end point
            # plt.plot([i.pose.x, next_target.x], [i.pose.y, next_target.y], '-', color='gray', linewidth=0.5)  # path is colored line
            # if output:
            #     x_vals = [node.pose.x for node in output]
            #     y_vals = [node.pose.y for node in output]
            #     plt.plot(x_vals, y_vals, '-', color='green', linewidth=1)  # path is colored line
            #     # plt.plot(x_vals, y_vals, 'o', color='green')
        plt.xlim(-5, 15)
        plt.ylim(-5, 15)
        
    plt.grid()
    plt.savefig(f"random.png")
    i.path = output
 
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
    
def calc_force(pose: Pose, center: Pose = Pose(0, 0), n: float = 100) -> Vector:
    direction = Vector(center.x - pose.x, center.y - pose.y)
    distance = math.sqrt(direction.x**2 + direction.y**2)
    if distance == 0:
        return Vector(0, 0)
    force_magnitude = (distance**2) / n
    force = Vector((direction.x / distance) * force_magnitude, (direction.y / distance) * force_magnitude)
    return force

def angle_nudge(pose: Pose, center: Pose, angle: float) -> float:
    # for caleb to do
    return angle
    
def form_groups(robot_list: list[Robot], obstacles: list[Square], capitals: list[Pose] = [Pose(3, 15), Pose(3, 5)]):
    group_0 = [r for r in robot_list if r.group_id == 0]
    group_1 = [r for r in robot_list if r.group_id == 1]
    group_2 = [r for r in robot_list if r.group_id == 2]
    group_3 = [r for r in robot_list if r.group_id == 3]
    group_4 = [r for r in robot_list if r.group_id == 4]
    
    if len(group_0) > 0:
        colors = plt.cm.get_cmap('hsv', len(group_0) + 1)
        
        for r in group_0:
            r.path = astar(r.pose, capitals[0].translate(-1 * calc_force(r.pose, capitals[0], 100)), obstacles, 0.25)
            # graph path
            if r.path:
                x_vals = [node.pose.x for node in r.path]
                y_vals = [node.pose.y for node in r.path]
                plt.plot(x_vals, y_vals, '-', color=colors(group_0.index(r)), linewidth=1)  # path is colored line
                plt.plot(x_vals, y_vals, 'o', color=colors(group_0.index(r)))
            plt.plot([r.pose.x], [r.pose.y], 'o', color='blue')  # start point
        plt.plot([capitals[0].x], [capitals[0].y], 'x', color='blue')  # end point
        
        plt.grid()
        plt.savefig("group0.png")

        
    if len(group_1) > 0:
        # clear matplotlib plot
        plt.clf()
        group_1[0].path = astar(group_1[0].pose, capitals[1], obstacles, 0.25)
        colors = plt.cm.get_cmap('hsv', len(group_1) + 1)
        
        for r in group_1:
            r.path = astar(r.pose, capitals[1].translate(-1 * calc_force(r.pose, capitals[1], 100)), obstacles, 0.25)
            # graph path
            if r.path:
                x_vals = [node.pose.x for node in r.path]
                y_vals = [node.pose.y for node in r.path]
                plt.plot(x_vals, y_vals, '-', color=colors(group_1.index(r)), linewidth=1)  # path is colored line
                plt.plot(x_vals, y_vals, 'o', color=colors(group_1.index(r)))
            plt.plot([r.pose.x], [r.pose.y], 'o', color='blue')  # start point
        plt.plot([capitals[1].x], [capitals[1].y], 'x', color='blue')  # end point
        
        plt.grid()
        plt.savefig("group1.png")

def do_groups(robot_list: list[Robot], n: int = 25, r_m: float = 0.5, obstacles: list[Square] = []):
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
    return path_1_vect
        

def form_circle(robot_list: list[Robot], center: Pose, rad: float = 1.0):
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
            x_vals = [node.pose.x for node in path]
            y_vals = [node.pose.y for node in path]
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

def do_circle(robot_list: list[Robot], center: Pose, rad: float = 1.0, n: int = 1):
    # generate n * 12 points in a circle around center
    circle_pts = circle(n * 12, center, rad)
    colors = plt.cm.get_cmap('hsv', len(robot_list) + 1)
    
    
    
    
    plt.xlim(0, 10)
    plt.ylim(0, 10)
    

# simulate pretend robots
robot_list = []
for i in range(12):
    robot_list.append(Robot(name=f"robot{i+1}"))
    robot_list[i].pose=Pose(x=random.randint(0, 10), y=random.randint(0, 10), heading=0.0)
    robot_list[i].globe_dia_cm = random.randint(15, 45)
    if (i < 5):
        robot_list[i].group_id = 0

# print(do_groups(robot_list, 25, 0.5, obstacles + group0_obstacles))
# print(form_groups(robot_list, obstacles))
print(form_circle(robot_list, Pose(5, 5), 3.0))
# graph_force()

# def check_collision(robot_list: list[Robot]):
#     # generate pair matrix of robot distances
#     positions = np.array([[robot.pose.x, robot.pose.y] for robot in robot_list])
#     dist_matrix = np.linalg.norm(positions[:, np.newaxis] - positions, axis=2)
#     np.fill_diagonal(dist_matrix, np.inf)  # ignore self-collision by setting diagonal to infinity
#     print("Distance Matrix:\n", dist_matrix)
#     collision_threshold = 0.5  # define a threshold distance for collision
#     collisions = np.where(dist_matrix < collision_threshold)
#     collision_pairs = set()
#     for i in range(len(collisions[0])):
#         r1 = collisions[0][i]
#         r2 = collisions[1][i]
#         if r1 < r2:  # avoid duplicate pairs
#             collision_pairs.add((robot_list[r1].name, robot_list[r2].name))
#     return list(collision_pairs)

# # test the check_collision function by making a list of robtos
# robot_list = [
#     Robot(name="robot1", pose=Pose(1, 1, 0.0)),
#     Robot(name="robot2", pose=Pose(1.2, 1.2, 0.0)),
#     Robot(name="robot3", pose=Pose(5, 5, 0.0)),
#     Robot(name="robot4", pose=Pose(5.1, 5.1, 0.0)),
#     Robot(name="robot5", pose=Pose(9, 9, 0.0))
# ]
# check_collision(robot_list)

# # print(do_circle(robot_list))
# # do_line(robot_list, Pose(1, 1, 0.0), Pose(9, 9, 0.0))

# # do_random(robot_list, 25, 0.5)

# # print(astar(start, target))

# # for direction in Direction:
#     # print(direction, direction.value)