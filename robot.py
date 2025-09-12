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
from formations import *

BROKER = "midway.breckstampede.org"  # Change to your broker address
TOPIC = "robots/#"  # Subscribe to all subtopics

robot: list[Robot] = []

def write_data():
    # write all current server data to pickles
    
    # write robot data
    with open('robots.pkl', 'wb') as f:
        pickle.dump(robots, f)
    
    # for i in range(len(robots)):

def load_robots():
    # read robot data
    with open('robots.pkl', 'rb') as f:
        return pickle.load(f)

# Official list of robots goes here:
# robots: list[Robot] = []
# for i in range(12):
#     robots.append(Robot(i, 0, Pose(0, 0, 0.0), 0, False, [Color(0, 0, 0)], [Pose(0, 0, 0.0)], 0, 0, 29.21, 0, None))
#     match (i):
#         case 1: 
# global variables holding important data

circle_center = Pose(5, 5)
static_obstacles = list[Square]
obstacles: list[Square] = [Square(Pose(6.7, 10.3),to_meters(24)), Square(Pose(6.7, 5.0), to_meters(24)), Square(Pose(6.7, 15.7), to_meters(24))]
circle_nums: list[int] = []
robots: list[Robot] = []
robots_dict: dict[str, Robot] = {}

# settings
# automatically continue once all robots arrive in position for circle
auto_continue: bool = True
# DANGEROUS: will force an override (ignore any safety checks)
# for any changes issued by the website
    # example of potential problem if enabled: path_interrupt called before
    # all robots have reached their position for circle

force_override: bool = False

# represents the current path being run
    # 0: random)
    # 1: line
    # 2: circle
    # 3: groups
selected_path: int = 0


for i in range(12):
    robots.append(Robot("", i, -1, Pose(1, 1), 0, False, None, RobotState.OFFLINE, 0, 0, 0, -1, None))
    match(i):
        case 0:
            robots[i].globe_rad_m = to_meters(11.75 / 2)
            robots[i].id = "6cc8404e2904"
        case 1:
            robots[i].globe_rad_m = to_meters(11.75 / 2)
            robots[i].id = "3c8a1f5d2798"
        case 2:
            robots[i].globe_rad_m = to_meters(14 / 2)
            robots[i].id = "004b128e3580"
        case 3:
            robots[i].globe_rad_m = to_meters(16 / 2)
            robots[i].id = "6cc8408a9584"
        case 4:
            robots[i].globe_rad_m = to_meters(18 / 2)
            robots[i].id = "6cc840862c9c"
        case 5:
            robots[i].globe_rad_m = to_meters(18 / 2)
            robots[i].id = "6cc8404f6930"
        case 6:
            robots[i].globe_rad_m = to_meters(19.5 / 2)
            robots[i].id = "3c8a1f5d75dc"
        case 7:
            robots[i].globe_rad_m = to_meters(21.5 / 2)
            robots[i].id = "6cc84087a038"
        case 8:
            robots[i].globe_rad_m = to_meters(21.5 / 2)
            robots[i].id = "004b12902ed0"
        case 9:
            robots[i].globe_rad_m = to_meters(21.5 / 2)
            robots[i].id = "004b12900210"
        case 10:
            robots[i].globe_rad_m = to_meters(23.5 / 2)
            robots[i].id = "6cc8404ff42c"
        case 11:
            robots[i].globe_rad_m = to_meters(23.5 / 2)
            robots[i].id = "004b12534944"

def on_connect(client, userdata, flags, reason_code, properties):
    if reason_code == 0:
        print("Connected to MQTT Broker!")
        client.subscribe(TOPIC)
    else:
        print(f"Failed to connect, return code {reason_code}")

def on_message(client, userdata, msg):
    topics = msg.topic.split('/')
    if topics[0] == "robots" and not topics[1] in robots_dict: # add the robot into the dictionary
        robots_dict[topics[1]] = next([x for x in robots if x.id == [topics[1]]])
        robots_dict[topics[1]].state = RobotState.LOADING
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
        robots_dict[topics[1]].heading = heading
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
        robots_dict[topics[1]].led = led

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
        robots_dict[topics[1]].telemetry = telemetry
    
    if topics[2] == "odometry":
        if topics[3] == "pose":
            data = json.loads(msg.payload.decode())
            pose = Pose(
                x=data['x'],
                y=data['y'],
                heading=data['heading']
            )
            robots_dict[topics[1]].pose = pose

        elif topics[3] == "pose2":
            data = json.loads(msg.payload.decode())
            pose2 = Pose(
                x=data['x'],
                y=data['y'],
                heading=data['heading']
            )
            robots_dict[topics[1]].pose2 = pose2

    if topics[2] == "wifi" and topics[3] == "rssi":
        data = json.loads(msg.payload.decode())
        rssi = data['rssi']
        robots_dict[topics[1]].wifi_rssi = rssi  

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
            robots_dict[topics[1]].tcs = tcs
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
            robots_dict[topics[1]].tcs = tcs

            robots_dict[topics[1]].tcs.color_temperature = 0
            robots_dict[topics[1]].tcs.status = status
            robots_dict[topics[1]].tcs = tcs
        
    if topics[2] == "reroute":
        data = json.loads(msg.payload.decode())
        robots_dict[topics[1]].reroute = data['reroute']

    if topics[2] == "path_step":
        data = json.loads(msg.payload.decode())
        robots_dict[topics[1]].path_step = data['path_step']
        
        if robots_dict[topics[1]].path_step >= len(robots_dict[topics[1]].path):
            # if forming circle, wait
            if robots_dict[topics[1]].path_id == 0:
                random_path(robots[topics[1]], 25, 0.5)
            if robots_dict[topics[1]].path_id == 1:
                # switch to waiting
                robots_dict[topics[1]].path_id = -1
            # if looping, restart
            elif robots_dict[topics[1]].path_id == 2:
                robots[topics[1]].path_step = 0
            # if #6 reaches target point, stop
            elif robots_dict[topics[1]].path_id == 4:
                robots_dict[topics[1]].path_id == -2
            elif robots_dict[topics[1]].path_id == 5:
                robots_dict[topics[1]].path_id = 0
                # plan path + insert force
            elif robots_dict[topics[1]].path_id == 6:
                robots_dict[topics[1]].path_id = 7

    if msg.topic.endswith("/ping") and msg.payload.decode() == "pong":
        robots_dict[topics[1]].last_ping = time.time()

def send_ping(target: Robot):
    client.publish(f"robots/{target.id}/ping", "ping")

def send_led(target: Robot, r: int, g: int, b: int):
    # make sure colors are in bounds
    r = max(0, min(255, r))
    g = max(0, min(255, g))
    b = max(0, min(255, b))
    client.publish(f"robots/{target.id}/command/pattern", f"[{r}, {g}, {b}]")

def send_path(target: Robot, path: list[Pose]):
    print(f"[" + ", ".join(str(x) for x in path.x) + "], [" + ", ".join(str(y) for y in path.y) + "]")
    client.publish(f"robots/{target.id}/command/pattern", f"[" + ", ".join(str(x) for x in path.x) + "], [" + ", ".join(str(y) for y in path.y) + "]")


client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
client.on_connect = on_connect
client.on_message = on_message
client.username_pw_set("esp", "esp32")
client.connect(BROKER, 8883, 60)
client.loop_start()

# obstacles = []

group0_obstacles = [
    Square(Pose(-20, 0, 0.0), 10),
    Square(Pose(0, 20, 0.0), 10),
    Square(Pose(20, 0, 0.0), 10),
    Square(Pose(0, -20, 0.0), 10)
]


def choose_path(input: int):
    # for caleb: make dropdown menu on website for the paths and a "Send" button
    # which calls this method with the correct path #
    # 0: random
    # 1: line
    # 2: circle
    # 3: groups
    
    # for some reason stores as a local variable
    # selected_path = input
    
    if input == 0:
        do_random()
    # elif input == 1:
    #     do_line()
    elif input == 2:
        form_circle()
        
    return input
    
def path_interrupt():
    # for caleb: to be called during certain paths (ex groups and line)
    # from website to advance path stage (ex 6 moves on or the line starts moving)
    if selected_path == 1:
        if force_override:
            move_line(robots)
        else:
            move_line(all(r.path_id == 7 for r in robots))
    elif selected_path == 2 and (force_override or all(r.path_id == -1 for r in robots)):
        do_circle(robots, Pose(0,0))
    # elif selected_path == 3:

def main():
    while True:
        time.sleep(0.1)
        # cycle once every 0.1s
        
        for r in robots:
            if r.state == RobotState.LOADING:
                print(f"Robot {robots[r]} is now loading path...")
                r.path_step = 0
                print(f"Checking for Robot {robots[r]}'s path")
                if r.path:
                    send_path(r, r.path)
                    print(f"Path sent to {robots[r]}. Status is now active.")
                    r.state = RobotState.ACTIVE
                    match(selected_path):
                        case 1:
                            r.path_id = 6
                            # forming line
                else:
                    print(f"No path found for Robot {robots[r]}. Going into idle...")
                    r.state = RobotState.IDLE
                
        
        obstacles.clear()
        obstacles = static_obstacles
        # for r in robots:
            # obstacles.append(Square(r.pose, r.globe_rad_m))
        
        # check if all robots path_id is -1
        all_waiting = auto_continue and all(r.path_id == -1 for r in robots)
        if all_waiting:
            print("All robots reached circle, running do_circle")
            do_circle(robots, circle_center, circle_nums, 3, 5)
            for r in robots:
                r.path_id = 2

# simulate pretend robots
# robot_list = []
# for i in range(12):
#     robot_list.append(Robot(num_ordered= i))
#     robot_list[i].pose=Pose(x=random.randint(0, 10), y=random.randint(0, 10), heading=0.0)
#     robot_list[i].globe_dia_cm = random.randint(15, 45)
#     if (i < 5):
#         robot_list[i].group_id = 0
#     if (i == 6):
#         robot_list[i].group_id = 1
#     if (i > 6):        robot_list[i].group_id = 2
#     if (i >= 10):
#         robot_list[i].group_id = 3

# robots = robot_list

# do_line()
# move_line()

# print(robots)
# write_data()
# robots = None
# print(robots)
# robots = load_robots()
# print(robots)

# print(do_groups(robot_list, 25, 0.5, obstacles + group0_obstacles))
# print(form_groups(robot_list, obstacles))

# print(do_6(robot_list))

# random_path(Robot(num_ordered=0, pose=Pose(0,0,0)), 25, 0.5, Pose(0,0,0))
# plt.grid()
# plt.savefig("forcefield_test.png")

# print(form_circle(robot_list, Pose(5, 5), 3.0))
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


# # print(do_circle(robot_list))
# # do_line(robot_list, Pose(1, 1, 0.0), Pose(9, 9, 0.0))

# # do_random(robot_list, 25, 0.5)



# TO DO: either make main loop take parameters or remove parameters from other functions

# robots = robot_list
# circle_nums = form_circle(robots, circle_center, 3.0)
# for r in robots:
#     r.path_id = -1
#     r.pose = r.path[len(r.path) - 1]
# main()

# selected_path = 1
# do_line()
main()