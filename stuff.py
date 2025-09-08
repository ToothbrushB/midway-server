from enum import Enum
from flask import Flask, Response, render_template
import cv2
import json
import numpy as np
import time
import math
import random
import paho.mqtt.client as mqtt
from scipy.optimize import linear_sum_assignment
from dataclasses import dataclass, field
import matplotlib.pyplot as plt
import heapq
import paho.mqtt.client as mqtt

class RobotState(Enum):
    OFFLINE = 0
    IDLE = 1
    ACTIVE_MOVING = 2
    ACTIVE_STOPPED = 3
    OBSTRUCTED_STOPPED = 4
    OBSTRUCTED_REROUTING = 5
    OBSTRUCTED_SCANNING = 6

@dataclass
class Direction(Enum):
  UP = (0,1, 1)
  RIGHT = (1,0, 1)
  DOWN = (0,-1, 1)
  LEFT = (-1,0, 1)
  UP_RIGHT = (1,1, math.sqrt(2))
  DOWN_RIGHT = (1,-1, math.sqrt(2))
  DOWN_LEFT = (-1,-1, math.sqrt(2))
  UP_LEFT=(-1,1, math.sqrt(2))
  
@dataclass
class Color:
    r: int
    g: int
    b: int

    def __str__(self):
        return f"[{self.r}, {self.g}, {self.b}]"

@dataclass
class LED:
    color: Color
    step: int

@dataclass
class RobotHeading:
    x: float
    y: float
    z: float
    rad_accuracy: float
    accuracy: str

@dataclass
class Telemetry:
    speed: float
    ticks: int
    output: float
    setpoint: float
    hasPower: bool
  
@dataclass
class Vector:
    x: float
    y: float
    
    def mag(self) -> float:
        return math.hypot(self.x, self.y)
    
    def angle(self) -> float:
        return math.atan2(self.y, self.x)
    
    def __rmul__(self, scalar: float) -> 'Vector':
        return Vector(self.x * scalar, self.y * scalar)

@dataclass
class Pose:
    x: float
    y: float
    heading: float = 0.0
    # used to maintain consistency
    inches_per_node: float = 8

    def dist (self, other: 'Pose') -> float:
        return math.hypot(self.x - other.x, self.y - other.y)

    def __eq__(self, other: 'Pose') -> bool:
        return (abs(self.x - other.x) < 0.00001 and 
                abs(self.y - other.y) < 0.00001 and
                abs(self.heading - other.heading) < 0.00001)

    # merge translate methods into a single method with overloading
    def translate(self, vec, mag: float = 1.0) -> 'Pose':
        # vec can either be a Vector or a Direction, otherwise self is returned
        if (isinstance(vec, Vector)):
            return Pose(
                x = self.x + vec.x * mag,
                y = self.y + vec.y * mag,
                heading = self.heading
            )
        if (isinstance(vec, Direction)):
            return Pose(
                x = self.x + vec.value[0] * mag,
                y = self.y + vec.value[1] * mag,
                heading = self.heading
            )
        return self
        
    # def translate(self, vec: Vector) -> 'Pose':
    #     return Pose(
    #         x = self.x + vec.x,
    #         y = self.y + vec.y,
    #         heading = self.heading
    #     )

@dataclass
class Node:
  pose: Pose = None
  g_cost: float = 0
  h_cost: float = 0
  parent = None
  
  def __lt__(self, other: 'Node') -> bool:
    return self.f_cost() < other.f_cost()

  def __le__(self, other: 'Node') -> bool:
    return self.f_cost() <= other.f_cost()
  

  def __eq__(self, other: object) -> bool:
    if not isinstance(other, Node):
        return False
    return self.pose == other.pose
  def __hash__(self) -> int:
    return hash((self.pose.x, self.pose.y))

  def f_cost (self):
    return self.g_cost + self.h_cost

  def dist (self, other: 'Node') -> float:
    return self.pose.dist(other.pose)


# May be implemented if more complex obstacles are needed
# @dataclass
# class Polygon:
#     points: list[float, float]

#     # def test_point(self, pose: Pose):

@dataclass
class Square:
    # a square obstacle 
    center: Pose
    # size: float
    half_size: float

    def test_point(self, pose: Pose):
        if (pose.x >= self.center.x - self.half_size and pose.x <= self.center.x + self.half_size and
            pose.y >= self.center.y - self.half_size and pose.y <= self.center.y + self.half_size):
            return True
        return False

def test_obstacles(pose: Pose, obstacles: list[Square]) -> bool:
    for obstacle in obstacles:
        if obstacle.test_point(pose):
            return True
    return False

def test_line(start: Pose, end: Pose, obstacles: list[Square]) -> bool:
    return False

def astar(start: Pose, end: Pose, obstacles: list[Square], grid: float = 1.0) -> list[Pose]:
  # snap start and end to grid
    # print("Before round s: ", start)
    start = Pose(round(start.x / grid) * grid, round(start.y / grid) * grid, start.heading)
    # print("After round s: ", start)
    # print("Before round e: ", end)
    end = Pose(round(end.x / grid) * grid, round(end.y / grid) * grid, end.heading)
    # print("After round e: ", end)
    open: list[Node] = []
    heapq.heappush(open, Node(start, 0, start.dist(end)))
    closed: set[Node] = set()

    while open:
        current = heapq.heappop(open)
        # current = min(open, key = lambda x: x.f_cost())
        closed.add(current)
        # print(current)
        # add currents to collection and put in matplotlib plot
        # plt.scatter(current.pose.x, current.pose.y, color='red', s=10)
        # plt.pause(0.01)
        # plt.savefig("whatthepath.png")
        
        if current.pose == end:
            output = []
            node = current
            while node.parent is not None:
                output.append(node)
                node = node.parent
            output.append(Node(start, 0, start.dist(end)))
            output.reverse()
            return output

        for dir in Direction:
            translated_pose = current.pose.translate(dir, grid)
            neighbor = Node(translated_pose, dir.value[2]*grid + current.g_cost, translated_pose.dist(end))
            if test_obstacles(neighbor.pose, obstacles) or neighbor in closed:
                continue
            old_neighbor = next((n for n in open if n.pose == neighbor.pose), None)

            if old_neighbor is None or old_neighbor.g_cost > neighbor.g_cost:
                if old_neighbor: # if the neighbor is already in open, remove it
                    open.remove(old_neighbor)
                neighbor.parent = current
                heapq.heappush(open, neighbor)
    return [] # no path found

def circle(n: int, translate: Pose, rad: float = 1.0) -> list[Pose]:
    # num: number of points in the circle
    points = [Pose(rad*math.cos(k*2*math.pi/n) + translate.x, rad*math.sin(k*2*math.pi/n) + translate.y, 0.0) for k in range(n)]
    return points

@dataclass
class TCS:
    r: int
    g: int
    b: int
    c: int
    lux: int
    color_temperature: int
    status: str

@dataclass
class Robot:
    name: str
    last_ping: float = 0
    led = LED(Color(0, 0, 0), 0)
    heading = RobotHeading(0.0, 0.0, 0.0, 0.0, "UNDEFINED")
    telemetry = Telemetry(0.0, 0, 0.0, 0.0, False)
    pose: Pose = field(default_factory=lambda: Pose(0.0, 0.0, 0.0))
    pose2 = Pose(0.0, 0.0, 0.0)
    # the current target for navigation
    target_pose = Pose(0.0, 0.0, 0.0)
    wifi_rssi: int = 0
    tcs = TCS(0, 0, 0, 0, 0, 0, "not_initialized")
    reroute: bool = False
    blink_pattern: list[Color] = field(default_factory=lambda: [Color(0,0,0)]*10)
    # x_pattern: list[float] = []
    # y_pattern: list[float] = []
    path: list[Pose] = field(default_factory=list)
    # 23in = 58.42cm
    globe_dia_cm: int = 58.42
    group_id: int = 1 # change to 0 after testing
    # 0: large group (upper left)
    # 1: 10 (bottom left)
    # 2: medium group (bottom right)
    # 3: roaming pair
    # 4: 6 while unattached

    client: mqtt.Client = None
    
    def send_ping(self):
        self.client.publish(f"robots/{self.name}/ping", "ping")

    def send_blink_pattern(self):
        self.client.publish(f"robots/{self.name}/command/blink_pattern", ", ".join(str(color) for color in self.blink_pattern))

    def send_pattern(self):
        self.client.publish(f"robots/{self.name}/command/pattern", "[" + ", ".join(str(x) for x in self.x_pattern) + "], [" + ", ".join(str(y) for y in self.y_pattern) + "]")
