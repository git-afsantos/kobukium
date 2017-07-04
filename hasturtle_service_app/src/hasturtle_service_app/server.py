
#Copyright (c) 2017 Andre Santos
#
#Permission is hereby granted, free of charge, to any person obtaining a copy
#of this software and associated documentation files (the "Software"), to deal
#in the Software without restriction, including without limitation the rights
#to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#copies of the Software, and to permit persons to whom the Software is
#furnished to do so, subject to the following conditions:

#The above copyright notice and this permission notice shall be included in
#all copies or substantial portions of the Software.

#THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
#THE SOFTWARE.


# Useful links:
# http://wiki.ros.org/actionlib/DetailedDescription
# http://wiki.ros.org/actionlib     look for "Full API reference"
# https://github.com/markwsilliman/turtlebot/


"""
Websocket messages:
S {
    "status":       string ("offline", "idle", "busy", "planning", "lost"),
    "location":     string,
    "mission":  ?{
        "location": string,
        "time":     string,
        "goal":     string,
        "status":   string ("in progress", "completed",
                            "successful", "failed", "aborted")
    }
}
C {
    "hint":         string
}
C {
    "goal":         string
}
C {
    "feedback":     string ("cancel", "success", "failure")
}
"""


# standard packages
# -----------------

import json
import math
import os

from datetime import datetime
from threading import Thread, Lock

from BaseHTTPServer import HTTPServer
from SimpleHTTPServer import SimpleHTTPRequestHandler

# third-party packages
# --------------------

import rospy
import actionlib

from actionlib_msgs.msg import GoalStatus
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Pose, Point, Quaternion, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# https://github.com/dpallot/simple-websocket-server
from SimpleWebSocketServer import SimpleWebSocketServer, WebSocket


# useful constants
# ----------------

TIME_FORMAT = "%Y-%m-%d %H:%M:%S"

DYNAMIC_JS_WS_TEMPLATE = """
(function () {{
    "use strict";
    window.App.wsAddress = "ws://{}:{}";
}})();
"""


###############################################################################
# HTTP Server Definition
###############################################################################

class TurtleRequestHandler(SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_my_headers()
        SimpleHTTPRequestHandler.end_headers(self)

    def send_my_headers(self):
        self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
        self.send_header("Pragma", "no-cache")
        self.send_header("Expires", "0")

    def do_GET(self):
        if self.path.startswith("/data/"):
    # -- NOTE: This requires some black magic.
    # -- Change the working directory and let the superclass handle it.
            self.path = self.path[5:]
            wd = self.directory
            try:
                self.directory = self.server.data_dir
                SimpleHTTPRequestHandler.do_GET(self)
            finally:
                self.directory = wd
        elif self.path == "js/ws.js" or self.path == "/js/ws.js":
            self.send_response(200)
            self.send_header("Content-type", "application/javascript")
            self.end_headers()
            self.wfile.write(self.server.ws_js)
        else:
            SimpleHTTPRequestHandler.do_GET(self)


class TurtleHTTPServer(HTTPServer):
    def __init__(self, host, port, ws_port, data_dir):
        self.data_dir = data_dir
        self.ws_js = DYNAMIC_JS_WS_TEMPLATE.format(host, ws_port)
        HTTPServer.__init__(self, (host, port), TurtleRequestHandler)


###############################################################################
# Websocket Server Definition
###############################################################################

class TurtleSocketHandler(WebSocket):
    def handleMessage(self):
        data = json.loads(self.data)
        try:
            if "goal" in data:
                print "received a new goal", data["goal"]
                self.server.robot.set_mission(data["goal"])
            elif "feedback" in data:
                print "received mission feedback", data["feedback"]
                self.server.robot.set_user_feedback(data["feedback"])
            elif "hint" in data:
                print "received initial pose", data["hint"]
                self.server.robot.set_location_hint(data["hint"])
        except RobotStatusError as e:
            print e.message
        message = json.dumps(self.server.robot.get_state())
        self.sendMessage(unicode(message))

    def handleConnected(self):
        print "CONNECTED", self.address
    # -- NOTE: cannot send message right away
    # -- it gets mixed with handshake headers and crashes client
        # message = json.dumps(self.server.robot.get_state())
        # self.sendMessage(unicode(message))
        self.server.robot.dirty = True

    def handleClose(self):
        print "DISCONNECTED", self.address


class TurtleSocketServer(SimpleWebSocketServer):
    def __init__(self, host, port, robot):
        self.robot = robot
        self.shutdown_requested = False
        SimpleWebSocketServer.__init__(self, host, port, TurtleSocketHandler)

    def update_all(self, message):
        for connection in self.connections.itervalues():
            connection.sendMessage(unicode(message))

    def serve_forever(self):
        while not self.shutdown_requested:
            state = self.robot.get_dirty_state()
            if not state is None:
                self.update_all(json.dumps(state))
            self.serveonce()
        self.close()


###############################################################################
# Robot Data Model
###############################################################################

class Robot(object):
    COVARIANCE = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                  0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                  0.0,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                  0.0,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                  0.0,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]

    def __init__(self, world):
        self.lock = Lock()
        self.status = "offline"
        self.location = None
        self.world = world
        self.missions = []
        self._mission = None
    # -- NOTE: I think we do not need to send all state updates to the client
    # -- so we just need a flag telling whether the client needs an update
        self.dirty = False
    # -- ROS stuff
        self.move_base = None
        self.diagnostics = None # used to check whether the robot is online

    # Thread: websocket
    def get_state(self):
        state = None
        with self.lock:
            state = {
                "status":   self.status,
                "location": self.location,
                "mission":  dict(self._mission) if self._mission else None
            }
        return state

    # Thread: websocket
    def get_dirty_state(self):
        state = None
        with self.lock:
            if self.dirty:
                state = {
                    "status":   self.status,
                    "location": self.location,
                    "mission":  dict(self._mission) if self._mission else None
                }
                self.dirty = False
        return state

    # Thread: websocket
    def set_mission(self, goal):
        with self.lock:
            self._check_status("idle")
            if goal == self.location or not goal in self.world:
                return
            self._mission = {
                "location": self.location,
                "time":     datetime.now().strftime(TIME_FORMAT),
                "goal":     goal,
                "status":   "in progress"
            }
            self.status = "planning"
        self._go_to(self.world.get(goal))

    # Thread: websocket
    def set_location_hint(self, location):
        with self.lock:
            self._check_status("lost")
            if not location in self.world:
                return
            self.location = location
            location = self.world.get(location)
            self.status = "idle"
        self._set_location(location)

    # Thread: websocket
    def set_user_feedback(self, feedback):
        with self.lock:
            if feedback == "cancel":
                self._check_status("busy")
                self._check_mission_status("in progress")
                print "cancelled current mission"
                self.move_base.cancel_goal()
                self.status = "idle"
                self._mission["status"] = "aborted"
                self._mission = None
            elif feedback == "success":
                self._check_status("idle")
                self._check_mission_status("completed")
                print "recording positive mission feedback"
                self._mission["status"] = "successful"
                self._mission = None
            elif feedback == "failure":
                self._check_status("idle")
                self._check_mission_status("completed")
                print "recording negative mission feedback"
                self._mission["status"] = "failed"
                self._mission = None

    # Thread: ros
    def init_ros(self):
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base.wait_for_server()
        self.initial_pose = rospy.Publisher("initialpose",
                                            PoseWithCovarianceStamped,
                                            queue_size = 10, latch = True)
        with self.lock:
            if self.status == "offline":
                self.diagnostics = rospy.Subscriber("diagnostics",
                                                    DiagnosticArray,
                                                    self._on_diagnostics)
        
        # sub amcl_pose (geometry_msgs/PoseWithCovarianceStamped)

    # Thread: ros
    def shutdown(self):
        if self._mission and self._mission["status"] == "in progress":
            self.move_base.cancel_all_goals()


    def _check_status(self, status):
        if self.status != status:
            raise RobotStatusError("expected " + status + "; found " + self.status)

    def _check_mission_status(self, status):
        if status is None:
            if not self._mission is None:
                raise RobotStatusError("expected not to have a mission")
            return
        if self._mission is None:
            raise RobotStatusError("expected to have a mission")
        if status != self._mission["status"]:
            raise RobotStatusError("expected mission status " + status
                                   + "; found " + self._mission["status"])

    # Thread: websocket
    def _set_location(self, location):
        hint = PoseWithCovarianceStamped()
        hint.header.frame_id = "map"
        hint.header.stamp = rospy.Time.now()
        hint.pose.pose.position.x = location.x
        hint.pose.pose.position.y = location.y
        hint.pose.pose.position.z = 0.0
        hint.pose.pose.orientation.x = 0.0
        hint.pose.pose.orientation.y = 0.0
        hint.pose.pose.orientation.z = location.orientation[2]
        hint.pose.pose.orientation.w = location.orientation[3]
        hint.pose.covariance = Robot.COVARIANCE
        self.initial_pose.publish(hint)

    # Thread: websocket
    def _go_to(self, location):
        position = location.to_position(Point)
        quaternion = location.to_quaternion(Quaternion)
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(position, quaternion)
        self.move_base.send_goal(goal, done_cb = self._on_move_done,
                                 active_cb = self._on_move_active,
                                 feedback_cb = self._on_move_feedback)

    # Thread: ros
    def _on_move_active(self):
        with self.lock:
            self.missions.append(self._mission)
            self.status = "busy"
            self.dirty = True

    # Thread: ros
    def _on_move_feedback(self, msg):
        x = msg.base_position.pose.position.x
        y = msg.base_position.pose.position.y
        w = msg.base_position.pose.orientation.w
        with self.lock:
            location = self.location
            self.location = self.world.where_is(x, y)
            if self.location != location:
                self.dirty = True
                # do not risk setting a True to False by direct assignment

    # Thread: ros
    def _on_move_done(self, state, msg):
        with self.lock:
            self.status = "idle"
            self._mission["completed"] = datetime.now().strftime(TIME_FORMAT)
            if state == GoalStatus.SUCCEEDED:
                self._mission["status"] = "completed"
                self.location = self._mission["goal"]
            elif state == GoalStatus.ABORTED:
                self._mission["status"] = "failed"
                self._mission = None
            else:   # REJECTED, PREEMPTED, RECALLED
                self._mission["status"] = "aborted"
                self._mission = None
            self.dirty = True

    # Thread: ros
    def _on_diagnostics(self, msg):
        with self.lock:
            if self.status == "offline":
                self.status = "lost"
                self.dirty = True
            self.diagnostics.unregister()
            self.diagnostics = None


###############################################################################
# Location Data
###############################################################################

class Location(object):
    def __init__(self, data):
        self.id     = data["id"]
        self.name   = data["name"]
        self.x      = data["position"]["x"]
        self.y      = data["position"]["y"]
        self.radius = data["radius"]
        orientation = data.get("orientation")
        if orientation:
            self.orientation = (orientation.get("x", 0.0),
                                orientation.get("y", 0.0),
                                orientation.get("z", 0.0), orientation["w"])
        else:
            self.orientation = (0.0, 0.0, 0.0, 1.0)

    def to_position(self, cls):
        return cls(self.x, self.y, 0.0)

    def to_quaternion(self, cls):
        return cls(*self.orientation)

    def contains(self, x, y):
        return self.radius > math.sqrt((x - self.x)**2 + (y - self.y)**2)


class World(object):
    def __init__(self):
        self.locations = {}

    def get(self, location):
        return self.locations.get(location)

    def read_from(self, data_dir):
        self.locations = {}
        with open(os.path.join(data_dir, "locations.json"), "r") as handle:
            locations = json.load(handle)
        for data in locations:
            self.locations[data["id"]] = Location(data)

    def where_is(self, x, y):
        for location in self.locations.itervalues():
            if location.contains(x, y):
                return location.id
        return None

    def __contains__(self, location):
        return location in self.locations


###############################################################################
# Application Controller
###############################################################################

class RobotManager(object):
    def __init__(self):
        self.world = World()
        self.robot = Robot(self.world)
        self.httpserver = None
        self.wsserver = None
        self._http_thread = None
        self._ws_thread = None

    def start(self, host = None, port = None, ws_port = None, data_dir = None):
        """Prioritise arguments, then ROS params, then defaults.
        """
        rospy.init_node("hasturtle")
        rospy.on_shutdown(self.shutdown)
        host = host or rospy.get_param("~host", "localhost")
        port = port or rospy.get_param("~port", 80)
        ws_port = ws_port or rospy.get_param("~ws_port", 8080)
        data_dir = data_dir or rospy.get_param("~data_dir", os.getcwd())
        self.world.read_from(data_dir)
        self._start_servers(host, port, ws_port, data_dir)
        try:
            self.robot.init_ros()
            rospy.spin()
        except rospy.ROSInterruptException:
            pass
        finally:
            self.shutdown()

    def shutdown(self):
        self.wsserver.shutdown_requested = True
        self.httpserver.shutdown()
        self.robot.shutdown()
        self._ws_thread.join()
        self._http_thread.join()
        self.wsserver       = None
        self.httpserver     = None
        self._ws_thread     = None
        self._http_thread   = None


    def _start_servers(self, host, port, ws_port, data_dir):
    # -- Websocket server
        self.wsserver = TurtleSocketServer(host, ws_port, self.robot)
        self._ws_thread = Thread(target = self.wsserver.serve_forever)
        self._ws_thread.daemon = True
        self._ws_thread.start()
    # -- HTTP server
        self.httpserver = TurtleHTTPServer(host, port, ws_port, data_dir)
        self._http_thread = Thread(target = self.httpserver.serve_forever)
        self._http_thread.daemon = True
        self._http_thread.start()


###############################################################################
# Custom Exceptions
###############################################################################

class RobotStatusError(Exception):
    pass
