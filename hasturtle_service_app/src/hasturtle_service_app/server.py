
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
    "status":       string ("offline", "idle", "busy", "planning"),
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
    "goal":         string
}
C {
    "feedback":     string ("cancel", "success", "failure")
}
"""


# standard packages
# -----------------

import json
import os
from datetime import datetime
from threading import Thread, Lock

from BaseHTTPServer import HTTPServer
from SimpleHTTPServer import SimpleHTTPRequestHandler

# third-party packages
# --------------------

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion

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
    def __init__(self):
        self.lock = Lock()
        self.status = "offline"
        self.location = None
        self.missions = []
        self._mission = None
    # -- NOTE: I think we do not need to send all state updates to the client
    # -- so we just need a flag telling whether the client needs an update
        self.dirty = False
        self.move_base = None

    def get_state(self):
        state = None
        with self.lock:
            state = {
                "status":   self.status,
                "location": self.location,
                "mission":  dict(self._mission) if self._mission else None
            }
        return state

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

    def set_mission(self, goal):
        """status can be one of
            {"in progress", "completed", "successful", "failed", "aborted"}
        """
        with self.lock:
            self._check_status("idle")
            if goal == self.location:
                return
            # TODO check if goal is valid
            self._mission = {
                "location": self.location,
                "time":     datetime.now().strftime(TIME_FORMAT),
                "goal":     goal,
                "status":   "in progress"
            }
            self.status = "planning"
            # pos, quat = goal to pose
            position = (1.22, 2.56)
            quaternion = (0.000, 0.000, 0.000, 1.000)
        self._go_to(position, quaternion)

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

    def init_ros(self):
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base.wait_for_server()
        
        # pub = rospy.Publisher("initialpose", String, queue_size=10)
        # initialpose (geometry_msgs/PoseWithCovarianceStamped)
        # http://wiki.ros.org/amcl

        # sub amcl_pose (geometry_msgs/PoseWithCovarianceStamped)

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

    def _go_to(self, position, quaternion):
        position = Point(position[0], position[1], 0.0)
        quaternion = Quaternion(quaternion[0], quaternion[1],
                                quaternion[2], quaternion[3])
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(position, quaternion)
        self.move_base.send_goal(goal, done_cb = self._on_move_done,
                                 active_cb = self._on_move_active,
                                 feedback_cb = self._on_move_feedback)

    def _on_move_active(self):
        with self.lock:
            self.missions.append(self._mission)
            self.status = "busy"
            self.dirty = True

    def _on_move_feedback(self, msg):
        x = msg.base_position.pose.position.x
        y = msg.base_position.pose.position.y
        w = msg.base_position.pose.orientation.w
        # check if coordinates fit a new location
        with self.lock:
            pass

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


###############################################################################
# Application Controller
###############################################################################

class RobotManager(object):
    def __init__(self):
        self.robot = Robot()
        self.httpserver = None
        self.wsserver = None
        self._http_thread = None
        self._ws_thread = None

    def start(self, host = "localhost", port = 80, ws_port = 8080,
              data_dir = os.getcwd()):
        rospy.init_node("hasturtle")
        rospy.on_shutdown(self.shutdown)
        # TODO get params from server and use these as defaults
        # http://wiki.ros.org/rospy_tutorials/Tutorials/Parameters
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
