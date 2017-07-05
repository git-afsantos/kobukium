
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


# standard packages
# -----------------

from datetime import datetime
from threading import Lock

# third-party packages
# --------------------

import rospy
import actionlib

from actionlib_msgs.msg import GoalStatus
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Pose, Point, Quaternion, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


# useful constants
# ----------------

TIME_FORMAT = "%Y-%m-%d %H:%M:%S"


###############################################################################
# Robot Model
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
        self.current_mission = None
        self._mission = None    # mission candidate
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
                "mission":  (dict(self.current_mission)
                             if self.current_mission else None)
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
                    "mission":  (dict(self.current_mission)
                                 if self.current_mission else None)
                }
                self.dirty = False
        return state

    # Thread: websocket
    def set_mission(self, goal):
        with self.lock:
            self._check_status("idle")
            if not goal in self.world:
                raise ValueError("The selected location is not valid!")
            self._mission = {
                "location": self.location,
                "time":     datetime.now().strftime(TIME_FORMAT),
                "goal":     goal,
                "status":   "in progress"
            }
            self.status = "planning"
            self.dirty = True
        self._go_to(self.world.get(goal))

    # Thread: websocket
    def set_location_hint(self, location):
        with self.lock:
            self._check_status("lost")
            if not location in self.world:
                raise ValueError("The selected location is not valid!")
            self.location = location
            location = self.world.get(location)
            self.status = "idle"
            self.dirty = True
        self._set_location(location)

    # Thread: websocket
    def set_user_feedback(self, feedback):
        with self.lock:
            if feedback == "cancel":
                self._check_status("busy")
                self._check_mission_status("in progress")
                print "[Robot] cancelled current mission"
                self.move_base.cancel_goal()
            elif feedback == "success":
                self._check_status("idle")
                self._check_mission_status("completed")
                print "[Robot] recording positive mission feedback"
                self.current_mission["status"] = "successful"
                self.current_mission = None
                self.dirty = True
            elif feedback == "failure":
                self._check_status("idle")
                self._check_mission_status("completed")
                print "[Robot] recording negative mission feedback"
                self.current_mission["status"] = "failed"
                self.current_mission = None
                self.dirty = True

    # Thread: ros
    def init_ros(self):
        rospy.init_node("hasturtle")
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base.wait_for_server()
        self.initial_pose = rospy.Publisher("initialpose",
                                            PoseWithCovarianceStamped,
                                            queue_size = 10, latch = True)
        if self.status == "offline":
            self.diagnostics = rospy.Subscriber("diagnostics", DiagnosticArray,
                                                self._on_diagnostics)
        
        # sub amcl_pose (geometry_msgs/PoseWithCovarianceStamped)

    # Thread: ros
    def shutdown(self):
        if (self.current_mission
                and self.current_mission["status"] == "in progress"):
            self.move_base.cancel_all_goals()

    # Thread: ros
    def get_param(self, name, default):
        return rospy.get_param(name, default)

    # Thread: ros
    def spin(self):
        try:
            rospy.spin()
        except rospy.ROSInterruptException as e:
            return False
        return True


    def _check_status(self, status):
        if self.status != status:
            raise RobotStatusError("expected " + status + "; found " + self.status)

    def _check_mission_status(self, status):
        if status is None:
            if not self.current_mission is None:
                raise RobotStatusError("expected not to have a mission")
            return
        if self.current_mission is None:
            raise RobotStatusError("expected to have a mission")
        if status != self.current_mission["status"]:
            raise RobotStatusError("expected mission status " + status
                                   + "; found " + self.current_mission["status"])

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
            self.current_mission = self._mission
            self._mission = None    # candidate graduates
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
            self.current_mission["completed"] = datetime.now().strftime(TIME_FORMAT)
            if state == GoalStatus.SUCCEEDED:
                self.current_mission["status"] = "completed"
                self.location = self.current_mission["goal"]
            elif state == GoalStatus.ABORTED:
                self.current_mission["status"] = "failed"
                self.current_mission = None
            else:   # REJECTED, PREEMPTED, RECALLED
                self.current_mission["status"] = "aborted"
                self.current_mission = None
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
# Custom Exceptions
###############################################################################

class RobotStatusError(Exception):
    pass
