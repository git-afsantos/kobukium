
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


# standard packages
# -----------------

import os
import sys
import time

from datetime import datetime
from random import randint
from threading import Lock


# useful constants
# ----------------

TIME_FORMAT = "%Y-%m-%d %H:%M:%S"


###############################################################################
# Fake Robot
###############################################################################

class Robot(object):
    def __init__(self, world):
        self.lock = Lock()
        self.status = "offline"
        self.location = None
        self.world = world
        self.missions = []
        self._mission = None
        self.dirty = False
        self.move_base = None
        self.simulate_online = False
        self.move_goal_set = False
        self.goal_cancelled = False
        self.navigating = -1
        self.navi_status = ["SUCCEEDED", "ABORTED"]

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
                self.goal_cancelled = True
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
        print "[FakeRobot] starting node"
        self.simulate_online = True

    # Thread: ros
    def shutdown(self):
        if self._mission and self._mission["status"] == "in progress":
            self.goal_cancelled = True

    # Thread: ros
    def get_param(self, name, default):
        return default

    # Thread: ros
    def spin(self):
        try:
            while True:
                time.sleep(1)
                if self.simulate_online:
                    self.simulate_online = False
                    self._on_diagnostics(None)
                if self.move_base.cancelled:
                    self.move_base.cancelled = False
                    self.navigating = -1
                    self._on_move_done("PREEMPTED", None)
                elif self.navigating > 0:
                    self.navigating -= 1
                elif self.navigating == 0:
                    self.navigating = -1
                    status = self.navi_status[randint(0, 1)]
                    self._on_move_done(status, None)
                elif self.move_goal_set:
                    self.move_goal_set = False
                    self._on_move_active()
                    self.navigating = 7
        except KeyboardInterrupt as e:
            return False
        return True


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
        print "[FakeRobot] publishing initial pose:", location.name

    # Thread: websocket
    def _go_to(self, location):
        print "[FakeRobot] publishing move base goal", location
        self.move_goal_set = True

    # Thread: ros
    def _on_move_active(self):
        with self.lock:
            self.missions.append(self._mission)
            self.status = "busy"
            self.dirty = True

    # Thread: ros
    def _on_move_feedback(self, msg):
        pass

    # Thread: ros
    def _on_move_done(self, state, msg):
        with self.lock:
            self.status = "idle"
            self._mission["completed"] = datetime.now().strftime(TIME_FORMAT)
            if state == "SUCCEEDED":
                self._mission["status"] = "completed"
                self.location = self._mission["goal"]
            elif state == "ABORTED":
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


###############################################################################
# Custom Exceptions
###############################################################################

class RobotStatusError(Exception):
    pass
