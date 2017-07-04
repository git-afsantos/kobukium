
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
import os

from threading import Thread

from BaseHTTPServer import HTTPServer
from SimpleHTTPServer import SimpleHTTPRequestHandler

# third-party packages
# --------------------

# https://github.com/dpallot/simple-websocket-server
from SimpleWebSocketServer import SimpleWebSocketServer, WebSocket

# project packages
# ----------------

from .world import World
try:
    from .robot import Robot, RobotStatusError
except ImportError:
    from .fake_robot import Robot, RobotStatusError


# useful constants
# ----------------

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
            wd = os.getcwd()
            try:
                os.chdir(self.server.data_dir)
                SimpleHTTPRequestHandler.do_GET(self)
            finally:
                os.chdir(wd)
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
        try:
            self.robot.init_ros()
            host = host or self.robot.get_param("~host", "localhost")
            port = port or self.robot.get_param("~port", 80)
            ws_port = ws_port or self.robot.get_param("~ws_port", 8080)
            data_dir = data_dir or self.robot.get_param("~data_dir", os.getcwd())
            self.world.read_from(data_dir)
            self._start_servers(host, port, ws_port, data_dir)
            self.robot.spin()
        finally:
            self.shutdown()

    def shutdown(self):
        if not self.wsserver is None:
            self.wsserver.shutdown_requested = True
            self._ws_thread.join()
        if not self.httpserver is None:
            self.httpserver.shutdown()
            self._http_thread.join()
        self.robot.shutdown()
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
