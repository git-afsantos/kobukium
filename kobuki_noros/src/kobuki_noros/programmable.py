
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


from collections import namedtuple
from math import pi, sqrt
from threading import Thread, Lock

import rospy
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

PI2     = 2 * pi
LINEAR  = 0.25  # m/s
ANGULAR = pi/4  # rad/s

def angle_distance(alpha, beta):
    d = alpha - beta
    s = 1 if (d >= 0.0 and d <= pi) or (d <= -pi and d >= -PI2) else -1
    d = abs(d) % PI2
    r = PI2 - d if d > pi else d
    return r * s

###############################################################################
#   Robot State
###############################################################################

State = namedtuple("State", ["translation", "rotation",
                             "bump_center", "bump_left", "bump_right"])


class Robot(object):
    def __init__(self):
        self.translation    = 0.0
        self.rotation       = 0.0
        self.bump_left      = False
        self.bump_right     = False
        self.bump_center    = False
        self.x              = 0.0
        self.y              = 0.0
        self.a              = 0.0
        self.lock           = Lock()

    # Thread: subscriber
    def set_odometry(self, x, y, a):
        with self.lock:
            if not (self.bump_center or self.bump_left or self.bump_right):
                self.translation += sqrt((x - self.x)**2 + (y - self.y)**2)
                self.rotation += angle_distance(a, self.a)
            self.x = x
            self.y = y
            self.a = a

    # Thread: subscriber
    def set_bumper(self, center, left, right):
        with self.lock:
            if center:
                self.bump_center = True
            if left:
                self.bump_left = True
            if right:
                self.bump_right = True

    # Thread: publisher
    def get_state(self):
        with self.lock:
            state = State(self.translation, self.rotation,
                          self.bump_center, self.bump_left, self.bump_right)
            self.translation    = 0.0
            self.rotation       = 0.0
            self.bump_center    = False
            self.bump_left      = False
            self.bump_right     = False
        return state



###############################################################################
#   Publisher
###############################################################################

class Publisher(object):
    def __init__(self, robot, callbacks):
        self.robot      = robot
        self.to_walk    = 0.0
        self.to_rotate  = 0.0
        self.commands   = []
        self.contador   = 0
        self.thread     = None
        self.twist      = None
        self.shutdown   = False
        self.init           = callbacks.get("init")         or self.skip
        self.bump_center    = callbacks.get("bump_center")  or self.skip
        self.bump_left      = callbacks.get("bump_left")    or self.skip
        self.bump_right     = callbacks.get("bump_right")   or self.skip
        self.walk_done      = callbacks.get("walk_done")    or self.skip
        self.rotate_done    = callbacks.get("rotate_done")  or self.skip

    def start(self):
        self.thread = Thread(target = self.spin)
        self.thread.daemon = True
        self.thread.start()

    def spin(self):
        rate    = rospy.Rate(5)
        cmd_vel = rospy.Publisher("cmd_vel", Twist, queue_size = 10)
        self.set_twist(0.0, 0.0)
        self.init(self)
        while not self.shutdown:
            state = self.robot.get_state()
            if self.to_walk > 0.0:
                self.to_walk -= state.translation
                if self.to_walk <= 0.0:
                    self.to_walk = 0.0
                    self.set_twist(0.0, 0.0)
                    self.walk_done(self)
                cmd_vel.publish(self.twist)
            if self.to_rotate > 0.0:
                self.to_rotate -= abs(state.rotation)
                if self.to_rotate <= 0.0:
                    self.to_rotate = 0.0
                    self.set_twist(0.0, 0.0)
                    self.rotate_done(self)
                cmd_vel.publish(self.twist)
            if state.bump_center:
                self.bump_center(self)
            elif state.bump_left:
                self.bump_left(self)
            elif state.bump_right:
                self.bump_right(self)
            rate.sleep()
        cmd_vel.unregister()

    def set_twist(self, vx, wz):
        self.twist = Twist()
        self.twist.linear.x = vx
        self.twist.linear.y = 0
        self.twist.linear.z = 0
        self.twist.angular.x = 0
        self.twist.angular.y = 0
        self.twist.angular.z = wz

    def skip(self, robot):
        if self.commands:
            cmd, val = self.commands.pop(0)
            cmd(val)
        else:
            self.terminar()


    def andar(self, meters):
        self.to_walk = meters
        self.to_rotate = 0.0
        self.set_twist(LINEAR, 0.0)

    def rodar(self, radians):
        self.to_walk = 0.0
        if radians > 0:
            self.to_rotate = radians
            self.set_twist(0.0, ANGULAR)
        elif radians < 0:
            self.to_rotate = -radians
            self.set_twist(0.0, -ANGULAR)

    def executar_depois(self, cmd, value):
        self.commands.append((getattr(self, cmd), value))

    def cancelar_comando(self):
        if self.to_walk > 0.0 or self.to_rotate > 0.0:
            self.to_walk = 0.0
            self.to_rotate = 0.0
            self.set_twist(0.0, 0.0)
            if self.commands:
                cmd, val = self.commands.pop(0)
                cmd(val)

    def conta(self):
        self.contador += 1

    def desconta(self):
        self.contador -= 1

    def terminar(self):
        self.to_walk = 0.0
        self.to_rotate = 0.0
        self.set_twist(0.0, 0.0)
        self.shutdown = True



###############################################################################
#   Robot Controller
###############################################################################

class RobotController(object):
    def __init__(self, callbacks):
        self.robot = Robot()
        self.publisher = Publisher(self.robot, callbacks)
        self.odom = None
        self.bump = None
        self.odom_callback = self._on_first_odom

    def run(self):
        rospy.init_node("turtlebot")
        self.odom = rospy.Subscriber("odom", Odometry, self._on_odom)
        self.bump = rospy.Subscriber("events/bumper", BumperEvent, self._on_bump)
        try:
            rospy.spin()
        except rospy.ROSInterruptException as e:
            pass
        finally:
            self.odom.unregister()
            self.odom = None
            self.bump.unregister()
            self.bump = None
            self.publisher.shutdown = True
            self.publisher.thread.join()
            self.publisher.thread = None


    def _on_odom(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        (roll, pitch, yaw) = euler_from_quaternion((q.x, q.y, q.z, q.w))
        self.odom_callback(x, y, yaw)

    def _on_first_odom(self, x, y, a):
        self.robot.x = x
        self.robot.y = y
        self.robot.a = a
        self.publisher.start()
        self.odom_callback = self.robot.set_odometry

    def _on_bump(self, msg):
        if msg.state == BumperEvent.PRESSED:
            if msg.bumper == BumperEvent.CENTER:
                self.robot.set_bumper(True, False, False)
            elif msg.bumper == BumperEvent.LEFT:
                self.robot.set_bumper(False, True, False)
            elif msg.bumper == BumperEvent.RIGHT:
                self.robot.set_bumper(False, False, True)
