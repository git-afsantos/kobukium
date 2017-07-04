
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

import json
import math
import os


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
