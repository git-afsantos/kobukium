/*
Copyright (c) 2017 Andre Santos

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

(function () {
    "use strict";

    var views = window.App.Views;

    views.RobotBoard = views.BaseView.extend({
        id: "robot-status-board",

        navigateOptions: { trigger: true, replace: true },

        initialize: function (options) {
            this.router = options.router;
            this.locations = options.locations;

            this.$status = this.$("#robot-status-tag");
            this.$location = this.$("#robot-location-tag");
            this.$missionHeader = options.$mission;
            this.$missionStatus = options.$mission.children(".title");
            this.$currentMission = options.$mission.children(".subtitle");

            this.strings = {
                robotStatus:    "Offline",
                robotLocation:  "Unknown Location",
                missionStatus:  "Currently Offline",
                currentMission: ""
            };

            this.listenTo(this.model, "change", this.onUpdate);
            this.listenTo(this.model, "error", this.onError);
        },

        render: function () {
            if (!this.visible) return this;
            this.$status.html(this.strings.robotStatus);
            this.$location.html(this.strings.robotLocation);
            if (this.model.get("status") == "offline") {
                this.$missionHeader.hide();
            } else {
                this.$missionHeader.show();
                this.$missionStatus.html(this.strings.missionStatus);
                this.$currentMission.html(this.strings.currentMission);
            }
            return this;
        },

        build: function () {
            return this.render();
        },


        onUpdate: function (robot) {
            var status = robot.get("status"),
                mission = robot.get("mission");
            this.strings.robotLocation = this._locname(robot.get("location"));
            if (mission == null) {
                this.strings.missionStatus = "Available for Missions";
                this.strings.currentMission = "[no target selected]";
            } else {
                this.strings.missionStatus = "Mission " + mission.status;
                this.strings.currentMission =
                        "From " + this._locname(mission.location)
                        + " to " + this._locname(mission.goal);
            }
            if (status == "offline") {
                this.strings.robotStatus = "Offline";
                this.strings.missionStatus = "";
                this.strings.currentMission = "";
                this.router.navigate("offline", this.navigateOptions);
            } else if (status == "busy") {
                this.strings.robotStatus = "Busy";
                this.router.navigate("progress", this.navigateOptions);
            } else if (status == "idle") {
                this.strings.robotStatus = "Idle";
                if (mission != null && mission.status == "completed") {
                    this.router.navigate("feedback", this.navigateOptions);
                } else {
                    this.router.navigate("locations", this.navigateOptions);
                }
            } else if (status == "planning") {
                this.strings.robotStatus = "Planning";
            } else if (status == "lost") {
                this.strings.robotStatus = "Lost";
                this.strings.missionStatus = "Help the robot locate itself";
                this.strings.currentMission = "[unknown location]";
                this.router.navigate("lost", this.navigateOptions);
            }
            this.render();
        },

        onError: function (robot, response) {
            alert(response.error);
            this.onUpdate(robot);
        },

        getRoute: function () {
            var status = this.model.get("status"),
                mission = this.model.get("mission");
            if (status == "offline") {
                return "offline";
            } else if (status == "busy") {
                return "progress";
            } else if (status == "idle") {
                if (mission != null && mission.status == "completed") {
                    return "feedback";
                } else {
                    return "locations";
                }
            }
            return "locations";
        },

        _locname: function (id) {
            if (id) {
                return this.locations.get(id).get("name");
            } else {
                return "Unspecified Location";
            }
        }
    });
})();