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
    var Models = window.App.Models;

    /*
        id,
        name,
        icon,
        detail1,
        detail2
    */
    Models.Location = Backbone.Model.extend({
        defaults: {
            name:       "Unspecified",
            icon:       null,
            detail1:    "",
            detail2:    ""
        }
    });

    Models.LocationCollection = Backbone.Collection.extend({
        model: Models.Location,

        url: "data/locations.json"
    });


    ////////////////////////////////////////////////////////////////////////////

    /*
        id,
        status,
        location,
        mission: {
            status,
            location,
            time,
            goal
        }
    */
    Models.Robot = Backbone.Model.extend({
        defaults: {
            status:     "offline",
            location:   null,
            mission:    null
        },

        wsAddress: window.App.wsAddress,

        initialize: function () {
            this.server = null;
        },

        isConnected: function () {
            return this.server != null;
            // return this.get("status") != "offline";
        },

        connect: function () {
            if (this.server == null) {
                console.log("Connecting to " + this.wsAddress);
                this.server = new WebSocket(this.wsAddress);
                this.server.onopen = _.bind(this.onConnect, this);
                this.server.onclose = _.bind(this.onDisconnect, this);
                this.server.onmessage = _.bind(this.onMessage, this);
                this.server.onerror = _.bind(this.onError, this);
            }
        },

        onConnect: function (event) {
            console.log("Connected to the server.");
        },

        onDisconnect: function (event) {
            console.log("Disconnected from the server.");
            this.set(this.defaults);
        },

        onMessage: function (event) {
            var attributes = JSON.parse(event.data);
            if ("error" in attributes) {
                this.trigger("error", this, attributes);
            } else {
                this.set(attributes);
            }
        },

        onError: function (event) {
            console.log("A communication error occurred.");
        },

        setMission: function (goal) {
            if (this.get("status") == "lost") {
                this.server.send(JSON.stringify({"hint": goal}));
            } else {
                this.server.send(JSON.stringify({"goal": goal}));
            }
        },

        abortMission: function () {
            this.server.send(JSON.stringify({"feedback": "cancel"}));
        },

        missionSuccess: function () {
            this.server.send(JSON.stringify({"feedback": "success"}));
        },

        missionFailure: function () {
            this.server.send(JSON.stringify({"feedback": "failure"}));
        }
    });
})();