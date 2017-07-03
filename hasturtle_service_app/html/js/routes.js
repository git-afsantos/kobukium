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
    var App = window.App,
        Router = Backbone.Router.extend({
            navigateOptions: { trigger: true, replace: true },

            routes: {
                "":                         "home",
                "offline":                  "offline",
                "locations":                "locations",
                "progress":                 "progress",
                "feedback":                 "feedback"
            },

            home: function() {
                this.navigate(App.robotBoard.getRoute(), this.navigateOptions);
            },

            offline: function() {
                if (App.robot.isConnected()) {
                    this.navigate("", this.navigateOptions);
                } else {
                    if (App.board != null) App.board.hide();
                    App.board = null;
                    //App.robot.connect();
                }
            },

            locations: function() {
                if (!App.robot.isConnected()) {
                    this.navigate("offline", this.navigateOptions);
                } else {
                    if (App.board != null) App.board.hide();
                    App.locationBoard.show().build();
                    App.board = App.locationBoard;
                }
            },

            progress: function() {
                if (!App.robot.isConnected()) {
                    this.navigate("offline", this.navigateOptions);
                } else {
                    if (App.board != null) App.board.hide();
                    App.progressBoard.show().build();
                    App.board = App.progressBoard;
                }
            },

            feedback: function() {
                if (!App.robot.isConnected()) {
                    this.navigate("offline", this.navigateOptions);
                } else {
                    if (App.board != null) App.board.hide();
                    App.feedbackBoard.show().build();
                    App.board = App.feedbackBoard;
                }
            }
        });
    App.router = new Router();
})();