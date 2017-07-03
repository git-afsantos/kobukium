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

    var App = window.App = {
        Views: {},
        Models: {},
        board: null, // current view
        robot: null,
        locations: null
    };

    $(document).ready(function () {
        delete window.App;

        App.robot = new App.Models.Robot({"id": "HASTurtle"});
        App.locations = new App.Models.LocationCollection();

        bootstrapViews();
        Backbone.history.start();
        preloadData();
    });



    function preloadData() {
        // App.robot.connect();
        App.locations.fetch();
    }


    function bootstrapViews() {
        // App.preloader = new App.Views.Preloader({
            // el: $("#preloader"),
            // model: App.robot,
            // locations: App.locations
        // });
        App.robotBoard = new App.Views.RobotBoard({
            el: $("#robot-status-board"),
            $mission: $("#mission-status-board"),
            model: App.robot,
            router: App.router
        });
        App.locationBoard = new App.Views.LocationBoard({
            el: $("#location-board"),
            collection: App.locations,
            robot: App.robot,
            router: App.router
        });
        App.progressBoard = new App.Views.ProgressBoard({
            el: $("#progress-board"),
            model: App.robot,
            router: App.router
        });
        App.feedbackBoard = new App.Views.FeedbackBoard({
            el: $("#feedback-board"),
            model: App.robot,
            router: App.router
        });
        // Hide everything. Each route shows its view.
        App.locationBoard.hide();
        App.progressBoard.hide();
        App.feedbackBoard.hide();
        App.robotBoard.show().build();
    }
})();
