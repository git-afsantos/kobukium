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

    views.LocationBoard = views.BaseView.extend({
        id: "location-board",

        navigateOptions: { trigger: true, replace: true },

        initialize: function (options) {
            this.robot = options.robot;
            this.router = options.router;

            this.$listing = this.$("#location-listing");

            this.locationTemplate = _.template($("#location-panel-template").html(),
                                               {variable: "data"});

            this.listenTo(this.collection, "sync", this.onSync);
        },

        render: function () {
            if (!this.visible) return this;
            return this;
        },

        build: function () {
            return this;
        },


        onSync: function (collection, response, options) {
            var $panels, data = collection.toJSON();
            this.$listing.html(_.map(data, this.locationTemplate).join("\n"));
            $panels = this.$listing.find(".clickable.panel");
            $panels.on("click touchstart", _.bind(this.onSelection, this));
            this.render();
        },

        onSelection: function (event) {
            var goal = $(event.currentTarget).data("modelid");
            if (goal != this.robot.get("location")) {
                this.robot.setMission(goal);
                this.hide();
            }
        }
    });
})();