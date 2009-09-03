var MapTile = Class.create({
    initialize: function(x, y, width, height, scale) {
        this.left = x;
        this.top = y;
        this.width = width;
        this.height = height;
        this.scale = scale;

        this.img = new Element("img");
        this.img.style.position = "absolute";
        this.img.style.left = this.left;
        this.img.style.top = this.top;
        this.load_image();

        $('map_panner').appendChild(this.img);
    },

    rescale : function(scale) {
        this.scale = scale;
        this.load_image();
    },

    load_image: function() {
        var url = 'http://' + window.location.hostname + ':8080/map/get_tile';
        url += '?x=' + this.left;
        url += '&y=' + this.top;
        url += '&width=' + this.width;
        url += '&height=' + this.height;
        url += '&scale=' + this.scale;
        this.img.src = url;
    },

    move: function(dx, dy) {
        this.left += dx;
        this.top += dy;

        this.img.style.left = this.left;
        this.img.style.top = this.top;

        this.load_image();
    },
});


var MapViewer = Class.create({
    initialize: function(domobj) {
        this.viewer = domobj;
        //this.topics = ['/robot_pose_visualization', '/move_base/NavfnROS/plan:simplified', '/base_pose_ground_truth'];
        this.topics = ['/robot_pose_visualization', '/move_base/NavfnROS/plan:simplified'];
    },

    init: function() {
        // Overlay a canvas the same size as this div
        this.canvas = new Element('canvas', {'id': 'map_canvas', 'width': this.viewer.getWidth(), 'height': this.viewer.getHeight(), 'style': 'z-index:1;position:absolute'});
        this.viewer.appendChild(this.canvas);

        // Create a div to contain the image tiles
        this.panner = new Element('div', {'id': 'map_panner', 'style': 'padding:0;position:absolute;top:0px;left:0px;z-index:0'});
        this.panner.left = this.panner.top = 0;
        this.viewer.appendChild(this.panner);

        this.sourceWidth = 2332;
        this.sourceHeight = 1825;
        this.sourceResolution = 0.025;
        this.tileWidth = 256;
        this.tileHeight = 256;
        this.scale = 1.0;
        this.dim = this.viewer.getDimensions();
        this.tilesWide = Math.floor((this.dim.width + this.tileWidth - 1) / this.tileWidth + 2);
        this.tilesHigh = Math.floor((this.dim.height + this.tileHeight - 1) / this.tileHeight + 2);

        // Create tiles
        this.tiles = [];
        for (var row = 0; row < this.tilesHigh; ++row) {
            for (var col = 0; col < this.tilesWide; ++col) {
                this.tiles.push(new MapTile((col-1)*this.tileWidth, (row-1)*this.tileHeight, this.tileWidth, this.tileHeight, this.scale));
            }
        }

        // Register event handlers
        this.viewer.observe('mousedown', this.handleMouseDown.bind(this));
        this.viewer.observe('dblclick', this.handleDblClick.bind(this));
        Event.observe(document, 'mouseup', this.handleMouseUp.bind(this));
        Event.observe(document, 'mousemove', this.handleMouseMove.bind(this));
        Event.observe(document, 'keypress', this.handleKeyPress.bind(this));
        this.panning = false;
        this.settingGoal = false;
        this.settingPose = false;

        this.robot_img = new Image();
        this.robot_img.src = window.location.pathname + '/images/pr2_small.png';
    },

    handleDblClick : function(e) {
        if (Event.isLeftClick(e)) {
            var off = this.viewer.cumulativeOffset();
            this.zoom(e.ctrlKey ? 0.25 : -0.25,
                      Event.pointerX(e) - off.left,
                      Event.pointerY(e) - off.top);
        }
    },

    handleMouseDown : function(e) {
        this.mark = [Event.pointerX(e), Event.pointerY(e)];
        if (Event.isLeftClick(e)) {
            this.panning = true;
        } else if (Event.isMiddleClick(e)) {
            if (e.ctrlKey)
              this.settingPose = true;
            else
              this.settingGoal = true;
        }
    },

    handleMouseUp : function(e) {
        if (Event.isLeftClick(e)) {
            this.panning = false;
        } else if (Event.isMiddleClick(e)) {
            if (this.settingGoal || this.settingPose) {
                var dx = Event.pointerX(e) - this.mark[0];
                var dy = Event.pointerY(e) - this.mark[1];
                var angle = Math.atan2(-dy, dx);
                var off = this.viewer.cumulativeOffset();
                var pos = this.pixelToMap([this.mark[0]-off.left, this.mark[1]-off.top]);
                var url = 'http://' + window.location.hostname + ':8080';
                url += this.settingGoal ? '/map/set_goal' : '/map/set_pose';
                url += '?x=' + pos.x;
                url += '&y=' + pos.y;
                url += '&angle=' + angle;
                getDataFromServer('_set_goal_pump', url);
                this.settingGoal = false;
                this.settingPose = false;
                delete this.robot_est;
                delete this.plan;
            }
        }
    },

    handleMouseMove : function(e) {
        if (this.panning) {
            var old_mark = this.mark;
            this.mark = [Event.pointerX(e), Event.pointerY(e)];
            this.panMap(this.mark[0] - old_mark[0],
                        this.mark[1] - old_mark[1]);
        } else if (this.settingGoal || this.settingPose) {
            var off = this.viewer.cumulativeOffset();
            var pos = this.pixelToMap([this.mark[0]-off.left, this.mark[1]-off.top]);
            var dx = Event.pointerX(e) - this.mark[0];
            var dy = Event.pointerY(e) - this.mark[1];
            var angle = Math.atan2(dy, dx);
            this.robot_est = {'x': pos.x, 'y': pos.y, 'angle': angle};
            this.updateCanvas();
        }
    },

    handleKeyPress : function(e) {
        if (e.keyCode == 37) { // Left
            this.panMap(-10, 0);
        } else if (e.keyCode == 38) { // Up
            if (e.ctrlKey)
                this.zoom(-0.25, this.dim.width/2, this.dim.height/2);
            else
                this.panMap(0, -10);
        } else if (e.keyCode == 39) { // Right
            this.panMap(10, 0);
        } else if (e.keyCode == 40) { // Down
            if (e.ctrlKey)
                this.zoom(0.25, this.dim.width/2, this.dim.height/2);
            else
                this.panMap(0, 10);
        }
    },

    zoom : function(factor, center_x, center_y) {
        var center = this.pixelToMap([center_x, center_y]);
        this.scale += factor;

        var x = Math.floor(center.x / this.scale / this.sourceResolution);
        var y = this.sourceHeight / this.scale - Math.floor(center.y / this.scale / this.sourceResolution);

        this.panMap(this.dim.width/2-x, this.dim.height/2-y, false);
        for (var i = 0; i < this.tiles.length; ++i) {
            var tile = this.tiles[i];
            tile.rescale(this.scale);
        }
        this.updateCanvas();
    }, 

    panMap : function(x, y, relative) {
        var left = x;
        var top = y;
        if (typeof(relative) != 'undefined' ? relative : true) {
            left = this.panner.left + x;
            top = this.panner.top + y;
        }
        if (left > 0) left = 0;
        if (top > 0) top = 0;
        if (left < (this.dim.width - this.sourceWidth/this.scale))
            left = this.dim.width - this.sourceWidth/this.scale;
        if (top < (this.dim.height - this.sourceHeight/this.scale))
            top = this.dim.height - this.sourceHeight/this.scale;
        this.panner.style.left = this.panner.left = left;
        this.panner.style.top  = this.panner.top = top;

        for (var i = 0; i < this.tiles.length; ++i) {
            var tile = this.tiles[i];
            var tileLeft = left + tile.left;
            var tileTop = top + tile.top;
            var dx = 0, dy = 0;
            if (tileLeft + tile.width < -this.tileWidth)
                dx += this.tilesWide * this.tileWidth;
            else if (tileLeft > (this.dim.width + this.tileWidth))
                dx -= this.tilesWide * this.tileWidth;
            if (tileTop + parseInt(tile.height) < -this.tileHeight)
                dy += this.tilesHigh * this.tileHeight;
            else if (tileTop > (this.dim.height + this.tileHeight))
                dy -= this.tilesHigh * this.tileHeight;
            if (dx || dy) tile.move(dx, dy);
        }
        this.updateCanvas();
    },

    pixelToMap: function(p) {
        result = {'x': 0, 'y': 0};
        var x = p[0] - this.panner.left;
        var y = p[1] - this.panner.top;
        result.x = x * this.sourceResolution * this.scale;
        result.y = (this.sourceHeight / this.scale - y) * this.sourceResolution * this.scale;
        return result;
    },

    mapToPixel: function(p) {
        var x = Math.floor(p.x / this.scale / this.sourceResolution);
        var y = this.sourceHeight / this.scale - Math.floor(p.y / this.scale / this.sourceResolution);
        x += this.panner.left;
        y += this.panner.top;
        return [x, y];
    },

    drawRobot: function(ctx, position, alpha) {
        if (this.robot_img.complete) {
            var coords = this.mapToPixel(position);
            ctx.save();
            ctx.globalAlpha = alpha;
            ctx.translate(coords[0], coords[1]);
            ctx.rotate(position.angle);
            var sx = 0.65 / (this.robot_img.width * this.sourceResolution * this.scale);
            var sy = 0.65 / (this.robot_img.height * this.sourceResolution * this.scale);
            ctx.scale(sx, sy);
            ctx.drawImage(this.robot_img, -this.robot_img.width / 2, -this.robot_img.height / 2);
            ctx.restore();
        }
    },

    updateCanvas: function() {
        var ctx = this.canvas.getContext('2d');
        ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);

        // Draw plan
        if (this.plan) {
            ctx.strokeStyle = "rgb(0, 255, 0)";
            ctx.beginPath();
            var p = this.mapToPixel(this.plan[0].pose.position);
            ctx.moveTo(p[0], p[1]);
            for (var i = 1; i < this.plan.length; ++i) {     
                p = this.mapToPixel(this.plan[i].pose.position);
                ctx.lineTo(p[0], p[1]);
            }
            ctx.stroke();
        }

        // Draw robot footprint
        if (this.footprint) {
            ctx.strokeStyle = "rgb(255, 0, 0)";
            ctx.beginPath();
            var p = this.mapToPixel(this.footprint[0]);
            ctx.moveTo(p[0], p[1]);
            for (var i = 1; i < this.footprint.length; ++i) {     
                p = this.mapToPixel(this.footprint[i]);
                ctx.lineTo(p[0], p[1]);
            }
            ctx.closePath();
            ctx.stroke();
        }

        if (this.ground_truth) {
            this.drawRobot(ctx, this.ground_truth, 0.25);
        }
        if (this.robot) {
            this.drawRobot(ctx, this.robot, 1.0);
        }
        if (this.robot_est) {
            this.drawRobot(ctx, this.robot_est, 0.75);
        }

    },

    quaternionToEuler: function (q)
    {
        result = {'r':0.0, 'p':0.0, 'y':0.0};

        var sqw = q.w * q.w;
        var sqx = q.x * q.x;
        var sqy = q.y * q.y;
        var sqz = q.z * q.z;

        // Roll
        result.r = Math.atan2(2 * (q.y*q.z + q.w*q.x), sqw - sqx - sqy + sqz);

        // Pitch
        result.p = Math.asin(-2 * (q.x*q.z - q.w * q.y));

        // Yaw
        result.y = Math.atan2(2 * (q.x*q.y + q.w*q.z), sqw + sqx - sqy - sqz);

        return result;
    },

    receive: function(topic, msg) {
        if (topic == '/robot_pose_visualization') {
          var angle = -this.quaternionToEuler(msg.pose.orientation).y;
          var coords = this.mapToPixel(msg.pose.position);
          coords[0] -= this.panner.left;
          coords[1] -= this.panner.top;
          //this.panMap(this.dim.width/2-coords[0], this.dim.height/2-coords[1], false);
          this.robot = {'x': msg.pose.position.x,
                        'y': msg.pose.position.y,
                        'angle': angle};
        } else if (topic == '/move_base/NavfnROS/plan' ||
                   topic == '/move_base/NavfnROS/plan:simplified') {
            this.plan = msg.poses;
        } else if (topic == '/move_base/TrajectoryPlannerROS/robot_footprint') {
            this.footprint = msg.polygon.points;
        } else if (topic == '/base_pose_ground_truth') {
            var angle = -this.quaternionToEuler(msg.pose.pose.orientation).y;
            this.ground_truth = {'x': msg.pose.pose.position.x,
                                 'y': msg.pose.pose.position.y,
                                 'angle': angle};
        }
        this.updateCanvas();
    },

});

gRosClasses['MapViewer'] = function(dom) {
    return new MapViewer(dom);
}
