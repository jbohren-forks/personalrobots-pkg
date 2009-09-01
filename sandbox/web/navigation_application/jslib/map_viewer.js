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
        var url = "http://cib.willowgarage.com:8080/map";
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
        this.topics = domobj.getAttribute("topic").split(',');
    },

    init: function() {
        // Overlay a canvas the same size as this div
        this.canvas = new Element('canvas', {'id': 'map_canvas', 'width': this.viewer.getWidth(), 'height': this.viewer.getHeight(), 'style': 'z-index:1;position:absolute'});
        this.viewer.appendChild(this.canvas);

        // Create a div to contain the image tiles
        this.panner = new Element('div', {'id': 'map_panner', 'style': 'padding:0;position:absolute;top:0px;left:0px;z-index:0'});
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
        this.pressed = false;
    },

    handleDblClick : function(e) {
        if (Event.isLeftClick(e)) {
            var off = this.viewer.cumulativeOffset();
            this.zoom(-0.25,
                      Event.pointerX(e) - off.left,
                      Event.pointerY(e) - off.top);
        }
    },

    handleMouseDown : function(e) {
        if (Event.isLeftClick(e)) {
            this.pressed = true;
            this.mark = [Event.pointerX(e), Event.pointerY(e)];
        }
    },

    handleMouseUp : function(e) {
        if (Event.isLeftClick(e)) {
            this.pressed = false;
        }
    },

    handleMouseMove : function(e) {
        if (this.pressed) {
            var old_mark = this.mark;
            this.mark = [Event.pointerX(e), Event.pointerY(e)];
            this.panMap(this.mark[0] - old_mark[0],
                        this.mark[1] - old_mark[1]);
        }
    },

    handleKeyPress : function(e) {
        if (e.keyCode == 37) { // Left
            this.panMap(-10, 0);
        } else if (e.keyCode == 38) { // Up
            if (e.ctrlKey)
                this.zoom(-0.25, 0, 0);
            else
                this.panMap(0, -10);
        } else if (e.keyCode == 39) { // Right
            this.panMap(10, 0);
        } else if (e.keyCode == 40) { // Down
            if (e.ctrlKey)
                this.zoom(0.25, 0, 0);
            else
                this.panMap(0, 10);
        }
    },

    zoom : function(factor, center_x, center_y) {
        this.scale += factor;
        for (var i = 0; i < this.tiles.length; ++i) {
            var tile = this.tiles[i];
            tile.rescale(this.scale);
        }
        this.drawRobot();
    }, 

    panMap : function(x, y) {
        var left = parseInt(this.panner.style.left) + x;
        var top = parseInt(this.panner.style.top) + y;
        if (left > 0) left = 0;
        if (top > 0) top = 0;
        if (left < (this.dim.width - this.sourceWidth/this.scale))
            left = this.dim.width - this.sourceWidth/this.scale;
        if (top < (this.dim.height - this.sourceHeight/this.scale))
            top = this.dim.height - this.sourceHeight/this.scale;
        this.panner.style.left = left;
        this.panner.style.top  = top;

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
        this.drawRobot();
    },

    mapToPixel: function(p) {
        var x = Math.floor(p.x / this.scale / this.sourceResolution);
        var y = this.sourceHeight / this.scale - Math.floor(p.y / this.scale / this.sourceResolution);
        x += parseInt(this.panner.style.left);
        y += parseInt(this.panner.style.top);
        return [x, y];
    },

    drawRobot: function() {
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
    },

    receive: function(msg) {
        var ctx = this.canvas.getContext('2d');
        ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);
        if (msg.polygon) {
            this.footprint = msg.polygon.points;
        } else if (msg.poses) {
            this.plan = msg.poses;
        }
        this.drawRobot();
    },

});

gRosClasses['MapViewer'] = function(dom) {
    return new MapViewer(dom);
}
