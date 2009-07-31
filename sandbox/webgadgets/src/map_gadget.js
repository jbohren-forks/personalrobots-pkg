var ROSMapGadget = Class.create(ROSGadget, {
  initialize: function(service)
  {
    // Create the ros gadget 
    this.create("Nav View", 600);

    // Create the set pose button
    this.setPoseButton = document.createElement("input");
    this.setPoseButton.value = "Set Pose";
    this.setPoseButton.type = "button";
    this.setPoseButton.style.cssFloat = "right";
    this.setPoseButton.style.margin = "2px 0 0 0";
    this.setPoseButton.observe('click', this.setPose.bind(this) );

    // Create the set goal button
    this.setGoalButton = document.createElement("input");
    this.setGoalButton.value = "Set Goal";
    this.setGoalButton.type = "button";
    this.setGoalButton.style.cssFloat = "right";
    this.setGoalButton.style.margin = "2px 0 0 0";
    this.setGoalButton.observe('click', this.setGoal.bind(this) );
   
    // Add the two buttons to the title span 
    this.titleSpan.appendChild(this.setPoseButton);
    this.titleSpan.appendChild(this.setGoalButton);

    this.contentDiv.style.position = 'relative';
    this.contentDiv.style.height = '300';

    // This div hold the map image, and the canvas (which is used for
    // painting)
    this.imgDiv = document.createElement('div');
    this.imgDiv.id = "imgdiv";
    this.imgDiv.style.overflow = 'hidden';

    // The canvas is used to draw shapes overlays
    this.canvas = document.createElement('div');
    this.canvas.id = "canvas";

    // The canvas is used to draw the robot's path
    this.pathCanvas = document.createElement('div');
    this.pathCanvas.id = "pathCanvas";

    // This div handles mouse interactions
    this.activeDiv = document.createElement('div');
    this.activeDiv.id = 'activediv';
    this.activeDiv.style.cursor = 'pointer';

    // This is actual img to be displayed
    this.img = document.createElement('img');
    this.img.id = "myimage";
    this.img.style.position = "absolute";

    this.imgDiv.appendChild(this.img);
    this.imgDiv.appendChild(this.canvas);
    this.imgDiv.appendChild(this.pathCanvas);

    this.contentDiv.appendChild(this.imgDiv);
    this.contentDiv.appendChild(this.activeDiv);

    this.service = "/static_map";
    this.mapresolution = 0.1;

    this.robotPose = {'x': 0.0, 'y':0.0, 't':0.0};
    this.imgpos = {'x' : 0.0, 'y': 0.0};
    this.start = {'x' : 0, 'y': 0};
    this.mapSize = {'w' : 0, 'h': 0};
    this.winSize = {'w' : 0, 'h':0};

    this.winSize.w = parseInt(this.activeDiv.offsetWidth);
    this.winSize.h = parseInt(this.activeDiv.offsetHeight);

    // Center the image
    this.imgDiv.style.left = parseInt(this.contentDiv.offsetWidth)/2 - this.winSize.w/2;
    this.imgDiv.style.top = parseInt(this.contentDiv.offsetHeight)/2 - this.winSize.h/2;

    this.activeDiv.style.left = this.imgDiv.style.left;
    this.activeDiv.style.top = this.imgDiv.style.top;

    this.jg = new jsGraphics("canvas");
    this.pathJG = new jsGraphics("pathCanvas");

    this.drag = false;
    this.zoom = 1.0;

    this.contentDiv.observe('mousedown', Event.stop);

    this.activeDiv.addEventListener('DOMMouseScroll', this.zoomCB.bind(this), false);

    this.initPoseTopic = new ROSTopic('/initialpose');
    this.initPoseTopic.announce(null, null);
    this.settingPose = false;
    this.settingGoal = false;
    this.initPoseAngle = 0.0;

    this.goalPoseTopic = new ROSTopic('/move_base/activate');
    this.goalPoseTopic.announce(null, null);

    this.pathTopic = new ROSTopic('/move_base/TrajectoryPlannerROS/global_plan');
    this.pathTopic.subscribe(this, this.pathSubscribedCB);

    this.path = [];

    this.poseTF = new ROSTF("/base_link");
    this.poseTF.subscribe(this, this.poseSubscribed);

    this.activeDiv.observe('mouseup', this.mouseup.bind(this));
    this.activeDiv.observe('mousedown', this.mousedown.bind(this));
    this.activeDiv.observe('mousemove', this.mousemove.bind(this));
    this.activeDiv.observe('mouseout', this.mouseup.bind(this));

    // Get the static map
    pump = new MessagePump();
    pump.sendAJAX('/ros/service' + this.service + '?x=0&y=0', 
                  this, this.mapResponseCB);
    var helpTxt="<dl><dt>Purpose:</dt><dd>This gadget is used to view the static map, set/get the robot's pose, and set/get the robot's goal pose.</dd><dt>Overview:</dt><dd>The two button located in the title bar provide the means to set the robot's pose and set the robot's goal pose. Once either button is selected, simply click anywhere on the map, and drag while holding the left mouse button. The click location will determin the robot's <x,y> position, and dragging will determine the robot's orientation. <p> The main content of this gadget show's the static map. The robot's current pose is depicted by a red circle and a line for orientation. A blue line will be present when the robot has a path planned.</p></dd></dl>";
    this.setHelpText(helpTxt);

  },

  setGoal: function()
  {
    this.settingGoal = true;
  },


  setPose: function()
  {
    this.settingPose = true;
  },

  mousemove: function (e)
  {
    if (this.drag)
    {
      delta = {'x' : e.pageX - this.start.x, 'y' : e.pageY - this.start.y};

      if (this.settingPose || this.settingGoal)
      {
        // Can't figure out why it's off by 10px
        topOffset = 10;
        leftOffset = 10;
        elem = this.activeDiv
          while (elem != null)
          {
            leftOffset += elem.offsetLeft;
            topOffset += elem.offsetTop;
            elem = elem.offsetParent;
          }

        this.jg.clear();
        this.jg.setColor("#0000FF");
        this.jg.setStroke(2);
        this.jg.drawLine(this.start.x-leftOffset, 
                         this.start.y-topOffset, 
                         e.pageX-leftOffset, 
                         e.pageY-topOffset );
        this.jg.paint();
        this.initPoseAngle = Math.atan2(-delta.y, delta.x);
      }
      else
      {

        this.imgpos.x += delta.x;
        this.imgpos.y += delta.y;

        if (this.imgpos.x + this.img.width < this.winSize.w )
          this.imgpos.x = this.winSize.w - this.img.width;

        if (this.imgpos.y + this.img.height < this.winSize.h )
          this.imgpos.y = this.winSize.h - this.img.height;

        this.imgpos.x = Math.min(0, this.imgpos.x);
        this.imgpos.y = Math.min(0, this.imgpos.y);

        this.img.style.top = this.imgpos.y + "px";
        this.img.style.left = this.imgpos.x + "px";

        this.start.x = e.pageX;
        this.start.y = e.pageY;

        this.drawRobot();
        this.drawPath();
      }
    }

  },

  zoomCB: function(e)
  {

    if (e.detail < 0)
      this.zoom *= 1.1;
    else
      this.zoom *= 0.9;

    w = this.img.width * this.zoom;
    h = this.img.height *this.zoom;

    this.img.width = this.mapSize.w * this.zoom;
    this.img.height = this.mapSize.h * this.zoom;

    this.drawRobot();
  },

  mousedown: function (e)
  {

    this.start.x = e.pageX;
    this.start.y = e.pageY;
    this.activeDiv.style.cursor = 'move';

    if (Event.isLeftClick(e))
    {
      this.drag = true;
    }
  },

  mouseup: function (e)
  {
    this.drag = false;
    this.activeDiv.style.cursor= 'pointer';

    if (this.settingPose == true)
    {
      pos = this.screenToMap(this.start);
      this.initPoseTopic.publish("?x="+pos.x+"&y="+pos.y+"&t=" + this.initPoseAngle);
      this.settingPose = false;
    }
    else if (this.settingGoal == true)
    {
      pos = this.screenToMap(this.start);

      this.goalPoseTopic.publish("?x="+pos.x+"&y="+pos.y+"&t=" + this.initPoseAngle );
      this.settingGoal = false;
    }

  },

  poseSubscribed: function(myself, pump)
  {
    myself.getPose();
  },

  drawRobot: function()
  {
    if (this.settingPose)
      return;

    pos = this.mapToScreen(this.robotPose);
    r = 0.8 / this.mapresolution;

    dx = (r*1.5) * Math.cos(this.robotPose.t);
    dy = (r*1.5) * Math.sin(this.robotPose.t);

    // Redraw the position of the robot
    this.jg.clear();
    this.jg.setColor("#ff0000");
    this.jg.setStroke(2);
    this.jg.drawEllipse(pos.x - r*0.5, pos.y - r*0.5, r, r);

    this.jg.drawLine(pos.x, pos.y, pos.x + dx, pos.y - dy);
    this.jg.paint();
  },

  screenToMap: function(pos)
  {
    result = {'x' :0.0, 'y':0.0};

    // Can't figure out why it's off by 10px
    topOffset = 10;
    leftOffset = 10;
    elem = this.activeDiv
    while (elem != null)
    {
      leftOffset += elem.offsetLeft;
      topOffset += elem.offsetTop;
      elem = elem.offsetParent;
    }

    result.x = (pos.x-leftOffset-this.imgpos.x)/this.zoom;

    result.y = (pos.y-topOffset-this.imgpos.y) / this.zoom;
    result.y = this.mapSize.h - result.y;

    result.x *= this.mapresolution;

    result.y *= this.mapresolution;

    return result;
  },

  mapToScreen: function(pos)
  {
    result = {'x' :0.0, 'y':0.0};

    result.x = this.imgpos.x + (this.zoom * pos.x) / this.mapresolution;
    result.y = this.imgpos.y + this.zoom * (this.mapSize.h - pos.y / this.mapresolution);

    return result;
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


  getPathCB: function(myself, pump)
  {
    jsonData = eval( '(' + pump.response + ')' );

    myself.path = [];

    for (i = 0; i < jsonData.points.length; i += 1)
    {
      pt = {'x':jsonData.points[i].x, 'y': jsonData.points[i].y};
      pt = myself.mapToScreen(pt);
      myself.path.push( pt );
    }

    myself.drawPath();

    setTimeout( function () {myself.getPath();}, 1000);
  },

  drawPath: function()
  {
    this.pathJG.clear();
    this.pathJG.setColor("#0000FF");
    this.pathJG.setStroke(2);

    for (i=0; i < this.path.length - 1; i+=1)
    {
      start = {"x": this.path[i].x,
               "y": this.path[i].y};

      end = {"x" : this.path[i+1].x,
             "y" : this.path[i+1].y};

      this.pathJG.drawLine(start.x, start.y, end.x, end.y);
    }

    this.pathJG.paint();
  },

  pathSubscribedCB: function(myself, pump)
  {
    myself.getPath();
  },

  getPath : function()
  {
    this.pathTopic.getMessage(this, this.getPathCB );
  },

  poseResponseCB: function(myself, pump)
  {
    if (pump.response != "")
    {
      jsonData = eval( '(' + pump.response + ')' );

      myself.robotPose.x = jsonData.position.x;
      myself.robotPose.y = jsonData.position.y;

      rpy = myself.quaternionToEuler(jsonData.orientation);

      myself.robotPose.t = rpy.y;

      myself.drawRobot();
    }

    setTimeout( function () {myself.getPose();}, 500);
  },

  getPose: function()
  {
    this.poseTF.getMessage(this, this.poseResponseCB);
  },


  mapResponseCB: function (myself, pump)
  {
    jsonData = eval( '(' + pump.response + ')' );

    myself.img.src = jsonData.data;
    myself.mapSize.w = jsonData.width;
    myself.mapSize.h = jsonData.height;
  }

});

