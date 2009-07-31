var ROSJoystickGadget = Class.create(ROSGadget, {

  initialize: function()
  {
    this.create("Joystick", 500);

    this.startButton = document.createElement('input');
    this.startButton.type = 'button';
    this.startButton.id = 'startbutton';
    this.startButton.value = 'Start';
    this.startButton.style.cssFloat = 'right';
    this.startButton.style.margin = '2px 0 0 0';
    this.startButton.observe('click', this.start.bind(this) );

    this.titleSpan.appendChild(this.startButton);

    statusDiv = document.createElement('div');
    statusDiv.innerHTML = '<div style="height:25px;line-height:15px;vertical-align:middle;font-size:120%;"><img id="joyStatusIcon"src="images/redbutton.png" width="15" style="border:0;float:left; display:inline; margin-right: 10px;"/> <b>Joystick</b>: <span id="joystatus"></span> </div>';

    this.contentDiv.appendChild(statusDiv);

    this.joyStatusIcon = document.getElementById("joyStatusIcon");
    this.joyStatus = document.getElementById('joystatus');
    this.joyStatus.innerHTML = 'stopped';

    buttonsDiv = document.createElement("div");
    buttonsDiv.align='center';
    buttonsDiv.style.margin='auto';
    buttonsDiv.style.width = "180px";
    
    upButtonDiv = document.createElement("div");
    leftButtonDiv = document.createElement("div");
    rightButtonDiv = document.createElement("div");
    downButtonDiv = document.createElement("div");
    stopButtonDiv = document.createElement("div");

    upButtonDiv.style.width = "50px";
    leftButtonDiv.style.width = "50px";
    leftButtonDiv.style.display = "inline";
    rightButtonDiv.style.width = "50px";
    rightButtonDiv.style.display = "inline";
    downButtonDiv.style.width = "50px";
    stopButtonDiv.style.width = "50px";
    stopButtonDiv.style.display = "inline";

    upButton = document.createElement("a");
    upButton.style.display = "block";
    upButton.style.width = "50px";
    upButton.style.height = "50px";
    
    leftButton = document.createElement("a");
    leftButton.style.width = "50px";
    leftButton.style.height = "50px";

    rightButton = document.createElement("a");
    rightButton.style.width = "50px";
    rightButton.style.height = "50px";

    downButton = document.createElement("a");
    downButton.style.width = "50px";
    downButton.style.height = "50px";

    stopButton = document.createElement("a");
    stopButton.style.width = "50px";
    stopButton.style.height = "50px";

    leftButton.href = '#';
    rightButton.href = '#';
    upButton.href = '#';
    downButton.href = '#';
    stopButton.href = '#';

    leftButton.innerHTML ="<img src=images/leftarrow.png style='border:none;'>";
    rightButton.innerHTML = "<img src=images/rightarrow.png style='border:none;'>";
    upButton.innerHTML = "<img src=images/uparrow.png style='border:none;'>";
    downButton.innerHTML = "<img src=images/downarrow.png style='border:none;'>";
    stopButton.innerHTML = "<img src=images/stopicon.png style='border:none;'>";

    upButtonDiv.appendChild(upButton);
    leftButtonDiv.appendChild(leftButton);
    rightButtonDiv.appendChild(rightButton);
    downButtonDiv.appendChild(downButton);
    stopButtonDiv.appendChild(stopButton);

    buttonsDiv.appendChild(upButtonDiv);
    buttonsDiv.appendChild(leftButtonDiv);
    buttonsDiv.appendChild(stopButtonDiv);
    buttonsDiv.appendChild(rightButtonDiv);
    buttonsDiv.appendChild(downButtonDiv);

    // Put this line back in to add in joystick GUI buttons
    //this.contentDiv.appendChild(buttonsDiv);


    if (this.pump != null)
      delete this.pump;

    this.pump = new MessagePump();

    var helpTxt="<dl><dt>Purpose:</dt><dd>This gadget is used to enable joystick control of the robot.</dd><dt>Overview:</dt><dd>The <Start> button located in this gadget's title bar will start the joystick controller.</dd> <dt>Driving the Robot with a Joystick Controller</dt><dd>To drive the robot, hold down the “dead man's switch” with your left index finger. The button is located at the top of the left hand grip. This switch helps prevent accidental operation of the robot. To drive the robot around, hold down the button while using the two joysticks to move the base of the robot. The right joystick moves the base to the right, left, forward, and backward, while the left joystick rotates the base clockwise and counterclockwise. The 4 button moves the spine up, while the 2 button moves the spine down.<br><img src='images/joystickimg.jpg'></dd></dl>";
    this.setHelpText(helpTxt);

  },

  start: function()
  {
    /*this.run = true;
    this.pump.sendAJAX('/ros/startup', this, this.started);
    this.robotStatus.innerHTML = 'processing...';
    this.robotStatusIcon.src = 'images/yellowbutton.png';
    */
  },
  
  stop: function()
  {
    /*this.pump.sendAJAX('/ros/shutdown', this, this.stopped);
    this.run = false;
    */
  },

  started: function(myself, pump)
  {
    /*myself.robotStatus.innerHTML = 'started';
    myself.robotStatusIcon.src = 'images/greenbutton.png';
    myself.startButton.value = 'Stop';
    myself.startButton.observe('click', this.stopt.bind(this) );
    */
  },
  
  stopped: function (myself, pump)
  {
    /*myself.robotStatus.innerHTML = 'stopped';
    myself.startButton.value = 'Start';
    myself.startButton.observe('click', this.stopt.bind(this) );
    */
  },
  
});



