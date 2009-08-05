var ROSJoystickGadget = Class.create(ROSGadget, {

  initialize: function($super)
  {

    $super("Joystick", 500);

    this.startButton = document.createElement('input');
    this.startButton.type = 'button';
    this.startButton.id = 'startbutton';
    this.startButton.value = 'Start';
    this.startButton.style.cssFloat = 'left';
    this.startButton.style.margin = '0 10px 0 0';
    this.startButton.observe('click', this.start.bind(this) );

    this.statusDiv = document.createElement('div');
    this.statusDiv.style.width = '100%';
    this.statusDiv.style.height = "30px";

    this.joyLabel = document.createElement("span");
    this.joyLabel.style.lineHeight = '20px';
    this.joyLabel.style.verticalAlign = 'middle';
    this.joyLabel.style.fontSize = '130%';
    this.joyLabel.style.cssFloat = 'left';
    this.joyLabel.style.fontWeight = 'bold';
    this.joyLabel.innerHTML = 'Joystick: ';

    this.joyStatus = document.createElement('span');
    this.joyStatus.id = 'joystatus';
    this.joyStatus.style.lineHeight = '20px';
    this.joyStatus.style.verticalAlign = 'middle';
    this.joyStatus.style.fontSize = '120%';
    this.joyStatus.style.marginLeft = '5px';
    this.joyStatus.style.marginTop = '1px';
    this.joyStatus.style.cssFloat = 'left';
    this.joyStatus.innerHTML = 'stopped';

    this.joyStatusIcon = document.createElement('img');
    this.joyStatusIcon.id = 'robotStatusIcon';
    this.joyStatusIcon.src = 'images/redbutton.png';
    this.joyStatusIcon.style.width = '20px';
    this.joyStatusIcon.style.cssFloat = 'right';

    this.statusDiv.appendChild(this.startButton);
    this.statusDiv.appendChild(this.joyLabel);
    this.statusDiv.appendChild(this.joyStatus);
    this.statusDiv.appendChild(this.joyStatusIcon);
 
    this.contentDiv.appendChild(this.statusDiv);

    /*buttonsDiv = document.createElement("div");
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
    */

    if (this.pump != null)
      delete this.pump;

    this.pump = new MessagePump();

    this.setHelpText('joystick_gadget_help.html');
  },

  start: function()
  {
    this.run = true;
    this.pump.sendAJAX('/ros/launch/ps3.launch', this, this.started);
    this.joyStatus.innerHTML = 'starting...';
    this.joyStatusIcon.src = 'images/yellowbutton.png';
  },
  
  stop: function()
  {
    this.pump.sendAJAX('/ros/stop/ps2.launch', this, this.stopped);
    this.run = false;
  },

  started: function(myself, pump)
  {
    myself.joyStatus.innerHTML = 'started';
    myself.joyStatusIcon.src = 'images/greenbutton.png';

    myself.startButton.value = 'Stop';
    myself.startButton.observe('click', myself.stop.bind(myself) );
  },
  
  stopped: function (myself, pump)
  {
    myself.joyStatus.innerHTML = 'stopped';
    myself.startButton.value = 'Start';
    myself.startButton.observe('click', myself.start.bind(myself) );
  },
  
});



