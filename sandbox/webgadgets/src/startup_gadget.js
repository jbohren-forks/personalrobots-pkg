var ROSStartupGadget = Class.create(ROSGadget, {

  initialize: function()
  {
    this.create("Robot Startup", 300);

    this.startButton = document.createElement('input');
    this.startButton.type = 'button';
    this.startButton.id = 'startbutton';
    this.startButton.value = 'Start';
    this.startButton.style.cssFloat = 'right';
    this.startButton.style.margin = '2px 0 0 0';
    this.startButton.observe('click', this.start.bind(this) );

    this.titleSpan.appendChild(this.startButton);

    statusDiv = document.createElement('div');
    statusDiv.innerHTML = '<div style="height:25px;line-height:15px;vertical-align:middle;font-size:120%;"><img id="robotStatusIcon"src="images/redbutton.png" width="15" style="border:0;float:left; display:inline; margin-right: 10px;"/> <b>Robot</b>: <span id="robotstatus"></span> </div>';

    this.contentDiv.appendChild(statusDiv);

    this.robotStatusIcon = document.getElementById("robotStatusIcon");
    this.robotStatus = document.getElementById('robotstatus');
    this.robotStatus.innerHTML = 'stopped';

    if (this.pump != null)
      delete this.pump;

    this.pump = new MessagePump();

    var helpTxt="This gadget is used to bring up the robot";
    this.setHelpText(helpTxt);
  },

  start: function()
  {
    this.run = true;
    this.pump.sendAJAX('/ros/startup', this, this.started);
    this.robotStatus.innerHTML = 'processing...';
    this.robotStatusIcon.src = 'images/yellowbutton.png';
  },
  
  stop: function()
  {
    this.pump.sendAJAX('/ros/shutdown', this, this.stopped);
    this.run = false;
  },

  started: function(myself, pump)
  {
    myself.robotStatus.innerHTML = 'started';
    myself.robotStatusIcon.src = 'images/greenbutton.png';
    myself.startButton.value = 'Stop';
    myself.startButton.observe('click', this.stopt.bind(this) );
  },
  
  stopped: function (myself, pump)
  {
    myself.robotStatus.innerHTML = 'stopped';
    myself.startButton.value = 'Start';
    myself.startButton.observe('click', this.stopt.bind(this) );
  },

});



