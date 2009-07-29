var ROSStartupGadget = Class.create(ROSGadget, {

  initialize: function()
  {
    this.create("Startup");

    this.titleSpan.innerHTML += "<input type='button' id='startbutton' value='Start' style='float:right; margin-top: 2px' onclick='start();'>";

    statusDiv = document.createElement('div');
    statusDiv.innerHTML = '<div style="height:25px;line-height:25px;vertical-align:middle;font-size:120%;margin-bottom:10px;"><img id="coreStatusIcon" src="images/redbutton.png" width="25" style="border:0;float:left; display:inline; margin-right: 10px;"/> <b>ROS Core</b>: <span id="corestatus"></span> </div>';

    statusDiv.innerHTML += '<div style="height:25px;line-height:25px;vertical-align:middle;font-size:120%;"><img id="robotStatusIcon"src="images/redbutton.png" width="25" style="border:0;float:left; display:inline; margin-right: 10px;"/> <b>Robot</b>: <span id="robotstatus"></span> </div>';

    this.contentDiv.appendChild(statusDiv);

    this.coreStatusIcon = document.getElementById("coreStatusIcon");
    this.coreStatus = document.getElementById('corestatus');
    this.coreStatus.innerHTML = 'stopped';

    this.robotStatusIcon = document.getElementById("robotStatusIcon");
    this.robotStatus = document.getElementById('robotstatus');
    this.robotStatus.innerHTML = 'stopped';

    if (this.pump != null)
      delete this.pump;

    this.pump = new MessagePump();
  },

  start: function()
  {
    this.run = true;
    this.pump.SendAJAX('/ros/startup', this, this.started);
  },
  
  stop: function()
  {
    this.pump.SendAJAX('/ros/shutdown', this, this.stopped);
    this.run = false;
  },

  started: function(myself, pump)
  {
    myself.coreStatus.innerHTML = 'started';
  },
  
  stopped: function (myself, pump)
  {
    myself.coreStatus.innerHTML = 'stopped';
  },
  
});



