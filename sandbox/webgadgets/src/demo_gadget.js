var ROSStartupGadget = Class.create(ROSGadget, {

  initialize: function($super)
  {
    $super("Demos", 500);

    this.contentDiv.style.height = "140px";

    this.startButton = document.createElement('input');
    this.startButton.type = 'button';
    this.startButton.id = 'startbutton';
    this.startButton.value = 'Start';
    this.startButton.style.cssFloat = 'left';
    this.startButton.style.margin = '0 10px 0 0';
    this.startButton.observe('click', this.start.bind(this) );

    label = document.createElement('label');
    label.style.cssFloat  = 'right';
    label.style.margin = '0 0 0 5px';

    this.titleSpan.appendChild (label);

    this.statusDiv = document.createElement('div');
    this.statusDiv.style.width = '100%';
    this.statusDiv.style.height = "30px";
    this.statusDiv.style.borderBottom = '4px dotted black';
    this.statusDiv.style.marginBottom = '10px';

    this.robotLabel = document.createElement("span");
    this.robotLabel.style.lineHeight = '20px';
    this.robotLabel.style.verticalAlign = 'middle';
    this.robotLabel.style.fontSize = '130%';
    this.robotLabel.style.cssFloat = 'left';
    this.robotLabel.style.fontWeight = 'bold';
    this.robotLabel.innerHTML = 'Robot: ';

    this.robotStatus = document.createElement('span');
    this.robotStatus.id = 'robotstatus';
    this.robotStatus.style.lineHeight = '20px';
    this.robotStatus.style.verticalAlign = 'middle';
    this.robotStatus.style.fontSize = '120%';
    this.robotStatus.style.marginLeft = '5px';
    this.robotStatus.style.marginTop = '1px';
    this.robotStatus.style.cssFloat = 'left';
    this.robotStatus.innerHTML = 'stopped';

    this.robotStatusIcon = document.createElement('img');
    this.robotStatusIcon.id = 'robotStatusIcon';
    this.robotStatusIcon.src = 'images/redbutton.png';
    this.robotStatusIcon.style.width = '20px';
    this.robotStatusIcon.style.cssFloat = 'right';

    this.statusDiv.appendChild(this.startButton);
    this.statusDiv.appendChild(this.robotLabel);
    this.statusDiv.appendChild(this.robotStatus);
    this.statusDiv.appendChild(this.robotStatusIcon);
    
    this.contentDiv.appendChild(this.statusDiv);

    if (this.pump != null)
      delete this.pump;

    this.pump = new MessagePump();

    this.setHelpText('startup_gadget_help.html');

    var gaugeWidth = 100;
    var gaugeHeight = 100;

    this.gaugeDiv = document.createElement("div");
    this.gaugeDiv.style.border = "0px solid black";
    this.gaugeDiv.style.display = "inline";
    this.gaugeDiv.style.cssFloat = "left";
    this.gaugeDiv.style.width = gaugeWidth + "px";
    this.gaugeDiv.style.height = gaugeHeight + "px";

    this.gauge = new google.visualization.Gauge(this.gaugeDiv);

    this.gaugeOptions = {width: gaugeWidth, height: gaugeHeight, 
                         redFrom: 0, redTo: 15, yellowFrom:15, 
                         yellowTo: 30, minorTicks: 5};

    this.gaugeData = new google.visualization.DataTable();
    this.gaugeData.addColumn('string', 'Label');
    this.gaugeData.addColumn('number', 'Value');
    this.gaugeData.addRows(1);
    this.gaugeData.setValue(0, 0, "% Battery");
    this.gaugeData.setValue(0, 1, 0);

    this.gauge.draw(this.gaugeData, this.gaugeOptions);

    this.contentDiv.appendChild(this.gaugeDiv);

    this.plugIcon = document.createElement("img");
    this.plugIcon.src = 'images/unplugged.png';
    this.plugIcon.style.display = "inline";
    this.plugIcon.style.cssFloat = "left";
    this.plugIcon.style.width = "80px";
    this.contentDiv.appendChild(this.plugIcon);

  },

  batteryStateChange: function(myself, pump)
  {
    jsonData = eval('(' + pump.response + ')');

    remaining = parseFloat(jsonData.energy_remaining);
    capacity = parseFloat(jsonData.energy_capacity);

    percent = (remaining / capacity) * 100.0;
    myself.gaugeData.setValue(0,1,percent|0);
    myself.gauge.draw(myself.gaugeData, myself.gaugeOptions);
  },

  start: function()
  {
    this.run = true;
    this.pump.sendAJAX('/ros/startup', this, this.started);
    this.robotStatus.innerHTML = 'starting...';
    this.robotStatusIcon.src = 'images/yellowbutton.png';
  },
  
  stop: function()
  {
    this.pump.sendAJAX('/ros/shutdown', this, this.stopped);
    this.run = false;

    this.robotStatus.innerHTML = 'stopping...';
    this.robotStatusIcon.src = 'images/yellowbutton.png';
  },

  started: function(myself, pump)
  {
    myself.robotStatus.innerHTML = 'started';
    myself.robotStatusIcon.src = 'images/greenbutton.png';
    myself.startButton.value = 'Stop';

    //myself.startButton.observe('click', Event.stop );
    myself.startButton.observe('click', myself.stop.bind(myself) );

    myself.batteryTopic = new ROSTopic("/battery_state");
    myself.batteryTopic.setCallback(myself, myself.batteryStateChange);
  },
  
  stopped: function (myself, pump)
  {
    myself.robotStatus.innerHTML = 'stopped';
    myself.startButton.value = 'Start';
    myself.robotStatusIcon.src = 'images/redbutton.png';

    myself.startButton.observe('click', myself.start.bind(myself) );
  },

});



