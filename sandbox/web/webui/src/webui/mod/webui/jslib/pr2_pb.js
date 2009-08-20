var PowerboardBreakerWidget = Class.create({
  initialize: function(domobj) {
    this.domobj = domobj;
    this.topics = [domobj.getAttribute("topic")];
    this.stateKey = domobj.getAttribute("state");
    this.voltageKey = domobj.getAttribute("voltage");
  },

  init: function() {
  },

  receive: function(msg) {
    var pbmsg = msg.status[0]
    if(pbmsg.name == "Power board 0") {
	var state = null;
	var voltage = null;
	for(var i=0; i<pbmsg.strings.length; i++) {
          if(pbmsg.strings[i].label == this.stateKey) {
	    state = pbmsg.strings[i].value;
  	  }
        }
	for(var j=0; j<pbmsg.values.length; j++) {
          if(pbmsg.values[j].label == this.voltageKey) {
	    voltage = pbmsg.values[j].value;
  	  }
        }
	if(state) this.domobj.className = "powerboard_breaker_" + state;
	var cb;
	if(voltage != null) {
	  voltage = voltage.toFixed(2);
	  cb = state + " @ " + voltage + "V";
	} else {
	  cb = state;
	}
	
        this.domobj.innerHTML = cb;
    }
  }
});

gRosClasses["PowerboardBreakerWidget"] = function(dom){
  return new PowerboardBreakerWidget(dom);
}


var PowerboardRunStopWidget = Class.create({
  initialize: function(domobj) {
    this.domobj = domobj;
    this.topics = [domobj.getAttribute("topic")];
  },

  init: function() {
  },

  receive: function(msg) {
    var pbmsg = msg.status[0]
    if(pbmsg.name == "Power board 0") {
	var estop_button_status = null;
	var estop_wireless_status = null;

	for(var j=0; j<pbmsg.values.length; j++) {
          if(pbmsg.values[j].label == "RunStop Button Status") {
	    var status = parseFloat(pbmsg.values[j].value);
	    if(status > 0.5) {
              estop_wireless_status = "Run";
            } else {
              estop_wireless_status = "Stop";
            }
  	  }
          if(pbmsg.values[j].label == "RunStop Status") {
	    var status = parseFloat(pbmsg.values[j].value);
            if(status > 0.5) {
              estop_button_status = "Run";
            } else {
	      if(estop_wireless_status == "Stop") {
	        estop_button_status = "Unknown";
              } else {
	        estop_button_status = "Stop";
              }
            }
  	  }
        }

	var estop_status = null;
	if(estop_button_status != "Run" || estop_wireless_status != "Run") {
	  estop_status = "Stop";
        } else {
	  estop_status = "Run";
	}

	this.domobj.className = "powerboard_runstop_" + estop_status;

        var cb = "<td class='powerboard_runstop_" + estop_status + "'>RunStop: " + estop_status + "</td><td class='powerboard_runstop_" + estop_button_status + "'>" + "Button: " + estop_button_status + "</td><td class='powerboard_runstop_" + estop_wireless_status + "'>" + "Wireless: " + estop_wireless_status + "</td>";
        this.domobj.innerHTML = cb;
    }
  }
});

gRosClasses["PowerboardRunStopWidget"] = function(dom){
  return new PowerboardRunStopWidget(dom);
}



var PowerboardRunStopWidget_Single = Class.create({
  initialize: function(domobj) {
    this.domobj = domobj;
    this.topics = [domobj.getAttribute("topic")];
    this.key = domobj.getAttribute("key");
  },

  init: function() {
  },

  receive: function(msg) {
    var pbmsg = msg.status[0]
    if(pbmsg.name == "Power board 0") {
	var estop_button_status = null;
	var estop_wireless_status = null;

	for(var j=0; j<pbmsg.values.length; j++) {
          if(pbmsg.values[j].label == "RunStop Button Status") {
	    var status = parseFloat(pbmsg.values[j].value);
	    if(status > 0.5) {
              estop_wireless_status = "Run";
            } else {
              estop_wireless_status = "Stop";
            }
  	  }
          if(pbmsg.values[j].label == "RunStop Status") {
	    var status = parseFloat(pbmsg.values[j].value);
            if(status > 0.5) {
              estop_button_status = "Run";
            } else {
	      if(estop_wireless_status == "Stop") {
	        estop_button_status = "Unknown";
              } else {
	        estop_button_status = "Stop";
              }
            }
  	  }
        }

	var estop_status = null;
	if(estop_button_status != "Run" || estop_wireless_status != "Run") {
	  estop_status = "Stop";
        } else {
	  estop_status = "Run";
	}


	if (this.key == "estop_status") {
	  this.domobj.innerHTML = estop_status;
	  this.domobj.className = "powerboard_runstop_" + estop_status;
	} else if (this.key == "estop_wireless_status") {
	  this.domobj.innerHTML = estop_wireless_status;
	  this.domobj.className = "powerboard_runstop_" + estop_wireless_status;
	} else if (this.key == "estop_button_status") {
	  this.domobj.innerHTML = estop_button_status;
	  this.domobj.className = "powerboard_runstop_" + estop_button_status;
	}
    }
  }
});

gRosClasses["PowerboardRunStopWidget_Single"] = function(dom){
  return new PowerboardRunStopWidget_Single(dom);
}
