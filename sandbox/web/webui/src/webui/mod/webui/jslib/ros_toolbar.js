
// *******************************************

var LaunchButtonWidget = Class.create({
  initialize: function(domobj) {
    this.pump = null;
    this.domobj = domobj;
    this.taskid = domobj.getAttribute("taskid");
    this.state = false;
    this.topics = [domobj.getAttribute("topic")];
  },

  init: function() {
    var obj = this;
    this.domobj.onmousedown = function() {obj.onmousedown();};

    if(this.state == true) {
      this.domobj.setAttribute("class", "buttonOn");
    } else {
      this.domobj.setAttribute("class", "buttonOff");
    }

    //this.pump.service_call("status_update", []);
  },

  receive: function(msg) {
    if(msg.taskid != this.taskid) return;

    var prev_state = this.state;
  
    var state = msg.status;
    if(state == "running") this.state = true;
    if(state == "stopped") this.state = false;

    if(prev_state != this.state) {
      var button = this.domobj;
      if(button != null) {
	this.domobj.style.border = "solid";
	if(this.state == true) {
	  button.setAttribute("class", "buttonOn");
	} else {
	  button.setAttribute("class", "buttonOff");
	}
      }
    }
  },

  onmousedown: function(evt) {
    var newstate = !this.state;

    this.domobj.style.border = "dotted";
    if(newstate) {
      this.pump.service_call("start_task", {'taskid':this.taskid, 'username':'anonymous'});
    } else {
      this.pump.service_call("stop_task", {'taskid':this.taskid, 'username':'anonymous'});
    }
    clearSelection();
  }
});

gRosClasses["LaunchButtonWidget"] = function(dom){
  return new LaunchButtonWidget(dom);
}

// *******************************************


var LaunchButtonWidget2 = Class.create({
  initialize: function(domobj) {
    this.pump = null;
    this.domobj = domobj;
    this.taskid = domobj.getAttribute("taskid");
    this.state = false;
    this.topics = [domobj.getAttribute("topic")];
  },

  init: function() {
    var obj = this;
    this.domobj.onmousedown = function() {obj.onmousedown();};

    this.set_state();
  },

  receive: function(msg) {
    if(msg.taskid != this.taskid) return;

    var prev_state = this.state;

    var state = msg.status;
    if(state == "running") this.state = true;
    if(state == "stopped") this.state = false;

    if(prev_state != this.state) {
      this.set_state(this.state);
    }
  },

  set_state: function() {
    if(this.domobj == null) return;

    if(this.state == true) {
      //      this.domobj.disabled = 0;
      this.domobj.value = "Stop";
    } else if (this.state == false) {
      //      this.domobj.disabled = 0;
      this.domobj.value = "Launch";
    } else {
      this.domobj.disabled = 1;
      this.domobj.value = "";
    }
  },

  onmousedown: function(evt) {
    var newstate = !this.state;

    if(newstate) {
      this.pump.service_call("start_task", {'taskid':this.taskid, 'username':'anonymous'});
    } else {
      this.pump.service_call("stop_task", {'taskid':this.taskid, 'username':'anonymous'});
    }
  }
});

gRosClasses["LaunchButtonWidget2"] = function(dom){
  return new LaunchButtonWidget2(dom);
}

// *******************************************

var ActiveTasks = Class.create({
  initialize: function(domobj) {
    this.pump = null;
    this.domobj = domobj;
    this.topics = [domobj.getAttribute("topic")];
  },

  init: function() {
    this.domobj.innerHTML = "";
  },

  receive: function(msg) {
      var s = "";
      for(var i=0; i<msg.active.length; i++) {
        s = s + "|" + "<a href='/webui/app/" + msg.active[i].taskid + "'>" + msg.active[i].name + "</a>";
      }
      this.domobj.innerHTML = s;
    }
});

gRosClasses["ActiveTasks"] = function(dom){
  return new ActiveTasks(dom);
}


