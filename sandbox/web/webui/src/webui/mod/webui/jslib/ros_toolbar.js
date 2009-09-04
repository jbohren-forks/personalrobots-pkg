
// *******************************************

var LaunchButtonWidget = Class.create({
  initialize: function(domobj) {
    this.pump = null;
    this.domobj = domobj;
    this.taskid = domobj.getAttribute("taskid");
    this.state = false;
    this.topics = ["/app_update", "/app_status"];
  },

  init: function() {
    var obj = this;
    this.domobj.onmousedown = function() {obj.onmousedown();};

    this.set_state();
  },

  set_state: function() {
    var button = this.domobj;
    if(button != null) {
      this.domobj.style.border = "solid";
      if(this.state == true) {
	button.setAttribute("class", "buttonOn");
      } else {
	button.setAttribute("class", "buttonOff");
      }
    }
  },

  receive: function(topic, msg) {
    if(topic == "/app_update") this.receive_app_update(msg);
    else if(topic == "/app_status") this.receive_app_status(msg);
  },

  receive_app_status: function(msg) {
    for(var i=0; i<msg.active.length; i++) {
      this.receive_app_update(msg.active[i]);
    }
  },

  receive_app_update: function(msg) {
    if(msg.taskid != this.taskid) return;

    var prev_state = this.state;
  
    var state = msg.status;
    if(state == "running") this.state = true;
    if(state == "stopped") this.state = false;

    if(prev_state != this.state) {
      this.set_state();
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
    this.state = null;
    this.topics = ["/app_status"];

    this.button = null;
    this.statusdiv = null;
  },

  init: function() {
    var obj = this;

    this.domobj.innerHTML = '<input class=app_button type=button value="">\n<div class=app_status>&nbsp;</div>';
    this.button = this.domobj.childNodes[0];
    this.statusdiv = this.domobj.childNodes[2];

    this.button.onmousedown = function() {obj.onmousedown();};

    this.set_state();
  },

  receive: function(topic, msg) {
    if(topic == "/app_update") this.receive_app_update(msg);
    else if(topic == "/app_status") this.receive_app_status(msg);
  },

  receive_app_status: function(msg) {
    var active = false;
    for(var i=0; i<msg.active.length; i++) {
      if(msg.active[i].taskid == this.taskid) {
	this.receive_app_update(msg.active[i]);
	active = true;
      }
    }
    if(active == false) {
      this.state = false;
      this.set_state();
    }
  },

  receive_app_update: function(msg) {
    if(msg.taskid != this.taskid) return;

    var prev_state = this.state;

    this.statusdiv.innerHTML = msg.status;

    var state = msg.status;
    if(state == "running") this.state = true;
    if(state == "stopped") this.state = false;

    if(prev_state != this.state) {
      this.set_state();
    }
  },

  set_state: function() {
    if(this.domobj == null) return;

    if(this.state == true) {
      this.button.disabled = 0;
      this.button.value = "Stop";
    } else if (this.state == false) {
      this.button.disabled = 0;
      this.button.value = "Launch";
    } else {
      this.button.disabled = 1;
      this.button.value = "";
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

  receive: function(topic, msg) {
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


