
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
  
    var state = msg.status;
    if(state == "running") this.state = true;
    if(state == "stopped") this.state = false;

    var button = this.domobj;
    if(button != null) {
      if(this.state == true) {
	button.setAttribute("class", "buttonOn");
      } else {
	button.setAttribute("class", "buttonOff");
      }
    }
  },

  onmousedown: function(evt) {
    var newstate = !this.state;

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
        s = s + "|" + msg.active[i];
      }
      this.domobj.innerHTML = s;
    }
});

gRosClasses["ActiveTasks"] = function(dom){
  return new ActiveTasks(dom);
}


