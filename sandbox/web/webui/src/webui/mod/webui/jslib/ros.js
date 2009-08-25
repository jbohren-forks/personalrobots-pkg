function clearDebug() {
  var myspan = document.getElementById('ErrorDiv');
  myspan.innerHTML = "";
}

function ros_debug(str) {
  var myspan = document.getElementById('ErrorDiv');
  myspan.innerHTML = str + "<br>" + myspan.innerHTML;
}

function clearSelection() {
  if (document.selection) {
    document.selection.clear();
  } else {
    if (window.getSelection) {
      sel = window.getSelection();
      sel.removeAllRanges();
    }
  }
}

// *******************************************************
function getDataFromServer(id, url, callback) { 
  var oScript = document.getElementById(id); 
  var head = document.getElementsByTagName("head").item(0); 
  if (oScript) { 
    // Destory object 
    head.removeChild(oScript); 
  }
  // Create object 
  oScript = document.createElement("script"); 
  oScript.setAttribute("type","text/javascript");
  oScript.setAttribute("charset","utf-8");
  oScript.setAttribute("src",url)
  oScript.setAttribute("id",id); 
  head.appendChild(oScript); 
} 

// *******************************************************

var MessagePump = Class.create({
 initialize: function(urlprefix) {
   this.urlprefix = urlprefix;
   this.lastTime = 0;
   this.topicListeners = new Hash();
   this.widgets = [];
   this.http_request = [];
 },

 registerWidget: function(widget) {
   widget.pump = this;

   this.widgets.push(widget);

   for(var i=0; i<widget.topics.length; i++) {
     var listeners = this.topicListeners.get(widget.topics[i]);
     if(listeners == null) {
       listeners = [];
       this.topicListeners.set(widget.topics[i], listeners);
     }
     listeners.push(widget);
   }
  },

 receive_service: function(json_result) {
      // implement this to return the result to the calling service call.
 },

 receive: function(json_result) {
   try {
     this.lastTime = json_result.since;
     this.evalMessages(json_result);
   } catch (e) {
     ros_debug("Error with evalMessages.");
   }
   setTimeout("gPump.pump();", 500);
 },
      
 evalMessages: function(json_result) {
      //       ros_debug("evalMessages()");

  for(var i=0; i<json_result.msgs.length; i++) {
    var msgEnv = json_result.msgs[i];
    //    ros_debug(msgEnv.topic);
    //ros_debug(msgEnv.msg.currentTask + "," + msgEnv.msg.status);
    var listeners = this.topicListeners.get(msgEnv.topic);
    if(listeners) {
      for(var j=0; j<listeners.length; j++) {
        try {
          listeners[j].receive(msgEnv.msg);
        } catch (e) {
          ros_debug("Error with receiver.");
        }
      }
    } else {
      if(msgEnv.topic != "/topics") {
	ros_debug("No listeners for " + msgEnv.topic);
      }
    }
  }
  },

 setupWidgets:  function() {
    var widget = null;
    var allHTMLTags=document.getElementsByTagName("*");
    for (i=0; i<allHTMLTags.length; i++) {
      var domobj = allHTMLTags[i];
      
      var objtype = domobj.getAttribute("objtype");
      if(objtype) {
	var clss = gRosClasses[objtype];
	if(clss) {
	  widget = clss(domobj);
	  this.registerWidget(widget);
	}
      }
    }

    for(var i=0; i<this.widgets.length; i++) {
      this.widgets[i].init();
    }

    var urlprefix = this.urlprefix;
    this.topicListeners.each(function(pair) {
       var uri = urlprefix + "/subscribe";
       //       new Ajax.Request(uri, {parameters: {topic:pair.key}, method: "get"});
       uri = uri + "?" + "topic=" + pair.key;
       //       getDataFromServer("_ros_subscribe", uri);
    });
  },

  service_call: function(service_name, parameterList) {
    var parameters = {args: parameterList};

    var uri = this.urlprefix + "/service/" + service_name;
    //new Ajax.Request(uri, {parameters: parameters, method: 'get'});
    uri = uri + "?callback=gPump.receive_service&json=" + Object.toJSON(parameterList);
    getDataFromServer("_ros_service_pump", uri);
  },
  receive_server: function(msg) {
    alert('receive_server');
  },

  pump: function() {
      var uri = this.urlprefix + "/receive?callback=gPump.receive&since=" + this.lastTime;
      this.topicListeners.each(function(pair) {uri = uri + "&topic=" + pair.key; });

      getDataFromServer("_ros_pump", uri);
    },

  pump_old: function() {
    var uri = this.urlprefix + "/receive?since=" + this.lastTime;
    this.topicListeners.each(function(pair) {
      uri = uri + "&topic=" + pair.key;
    });

    var obj = this;
    new Ajax.Request(uri, {
      method: 'get',
      requestHeaders: {Accept: 'application/json'},
      onSuccess: function(transport) {
        try {
          var json = transport.responseText.evalJSON();
  	  obj.lastTime = json.since;
          obj.pump();
	  obj.evalMessages(json);
        } catch(e) {
          obj.pump();
        }
      },
      onFailure: function(transport) {
        obj.pump();
      }
    });
  }
});

var gPump = null;

function ros_handleOnLoad(prefix) 
{
  gPump = new MessagePump("http://" + window.location.hostname + ":8080" + prefix);
  gPump.setupWidgets();
  gPump.pump();
}

// *******************************************
gRosClasses = [];

// *******************************************

var OnOffButtonWidget = Class.create({
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

gRosClasses["OnOffButtonWidget"] = function(dom){
  return new OnOffButtonWidget(dom);
}

// *******************************************

var TextWidget = Class.create({
  initialize: function(domobj) {
    this.pump = null;
    this.domobj = domobj;
    this.topics = [domobj.getAttribute("topic")];
    this.key = domobj.getAttribute("key");
    this.selector = domobj.getAttribute("selector");
    this.selectorValue = domobj.getAttribute("selectorValue");
  },

  init: function() {
  },

  receive: function(msg) {
    if(this.selector && this.selectorValue) {
      if(msg[this.selector] != this.selectorValue) return;
    }
    if(msg[this.key] != null) {
      this.domobj.innerHTML = msg[this.key];
    }
  }
});

gRosClasses["TextWidget"] = function(dom){
  return new TextWidget(dom);
}

// *******************************************

var PercentTextWidget = Class.create({
  initialize: function(domobj) {
    this.pump = null;
    this.domobj = domobj;
    this.topics = [domobj.getAttribute("topic")];
    this.key = domobj.getAttribute("key");
    this.key2 = domobj.getAttribute("key2");
  },

  init: function() {
  },

  receive: function(msg) {
    if(msg[this.key] != null) {
      var percent = parseFloat(msg[this.key]) / parseFloat(msg[this.key2]);
      this.domobj.innerHTML = (100. * percent).toFixed(2) + "%";
    }
  }
});

gRosClasses["PercentTextWidget"] = function(dom){
  return new PercentTextWidget(dom);
}

// *******************************************

var ScrollingTextWidget = Class.create({
  initialize: function(domobj) {
    this.pump = null;
    this.domobj = domobj;
    this.topics = [domobj.getAttribute("topic")];
    this.maxlines = parseInt(domobj.getAttribute("maxlines", "100"));

    this.textdiv = null;
  },

  init: function() {
      this.domobj.style.position = "absolute";
      this.domobj.style.overflow = "auto";

      this.textdiv = document.createElement("div");
      this.domobj.appendChild(this.textdiv);
  },

  new_message: function(msg) {
     var d = document.createElement("div");
     d.innerHTML = Object.toJSON(msg) + "<br>";
     this.textdiv.appendChild(d);      
  },

  receive: function(msg) {
    this.new_message(msg);
    if(this.textdiv.childNodes.length > this.maxlines) {
      this.textdiv.removeChild(this.textdiv.childNodes[0]);
    } else {
      this.domobj.scrollTop = this.domobj.scrollHeight;
    }
  }
});

gRosClasses["ScrollingTextWidget"] = function(dom){
  return new ScrollingTextWidget(dom);
}

var TerminalTextWidget = Class.create(ScrollingTextWidget, {
  initialize: function($super, domobj) {
    this.key = domobj.getAttribute("key");
    $super(domobj);
  },
  new_message: function($super, msg) {
    if(msg[this.key] != null) {
      var d = document.createElement("div");
      d.innerHTML = msg[this.key] + "<br>";
      this.textdiv.appendChild(d);      
    }
  }
});

gRosClasses["TerminalTextWidget"] = function(dom){
  return new TerminalTextWidget(dom);
}



// *******************************************

var ListWidget = Class.create({
  initialize: function(domobj) {
    this.pump = null;
    this.domobj = domobj;
    this.topics = [domobj.getAttribute("topic")];
    this.key = domobj.getAttribute("key");
    this.itemhtml = null;
  },

  init: function() {
    this.itemhtml = this.domobj.innerHTML;
    this.domobj.innerHTML = "";
  },

  receive: function(msg) {
    if(msg[this.key] != null) {
      var lst = msg[this.key];
      var s = "";
      for(var i=0; i<lst.length; i++) {
	if(this.itemhtml) {
	  var t = this.itemhtml.replace(/__item__/g, lst[i]);
	  s = s + t;
	} else {
	  s = s + "<li>" + lst[i];
	}
      }
      this.domobj.innerHTML = s;
    }
  }
});

gRosClasses["ListWidget"] = function(dom){
  return new ListWidget(dom);
}


var MessageWidget = Class.create({
  initialize: function(domobj) {
    this.pump = null;
    this.domobj = domobj;
    this.topics = [domobj.getAttribute("topic")];
  },

  init: function() {
    this.domobj.innerHTML = "";
  },

  receive: function(msg) {
      var s = "<ul>";
      for(var key in msg) {
        s = s + "<li>" + key + ": " + msg[key];
      }
      s = s + "</ul>";
      this.domobj.innerHTML = s;
    }
});

gRosClasses["MessageWidget"] = function(dom){
  return new MessageWidget(dom);
}





var RosOut_Widget = Class.create({
  initialize: function(domobj) {
    this.pump = null;
    this.domobj = domobj;
    this.topics = ['/rosout'];
    this.maxlines = parseInt(domobj.getAttribute("maxlines", "100"));

    this.tbl = null;
  },

  init: function() {
      this.domobj.style.position = "absolute";
      this.domobj.style.overflow = "auto";

      this.tbl = document.createElement("table");
      this.tbl.width = "100%";
      this.tbl.className = "rosout";
      this.domobj.appendChild(this.tbl);
  },

  new_message: function(msg) {
      var lastRow = this.tbl.rows.length;

      var row = this.tbl.insertRow(lastRow);
      row.className = "rosout";
      var c, tn;
      c = row.insertCell(0);
      c.className = "rosout";
      c.style.width = "5%";
      var level = "";
      if(msg.level == "16") level="Fatal";
      if(msg.level == "8") level="Error";
      if(msg.level == "4") level="Warn";
      if(msg.level == "2") level="Info";
      if(msg.level == "1") level="Debug";
      tn = document.createTextNode(level);
      c.appendChild(tn);

      c = row.insertCell(1);
      c.className = "rosout";
      c.style.width = "80%";
      tn = document.createTextNode(msg.msg);
      c.appendChild(tn);


      c = row.insertCell(2);
      c.style.width = "15%";
      c.className = "rosout";
      tn = document.createTextNode(msg.name);
      c.appendChild(tn);

      //      c = row.insertCell(3);
      //      c.style.width = "5%";
      //      c.className = "rosout";
      //      tn = document.createTextNode(msg.file + ":" + msg.line);
      //      c.appendChild(tn);

      //c = row.insertCell(3);
      //c.style.width = "10%";
      //c.className = "rosout";
      //tn = document.createTextNode(Object.toJSON(msg.topics));
      //c.appendChild(tn);
  },

  receive: function(msg) {
    this.new_message(msg);
    if(this.tbl.rows.length > this.maxlines) {
      this.tbl.deleteRow(0);
    } else {
      this.domobj.scrollTop = this.domobj.scrollHeight;
    }
  }
});

gRosClasses["RosOut_Widget"] = function(dom){
  return new RosOut_Widget(dom);
}




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


