////////////////////////////////////////////////////////////////////////////////
// Message Pump class. Handles sending and receiving AJAX messages
var MessagePump = Class.create({
  emptyFunction: function() {},

  initialize: function()
  {
    this.response = 'no message';
    this.responseFunc = this.emptyFunction;
    this.responseObj = null;
  },
  
  sendAJAX: function (request, _responseObj, _responseFunc)
  {
    if (_responseFunc != null)
    {
      this.responseFunc = _responseFunc;
      this.responseObj = _responseObj;
    }

    new Ajax.Request(request, 
        {method: 'get', 
         onSuccess: function(transport) 
         { this.response = transport.responseText; }.bind(this),
         onFailure: function()
         { this.response = 'error'; }.bind(this),
         onComplete: function()
         { this.completeFunction(); }.bind(this)
        });
  },
     
  completeFunction: function()
  {
    this.responseFunc(this.responseObj, this);
  }
  
});


////////////////////////////////////////////////////////////////////////////////
// ROS TF class. 
var ROSTF = Class.create({
  initialize: function(_tf)
  {
    this.tf = _tf
  },

  subscribe: function (obj, callback)
  {
    pump = new MessagePump();
    pump.sendAJAX('/ros/tfsub' + this.tf, obj, callback);
  },

  getMessage: function(obj, callback)
  {
    pump = new MessagePump();
    pump.sendAJAX('/ros/tfget' + this.tf, obj, callback);
  }

});


////////////////////////////////////////////////////////////////////////////////
// ROS Topic class, a simple interface to ros topics
var ROSTopic = Class.create({
  initialize: function(_topic)
  {
    this.topic = _topic
  },

  subscribe: function (obj, callback)
  {
    pump = new MessagePump();
    pump.sendAJAX('/ros/subscribe' + this.topic, obj, callback);
  },

  announce: function (obj, callback)
  {
    pump = new MessagePump();
    pump.sendAJAX('/ros/announce' + this.topic, obj, callback);
  },

  unsubscribe: function (obj, callback)
  {
    pump = new MessagePump();
    pump.sendAJAX('/ros/unsubscribe' + this.topic, obj, callback);
  },

  getMessage: function(obj, callback)
  {
    pump = new MessagePump();
    pump.sendAJAX('/ros/get' + this.topic, obj, callback);
  },

  publish: function(msg, obj, callback)
  {
    pump = new MessagePump();
    pump.sendAJAX('/ros/pub' + this.topic + msg, obj, callback);
  },

});


////////////////////////////////////////////////////////////////////////////////
// A ROS Gadget. This will load a ros gadget
var ROSGadget = Class.create({
  initialize: function(name)
  {
    var html = "<iframe src='" + name + ".html' width='600' height='600' frameborder='0' scrolling='no'/>";
    element = document.body;
    element.innerHTML += html;
  },
});
