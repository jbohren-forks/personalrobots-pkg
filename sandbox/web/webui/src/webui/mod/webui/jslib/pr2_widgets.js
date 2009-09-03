var BatteryMonitor = Class.create({
  initialize: function(domobj) {
    this.domobj = domobj;
    this.topics = [domobj.getAttribute("topic")];
    this.img = new Element('img', {'src' : '/webui/webui/templates/images/toolbar/battery_gray.png', 'width': 41, 'height': 16});
    this.canvas = new Element('canvas', {'width': this.img.width, 'height': this.img.height});
    this.domobj.appendChild(this.canvas);
    this.span = new Element('span', {'style':"font-size:11px;position:relative;top:-3"});
    this.domobj.appendChild(this.span);
    this.img.onload = this.draw.bind(this);
  },

  draw: function() {
    if (this.img.complete) {
      var ctx = this.canvas.getContext('2d');  
      ctx.drawImage(this.img,0,0);  
      var width = 32 * this.percent / 100;
      ctx.beginPath();  
      ctx.moveTo(38-width, 1);  
      ctx.lineTo(38, 1);  
      ctx.lineTo(40, 4);  
      ctx.lineTo(40,11);  
      ctx.lineTo(38,15);  
      ctx.lineTo(38-width,15);  
      ctx.lineTo(38-width,1);  
      if (this.percent > 30)
        ctx.fillStyle = "rgb(0, 200, 0)";
      else
        ctx.fillStyle = "rgb(200, 0, 0)";
      ctx.fill();  
    }
    this.span.innerHTML = ' ' + this.percent + '%';
  },

  init: function() {
    this.percent = 0;
    this.draw();
  },

  receive: function(topic, msg) {
    if(msg['energy_remaining'] != null) {
      this.percent = Math.round(100. * parseFloat(msg['energy_remaining']) / parseFloat(msg['energy_capacity']));
      if (this.percent > 100) this.percent = 100;
      else if (this.percent < 0) this.percent = 0;
      this.draw();
    }
  }
});

gRosClasses["BatteryMonitor"] = function(dom){
  return new BatteryMonitor(dom);
}

var CircuitMonitor = Class.create({
  initialize: function(domobj) {
    this.domobj = domobj;
    this.topics = [domobj.getAttribute("topic")];
  },

  init: function() {
    var html = '<span style="font-size:11px;position:relative;top:-3">Circuits:';
    html += '</span>';
    this.domobj.innerHTML = html;
  },

  receive: function(topic, msg) {
    var html = '<span style="font-size:11px;position:relative;top:-3">Circuits:</span><span>';
    var states = msg['circuit_state'];
    for (var i = 0; i < 3; ++i) {
      var src;
      if (states[i] == 3) // Running
        src = 'green_ball.png';
      else if (states[i] == 1) // Standby
        src = 'orange_ball.png';
      else
        src = 'red_ball.png';
      html += '<img src="/webui/webui/templates/images/toolbar/' + src + '"> ';
    }
    html += '</span>';
    this.domobj.innerHTML = html;
  }
});

gRosClasses["CircuitMonitor"] = function(dom){
  return new CircuitMonitor(dom);
}

var ChargeMonitor = Class.create({
  initialize: function(domobj) {
    this.domobj = domobj;
    this.topics = [domobj.getAttribute("topic")];
  },

  init: function() {
  },

  receive: function(topic, msg) {
    var voltage = msg['input_voltage'];
    var src;

    if (voltage >= 68.0) {
      src = 'ac_power.png';
    } else {
      src = 'battery_power.png';
    }
    this.domobj.innerHTML = '<img src="/webui/webui/templates/images/toolbar/' + src + '"> ';
  }
});

gRosClasses["ChargeMonitor"] = function(dom){
  return new ChargeMonitor(dom);
}

