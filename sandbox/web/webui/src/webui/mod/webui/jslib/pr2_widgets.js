var BatteryMonitor = Class.create({
  initialize: function(domobj) {
    this.domobj = domobj;
    this.topics = [domobj.getAttribute("topic")];

    this.key = domobj.getAttribute("key");
    this.key2 = domobj.getAttribute("key2");
  },

  draw: function() {
    var canvas = this.domobj.childNodes[0];
    var span = this.domobj.childNodes[1];
    var ctx = canvas.getContext('2d');  
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
    span.innerHTML = ' ' + this.percent + '%';
  },

  init: function() {
    this.img = new Image();  
    this.img.src = 'battery_gray.png';  
    this.domobj.innerHTML = '<canvas width=41 height=16></canvas><span style="font-size:11px;position:relative;top:-3"></span>'

    var width = this.domobj.getAttribute("width");
    if(!width) width=120;
    else width = parseInt(width);

    var height = this.domobj.getAttribute("height");
    if(!height) height=120;
    else height = parseInt(height);

    this.percent = 0;
    this.draw();
  },

  receive: function(msg) {
    if(msg[this.key] != null) {
      this.percent = 100. * parseFloat(msg[this.key]) / parseFloat(msg[this.key2]);
      if (this.percent > 100) this.percent = 100;
      else if (this.percent < 0) this.percent = 0;
      this.draw();
    }
  }
});

gRosClasses["BatteryMonitor"] = function(dom){
  return new BatteryMonitor(dom);
}

