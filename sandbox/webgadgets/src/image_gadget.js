var ROSImageWidget = Class.create({
  initialize: function(topic, parentElem)
  {
    if (this.run)
      this.stop();

    this.run = false;

    parentElem = document.getElementById('content');

    if (parentElem.hasChildNodes())
    {
      this.img = parentElem.childNodes[0];
      this.img.height='240';
      this.img.width='320';
    }
    else
    {
      this.img = document.createElement('img');
      this.img.height='240';
      this.img.width='320';
      
      parentElem.appendChild(this.img);
    }

    if (this.ros != null)
      delete this.ros;

    this.ros = new ROSTopic(topic);
  },

  start: function(topic)
  {
    this.run = true;
    this.ros.subscribe(this, this.subscribed);
  },
  
  stop: function()
  {
    this.ros.unsubscribe(null, null);
    this.run = false;
  },

  subscribed: function(myself, pump)
  {
    myself.getImages();
  },
  
  responseCB: function (myself, pump)
  {
    jsonData = eval( '(' + pump.response + ')' );

    myself.img.src = jsonData.data;

    if (myself.run)
    {
      setTimeout( function () {myself.getImages();} , 100);
    }
  },
  
  getImages: function ()
  {
    if (this.run)
      this.ros.getMessage(this, this.responseCB);
  }

});



