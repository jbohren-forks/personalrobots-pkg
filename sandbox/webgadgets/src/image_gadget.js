var ROSImageGadget = Class.create(ROSGadget, {
  initialize: function()
  {
    this.create("Image View", 600);

    this.subscribeButton = document.createElement('input');

    this.subscribeButton.id = 'subscribeButton';
    this.subscribeButton.type = 'button';
    this.subscribeButton.value = 'Subscribe';
    this.subscribeButton.style.cssFloat = 'right';
    this.subscribeButton.style.margin = '2px 0 0 0';

    this.titleSpan.appendChild(this.subscribeButton);

    this.img = document.createElement('img');
    this.img.style.height='240';
    this.img.style.width='320';
    this.img.style.border='1px solid black';
      
    this.contentDiv.appendChild(this.img);
    this.contentDiv.align='center';

    this.contentDiv.style.bgColor= "#FF00FF";
    this.contentDiv.height = '320';

    this.ros = null;
    this.topicSelect = document.createElement('select');
    this.topicSelect.id = 'topicSelect';
    this.topicSelect.style.cssFloat = "right"
    this.topicSelect.style.margin = "2px 0 0 0";
    this.topicSelect.add(new Option("----",""), null);
    this.titleSpan.appendChild(this.topicSelect);

    label = document.createElement('label');
    label.style.cssFloat  = 'right';
    label.style.margin = '0 0 0 5px';
    label.innerHTML = "Topic:";
    this.titleSpan.appendChild(label);

    this.subscribeButton.observe( 'click', this.start.bind(this));

    pump = new MessagePump();
    pump.sendAJAX("/ros/topics", this, this.initCB);

    this.setHelpText('image_gadget_help.html');

  },

  initCB: function(myself, pump)
  {
    topics = eval( '(' + pump.response + ')' );

    for (i = 0; i < topics.topics.length; i+=1)
    {
      if (topics.topics[i].type == "sensor_msgs/CompressedImage" ||
          topics.topics[i].type == "sensor_msgs/Image")
      {
        var optNew = new Option(topics.topics[i].name, topics.topics[i].type);
        myself.topicSelect.add(optNew, null);
      }
    } 
  },

  start: function(e)
  {
    topic = this.topicSelect.options[this.topicSelect.selectedIndex].text;

    if (this.ros != null)
      delete this.ros;

    this.ros = new ROSTopic(topic);
    this.ros.setCallback(this, this.imageCB);
  },
  
  stop: function()
  {
    this.ros.unsubscribe(null, null);
  },
 
  imageCB: function (myself, pump)
  {
    jsonData = eval( '(' + pump.response + ')' );

    myself.img.src = jsonData.data;
  },
  
});
