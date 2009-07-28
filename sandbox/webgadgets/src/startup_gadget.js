var ROSStartupGadget = Class.create({
  initialize: function()
  {
    /*if (this.run)
      this.stop();

    this.run = false;
*/

    parentElem = document.getElementById('content');

    statusDiv = document.createElement('div');
    statusDiv.style.height = '300px';
    //statusDiv.innerHTML = 'Status';

    parentElem.appendChild(statusDiv);

    /*if (this.pump != null)
      delete this.pump;

    this.pump = new MessagePump();
*/
  },

  start: function()
  {
    this.run = true;
    this.pump.SendAJAX('/ros/startup', this, this.started);
  },
  
  stop: function()
  {
    this.pump.SendAJAX('/ros/shutdown', this, this.stopped);
    this.run = false;
  },

  started: function(myself, pump)
  {
  },
  
  stopped: function (myself, pump)
  {
  },
  
});



