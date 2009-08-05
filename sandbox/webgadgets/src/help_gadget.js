
var HelpGadget = Class.create( Gadget, {
  initialize : function($super, titleName, gadget)
  {
    titleName = "Help: " + titleName;
    $super(titleName);

    this.gadget = gadget;

    this.mainDiv.style.display = "none";
    this.mainDiv.style.width = "500px";
    this.mainDiv.style.zIndex = "2000";

    title = document.createElement("b");
    title.style.id = "gadgetTitle";
    title.style.cssFloat = "left";
    title.style.margin= "0 0 0 10px";
    title.innerHTML = titleName;

    this.titleSpan.appendChild(title);

    this.closeButton.stopObserving('click');
    this.closeButton.observe('click', this.hideHelp.bind(this) );
    this.pump = new MessagePump();
  },

  gotHelp: function(myself, pump)
  {
    myself.contentDiv.innerHTML = pump.response;
  },

  showHelp : function(e)
  {
    left = parseInt(this.gadget.offsetLeft) + parseInt(this.gadget.clientWidth) + 10;
    this.mainDiv.style.left = left + "px";
    this.mainDiv.style.top = parseInt(this.gadget.offsetTop) + "px";
    this.mainDiv.style.display = "";
  },

  hideHelp : function(e)
  {
    this.mainDiv.style.display = "none";
  },

  setContent : function (file)
  {
    this.pump.sendAJAX(file, this, this.gotHelp);
  }

});
