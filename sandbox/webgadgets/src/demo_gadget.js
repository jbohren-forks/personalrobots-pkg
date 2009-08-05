var ROSDemoGadget = Class.create(ROSGadget, {

  initialize: function($super)
  {
    $super("Demos", 200);

    this.contentDiv.style.height = "140px";

    this.demoSelect = document.createElement('select');
    this.demoSelect.size = '6';
    this.demoSelect.style.width = '200px';
    this.demoSelect.style.marginBottom = '5px';
    this.demoSelect.name = 'demoSelect';
    this.demoSelect.add(new Option("Door Open",""), null);
    this.demoSelect.add(new Option("Person Follow",""), null);
    this.demoSelect.add(new Option("Plugin",""), null);
    this.demoSelect.add(new Option("Milestone 2",""), null);

    this.startButton = document.createElement('input');
    this.startButton.type = 'button';
    this.startButton.id = 'startbutton';
    this.startButton.value = 'Start Demo';
    this.startButton.style.cssFloat = 'left';
    this.startButton.style.margin = '0 10px 0 0';
    this.startButton.observe('click', this.start.bind(this) );

    this.stopButton = document.createElement('input');
    this.stopButton.type = 'button';
    this.stopButton.id = 'stopbutton';
    this.stopButton.value = 'Stop Demo';
    this.stopButton.style.cssFloat = 'left';
    this.stopButton.style.margin = '0 10px 0 0';
    this.stopButton.observe('click', this.start.bind(this) );

    this.contentDiv.appendChild(this.demoSelect);
    this.contentDiv.appendChild(this.startButton);
    this.contentDiv.appendChild(this.stopButton);

    this.setHelpText('demo_gadget_help.html');
  },

  start: function()
  {
  },
  
  stop: function()
  {
  },

});



