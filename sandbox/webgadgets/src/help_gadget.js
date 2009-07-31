
var HelpGadget = Class.create({
  initialize : function(titleName)
  {
    // The help DIV
    this.helpDiv = document.createElement("div");
    this.helpDiv.id = "helpbox";
    this.helpDiv.name = "help";
    this.helpDiv.style.display = "none";

    this.helpDivHeader = document.createElement("div");
    this.helpDivHeader.id = "header";

    // This is not the cleanest way to close the help gadgets 
    this.helpDivHeader.innerHTML += "<a href='#' onclick='hideHelp();' id='xbutton'><img src='images/xicon.png'></a>Help:"+titleName;
    
    this.helpDivContent = document.createElement("div");
    this.helpDivContent.id = "content";
  
    this.helpDiv.appendChild(this.helpDivHeader);
    this.helpDiv.appendChild(this.helpDivContent);
  
    document.body.appendChild(this.helpDiv);
  },

  showHelp : function(e)
  {
    this.helpDiv.style.display = "";
  },

  hideHelp : function(e)
  {
    this.helpDiv.style.display = "none";
  },

  setContent : function (txt)
  {
    this.helpDivContent.innerHTML = txt;
  }

});
