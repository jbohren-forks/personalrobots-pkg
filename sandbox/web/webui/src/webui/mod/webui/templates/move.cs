<html>
  <head>
    <?cs include:"includes.cs" ?>

<style>
div.outlet_available {
  display: block;
  background: #ff9900;
  border: 1px solid #000;
  padding: 1px;
  position: absolute;
}

div.outlet_unavailable {
  display: block;
  background: #ff0000;
  border: 1px solid #000;
  padding: 1px;
  position: absolute;
}

div.robot {
  display: block;
  padding: 1px;
  position: absolute;
  width: 48px;
  height: 48px;
}
</style>

<script type="text/javascript">

// *******************************************

var RobotWidget = Class.create({
  initialize: function(domobj) {
    this.pump = null;
    this.domobj = domobj;
    this.topics = [domobj.getAttribute("topic")];
  },

  init: function() {
  },

  receive: function(msg) {
    this.domobj.innerHTML = msg.pose.position.x + " " + msg.pose.position.y;

    moveRobot(msg.pose);
  }
});

gRosClasses["RobotWidget"] = function(dom){
  return new RobotWidget(dom);
}

var gBuilding = {};

function setupBuilding()
{
  var building = $('map');
  var rez = building.getAttribute("rez");
  if(rez) {
    rez = parseFloat(rez);
  } else {
    rez = 1;
  }
  gBuilding.rez = rez;

  [gBuilding.left, gBuilding.top] = findPos(building);
  gBuilding.width = building.width;
  gBuilding.height = building.height;

  gBuilding.bwidth = building.width * rez;
  gBuilding.bheight = building.height * rez;
}
	

function heading_from_pose(p)
{
  var heading;
  var attitude;
  var test = p.y*p.z + p.x*p.w;
  if(test > 0.499) {
    heading = 2. * Math.atan2(p.y,p.w);
    attitude = Math.PI / 2;
    return heading;
  }
  if(test < -0.499) {
    heading = -2. * Math.atan2(p.y,p.w);
    attitude = -Math.PI / 2;
    return heading;
  }
  var sqx = p.x * p.x;
  var sqy = p.y * p.y;
  var sqz = p.z * p.z;

  heading = Math.atan2(2.*p.z*p.w - 2.*p.y*p.z, 1. - 2.*sqz - 2.*sqx);
  if(heading < 0) {
    heading = heading + Math.PI*2;
  }

  //$('heading').innerHTML = "test: " + test + " heading: " + heading;
  return heading;
}

function moveRobot(pose) 
{
  var heading = heading_from_pose(pose.orientation);
  var d = parseInt(heading * 180. / Math.PI);
  $('robotimg').src = "templates/images/pr2_" + d + ".png";

  var left = parseInt(pose.position.x / gBuilding.rez);
  var top = parseInt(pose.position.y / gBuilding.rez);

  var robot = $('robot');

  var width = 48;
  var height = 48;  

  robot.style.left = gBuilding.left + left - (width/2);
  robot.style.top = gBuilding.top + gBuilding.height - (top + (height/2));
}

// *******************************************

function findPos(obj) {
  var curleft = curtop = 0;
  if (obj.offsetParent) {
    do {
      curleft += obj.offsetLeft;
      curtop += obj.offsetTop;
    } while (obj = obj.offsetParent);
    return [curleft,curtop];
  }
}

function submit_goals() {
  goalElements = document.getElementsByClassName("goal");

  var goalList = new Array();
  for(i = 0; i<goalElements.length; i++) {
    goalList.push(parseInt(goalElements[i].innerHTML));
  }
  gPump.service_call('execute_goals', goalList.toString());
  return true;
}

function reset_goals() {
  var goal_list = $("goal_list");

  for(var i=goal_list.rows.length-1; i>0; i--) {
    goal_list.deleteRow(i);
  }
}

function delete_handler(row) {
  var goal_list = $("goal_list");

  alert("delete_handler");
  for(var i=goal_list.rows.length-1; i>0; i--) {
    if(row == goal_list.rows[i]) {
      goal_list.deleteRow(i);
    }
  } 
}

function click_handler(tagid) {
  var goal_list = $("goal_list");
 
  var lastRow = goal_list.rows.length;
  var iteration = lastRow;
  row = goal_list.insertRow(lastRow);

  var textNode;

  var cell2 = row.insertCell(0);
  cell2.innerHTML = "<div class=goal>" + tagid + "</div>";
}

function creatediv(tagid, html, width, height, x, y, available) {
  var building = $('map');
  var rez = building.getAttribute("rez");
  if(rez) {
    rez = parseFloat(rez);
  } else {
    rez = 1;
  }

  var left = parseInt(x / rez);
  var top = parseInt(y / rez);

  [bleft, btop] = findPos(building);
  bwidth = building.width;
  bheight = building.height;

  var newdiv = document.createElement('div');

  newdiv.setAttribute('id', "tag" + tagid);
  newdiv.id = "tag"+tagid;
   
  if(available == "true") {
    newdiv.className = "outlet_available";
  } else {
    newdiv.className = "outlet_unavailable";
  }

  newdiv.style.width = width;
  newdiv.style.height = height;

  newdiv.style.left = bleft + left - (width/2);
  newdiv.style.top = btop + bheight - (top + (height));

  newdiv.onclick = function() { click_handler(tagid); };

  if (html) {
    newdiv.innerHTML = html;
  } else {
    newdiv.innerHTML = "?";
  }
   
  document.body.appendChild(newdiv);
} 

function add_outlet(tagid, x, y, available) {
  if(available == "false") return;
  creatediv(tagid, "<center>" + tagid, 20, 20, x, y, available);
}


function objectLoad(xmldoc) {
  var outlets = xmldoc.getElementsByTagName('outlet');

  for (var iNode = 0; iNode < outlets.length; iNode++) {
    var node = outlets[iNode];
    var outletid = node.getAttribute("id");
    var availableNode = node.getElementsByTagName('available')[0];
    var available = "false";

    if(availableNode) {
      available = availableNode.childNodes[0].nodeValue;
    } 
    
    var origin = node.getElementsByTagName('origin')[0];
    var xyz = origin.getAttribute("xyz");
    var parts = xyz.split(" ");

    add_outlet(outletid, parseFloat(parts[0]), parseFloat(parts[1]), available); 
  }
}

function handleOnLoad() {
  ros_handleOnLoad('/ros');

  new Ajax.Request("templates/outlets-willow-full-0.025.xml", {
  	method: 'get',
 	onSuccess: function(transport) {  
          objectLoad(transport.responseXML);
	},
        onFailure: function() {alert("something went wrong...")}
  });

  //new PeriodicalExecuter(wander, 0.1);

  setupBuilding();
}

var gpose = {};
gpose.position = {};
gpose.position.x = 10.0;
gpose.position.y = 10.0;
gpose.position.z = 0.5;
gpose.position.ddir = 0;
gpose.position.ddir = 90;
gpose.position.dir = gpose.position.ddir * Math.PI / 180.

var switchTime = 30;

function wander() 
{
  var m = 0.1;

  switchTime = switchTime - 1;
  if(switchTime <= 0) {
    switchTime = parseInt(Math.floor(Math.random() * 50));

    //gpose.position.dir = Math.random() * 2 * Math.PI;
    gpose.position.ddir = parseInt(Math.random() * 360);
    gpose.position.dir = gpose.position.ddir * Math.PI / 180.
  }

  gpose.position.x = gpose.position.x + Math.cos(gpose.position.dir) * m;
  gpose.position.y = gpose.position.y + Math.sin(gpose.position.dir) * m;

  if(gpose.position.x < 0) gpose.position.x = 0;
  if(gpose.position.y < 0) gpose.position.y = 0;
  if(gpose.position.x > gBuilding.bwidth) gpose.position.x = gBuilding.bwidth;
  if(gpose.position.y > gBuilding.bheight) gpose.position.y = gBuilding.bheight;

  moveRobot(gpose);
}

</script>
</head>

<body onload="handleOnLoad();">
<?cs include:"header.cs" ?>

<body>
<br>
<br>
<table>
<tr>
<td>
<img id=map src="templates/68willow-<?cs var:CGI.cur.rez?>.jpg" rez=<?cs var:CGI.cur.rez?>>
</td>
<td valign=top>
  <table id=goal_list border=1 style="width:200px;">
   <tr><td>Goal</td></tr>
  </table>
  <input type=button onclick="submit_goals()" value="Submit">
  <input type=button onclick="reset_goals()" value="Reset">

</td>
</tr>
</table>
</center>
<br>
<br>
<a href="?rez=0.10">0.10</a> | <a href="?rez=0.075">0.075</a> | <a href="?rez=0.05">0.05</a> | <a href="?rez=0.025">0.025</a>
<br>

<div id=heading></div><br>

<div id=robot class=robot><img id=robotimg src="templates/pr2_small.png"></div>
</body>
</html>
