<!DOCTYPE html PUBLIC "-//W3C//DTD HTML 4.01 Transitional//EN">
<html>
  <head>
    <link rel='stylesheet' type='text/css' media='Screen' href='style.css'/>
    <script type="text/javascript" src="http://www.google.com/jsapi"></script>
    <script type="text/javascript" src="3rdparty/curvycorners.js"></script>
    <script type="text/javascript" src="3rdparty/prototype.js"></script>
    <script type="text/javascript" src="3rdparty/wz_jsgraphics.js"></script>
    <script type="text/javascript" src="ros.js"></script>
    <script type="text/javascript" src="help_gadget.js"></script>
    <script type="text/javascript" src="image_gadget.js"></script>
    <script type="text/javascript" src="nav_gadget.js"></script>
    <script type="text/javascript" src="startup_gadget.js"></script>
    <script type="text/javascript" src="joystick_gadget.js"></script>
    <script type="text/javascript" src="demo_gadget.js"></script>
    <script type="text/javascript" src="action_select_gadget.js"></script>
    <script type="text/javascript">

      function showGadgets()
      {
        width = document.body.clientWidth;
        height = document.body.clientHeight;

        l= width * 0.1;
        t= height * 0.1;

        newDiv = document.getElementById('displaybox');
        newDiv.style.display = "";
        newDiv.style.top = t;
        newDiv.style.left = l;
      }

      // This is not the cleanest way to close the help gadgets....
      function hideHelp()
      {
        allHelp = document.getElementsByTagName('div');

        for (i=0; i<allHelp.length; i++)
        {
          if (allHelp[i].id == "helpbox")
            allHelp[i].style.display = "none";
        }
      }

      function hideGadgets()
      {
        document.getElementById('displaybox').style.display="none";
      }

      function addGadget(name)
      {
        if (name == "image_gadget")
          gadget = new ROSImageGadget();
        else if (name == "nav_gadget")
          gadget = new ROSNavGadget();
        else if (name == "joystick_gadget")
          gadget = new ROSJoystickGadget();
        else if (name == "demo_gadget")
          gadget = new ROSDemoGadget();

      }

      function start()
      {
        pump = new MessagePump();
        pump.sendAJAX('/ros/startup', null, init);
      }

      function init()
      {
        gadget = new ROSImageGadget();
        gadget = new ROSActionSelectGadget();
      }

      google.load('visualization', '1', {packages:['gauge']});
      google.setOnLoadCallback(init);
      //window.onload = init;
    </script>
  </head>

  <body>

    <?php
      $gadgets = array(
      "image_gadget"=>array("image_gadget.png", "Image Viewer", "The image viewer gadget displays and image from a topic that publishes messages of type sensor_msgs/CompressedImage or sensor_msgs/Image."),

      "nav_gadget"=>array("nav_gadget.png", "Navigation Viewer", "The navigation viewer gadget displays the static map, the pose of the robot, and it's planned path. It is possible to set the robot's pose, and assign a goal pose.  The map can be panned by clicking the left mouse button and dragging.  The map can be zoomed using the middle scroll wheel."),

      "joystick_gadget"=>array("joystick_gadget.png","Joystick", "The joystick gadget is capable of starting the joystick controller for the robot. Once started the external PS3 jostick can be used to drive the robot."),

      "demo_gadget"=>array("demo_gadget.png", "Demo", "The demo gadget is capable of running a number of pre-defined demo scripts."),
      );
    ?>

    <div id="mainheader">
      <!--<div id="title"><object data='images/maintitle.svg' width="100%" height="100%" type='image/svg+xml'></div>-->
      <div style="width:100%; text-align: right;"><a href='#' onclick='start();'>Start</a></div>
    </div>

    <!-- This is the pop-up window that allows a user to add gadgets -->
    <div id='displaybox' style="display:none;">

      <div style='border:1px solid black;display:block;width:100%;background-image: url("images/gadgetHeaderBkg.png");background-repeat:repeat-x; height: 30px; vertical-align:middle;'>
        <a href='#' style="float:left; float:top; padding-top:2px;font-weight: bold;" onclick="hideGadgets();"><img src='images/xicon.png' height="15px" style="border:none;margin-left:2px;"></a>
        <b style="color: #ffffff;font-size:180%;vertical-align:middle;">Add Gadget</b>
      </div>

      <?php 
      foreach ($gadgets as $name=>$data)
      {
        $img = $data[0];
        $title = $data[1];
        $desc = $data[2];

        print "<div id='gadgetdesc'> <div id='imgbox'>";

        print "<a href='#' style='display:block;' onclick='addGadget(\"$name\");'>";
        print "<img src='images/$img' style='margin-bottom:10px;display:block;width:200px'> <b>Add Now</b></a> </div>";

        print "<p> <h2>$title</h2> $desc </p> </div>";
      }
      ?>

    </div>

    <div id='maincontent'> </div>
  </body>
</html>
