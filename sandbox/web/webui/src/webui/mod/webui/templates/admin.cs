<html>
  <head>
    <?cs include:"includes.cs" ?>
</head>

<body onload="ros_handleOnLoad('/ros')">
<?cs include:"header.cs" ?>
<br>

<a href="?Action_halt=1">Shutdown</a><br>
<a href="?Action_reboot=1">Reboot</a><br>
<a href="?Action_rosinit_demo=1">Demo</a><br>
<a href="?Action_rosinit_dev=1">Development</a><br>

<br>
<div id=ErrorDiv></div>


</body>
</html>
