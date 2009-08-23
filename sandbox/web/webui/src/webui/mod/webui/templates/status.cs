<html>
  <head>

    <title>webui</title>
    <?cs include:"includes.cs" ?>

</head>

<body onload="ros_handleOnLoad('/ros')">
<?cs include:"header.cs" ?>
<br>

<br>

<div objtype=PowerboardGraphWidget topic="/diagnostics/Power board 0" width=640></div><br>

<div objtype=PowerboardGraph2Widget topic="/diagnostics/Power board 0" width=640></div><br>

<div objtype=PowerboardGraph3Widget topic="/battery_state" width=640></div><br>

<div id=ErrorDiv></div>

</body>
</html>
