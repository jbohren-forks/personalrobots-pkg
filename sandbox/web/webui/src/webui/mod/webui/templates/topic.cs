<html>
  <head>
    <?cs include:"includes.cs" ?>

</head>

<body onload="ros_handleOnLoad('/ros')">
<?cs include:"header.cs" ?>
<br>
<?cs include:"status_header.cs" ?>

<h3>Topic: <?cs var:CGI.cur.topic?></h3>

<div style="font-size: 10pt; " objtype=MessageWidget topic="<?cs var:CGI.cur.topic ?>"></div><br>

<div style="border: 2px solid white; font-size: 10pt; font-family: courier; height: 40em; width: 60em;" objtype=ScrollingTextWidget topic="<?cs var:CGI.cur.topic ?>"></div><br>

<br>

<?cs include:"rosfooter.cs"?>
</body>
</html>
