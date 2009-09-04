<html>
  <head>
    <?cs include:"includes.cs" ?>
</head>

<body onload="ros_handleOnLoad('/ros')">
<?cs include:"header.cs" ?>
<br>
<?cs include:"status_header.cs" ?>

<h3>Nodes:</h3>
<ul objtype="ListWidget" topic="/topics" key="nodes">
  <li>__item__
</ul>

<br>

<?cs include:"rosfooter.cs"?>
</body>
</html>
