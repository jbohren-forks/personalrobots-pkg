<html>
<head>
<?cs include:"includes.cs" ?>
</head>

<body onload="ros_handleOnLoad('/ros')">
<?cs include:"header.cs" ?>
<h3>Sample Application</h3>

<table align=right>
<td>
<div class=app_button objtype="LaunchButtonWidget2" taskid="sample_application/sample_app.app">
</td>
</table>

<table width=80% align=center>
<tr>
<td>
<div style="align: center; border: 1px solid black; font-size: 10pt; font-family: courier; height: 40em; width: 60em;" objtype=TerminalTextWidget topic="/chatter:more" key="data"></div><br>
</td>
</table>

<div id=ErrorDiv></div>

<?cs include:"rosfooter.cs"?>
</body>
</html>
