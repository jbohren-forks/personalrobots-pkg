<html>
<head>
<?cs include:"includes.cs" ?>
</head>

<body onload="ros_handleOnLoad('/ros')">
<?cs include:"header.cs" ?>
<h3>Sample Application</h3>

<table align=right>
<td>
<input class=app_button type=button value="" objtype="LaunchButtonWidget2" topic="/app_update" taskid="<?cs var:CGI.cur.app.taskid ?>">
<div class=app_status objtype=TextWidget topic="/app_update" key=status selector="taskid" selectorValue="<?cs var:CGI.cur.app.taskid?>">&nbsp;</div>
</td>
</table>

<table width=80% align=center>
<tr>
<td>
<div style="align: center; border: 1px solid black; font-size: 10pt; font-family: courier; height: 40em; width: 60em;" objtype=ScrollingTextWidget topic="/chatter"></div><br>
</td>
</table>

<div id=ErrorDiv></div>

</body>
</html>
