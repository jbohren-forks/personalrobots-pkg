<html>
<head>
<?cs include:"includes.cs" ?>
<script type="text/javascript" src="jslib/map_viewer.js"></script>
</head>

<body onload="ros_handleOnLoad('/ros')">
<?cs include:"header.cs" ?>
<h3>Navigation</h3>

<table align=right>
<td>
<input class=app_button type=button value="" objtype="LaunchButtonWidget2" topic="/app_update" taskid="<?cs var:CGI.cur.app.taskid ?>">
<div class=app_status objtype=TextWidget topic="/app_update" key=status selector="taskid" selectorValue="<?cs var:CGI.cur.app.taskid?>">&nbsp;</div>
</td>
</table>

<table width=80% align=center>
<tr>
<td>
<div style="position:absolute;overflow:hidden;width:80%;height:600;border:2px solid" objtype=MapViewer></div><br>
</td>
</table>

<div id=ErrorDiv></div>

</body>
</html>
