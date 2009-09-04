<html>
  <head>

    <?cs include:"includes.cs" ?>
</head>

<body onload="ros_handleOnLoad('/ros')">
<?cs include:"header.cs" ?>
<br>

<?cs if:0 ?>
<table align=center cellspacing=5>
<tr>
<?cs each:_app=CGI.cur.apps ?>
  <td class=buttonOff objtype="LaunchButtonWidget" topic="/app_update" taskid="<?cs var:_app.taskid ?>"><div class=buttonTitle valign=top><?cs var:_app.name ?></div><div class=buttonStatus objtype=TextWidget topic="/app_update" key=status selector="taskid" selectorValue="<?cs var:_app.taskid?>">&nbsp;</div></td>
<?cs /each ?>
</tr>
</table>
<?cs /if ?>

<table width=30% cellspacing=5>
<?cs each:_app=CGI.cur.apps ?>
<tr><td>
<table class=app border=0>
<tr>
<td width=1% onclick="document.location='<?cs var:CGI.ScriptName?>/app/<?cs var:_app.package ?>/';"><img src="<?cs var:CGI.ScriptName?>/app/<?cs var:_app.package ?>/<?cs var:_app.icon?>" width=100 height=100 border=0></td>
<td class=app_bar>
<div class=app_name><?cs var:_app.name ?></div>
<div class=app_description><?cs var:_app.description ?></div>
</td>
<td valign=top>
<div class=app_button objtype="LaunchButtonWidget2" taskid="<?cs var:_app.taskid ?>">
</td>
</tr>
</table>
</td></tr>
<?cs /each ?>
</table>


<br>
<div id=ErrorDiv></div>


<?cs include:"rosfooter.cs"?>
</body>
</html>
