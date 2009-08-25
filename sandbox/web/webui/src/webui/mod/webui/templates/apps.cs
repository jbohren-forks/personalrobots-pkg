<html>
  <head>

    <?cs include:"includes.cs" ?>
</head>

<body onload="ros_handleOnLoad('/ros')">
<?cs include:"header.cs" ?>
<br>


<table align=center cellspacing=5>
<tr>
<?cs each:_app=CGI.cur.apps ?>
  <td class=buttonOff objtype="LaunchButtonWidget" topic="/app_update" taskid="<?cs var:_app.taskid ?>"><div class=buttonTitle valign=top><?cs var:_app.name ?></div><div class=buttonStatus objtype=TextWidget topic="/app_update" key=status selector="taskid" selectorValue="<?cs var:_app.taskid?>">&nbsp;</div></td>
<?cs /each ?>
</tr>
</table>


<table width=80% align=center cellpadding=0 cellspacing=0>
<tr>
<td width=100%>
<table class=rosout><tr>
<td class=rosoutHeading style="width: 5%;">Severity</td>
<td class=rosoutHeading style="width: 80%;">Message</td>
<td class=rosoutHeading style="width: 15%;">Node</td>
</table>
<div class=rosoutWidget objtype=RosOut_Widget></div><br>
</td>
</tr>
</table>

<br>
<div id=ErrorDiv></div>


</body>
</html>
