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
  <td class=buttonOff height=100 bgcolor=white objtype="OnOffButtonWidget" topic="/app_update" taskid="<?cs var:_app.taskid ?>"><div valign=top style="height: 70%;"><font color=black><nobr><?cs var:_app.name ?></div><div style="position: relative; bottom: 0px; font-size: 10pt;" objtype=TextWidget topic="/app_update" key=status selector="taskid" selectorValue="<?cs var:_app.taskid?>">&nbsp;</div></td>
<?cs /each ?>
</tr>
</table>


<table width=80% align=center cellpadding=0 cellspacing=0>
<tr>
<td width=100%>
<table class=rosout><tr>
<td class=rosoutHeading style="width: 1%;">Severity</td>
<td class=rosoutHeading style="width: 90%;">Message</td>
</table>
<div style="border: 2px solid white; height: 30em; width: 80%;" objtype=RosOut_Widget></div><br>
</td>
</tr>
</table>

<br>
<div id=ErrorDiv></div>


</body>
</html>
