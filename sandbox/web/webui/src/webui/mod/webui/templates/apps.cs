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
  <td class=buttonOff height=100 bgcolor=white objtype="OnOffButtonWidget" topic="/TaskStatus" taskid="<?cs var:_app.appid ?>"><div valign=top style="height: 70%;"><font color=black><?cs var:_app.appname ?></div><div style="position: relative; bottom: 0px; font-size: 10pt;" objtype=TextWidget topic="/TaskStatus" key=status selector="taskid" selectorValue="<?cs var:_app.appid?>">&nbsp;</div></td>
<?cs /each ?>
</tr>
</table>


<table width=80% align=center>
<tr>
<td width=100%>
<table style="width: 100%;" class=rosout><tr>
<td class=rosoutHeading style="width: 50%;">Message</td>
<td class=rosoutHeading style="width: 10%;">Severity</td>
<td class=rosoutHeading style="width: 10%;">Node</td> 
<td class=rosoutHeading style="width: 20%;">File</td>
<td class=rosoutHeading style="width: 10%;">Topics</td>
</table>
<div style="border: 2px solid white; font-size: 10pt; height: 30em; width: 80%;" objtype=RosOut_Widget></div><br>
</td>
</tr>
</table>

<br>
<div id=ErrorDiv></div>


</body>
</html>
