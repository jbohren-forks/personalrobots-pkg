<html>
<head>
<?cs include:"includes.cs" ?>
</head>

<body onload="ros_handleOnLoad('/ros')">
<?cs include:"header.cs" ?>
<br>
<?cs include:"status_header.cs" ?>

<table style="border: 1px solid black; width: 150px; float: right">
<tr>
<td objtype="ListWidget" topic="/users" key="users">
  __item__<br>
</td>
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



<?cs include:"rosfooter.cs"?>
</body>
</html>
