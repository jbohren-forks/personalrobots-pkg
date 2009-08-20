<html>
<head>
<title>New Account Registration</title>

<script language="JavaScript">
<!--
function handleOnload() {
  var top_parent = window;
  while (top_parent.parent && (top_parent.parent != top_parent)) {
    top_parent = top_parent.parent;
  }
  if (top_parent != window) {
    top_parent.location.href="<?cs var:js_escape(CGI.RequestURI) ?>";
  }
}
//-->
</script>

</head>
<body onload="handleOnload()">

<table width=100% height=70%>
<form action="<?cs var:CGI.BaseURI?>_/login/register.py" method=post>
<tr><td align=center valign=middle>


<?cs if:Query.err ?>
<center>
Error: <font color=red><?cs var:Query.err ?></font>
</center>
<?cs /if ?>



<table width=150 cellspacing=0 cellpadding=2 style="border:1px solid #777799;" >
<tr><td colspan=2 style="color:white;background:#777799;" align=center>
Register
</td></tr>
<tr><td align=right>Choose login name:</td>
<td><input name=login type=text size=20 value="<?cs var:CGI.login ?>"></td></tr>

<tr><td nowrap align=right>Your current email address:</td>
<td><input name=out_address type=text size=50></td></tr>


<tr><td nowrap align=right>Password:</td>
<td><input name=pw1 type=password size=20></td></tr>

<tr><td nowrap align=right>Confirm Password:</td>
<td><input name=pw2 type=password size=20></td></tr>

<tr><Td colspan=2 align=center>

<input type=submit value="Create" name="Action.Create">
</td></tr>

</form>
</table>

</td></tr></table>

</body>
</html>
