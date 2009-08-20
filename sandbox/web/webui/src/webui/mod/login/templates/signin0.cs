<html>
<head>
<title>Mail Sign-In</title>

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
<form action="<?cs var:CGI.BaseURI?>/login/signin0.py" method=post>
<input type=hidden name=request value="<?cs var:CGI.cur.request?>">
<tr><td align=center valign=middle>

<?cs if:Query.reason == "timeout"?>
  <b>Your login has timed-out due to inactivity, please sign in
  again</b><p>
<?cs /if ?>



<table width=200 cellspacing=0 cellpadding=2 style="border:1px solid #777799;" >
<tr><td colspan=2 style="color:white;background:#777799;" align=center>
Sign-In
</td></tr>
<tr align=center><td colspan=2><?cs if:Query.err?><font color=red><?cs var:html_escape(Query.err)?><?cs /if ?></td></tr>
<tr><td align=right>Username:</td>
<td><input name=username type=text size=20 value="<?cs var:url_escape(CGI.username) ?>"></td></tr>
<tr><td align=right>Password:</td>
<td><input name=password type=password size=20></td></tr>
<tr><td><Td colspan=1>
<!--<font size=-2><input type=checkbox value="1" name="persist" <?cs if:Cookie.MB_persist==1?>CHECKED<?cs /if ?>> Don't ask for my password for 2 weeks.</font>-->
</td></tr>
<tr><Td colspan=2 align=center>
<br>
<input type=submit value="Sign In" name="Action.Login">
</td></tr>


<tr><td align=center colspan=2>
<?cs if:0 ?>
  <a href="<?cs var:CGI.BaseURI?>/login/forgotpw.py">Forgot Password</a>
<?cs /if ?>
</td></tr>

</form>
</table>

</td></tr></table>

</body>
</html>
