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

<table width=50% height=70% align=center>
<tr><td valign=middle>

<center>
<b> Your account is created ! </b><p>
</center>

Forward mail to 
<tt><?cs var:CGI.mailAddrPattern?></tt> 

to deliver it to this account. 

<p>
Sorry, there is no POP3 or IMAP support yet.

<p>

<center>

<a href="<?cs var:CGI.BaseURI?><?cs var:CGI.Login?>/mail/index.py">See your new Inbox</a><p>

</center>


</td></tr></table>

</body>
</html>
