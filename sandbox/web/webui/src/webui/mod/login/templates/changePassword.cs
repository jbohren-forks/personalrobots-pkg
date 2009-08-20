<html>
<head>
<title>Mail - Change Password</title>
</head>
<body>



<table width=100% height=70%>


<form action="changePassword.py" method=post>
<input type=hidden name="request" value="<?cs var:CGI.cur.request?>">
<input type=hidden name="login" value="<?cs var:CGI.Login?>">
<tr><td align=center valign=middle>

<?cs if:Query.err ?>
Error: <font color=red><?cs var:Query.err ?></font><br>
<?cs /if ?>

<table width=150 cellspacing=0 cellpadding=2 style="border:1px solid #777799;" >
<tr><td colspan=2 style="color:white;background:#777799;" align=center>
Change Password
</td></tr>
<tr><td nowrap align=right>Old Password:</td>
<td><input name=pw0 type=password size=20></tr>

<tr><td nowrap align=right>New Password:</td>
<td><input name=pw1 type=password size=20></tr>

<tr><td nowrap align=right>Confirm Password:</td>
<td><input name=pw2 type=password size=20></tr>

<tr><Td colspan=2 align=center>
<input type=hidden name="Action.changePassword" value="1">
<input type=submit value="Change Password" name="Action.changePassword">
</td></tr>
</form>
</table>
</table>

</body>
</html>
