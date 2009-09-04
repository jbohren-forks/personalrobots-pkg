<table class=toolbar width=100% cellspacing=0 cellpadding=2>
<tr>
<td width=1%><nobr>Robot: <?cs var:CGI.ServerName?></td>
<td width=1%><img src="<?cs var:CGI.ScriptName?>/webui/templates/images/toolbar/grid-blue-split.gif"></td>
<td width=90%><nobr>Core Status: <font color=green>Running</font> 
<span objtype=ActiveTasks topic="/app_status"/>
</nobr>
</td>
<td width=1%><img src="<?cs var:CGI.ScriptName?>/webui/templates/images/toolbar/grid-blue-split.gif"></td>
<td width=1%><nobr><div objtype=CircuitMonitor topic="/power_board_state"/></td>
<td width=1%><img src="<?cs var:CGI.ScriptName?>/webui/templates/images/toolbar/grid-blue-split.gif"></td>
<td width=1%><nobr><div objtype=ChargeMonitor topic="/power_board_state"/></td>
<td width=1%><nobr><div objtype=BatteryMonitor topic="/battery_state"/></td>
<td width=1%><img src="<?cs var:CGI.ScriptName?>/webui/templates/images/toolbar/grid-blue-split.gif"></td>
<?cs if:CGI.Login ?>
<td align=right><a href="<?cs var:CGI.ScriptName ?>/login/signin.py?signout=1" class=tablink>Logout</a>(<?cs var:CGI.Login ?>)</td>
<?cs else ?>
<td align=right><a href="<?cs var:CGI.ScriptName ?>/login/signin0.py?q=1" class=tablink>Login</a></td>
<?cs /if ?>
</tr>
</table>

<table class=head_buttons width=80%>
<tr>
<td class=head_buttons width=1% onclick="javascript:location.href='<?cs var:CGI.ScriptName?>/webui/apps.py'">Apps</a></td>
<td class=head_buttons width=1% onclick="javascript:location.href='<?cs var:CGI.ScriptName?>/webui/status.py'">Status</a></td>
<td class=head_buttons width=1% onclick="javascript:location.href='<?cs var:CGI.ScriptName?>/webui/admin.py'">Admin</a></td>

<?cs if:CGI.Login=="hassan"?><td width=1%><a href="<?cs var:CGI.ScriptName ?>/webui/tables.py/" class=tablink>Tables</a></td><?cs /if ?>

</tr>
</table>

</td></tr>
<tr><td>
</td></tr>
</table>

