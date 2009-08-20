<html>
  <head>

    <title>webui</title>
    <?cs include:"includes.cs" ?>

</script>

</head>

<body onload="ros_handleOnLoad('/ros')">
<?cs include:"header.cs" ?>
<br>

<br>

<table class=powerboard align=center>
  <tr align=center>
    <td>Circuit Breaker 0</td>
    <td>Circuit Breaker 1</td>
    <td>Circuit Breaker 2</td>
  </tr>
  <tr align=center>
    <td>(Left Arm)</td>
    <td>(Base Spine)</td>
    <td>(Right Arm)</td>
  </tr>
 <tr>
   <td class=powerboard>
     <div objtype=PowerboardBreakerWidget topic="/diagnostics/Power board 0" state="Breaker 0 State" voltage="Breaker 0 Voltage">No Status</div><br>
     <input class=pbbutton type=button value=On onclick="javascript:gPump.service_call('power_board_control', [0, 'start', 0]);"><br>
     <input class=pbbutton type=button value=Standby onclick="javascript:gPump.service_call('power_board_control', [0, 'stop', 0]);"><br>
     <input class=pbbutton type=button value="Reset Disable" onclick="javascript:gPump.service_call('power_board_control', [0, 'reset', 0]);"><br>
     <input class=pbbutton type=button value=Disable onclick="javascript:gPump.service_call('power_board_control', [0, 'disable', 0]);"><br>
   </td>

   <td class=powerboard>
     <div objtype=PowerboardBreakerWidget topic="/diagnostics/Power board 0" state="Breaker 1 State" voltage="Breaker 1 Voltage">No Status</div><br>
     <input class=pbbutton type=button value=On onclick="javascript:gPump.service_call('power_board_control', [1, 'start', 0]);"><br>
     <input class=pbbutton type=button value=Standby onclick="javascript:gPump.service_call('power_board_control', [1, 'stop', 0]);"><br>
     <input class=pbbutton type=button value="Reset Disable" onclick="javascript:gPump.service_call('power_board_control', [1, 'reset', 0]);"><br>
     <input class=pbbutton type=button value=Disable onclick="javascript:gPump.service_call('power_board_control', [1, 'disable', 0]);"><br>
   </td>

   <td class=powerboard>
     <div objtype=PowerboardBreakerWidget topic="/diagnostics/Power board 0" state="Breaker 2 State" voltage="Breaker 2 Voltage">No Status</div><br>
     <input class=pbbutton type=button value=On onclick="javascript:gPump.service_call('power_board_control', [2, 'start', 0]);"><br>
     <input class=pbbutton type=button value=Standby onclick="javascript:gPump.service_call('power_board_control', [2, 'stop', 0]);"><br>
     <input class=pbbutton type=button value="Reset Disable" onclick="javascript:gPump.service_call('power_board_control', [2, 'reset', 0]);"><br>
     <input class=pbbutton type=button value=Disable onclick="javascript:gPump.service_call('power_board_control', [2, 'disable', 0]);"><br>
   </td>
 </tr>
 <tr class=powerboard_runstop_stall objtype=PowerboardRunStopWidget topic="/diagnostics/Power board 0">
  <td>No Status</td><td>No Status</td><td>No Status</td>
 </tr>
</table>



    <div id='chart_div'></div>

<div id=ErrorDiv></div>

</body>
</html>
