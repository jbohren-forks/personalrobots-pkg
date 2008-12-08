#!/bin/bash

MPBENCH=`rospack find mp_benchmarks`/mpbench-incremental

CONSTANT_OPTS="-e 3d -i 0 -d 0.75 -H 3 -a 100"
CONSTANT_HR="setup:cubicle environmentType:3d inscribedRadius:0 doorWidth:0.75 hallWidth:3 allocatedTime:100ms"

A_OPT="-s"
A_VAR="cubicle office1"
A_HR="setup"

B_OPT="-p"
B_VAR="ARAPlanner ADPlanner"
B_HR="plannerType"
B_NOPTS="2"

REST="-r:0.1:-c:0:-I:0:-m:costmap_2d -r:0.1:-c:0.135:-I:0.225:-m:costmap_2d -r:0.1:-c:0.135:-I:0.225:-m:sfl    -r:0.05:-c:0:-I:0:-m:costmap_2d -r:0.05:-c:0.135:-I:0.225:-m:costmap_2d -r:0.05:-c:0.135:-I:0.225:-m:sfl    -r:0.025:-c:0:-I:0:-m:costmap_2d -r:0.025:-c:0.135:-I:0.225:-m:costmap_2d -r:0.025:-c:0.135:-I:0.225:-m:sfl"
REST_HR="-r:resolution -c:circumscribedRadius -I:inflationRadius -m:costmapType"

rm -f index.html
cat>>index.html<<EOF
<html>
<body>
<h1>Setup</h1>
<ul>
EOF

for opt in $CONSTANT_HR; do
    hr=`echo $opt | sed 's/:/ /g'`
    cat>>index.html<<EOF
<li>$hr</li>
EOF
done

cat>>index.html<<EOF
</ul>
<h1>Categories</h1>
<ul>
 <li>$A_HR <code>$A_OPT</code>: $A_VAR</li>
 <li>$B_HR <code>$B_OPT</code>: $B_VAR</li>
</ul>
<h1>Run Variables</h1>
<ul>
EOF

for opt in $REST_HR; do
    hr=`echo $opt | sed 's/:/ /g'`
    cat>>index.html<<EOF
<li>$hr</li>
EOF
done

cat>>index.html<<EOF
</ul>
<table border="1" cellpadding="2">
<tr>
EOF

for aa in $A_VAR; do
    cat>>index.html<<EOF
<th colspan="$B_NOPTS">$A_HR <code>$A_OPT</code> $aa</th>
EOF
done

cat>>index.html<<EOF
</tr>
<tr>
EOF

for aa in $A_VAR; do
    for bb in $B_VAR; do
	cat>>index.html<<EOF
<th>$B_HR <code>$B_OPT</code> $bb</th>
EOF
    done
done

cat>>index.html<<EOF
</tr>
EOF


for rest in $REST; do
    opts=`echo $rest | sed 's/:/ /g'`
    echo "<tr>" >> index.html    
    for aa in $A_VAR; do
	for bb in $B_VAR; do
	    allopts="$A_OPT $aa $B_OPT $bb $opts $CONSTANT_OPTS"
	    basename=`$MPBENCH $allopts -X`

	    echo "running with $allopts -W"
	    $MPBENCH $allopts -W 2>&1 | tee cons-$basename.txt
	    
	    echo "invoking gnuplot $basename.plot"
	    gnuplot $basename.plot 2>&1 | tee gnuplot-$basename.txt

cat>>index.html<<EOF
<td>
 <table border="0" cellpadding="1">
  <tr>
   <td colspan="3"><code>$A_OPT $aa $B_OPT $bb $opts</code></td>
  </tr>
  <tr>
   <td><a href="$basename.txt">bench log</a></td>
   <td><a href="cons-$basename.txt">bench cons</a></td>
   <td><a href="gnuplot-$basename.txt">plot cons</a></td>
  </tr>
  <tr>
   <td colspan="3"><a href="$basename.png"><img src="small-$basename.png" alt="$basename.png"></a></td>
  </tr>
  <tr>
   <td><a href="$basename.png">plan png</a></td>
   <td><a href="table-$basename.html">table</a></td>
   <td><a href="plots-$basename.html">plots</a></td>
  </tr>
 </table>
</td>
EOF

cat>>table-$basename.html<<EOF   
<html>
<body>
<h1>summary table</h1>
<p>command line options: <code>$allopts</code></p>
EOF
cat $basename.html >> table-$basename.html
cat>>table-$basename.html<<EOF   
</body>
</html>
EOF

cat>>plots-$basename.html<<EOF
<html>
<body>
<h1>Summary plots</h1>
 <p>command line options: <code>$allopts</code></p>
 <h2>cost histogram</h2>
  <img src="$basename--costhist.png" alt="no image? no solution...">
 <hr>
 <h2>relative path quality vs relative time</h2>
  <img src="$basename--rqual-rtime.png" alt="no image? no solution...">
 <hr>
 <h2>expansion speed vs relative time</h2>
  <img src="$basename--speed-rtime.png" alt="no image? no solution...">
 <hr>
 <h2>relative path quality vs absolute time</h2>
  <img src="$basename--rqual-atime.png" alt="no image? no solution...">
 <hr>
 <h2>expansion speed vs absolute time</h2>
  <img src="$basename--speed-atime.png" alt="no image? no solution...">
 <hr>
 <h2>solution cost vs absolute time</h2>
  <img src="$basename--cost-atime.png" alt="no image? no solution...">
 <hr>
 <h2>plan length vs absolute time</h2>
  <img src="$basename--length-atime.png" alt="no image? no solution...">
 <hr>
 <h2>plan tangent rotation vs absolute time</h2>
  <img src="$basename--rotation-atime.png" alt="no image? no solution...">
</body>
</html>
EOF

	done
    done
    echo "</tr>" >> index.html
done

echo "</table></body></html>" >> index.html
