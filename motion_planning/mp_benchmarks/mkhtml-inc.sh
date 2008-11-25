#!/bin/bash

MPBENCH=`rospack find mp_benchmarks`/mpbench-incremental

CONSTANT_OPTS="-s cubicle -i 0.325 -c 0.46 -I 0.55 -d 1.2 -H 3 -e 2d"
CONSTANT_HR="setup:cubicle inscribedRadius:0.325 circumscribedRadius:0.46 inflationRadius:0.55 doorWidth:1.2 hallWidth:3 environmentType:2D"

A_OPT="-p"
A_VAR="ARAPlanner ADPlanner"
A_HR="plannerType"

B_OPT="-m"
B_VAR="costmap_2d sfl"
B_HR="costmapType"
B_NOPTS="2"

REST="-r:0.1"
#REST="-r 0.1 -r 0.05 -r 0.025"
REST_HR="-r:resolution"

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
	    echo "<td>" >> index.html
	    echo "<code>$A_OPT $aa $B_OPT $bb $opts</code><br>" >> index.html

	    echo "extracting basename for $allopts"
	    basename=`$MPBENCH $allopts -X`
	    echo "<a href=\" $basename.txt \"> log </a>&nbsp;<a href=\" cons-$basename.txt \"> cons </a>&nbsp;<a href=\" vg-$basename.txt \"> vg </a>&nbsp;<a href=\" $basename.png \"> png </a><br>" >> index.html
	    echo "<a href=\" $basename.png \"><img src=\" small-$basename.png \" alt=\" $basename.png \"></a><br>" >> index.html
	    echo "running with $allopts -W"
	    valgrind --log-file-exactly=vg-$basename.txt $MPBENCH $allopts -W 2>&1 | tee cons-$basename.txt
	    
	    cat $basename.html >> index.html
	    
	    echo "</td>" >> index.html
	done
    done
    echo "</tr>" >> index.html
done

echo "</table></body></html>" >> index.html
