#!/bin/bash

MPBENCH=`rospack find mpbench`/mpbench-base

CONSTANT_OPTS="-i 0.325 -c 0.46 -I 0.55 -d 1.2 -H 3 -m costmap_2d"

A_OPT="-s"
A_VAR="office1 cubicle"

B_OPT="-p"
B_VAR="ARAPlanner ADPlanner NavFn"
B_NOPTS="3"

REST="-r:0.1 -r:0.05 -r:0.025"

rm -f index.html

echo "<html><body>" >> index.html
echo "<p><em>Constant options:</em> $CONSTANT_OPTS </p>" >> index.html
echo "<table border=\"1\" cellpadding=\"2\">" >> index.html

echo "<tr>" >> index.html
for aa in $A_VAR; do
    echo "<th colspan=\" $B_NOPTS \"> $aa </th>" >> index.html
done
echo "</tr>" >> index.html

echo "<tr>" >> index.html
for aa in $A_VAR; do
    for bb in $B_VAR; do
	echo "<th> $bb </th>" >> index.html
    done
done
echo "</tr>" >> index.html

for rest in $REST; do
    opts=`echo $rest | sed 's/:/ /g'`
    echo "<tr>" >> index.html    
    for aa in $A_VAR; do
	for bb in $B_VAR; do
	    allopts="$A_OPT $aa $B_OPT $bb $opts $CONSTANT_OPTS"
	    echo "<td>" >> index.html
	    echo "$A_OPT $aa <br> $B_OPT $bb <br> $opts <br>" >> index.html

	    echo "mkhtml.sh: extracting basename for $allopts"
	    basename=`$MPBENCH $allopts -X`
	    echo "<a href=\" $basename.txt \"> log </a>&nbsp;<a href=\" cons-$basename.txt \"> cons </a>&nbsp;<a href=\" $basename.png \"> png </a><br>" >> index.html
	    echo "<a href=\" $basename.png \"><img src=\" small-$basename.png \" alt=\" $basename.png \"></a>" >> index.html
	    echo "mkhtml.sh: running with $allopts -W"
	    $MPBENCH $allopts -W 2>&1 | tee cons-$basename.txt
	    
	    cat $basename.html >> index.html
	    
	    echo "</td>" >> index.html
	done
    done
    echo "</tr>" >> index.html
done

echo "</table></body></html>" >> index.html
