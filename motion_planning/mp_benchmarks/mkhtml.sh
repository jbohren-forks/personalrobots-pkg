#!/bin/bash

MPBENCH=`rospack find mp_benchmarks`/mpbench

#CONSTANT_OPTS="-s office1 -r 0.05 -i 0.5 -d 1.3 -H 3.0"
CONSTANT_OPTS="-s office1 -r 0.025 -i 0.5 -d 1.3 -H 3.0"

A_OPT="-p"
A_VAR="ARAPlanner ADPlanner"

B_OPT="-m"
B_VAR="costmap_2d sfl"
B_NOPTS="2"

#REST="-c:0.6:-I:1 -c:0.65:-I:1"
REST="-c:0.6:-I:1 -c:0.65:-I:1 -c:0.7:-I:1 -c:0.9:-I:1 -c:0.6:-I:2 -c:0.65:-I:2 -c:0.7:-I:2 -c:0.9:-I:2 -c:1.4:-I:2 -c:1.9:-I:2"

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
