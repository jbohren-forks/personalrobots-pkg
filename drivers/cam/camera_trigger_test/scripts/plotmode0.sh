#!/bin/sh
for s in data/mode0out-*.txt
do
mode=`echo $s|sed 's/data\/mode0out-\(.*\).txt/\1/'`
rate=`echo $mode|sed 's/.*x.*x\(.*\)/\1/'`
echo $mode $rate
  gnuplot << EOF
set terminal gif
set output "data/mode0out-$mode.gif"
set title "Exposure delay for mode $mode"
set xlabel "LED pulse delay from trigger (s)"  
set ylabel "intensity (A.U.)
plot "data/mode0out-$mode.txt" using 2:4 title "intensity", x < 1./$rate axis x1y2 title "1/$rate seconds"
EOF
done

