#! /usr/bin/gnuplot
set terminal gif
set output "data/mode1out.gif"
set title "/forearm_cam/image_raw delay"
set xlabel "time (s)"  
set ylabel "intensity (A.U.)
set y2tics 
set ytics nomirror
set y2range [-0.1:1.1]
plot "data/mode1out.txt" using 2:4 title "image_raw intensity", x < 1 axis x1y2 title "LED state"
#pause mouse any
