# Gnuplot script file for plotting data in file "/tmp/run"
# This file is called   plotutil
  
reset
set style function lines
set size 1.0, 1.0
set origin 0.0, 0.0
set multiplot
set size 0.5,0.5
set origin 0.0,0.5
set grid
unset key
set   autoscale                        # scale axes automatically
      unset log                              # remove any log-scaling
      unset label                            # remove any previous labels
      set xtic auto                          # set xtics automatically
      set ytic auto                          # set ytics automatically
      set title "Current Control Graph"
      set xlabel "Count"
      set ylabel "Current (?)"
      set key 8,1

      plot    "output" using 1:3 title 'Current' with line , \
              "output" using 1:5 title 'Desired' with line
#  Plot PWM Response
set size 0.5,0.5
set origin 0.0,0.0
set   autoscale                        # scale axes automatically
      unset log                              # remove any log-scaling
      unset label                            # remove any previous labels
      set xtic auto                          # set xtics automatically
      set ytic auto                          # set ytics automatically
      set title "PWM & Encoder Graph"
      set xlabel "Count"
      set ylabel "Value"
      set key 8,1
      
      plot    "output" using 1:2 title 'Encoder' with line , \
              "output" using 1:4 title 'PWM' with line
unset multiplot
      
    
