#! /usr/bin/perl

use strict;

open(IN, "rostopic echo pressure/r_gripper_motor |");
open(GNUPLOT, "|gnuplot -persist");
#open(GNUPLOT, ">-");

while (<IN>)
{
  my @line = split/ |\(|,|\)/;
  
  if ($line[0] eq "data1:")
  {
    print GNUPLOT "set terminal x11\n";
    print GNUPLOT "set yrange [0:10000]\n";
    print GNUPLOT "plot \"-\" with linespoints\n";
    foreach my $value (@line)
    {
      if ($value =~ /^(\d+\.?\d*|\.\d+)$/)
      {
        print GNUPLOT "$value\n";
      }
    }
    print GNUPLOT "e\n";
  }
}
