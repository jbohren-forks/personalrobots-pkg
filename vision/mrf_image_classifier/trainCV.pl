#!/usr/bin/perl

use warnings; use strict;

my $datadir = $ARGV[0] || die "You must provide a data directory";
my $outname = $ARGV[1] || die "You must provide an output file name";
my $epochs = 10;

for my $dir (glob("$datadir/*")) {
    if ( -d $dir ) {
	print "training fold $dir\n";
	my $cmd = "oOutputFile=$dir/$outname oEpochs=$epochs ./naryTrainBatch $dir/train";
	print "command: $cmd\n";
	system("$cmd");
    }
}
