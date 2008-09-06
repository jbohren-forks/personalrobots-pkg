#!/usr/bin/perl -I ../../external/perl5
#
# STAIR VISION LIBRARY
# Copyright (c) 2007-2008, Stephen Gould
#
# FILENAME:    video2imageLabeler.pl
# AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
# DESCRIPTION:
#   Converts from pre-release Object2dVideo XML files to the new
#   Object2dSequence XML files.
#

use File::Copy;
use strict;

if (($#ARGV != 0) && ($#ARGV != 1)) {
    print STDERR "USAGE: ./video2imageLabeler.pl <Object2dVideo> [<ImageSequence>]\n";
    exit(-1);
}

my $labelsFilename = $ARGV[0];
my $imagesFilename;
$imagesFilename = $ARGV[1] if ($#ARGV == 1);

# backup files
my $timestamp = time();
copy($labelsFilename, "${labelsFilename}.${timestamp}.bak");

# load in labels
open FILE, $labelsFilename;
my @labelsData = <FILE>;
close FILE;

# load id's if available
my @idData = ();
if (length($imagesFilename)) {    
    open FILE, $imagesFilename;
    while (<FILE>) {
        if (m/<Image\s+name=\"(.*).jpg\"/) {
            push @idData, $1;
        }
    }
    close FILE;
}

# convert to new format
my $n = 0;
for (my $i = 0; $i <= $#labelsData; $i++) {
    $labelsData[$i] =~ s/Object2dVideo/Object2dSequence/;
    $labelsData[$i] =~ s/Object2dFrame\s+index/Object2dFrame id/;
    if (($labelsData[$i] =~ m/Object2dFrame id/) && ($n <= $#idData)) {
        $labelsData[$i] =~ s/id=\"(.*)\"/id=\"$idData[$n]\"/;
        $n++;
    }
}

# write out
open FILE, ">$labelsFilename";
print FILE join("", @labelsData);
close FILE;
