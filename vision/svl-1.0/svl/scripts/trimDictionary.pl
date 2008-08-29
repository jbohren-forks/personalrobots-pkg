#!/usr/bin/perl -I ../../external/perl5
#
# STAIR VISION LIBRARY
# Copyright (c) 2007-2008, Stanford University
#
# FILENAME:    trimDictionary.pl
# AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
# DESCRIPTION:
#   Reduces the size of a patch dictionary by examining which patches
#   get used in a particular model and removing the unused patches from
#   the dictionary. All indices are renumbered to take account of the new
#   dictionary.
#

use File::Copy;
use Getopt::Std;
use strict;

my %opts = ();
getopts("r", \%opts);

if ($#ARGV != 1) {
    print STDERR "USAGE: ./trimDictionary.pl [options] <dictionaryName> <modelName>\n";
    print STDERR "OPTIONS:\n";
    print STDERR "  -r        :: remove redundant entries from model file\n";
    print STDERR "\n";
    exit(-1);
}

my $dictionaryName = $ARGV[0];
my $modelName = $ARGV[1];

# backup files
#copy($dictionaryName, "${dictionaryName}.bak") or die("could not backup dictionary");
#copy($modelName, "${modelName}.bak") or die("could not backup model parameters");

# find which dictionary entries are actually used
my @indices = ();
my %seenIndex = ();
open FILE, $modelName;
while (<FILE>) {
    if (m/\{\s*var:\s*(\d+)/) {
	push (@indices, $1) unless $seenIndex{$1}++;
    }
}
close FILE;

print STDERR "can reduce dictionary to " . ($#indices + 1) ." entries\n";

if ($#indices < 1) {
    print STDERR "but something don't feel right!\n";
    exit(0);
}

# sort in order and build reverse index
my %reverseIndex = ();
@indices = sort { $a <=> $b } @indices;
for (my $i = 0; $i <= $#indices; $i++) {
    $reverseIndex{$indices[$i]} = $i;
}

# replace indices in model file
open FILE, $modelName;
my @model = <FILE>;
close FILE;

for (my $i = 0; $i <= $#model; $i++) {
    if ($model[$i] =~ m/\s+(var_all|var_count|ord_var_count):\s*(\d+)/) {
	my $n = $2;
	my $m = $#indices + 1;
	$model[$i] =~ s/$n/$m/;
    } elsif ($model[$i] =~ m/\{\s*var:\s*(\d+)/) {
	my $index = $1;
	$model[$i] =~ s/$index/$reverseIndex{$index}/;
    } elsif ($model[$i] =~ m/^(\s*var_type:\s*\[)/) {
        my $prefix = $1;
        while ($model[$i] !~ m/\]/) {
            $model[$i++] = "";
        }
        $model[$i] = $prefix . (("0, ") x $#indices) . "0 ]\n";
    }

    # removes unnecessary lines from OpenCV model file to save space
    if (defined($opts{r})) {
        if (($model[$i] =~ m/Tn:\s*0/) || ($model[$i] =~ m/complexity:\s*0/) ||
            ($model[$i] =~ m/alpha:\s*0/) || ($model[$i] =~ m/node_risk:/) ||
            ($model[$i] =~ m/tree_risk:/) || ($model[$i] =~ m/tree_error:/)) {
            $model[$i] = "";
        }
    }
}

open FILE, ">$modelName";
print FILE join("", @model);
close FILE;

# remove unused entries from dictionary
open FILE, $dictionaryName;
my @dictionary = <FILE>;
close FILE;

open FILE, ">$dictionaryName";
my $i = 0;
while ($dictionary[$i] !~ m/\<PatchDefinition/) {
    if ($dictionary[$i] =~ m/numEntries=\"(\d+)\"/) {
        my $m = $#indices + 1;
        $dictionary[$i] =~ s/numEntries=\"(\d+)\"/numEntries=\"$m\"/;
    }
    print FILE $dictionary[$i++];
}

my $index = 0;
while ($i < $#dictionary) {
    while ($dictionary[$i] !~ m/\<\/PatchDefinition/) {
        if (defined($reverseIndex{$index})) {
            print FILE $dictionary[$i];
        }
        $i++;
    }
    if (defined($reverseIndex{$index})) {
        print FILE $dictionary[$i];
    }
    $i++;
    $index++;
}

print FILE "</PatchDictionary>\n";
close FILE;

