#!/usr/bin/perl -I ../../external/perl5
#
# STAIR VISION LIBRARY
# Copyright (c) 2007-2008, Stanford University
#
# FILENAME:    generateCrossValidationFolds.pl
# AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
# DESCRIPTION:
#   Generates cross-validation folds, by randomly permuting the rows
#   in a given file (and corrsponding rows in associated files) and
#   partitions into training and hold-out sets.
#

use File::Spec;
use Getopt::Std;
use List::Util 'shuffle';
use strict;

my $FOLDS = 5;
my $OUTPUTDIR = ".";
my $HEADERLINES = 0;
my $FOOTERLINES = 0;

my %opts = ();
getopts("f:o:h:t:", \%opts);
if ($#ARGV < 0) {
    print STDERR "USAGE: ./generateCrossValidationFolds.pl [options] (<data>)\n";
    print STDERR "OPTIONS:\n";
    print STDERR "  -f <n>    :: number of folds (default: $FOLDS)\n";
    print STDERR "  -h <n>    :: number of header lines (default: $HEADERLINES)\n";
    print STDERR "  -t <n>    :: number of footer lines (default: $FOOTERLINES)\n";
    print STDERR "  -o <dir>  :: output directory (default: $OUTPUTDIR)\n";
    print STDERR "\n";
    exit(-1);
}

$FOLDS = $opts{f} if (defined($opts{f}));
$OUTPUTDIR = $opts{o} if (defined($opts{o}));
$HEADERLINES = $opts{h} if (defined($opts{h}));
$FOOTERLINES = $opts{t} if (defined($opts{t}));
die "Invalid commandline option" if (($FOLDS < 0) || ($HEADERLINES < 0) || ($FOOTERLINES < 0));

# load first data file
open FILE, $ARGV[0];
my @data = <FILE>;
close FILE;

# remove empty trailing lines
while ($data[$#data] =~ m/^\s*$/) {
    $#data -= 1;
}

my $numInstances = $#data + 1 - $HEADERLINES - $FOOTERLINES;
my $testInstancesPerFold = $numInstances / $FOLDS;
print STDOUT "Generating data for ${FOLDS}-fold cross-validation on $numInstances instances...\n";
die "Too few test instances per fold" if ($testInstancesPerFold < 1);

# generate permutations
my @index = ();
for (my $i = 0; $i < $numInstances; $i++) {
    $index[$i] = $i;
}
@index = shuffle(@index);

# write out folds for each input file
for (my $i = 0; $i <= $#ARGV; $i++) {
    # extract filename
    my ($volume, $directory, $filename) = File::Spec->splitpath($ARGV[$i]);

    print STDOUT "...processing $filename\n";
    open FILE, $ARGV[$i];
    @data = <FILE>;
    close FILE;

    # remove empty trailing lines
    while ($data[$#data] =~ m/^\s*$/) {
        $#data -= 1;
    }

    # remove header and footer lines
    my @header = ();
    my @footer = ();
    for (my $j = 0; $j < $HEADERLINES; $j++) {
        push @header, shift(@data);
    }
    for (my $j = 0; $j < $FOOTERLINES; $j++) {
        unshift @footer, pop(@data);
    }

    # construct folds
    die "Incorrect number of instances" if ($#data != $numInstances - 1);

    for (my $fold = 0; $fold < $FOLDS; $fold++) {
        open TRAINFILE, ">${OUTPUTDIR}/${filename}.${fold}.train";
        open TESTFILE, ">${OUTPUTDIR}/${filename}.${fold}.test";
        print TRAINFILE join("", @header);
        print TESTFILE join("", @header);

        for (my $n = 0; $n < $numInstances; $n++) {
            if (($n >= $fold * $testInstancesPerFold) && 
                ($n < ($fold + 1) * $testInstancesPerFold)) {
                print TESTFILE $data[$index[$n]];
            } else {
                print TRAINFILE $data[$index[$n]];
            }
        }

        print TRAINFILE join("", @footer);
        print TESTFILE join("", @footer);
        close TESTFILE;
        close TRAINFILE;        
    }
}

print STDOUT "...done\n";

