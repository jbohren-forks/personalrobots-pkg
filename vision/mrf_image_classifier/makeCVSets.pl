#!/usr/bin/perl

use warnings; use strict;

use File::Basename;

use POSIX qw/floor/;

my $FOLDS = 5;

my $dirIn = $ARGV[0];
my $dirOut = $ARGV[1];

my @files = glob("$dirIn/*png");

##
my $nTest = floor(@files / $FOLDS);
my $nTrain = @files - $nTest;

for my $fold (1 .. $FOLDS) {
    mkdir "$dirOut/$fold";
    mkdir "$dirOut/$fold/train";
    mkdir "$dirOut/$fold/test";

    my @inds = 0..(@files-1);
    @inds = sort { rand() <=> 0.5 } @inds;

    my @trainingInds = @inds[0 .. ($nTrain - 1)];
    my @testInds = @inds[$nTrain .. (@files - 1)];

    print "nt $nTrain\n";
    print "FOLD $fold\n";
    print "TRAIN @trainingInds\n";
    print "TEST @testInds\n\n";

    system("cp graspObjects.dat $dirOut/$fold/train/objects.dat");

    for my $ind (@trainingInds) {
	my $file = $files[$ind];
	(my $prefix) = ($file =~ /(.*)\.png/);
	my $segfile = "$prefix.seg";
	system("cp $file $dirOut/$fold/train");
	system("cp $segfile $dirOut/$fold/train");
	system("echo cp $file $dirOut/$fold/train");
	system("echo cp $segfile $dirOut/$fold/train");
    }
    for my $ind (@testInds) {
	my $file = $files[$ind];
	(my $prefix) = ($file =~ /(.*)\.png/);
	my $segfile = "$prefix.seg";
	system("cp $file $dirOut/$fold/test");
	system("cp $segfile $dirOut/$fold/test");
	system("echo cp $file $dirOut/$fold/test");
	system("echo cp $segfile $dirOut/$fold/test");
    }
}
