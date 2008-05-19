#!/usr/bin/perl
#
# STAIR VISION PROJECT
# Copyright (c) 2007-2008, Stephen Gould
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Stanford University nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
# EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <copyright holder> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
##############################################################################
#
# FILENAME:    trainObjectDetector.pl
# AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
# DESCRIPTION:
#  Trains a patch-based object detector for a given class using
#  the following steps:
#   1. constucts a dictionary by randomly selecting patches from
#      the positive training examples (~/scratch/data/<object>)
#   2. computes a patch cache for positive and negative training
#      examples (in /tmp/response-cache/). Negative samples come
#      from ~/scratch/data/other.
#   3. trains the (patch-based) object detector
#   4. trims the dictionary and model file to remove patches
#      not selected by the learning algorithm
#   5. visualizes the trimmed dictionary
#

use strict;
use Getopt::Std;
use Cwd 'realpath';

my $EXEBASE = "bin/";
#my $DATADIR = "~/scratch/data/objects/";
my $DATADIR = "/home/sgould/data/objects/";
my $CACHEDIR = "/tmp/response-cache/";
my $MODELDIR = "${EXEBASE}../models/";
my $NUM_PATCHES_PER_IMAGE = 10;
my $BASE_WIDTH = 32;
my $BASE_HEIGHT = 32;
my $BOOSTING_ROUNDS = 50;
my $WEAK_LEARNER_SPLITS = 2;
my $MAX_IMAGES = 10000;

my %opts = ();
getopts("cw:h:r:s:n:m:v", \%opts);
if ($#ARGV < 0) {
    print STDERR "USAGE: ./trainObjectDetector.pl [options] <object>\n";
    print STDERR "OPTIONS:\n";
    print STDERR "  -c        :: clear all\n";
    print STDERR "  -w <n>    :: base image width (default: $BASE_WIDTH)\n";
    print STDERR "  -h <n>    :: base image height (default: $BASE_HEIGHT)\n";
    print STDERR "  -r <n>    :: number of boosting rounds (default: $BOOSTING_ROUNDS)\n";
    print STDERR "  -s <n>    :: number of splits in weak learner (default: $WEAK_LEARNER_SPLITS)\n";
    print STDERR "  -n <n>    :: number patches to sample per image (default: $NUM_PATCHES_PER_IMAGE)\n";
    print STDERR "  -m <n>    :: maximum number of training instances (default: $MAX_IMAGES)\n";
    print STDERR "  -v        :: visualize dictionary after training\n";
    print STDERR "\n";
    exit(-1);
}

$BASE_WIDTH = $opts{'w'} if (exists($opts{'w'}));
$BASE_HEIGHT = $opts{'h'} if (exists($opts{'h'}));
$BOOSTING_ROUNDS = $opts{'r'} if (exists($opts{'r'}));
$WEAK_LEARNER_SPLITS = $opts{'s'} if (exists($opts{'s'}));
$NUM_PATCHES_PER_IMAGE = $opts{'n'} if (exists($opts{'n'}));
$MAX_IMAGES = $opts{'m'} if (exists($opts{'m'}));

my $object = $ARGV[0];

my $dictFilename = "${MODELDIR}dictionary.${object}.txt";
my $modelFilename = "${MODELDIR}${object}.model";

my $cmdline;

# delete existing dictionary and model files
if (exists($opts{'c'})) {
    unlink($dictFilename);
    unlink($modelFilename);
}

# create cache directory
if (!-d "${CACHEDIR}") {
    `mkdir ${CACHEDIR}`;
}

# build dictionary (clear cache and model file)
if (!-e $dictFilename) {
    if (-d "${CACHEDIR}${object}-pos") {
	$cmdline = "rm -rf ${CACHEDIR}${object}-pos";
	print "$cmdline\n";
	`$cmdline`;
    }

    if (-d "${CACHEDIR}${object}-neg") {
	$cmdline = "rm -rf ${CACHEDIR}${object}-neg";
	print "$cmdline\n";
	`$cmdline`;
    }

    `rm -f $modelFilename` if (-e $modelFilename);

    $cmdline = "${EXEBASE}buildPatchDictionary -o $dictFilename -n $NUM_PATCHES_PER_IMAGE ${DATADIR}${object} $BASE_WIDTH $BASE_HEIGHT";
    print "$cmdline\n";
    `$cmdline`;
}

# compute response cache (in <object>-pos and <object>-neg)
if (!-d "${CACHEDIR}${object}-pos") {
    `mkdir ${CACHEDIR}${object}-pos`;
    $cmdline = "${EXEBASE}buildPatchResponseCache -maxImages ${MAX_IMAGES} ${DATADIR}${object} ${CACHEDIR}${object}-pos $dictFilename";
    print "$cmdline\n";
    `$cmdline`;
}

if (!-d "${CACHEDIR}${object}-neg") {
    `mkdir ${CACHEDIR}${object}-neg`;
    $cmdline = "${EXEBASE}buildPatchResponseCache -maxImages ${MAX_IMAGES} ${DATADIR}other ${CACHEDIR}${object}-neg $dictFilename";
    print "$cmdline\n";
    `$cmdline`;
}

# train object classifier
if (!-e $modelFilename) {
    $cmdline = "${EXEBASE}trainObjectClassifier -c PATCH -o $modelFilename -maxImages $MAX_IMAGES -rounds $BOOSTING_ROUNDS -splits $WEAK_LEARNER_SPLITS -cached ${CACHEDIR}${object}-pos ${CACHEDIR}${object}-neg";
    print "$cmdline\n";
    `$cmdline`;
}

# trim dictionary (for faster detection)
$cmdline = "${EXEBASE}../scripts/trimDictionary.pl $dictFilename $modelFilename";
print "$cmdline\n";
`$cmdline`;

# visualize
if (defined($opts{'v'})) {
    $cmdline = "${EXEBASE}visualizeClassifier $dictFilename $modelFilename";
    print "$cmdline\n";
    `$cmdline`;
}

exit(0);

