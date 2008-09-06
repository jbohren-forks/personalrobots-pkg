#!/usr/bin/perl -I ../../external/perl5
#
# STAIR VISION LIBRARY
# Copyright (c) 2008, Stanford University
#
# FILENAME:    trainObjectDetector.pl
# AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
# DESCRIPTION:
#  Trains a patch-based object detector for a given class using
#  the following steps:
#   1. constucts a dictionary by randomly selecting patches from
#      the positive training examples (data/objects/<object>)
#   2. computes a patch cache for positive and negative training
#      examples (in /tmp/response-cache/) for a subset of the
#      training data. Negative samples come from data/negative.
#   3. trains the (patch-based) object detector
#   4. trims the dictionary and model file to remove patches
#      not selected by the learning algorithm
#   5. repeats steps 2-4 on all training data
#

use strict;
use Getopt::Std;
use File::Path;
use Cwd 'realpath';

my $EXEBASE = "bin/";
my $DATADIR = "${EXEBASE}../data/objects/";
my $CACHEDIR = "/tmp/response-cache/";
my $MODELDIR = "${EXEBASE}../models/";
my $NUM_PATCHES_PER_IMAGE = 10;
my $BASE_WIDTH = 32;
my $BASE_HEIGHT = 32;
my $BOOSTING_ROUNDS = 100;
my $WEAK_LEARNER_SPLITS = 2;
my $MAX_IMAGES_A = 2000;
my $MAX_IMAGES_B = 20000;
my $NEGATIVE_OBJECTS = "negative";

my %opts = ();
getopts("b:cd:w:h:r:s:n:m:M:N:", \%opts);
if ($#ARGV < 0) {
    print STDERR "USAGE: ./trainObjectDetector.pl [options] <object>\n";
    print STDERR "OPTIONS:\n";
    print STDERR "  -b <dir>  :: base bin directory (default: $EXEBASE)\n";
    print STDERR "  -d <dir>  :: base data directory (default: $DATADIR)\n";
    print STDERR "  -c        :: clear all\n";
    print STDERR "  -w <n>    :: base image width (default: $BASE_WIDTH)\n";
    print STDERR "  -h <n>    :: base image height (default: $BASE_HEIGHT)\n";
    print STDERR "  -r <n>    :: number of boosting rounds (default: $BOOSTING_ROUNDS)\n";
    print STDERR "  -s <n>    :: number of splits in weak learner (default: $WEAK_LEARNER_SPLITS)\n";
    print STDERR "  -n <n>    :: number patches to sample per image (default: $NUM_PATCHES_PER_IMAGE)\n";
    print STDERR "  -m <n>    :: maximum number of stage 1 training instances (default: $MAX_IMAGES_A)\n";
    print STDERR "  -M <n>    :: maximum number of stage 2 training instances (default: $MAX_IMAGES_B)\n";
    print STDERR "  -N <name> :: negative images name (default: $NEGATIVE_OBJECTS)\n";
    print STDERR "\n";
    exit(-1);
}

if (exists($opts{'b'})) {
    $EXEBASE = "$opts{'b'}/";
    $DATADIR = "${EXEBASE}../data/objects/";
    $CACHEDIR = "/tmp/response-cache/";
    $MODELDIR = "${EXEBASE}../models/";
}
$DATADIR = $opts{'d'} if (exists($opts{'d'}));
$BASE_WIDTH = $opts{'w'} if (exists($opts{'w'}));
$BASE_HEIGHT = $opts{'h'} if (exists($opts{'h'}));
$BOOSTING_ROUNDS = $opts{'r'} if (exists($opts{'r'}));
$WEAK_LEARNER_SPLITS = $opts{'s'} if (exists($opts{'s'}));
$NUM_PATCHES_PER_IMAGE = $opts{'n'} if (exists($opts{'n'}));
$MAX_IMAGES_A = $opts{'m'} if (exists($opts{'m'}));
$MAX_IMAGES_B = $opts{'M'} if (exists($opts{'M'}));
$NEGATIVE_OBJECTS = $opts{'N'} if (exists($opts{'N'}));

my $object = $ARGV[0];

my $dictFilename = "${MODELDIR}${object}.dictionary.xml";
my $modelFilename = "${MODELDIR}${object}.model";

my $cmdline;

# delete existing dictionary and model files
if (exists($opts{'c'})) {
    unlink($dictFilename);
    unlink($modelFilename);
}

# create cache directory
if (!-d $CACHEDIR) {
    mkdir($CACHEDIR);
}

# build dictionary (clear cache and model file)
if (!-e $dictFilename) {
    if (-d "${CACHEDIR}${object}-pos") {
        rmtree("${CACHEDIR}${object}-pos", 1, 1);
    }
    if (-d "${CACHEDIR}${object}-neg") {
        rmtree("${CACHEDIR}${object}-neg", 1, 1);
    }
    
    unlink($modelFilename) if (-e $modelFilename);

    $cmdline = "${EXEBASE}buildPatchDictionary -o $dictFilename -n $NUM_PATCHES_PER_IMAGE ${DATADIR}${object} $BASE_WIDTH $BASE_HEIGHT";
    print "$cmdline\n";
    `$cmdline`;
}

# compute response cache (in <object>-pos and <object>-neg)
if (!-d "${CACHEDIR}${object}-pos") {
    mkdir("${CACHEDIR}${object}-pos");
    $cmdline = "${EXEBASE}buildPatchResponseCache -maxImages ${MAX_IMAGES_A} ${DATADIR}${object} ${CACHEDIR}${object}-pos $dictFilename";
    print "$cmdline\n";
    `$cmdline`;
}

if (!-d "${CACHEDIR}${object}-neg") {
    mkdir("${CACHEDIR}${object}-neg");
    $cmdline = "${EXEBASE}buildPatchResponseCache -maxImages ${MAX_IMAGES_A} ${DATADIR}${NEGATIVE_OBJECTS} ${CACHEDIR}${object}-neg $dictFilename";
    print "$cmdline\n";
    `$cmdline`;
}

# train object detector
if (!-e $modelFilename) {
    $cmdline = "${EXEBASE}trainObjectDetector -c PATCH -o $modelFilename -maxInstances $MAX_IMAGES_A -rounds $BOOSTING_ROUNDS -splits $WEAK_LEARNER_SPLITS ${CACHEDIR}${object}-pos ${CACHEDIR}${object}-neg";
    print "$cmdline\n";
    print `$cmdline`;
}

# trim dictionary (for faster detection)
$cmdline = "${EXEBASE}../svl/scripts/trimDictionary.pl $dictFilename $modelFilename";
print "$cmdline\n";
`$cmdline`;

# repeat with more images
if ($MAX_IMAGES_B > $MAX_IMAGES_A) {
    # update response cache
    $cmdline = "${EXEBASE}buildPatchResponseCache -maxImages ${MAX_IMAGES_B} ${DATADIR}${object} ${CACHEDIR}${object}-pos $dictFilename";
    print "$cmdline\n";
    `$cmdline`;

    $cmdline = "${EXEBASE}buildPatchResponseCache -maxImages ${MAX_IMAGES_B} ${DATADIR}${NEGATIVE_OBJECTS} ${CACHEDIR}${object}-neg $dictFilename";
    print "$cmdline\n";
    `$cmdline`;

    # train object detector
    $cmdline = "${EXEBASE}trainObjectDetector -c PATCH -o $modelFilename -maxInstances $MAX_IMAGES_B -rounds $BOOSTING_ROUNDS -splits $WEAK_LEARNER_SPLITS ${CACHEDIR}${object}-pos ${CACHEDIR}${object}-neg";
    print "$cmdline\n";
    print `$cmdline`;

    # trim dictionary
    $cmdline = "${EXEBASE}../svl/scripts/trimDictionary.pl $dictFilename $modelFilename";
    print "$cmdline\n";
    `$cmdline`;
}

exit(0);

