EXPECTED_ARGS=1
E_BADARGS=65

if [ $# -ne $EXPECTED_ARGS ]
then
    echo 'Usage: run_planner.sh [environment file]'
    exit $E_BADARGS
fi

logfile=logs/$(date +%Y-%m-%d-%H-%M-%S)-output.log
solutionfile=logs/$(date +%Y-%m-%d-%H-%M-%S)-solution.log

echo Running planning with config file - $1

bin/test_nav3ddyn $1 > $logfile

grep 'Printing actions to file' $logfile
grep 'Time to precompute actions' $logfile

echo Logging info output to file - $logfile

grep 'planning time' $logfile

EXPECTED_RETURN=0

grep 'failed to find a solution' $logfile > /dev/null
RETURN_VAL=$?

if [ $RETURN_VAL -ne $EXPECTED_RETURN ]
then
    grep 'Computing Successors' $logfile > $solutionfile
    grep 'Adding goal state to list' $logfile >> $solutionfile
    grep 'Original Coords' $logfile >> $solutionfile

    echo Solution info output to file - $solutionfile

else
    echo Failed to find a solution
fi

echo Done