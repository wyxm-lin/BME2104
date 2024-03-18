SEED=$(date +%s)
RANDOM=$SEED
../../LinuxReleasev1.1/PreliminaryJudge ./bin/test-main -f 0 -s 2 -m ./maps/map1.txt -d output.txt
# ../ContestProblem/LinuxRelease/PreliminaryJudge ./bin/$1 -f 0 -s $RANDOM -m ./maps/map$2.txt -d output.txt
# ../ContestProblem/LinuxRelease/PreliminaryJudge ./bin/$1 -f 0 -m ./maps/map$2.txt -d output.txt