# change to current workspace
export ACKERMANN_WS=`echo "$( cd "$( dirname "$0" )" && pwd )"`
cd `echo $ACKERMANN_WS`

rm -rf devel build src/CMakeLists.txt
source make.sh
