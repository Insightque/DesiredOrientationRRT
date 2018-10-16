#!/bin/bash
ROSVERSION=`rosversion -d`
retval=`dpkg -l|grep ros-$ROSVERSION-ompl`
curpath=$(pwd)
omplpath="$2"

function echocolor(){
RED='\033[0;31m'
NC='\033[0m' # No Color
printf "${RED}$1${NC}\n"
}

function bindros(){
if [ "$retval" = "" ]
then
    echo "There is no ROS-$ROSVERSION-OMPL package"
    sudo apt-get install ros-$ROSVERSION-ompl
else
    echo "ORIGINAL OMPL >>>>> DO OMPL"
    sudo rm -rf lib
    sudo rm -rf ompl
    sudo cp -r $omplpath/lib ./
    sudo cp -r $omplpath/src/ompl ./
    cd ./ompl
    sudo rm -rf CMake*
    sudo rm CTestTestfile.cmake
    sudo rm Makefile
    sudo rm config.h.in
    sudo rm cma*
    find . -name "src" -type d -exec sudo rm -rf {} \;
    cd $curpath
    sudo rm -rf /opt/ros/$ROSVERSION/include/ompl
    sudo cp -r ./ompl/ /opt/ros/$ROSVERSION/include/
    sudo rm /opt/ros/$ROSVERSION/lib/x86_64-linux-gnu/libompl*.*
    sudo cp -i -b -r ./lib/* /opt/ros/$ROSVERSION/lib/x86_64-linux-gnu/
    sudo rm -rf ./ompl/ ./lib/
fi

}


function buildompl(){
    echo $omplpath
    cd $omplpath
    cmake .
    make -j 4
    cd $curpath
}

option="$1"
case $option in
    bindros)
    bindros
    ;;
    buildompl)
    buildompl
    ;;
    *)
    echocolor "Need Argument - git repo name [tmp.git]"
    echocolor "1. buildompl \OMPL_LIBRARY_PATH(ABSOLUTE PATH)"
    echocolor "2. bindros \OMPL_LIBRARY_PATH(ABSOLUTE PATH)"
    ;;
esac
