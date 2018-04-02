#!/bin/bash


./premake5 gmake
make config=release_x64
makeExitStatus=$?
if [ "$makeExitStatus" = "0" ];
then
	echo "Build success"
else
	echo "\n\n\n\n\n\n"
	echo "****** Error building library *****"
fi

