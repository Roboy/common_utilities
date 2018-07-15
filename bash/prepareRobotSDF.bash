#!/bin/bash
if [ "$#" -ne 1 ]; then
    echo "USAGE:     ./prepareRobotSDF.bash path/to/directory"
else
    currentworkingdirectory=$(pwd)
    cd $1
    rosrun pysdf sdf2urdf.py --no-prefix model.sdf model.urdf
    if [ $? -ne 0 ]; then
      echo FAIL
      cd $currentworkingdirectory
      exit 1
    fi
    projectname=$(ls model.urdf|sed -e "s/.urdf//")
    read -r -p "reduce meshes? [y/n] " response
    response=${response,,}    # tolower
    if [[ $response =~ ^(yes|y)$ ]]; then
	    datetimestring=$(date +'%d%m%Y_%H-%M')
	    cd $currentworkingdirectory/../python
        python reduceMeshesInDirectory.py "$1/meshes/CAD/" "$1/meshes/CAD/" 0.1 1
        if [ $? -ne 0 ]; then
          echo failed to reduce meshes...got blender?
          cd $currentworkingdirectory
          exit 1
        fi
    fi
    echo DONE
    cd $currentworkingdirectory
fi 
