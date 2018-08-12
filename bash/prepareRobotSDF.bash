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
    check_urdf model.urdf
    if [ $? -ne 0 ]; then
      echo "urdf check failed, check your sdf"
      cd $currentworkingdirectory
      exit 1
    fi
    projectname=$(echo $1| cut -d "/" -f4)
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
    ls ~/.gazebo/models/$projectname
    if [ $? -ne 0 ]; then
      read -r -p "create symbolic link to gazebo models? [y/n] " response
      response=${response,,}    # tolower
      if [[ $response =~ ^(yes|y)$ ]]; then
        ln -s $currentworkingdirectory/$1 ~/.gazebo/models/$projectname
      fi
    fi

    echo DONE
    cd $currentworkingdirectory
fi 
