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
      read -r -p "use collada format? [y/n] " response
      response=${response,,}    # tolower
      collada=false
      if [[ $response =~ ^(yes|y)$ ]]; then
        collada=true
      fi
        read -r -p "ratio?" ratio
        if [[ $ratio =~ ^[+-]?[0-9]+\.?[0-9]*$ ]]; then
            read -r -p "use bounding boxes for collision models? [y/n] " response
            response=${response,,}
            if [[ $response =~ ^(yes|y)$ ]]; then
                blender --background --python reduce-stl-dae-bpy.py -- "$1/meshes/" "$1/meshes/" $ratio "$collada"
                if [ $? -ne 0 ]; then
                  echo failed to reduce meshes...got blender>=2.8?
                  cd $currentworkingdirectory
                  exit 1
                fi
            else
                read -r -p "ratio for collision models?" collision_ratio
                if [[ $collision_ratio =~ ^[+-]?[0-9]+\.?[0-9]*$ ]]; then
                    blender --background --python reduce-stl-dae-bpy.py -- "$1/meshes/" "$1/meshes/" $ratio "$collada" $collision_ratio
                    if [ $? -ne 0 ]; then
                      echo failed to reduce meshes...got blender>=2.8?
                      cd $currentworkingdirectory
                      exit 1
                    fi
                else
                    echo "invalid input"
                    cd $currentworkingdirectory
                    exit 1
                fi

            fi
        else
            echo "invalid input"
            cd $currentworkingdirectory
            exit 1
        fi
    fi

    read -r -p "use reduced meshes in model.sdf and model.urdf? [y/n] " response
    response=${response,,}    # tolower
    if [[ $response =~ ^(yes|y)$ ]]; then
      cd $currentworkingdirectory/../python
      python update_mesh_uri.py "$1/model.sdf" "$collada"
      python update_mesh_uri.py "$1/model.urdf" "$collada"
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
