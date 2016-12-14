#!/bin/bash
if [ "$#" -ne 1 ]; then
    echo "USAGE:     ./makeSDFfromUREDFexport.bash path/to/directory"
else
    currentworkingdirectory=$(pwd)
    cd $1
    projectname=$(ls urdf|sed -e "s/.urdf//")
    mkdir $projectname
    cd $projectname
    mkdir cad
    cp ../meshes/* cad
    #python $currentworkingdirectory/../python/step-stl-dae/stl-to-dae.py cad/ dae/
    gz sdf -p ../urdf/$projectname.urdf > model.sdf
    grep -rl "stl" model.sdf | xargs sed -i "s/stl/STL/g"
    grep -rl "meshes" model.sdf | xargs sed -i "s/meshes/cad/g"
    cp $currentworkingdirectory/model.config .
    grep -rl "PROJECTNAME" model.config | xargs sed -i "s/PROJECTNAME/$projectname/g"
    cd $currentworkingdirectory 
fi 
