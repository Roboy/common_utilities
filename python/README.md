# Python Scripts and Blender Plugins

This folder contains scripts which can be installed in blender as a plugin, or in case this is not possible, can be used in the python console inside of blender.  
__IMPORTANT:__ The python library `bpy` can no longer be used outside of blender, all scripts using this module need to be run in blender. One option to run a script in Blender can be seen in the following commands. For this, open blender and go to the python console in blender. 

	import sys
	sys.argv = ['/path/to/common_utilities/python/FileOfInterest.py', args] 
	exec(open('/path/to/common_utilities/python/FileOfInterest.py').read())


## Dependencies

#### Blender

download and install python3 and blender, add blender to your python path if need be (since all scripts depending on blender can only be run inside of blender, this should not be necessary).

	sudo apt install blender

## Usage

#### ReduceMeshesInDirectory

	python reduceMeshesInDirectory.py ~/workspace/roboy_models/Roboy2.0/meshes/CAD/ ~/workspace/roboy_models/Roboy2.0/meshes/CAD 0.1 1

The first two parameters define paths to input/output directories, the last parameters defines the ratio of mesh reduction (c.f. [decimate modifier](https://www.blender.org/manual/modeling/modifiers/generate/decimate.html)). The script search through the input directory and transforms every .stl file to a .stl file with the same name into the output directory. NOTE: mind the additional '/' in the input/output paths. The last parameter is the scale. 

__Note__:
There are no sanity checks regarding your original files. They will be overwritten, if you set the output directory to your input directory!
Also reducing the mesh faces below some value will degenerate your mesh. To avoid this you can try different values first in blender.

#### ReduceMeshSingleFile

	python reduceMeshSingleFile.py ../../roboy_models/Roboy2.0_Head_simplified/meshes/CAD/torso.stl ../../roboy_models/Roboy2.0_Head_simplified/meshes/CAD/torso.stl 0.1 1

The first two parameters define paths to input/output .dae file, while the thir parameter defines the ratio of mesh reduction, the last parameter is the scale.

__Note__:
There are no sanity checks regarding your original files. They will be overwritten, if you set the output directory to your input directory!
Also reducing the mesh faces below some value will degenerate your mesh. To avoid this you can try different values first in blender.


#### ReduceAndExportSeparatelyAsObj

This file reads in all .stl files in the specified folder or the file itself -depending on what was specified- and imports it. Then, it applies the specified reduction scale by which the number of vertices is reduced. Finally, the scale is applied to the models which are then exported separately as individual .obj files. The file name consists of the object name, written in lowercase and words separated by '_'. The corresponding .mtl are deleted since these are not needed. The original files are not changed neither overwritten. This script needs to be run in blender, one exemplary use case can be seen here:

	import sys
	sys.argv = ['/home/roboy/workspace/common_utilities/python/reduceAndExportSeparatelyAsObj.py', '/home/roboy/workspace/roboy_models/roboy_xylophone_left_arm/meshes/CAD/', '/home/roboy/workspace/DartTracker/src/dart_tracker/models/roboy_xylophone_left_arm/meshes/', 0.3, 0.001]
	exec(open('/home/roboy/workspace/common_utilities/python/reduceAndExportSeparatelyAsObj.py').read())


#### SdfToXml 

This script can be run independently from blender. It is used to parse all information needed by the DartTracker from a given .sdf file to an .xml file. Additionally, an optional subfolder in which to find the meshes can be specified (relative to the position of the .xml file) as well as an optional color of the model. Mind the order of variables. 

	python SdfToXml.py /path/to/in.sdf /path/to/out.xml subfolder/subsubfolder/ 255 255 255 


One example: 

	python SdfToXml.py /home/roboy/workspace/roboy_models/roboy_xylophone_left_arm/model.sdf /home/roboy/workspace/DartTracker/src/dart_tracker/models/roboy_xylophone_left_arm/roboy_xylophone_left_arm.xml meshes/ 220 220 220