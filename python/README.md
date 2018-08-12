## reduce dae meshes
The reduceMeshesInDirectory.py is a helper script for reducing meshes of file type .dae

## Dependencies
python3 blender 
### Ubuntu16 
```
#!/bin/bash
sudo apt install blender
```
## Usage
Example directory:
```
#!/bin/bash
python reduceMeshesInDirectory.py ~/workspace/roboy_models/Roboy2.0/meshes/CAD/ ~/workspace/roboy_models/Roboy2.0/meshes/CAD 0.1 1
```
The first two parameters define paths to input/output directories, the last parameters defines the ratio of mesh reduction (c.f. [decimate modifier](https://www.blender.org/manual/modeling/modifiers/generate/decimate.html)). The script search through the input directory and transforms every .stl file to a .stl file with the same name into the output directory. NOTE: mind the additional '/' in the input/output paths. The last parameter is the scale. 

Example single file:
```
#!/bin/bash
python reduceMeshSingleFile.py ../../roboy_models/Roboy2.0_Head_simplified/meshes/CAD/torso.stl ../../roboy_models/Roboy2.0_Head_simplified/meshes/CAD/torso.stl 0.1 1
```
The first two parameters define paths to input/output .dae file, while the thir parameter defines the ratio of mesh reduction, the last parameter is the scale.

## Note
There are no sanity checks regarding your original files. They will be overwritten, if you set the output directory to your input directory!
Also reducing the mesh faces below some value will degenerate your mesh. To avoid this you can try different values first in blender.
