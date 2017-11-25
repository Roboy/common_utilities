from subprocess import call
import sys
from os import listdir
from os.path import isfile, join

argv = sys.argv

print(argv)

if len(argv)<=4 and len(argv)<6:
    print('USAGE: path/to/meshes/directory/ outputpath/directory/ ration[0-1] OPTIONAL: scale[0.01-1000]')
else:
    files = [f for f in listdir(argv[1]) ]
    print('Converting '+ str(len(files)) +' files')
    i=0
    scale = 1
    if(len(argv)==5):
	scale = argv[4]
    for file in files:
        if file.endswith('.stl') or file.endswith('.STL') :
            i+=1
            print('Converting ( ' + str(i) + '/' + str(len(files)) + ' )')
            call(["blender", "--background", "--python", "reduceMesh.py", "--", join(argv[1],file), join(argv[2],file), argv[3], scale])
    if(i<len(files)):
	print('there where some none stl files, didn\'t touch them though');
