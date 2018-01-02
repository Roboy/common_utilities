from subprocess import call
import sys
from os.path import isfile

argv = sys.argv

print(argv)

if len(argv)!=5:
    print('USAGE: path/to/stl outputpath ration[0-1] scale')
else:
    if isfile(argv[1]):
        if argv[1].endswith('.stl'):
            call(["blender", "--background", "--python", "reduceMesh.py", "--", argv[1], argv[2], argv[3], argv[4] ])
        else:
            print(argv[1] + ' is not a .stl file')
    else:
        print(argv[1]+' is not a valid file path')
