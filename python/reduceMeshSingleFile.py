from subprocess import call
import sys
from os.path import isfile

argv = sys.argv

print(argv)

if len(argv)!=4:
    print('USAGE: path/to/dae outputpath ration[0-1]')
else:
    if isfile(argv[1]):
        if argv[1].endswith('.dae'):
            call(["blender", "--background", "--python", "reduceMesh.py", "--", argv[1], argv[2], argv[3] ])
        else:
            print(argv[1] + ' is not a .dae file')
    else:
        print(argv[1]+' is not a valid file path')