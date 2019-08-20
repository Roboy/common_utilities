from subprocess import call
import sys
import os
from os import listdir
from os.path import isfile, join

argv = sys.argv

print(len(argv), argv)

if len(argv) < 4 or len(argv) > 6:
    print('USAGE: path/to/meshes/directory/ outputpath/directory/ ration[0-1] OPTIONAL: scale[0.01-1000]')
else:
    files = [f for f in listdir(argv[1])]
    print('Converting ' + str(len(files)) + ' files')
    i = 0
    scale = 1
    if (len(argv) == 5):
        scale = argv[4]
    for file in files:
        if (file.endswith('.stl') or file.endswith('.STL')) and os.path.getsize(join(argv[1], file)) > 100000:  # for files bigger that 100kB
            i += 1
            print('Converting ( ' + str(i) + '/' + str(len(files)) + ' )')
            call(
                ["blender", "--background", "--python", "reduceMesh.py", "--", join(argv[1], file), join(argv[2], file),
                 argv[3], str(scale)])
        else:
            print('ignoring ' + join(argv[1], file))
    if (i < len(files)):
        print('there where some files that were either no stls or too small (<100kB), didn\'t touch them though');
