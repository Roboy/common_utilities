#!/usr/bin/env python

import sys
import xml.etree.ElementTree as ET

if len(sys.argv) < 2:
    print "Please provide the xml filename"
    sys.exit(-1)

cardsflow = ET.parse(sys.argv[1])
root = cardsflow.getroot()

children = sorted(root, key=lambda child: int(child.attrib['name'].strip("motor")))

root.clear()

for c in children:
    root.append(c)

cardsflow.write(sys.argv[1] + '.ordered')




