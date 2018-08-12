import xml.etree.ElementTree as ET
import sys, os
from anytree import Node, RenderTree

dest_dir = ""

def ensure_dirs(file_path, subpath):
    directory = os.path.dirname(file_path)
    if not os.path.exists(directory):
        os.makedirs(directory)
    # ensure subpath exists
    if not os.path.exists(directory + subpath):
        os.makedirs(directory + subpath)


def warn_missing_file(relativepath):
    if not os.path.exists(dest_dir +  "/" + relativepath):
        #print("Warning: " + relativepath.split('/')[-1] + " not found.")
        print("Warning: " + dest_dir + "/" + relativepath + " not found.")


def main(out=None):
    # Get the total number of args passed to the demo.py
    global dest_dir
    color = [200, 200, 200]
    subpath = ""
    if(len(sys.argv) < 3):
        print("Expected at least 2 args: sdf in, xml out.\nAdditional options: obj subpath (relative) and obj color. Mind the order!")
        return

    if len(sys.argv) >= 6:
        for i in range(3):
            try:
                var = int(sys.argv[i +4])
                if var >= 0 and var < 255:
                    color[i] = var
                else:
                    print("Wrong color code. Expected 3 int numbers elem [0,255]. Aborting.")
                    return
            except ValueError:
                print("Wrong color code. Expected 3 int numbers elem [0,255], received no ints. Aborting.")
                return
    if len(sys.argv) == 7 or len(sys.argv) == 4:
        subpath = sys.argv[3]
    sdftree = ET.parse(sys.argv[1])
    if not sdftree:
        print("Couldn't parse given sdf file!")
        return

    root = sdftree.getroot()
    links = dict()
    tree = dict()
    joints = dict()
    rootList = []
    destination = sys.argv[2]
    dest_dir = os.path.dirname(destination)

    for link in root.find('model').findall('link'):
        links.update(handleLink(link, ""))
        rootList.append(link.get('name'))

    for joint in root.find('model').findall('joint'):
        joints.update(handleJoint(joint))
        child = joint.find('child').text
        parent = joint.find('parent').text

        if  parent not in tree:
            tree.update({parent: Node(parent)})

        if child not in tree:
            tree.update({child: Node(child, parent=tree[parent])})
        else:
            # If parent was previously created which now is a child
            tree[child].parent = tree[parent]

        #slowly find root by eliminating children
        rootList.remove(child)

    rootname = rootList[0]

    ensure_dirs(destination, subpath)
    f = open(destination, 'w+')
    printToXML(tree[rootname], joints, links, f, color, subpath)
    f.close()
    print("Successfully parsed SDF to XML.")


# not racing condition proof


def printToXML(root, joints, links, f, color=(125,125,125), path=''):
    start = "<?xml version=\"1.0\" ?>"
    model = "<model version=\"1\">"

    f.write(start + "\n")
    f.write(model + "\n")
    for child in root.children:
        printJoint("\t", joints[child.name], f)
        printChild(child, '\t\t', joints, links, f, color, path)
        f.write("\t" + "</frame>" + "\n")
    f.write("</model>" + "\n")


# merely prints every entry
def printChild(entry, tabs, joints, links, f, color, path):
    printLink(tabs, links[entry.name], f, color, path)
    for child in entry.children:
        printJoint(tabs, joints[child.name], f)
        printChild(child, tabs + '\t', joints, links, f, color, path)
        f.write(tabs + "</frame>" + "\n")


def printJoint(tabs, joint, f):
    # pos, rot, axis, geometry
    name = tabs + "<frame jointName=\"" + joint[0] + "\" "
    name += "jointType=\"" + joint[5].replace('revolute', 'rotational') + "\" "
    name += "jointMin=\"" + joint[3] + "\" jointMax=\"" + joint[4] + "\">"
    position = tabs + "\t<position x=\"" + joint[1][0] + "\" " + "y=\"" + joint[1][1] \
              + "\" " + "z=\"" + joint[1][2] + "\" />"
    orientation = tabs + "\t<orientation x=\"" + joint[1][3] + "\" " + "y=\"" + joint[1][4] \
              + "\" " + "z=\"" + joint[1][5] + "\" />"

    axis = tabs + "\t<axis x=\"" + joint[2][0] + "\" " + "y=\"" + joint[2][1] + "\" " + "z=\"" + joint[2][2] + "\" />"

    f.write(name + "\n")
    f.write(position + "\n")
    f.write(orientation + "\n")
    f.write(axis + "\n")


def printLink(tabs, link, f, color, path):
    name = tabs + "<geom type=\"mesh\" "
    name += "sx=\"" + link[2][0] + "\" " + "sy=\"" + link[2][1] + "\" " + "sz=\"" + link[2][2] + "\" "
    name += "tx=\"" + link[0][0] + "\" " + "ty=\"" + link[0][1] + "\" " + "tz=\"" + link[0][2] + "\" "
    name += "rx=\"" + link[0][3] + "\" " + "ry=\"" + link[0][4] + "\" " + "rz=\"" + link[0][5] + "\" "
    name += "red=\"" + str(color[0]) + "\" blue=\"" + str(color[1]) + "\" green = \"" + str(color[2]) + "\" "
    name += "meshFile=\"" + path + link[1] + "\" />"
    f.write(name + "\n")
    warn_missing_file(path + link[1])


# Returns a dictionary entry consisting of name: pose, axis, lower_limit, upper_limit, type
def handleJoint(joint):
    pose = joint.find('pose').text.split(' ')
    axis = joint.find('axis')
    axis_coords = axis.find('xyz').text.split(' ')
    axis_lower = axis.find('limit').find('lower').text
    axis_upper = axis.find('limit').find('upper').text
    child = joint.find('child').text
    type = joint.attrib['type']
    return {child: (joint.get('name'), pose, axis_coords, axis_lower, axis_upper, type)}


# Returns a dictionary entry consisting of child_name: jointname, pose, mesh-file (not path), mesh scale
def handleLink(link, tabs):
    pose = link.find('pose').text.split(' ')
    mesh = link.find('collision').find('geometry').find('mesh')
    meshfile = mesh.find('uri').text.replace('model://', '').replace('.stl', '.obj').split('/')[-1]
    meshscale = mesh.find('scale').text.split(' ')
    return {link.get('name'): (pose, meshfile, meshscale)}


if __name__ == "__main__":
    main()