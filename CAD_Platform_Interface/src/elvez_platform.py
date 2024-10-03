import os
import PyKDL
import xml.etree.ElementTree as ET
import numpy as np
import math
from PyKDL import Frame

class ItemID(object): 
    """Class that saves the different elements of the ELVEZ platform with its CAD ID, its label and its commercial number"""
    def __init__(self, cad_id, label='', commercial=''): 
        self.id_list = [cad_id, label, commercial] 
 
    def getCadID(self): 
        if len(self.id_list) > 0: 
            return self.id_list[0] 
        return None 
 
    def getLabel(self): 
        if len(self.id_list) > 1: 
            return self.id_list[1] 
        return None 
 
    def getCommercialID(self): 
        if len(self.id_list) > 2: 
            return self.id_list[2] 
        return None 
 
    def getType(self): 
        label = self.getLabel()
        #Table
        if(label[0]=='T'):
            return 0
        #Jig
        elif(label[0]=='J'):
            return 1
        #Box
        if(label[0]=='B'):
            return 2
        #Comb
        if(label[0]=='C'):
            return 3
        #ATC
        if(label[0]=='A'):
            return 4
 

class Transform(PyKDL.Frame):
    def __init__(self, node, scale, file_name, parent=None):
        super(PyKDL.Frame, self).__init__() 
        self.node = node 
        self.scale = scale
        self.file_name = file_name
        #self.shape
        self.children_transforms = []
        self.id_string = ''
        self.item_id = None
        self.parent = parent
        self.translation = np.array([0.0, 0.0, 0.0])
        self.rotation = np.array([1.0, 0.0, 0.0, 0.0])
        self.parseAttributes(node) 
        for element in self.node:
            if element.tag.lower() == 'transform':
                self.children_transforms.append(Transform(element, self.scale, file_name, self)) 

    def parseAttributes(self, node):  
        """ Saves the attributes of the transform: translation, rotation and ID """
        attrib = node.attrib 
        if 'scale' in attrib: 
            self.scale_current = np.array(map(float, attrib['scale'].split(' ')))
            self.scale = self.scale * self.scale_current
        if 'translation' in attrib: 
            self.translation = np.array(map(float, attrib['translation'].split(' '))) 
            self.p_mm = PyKDL.Vector(
                float(self.translation[0]), 
                float(self.translation[1]), 
                float(self.translation[2])
            )
            self.p = PyKDL.Vector(
                (float(self.translation[0]))*self.scale[0], 
                (float(self.translation[1]))*self.scale[1], 
                (float(self.translation[2]))*self.scale[2]
            )
        if 'rotation' in attrib: 
            self.rotation = np.array(map(float, attrib['rotation'].split(' ')))
            quat = self.quaternion_about_axis(self.rotation[3], self.rotation[0:3]) 
            self.M = PyKDL.Rotation.Quaternion(quat[0], quat[1], quat[2], quat[3])
        if 'DEF' in attrib: 
            self.id_string = attrib['DEF']

    def quaternion_about_axis(self, angle, axis):
        """ Method for creating quaternions """
        qx = axis[0] * math.sin(angle / 2) 
        qy = axis[1] * math.sin(angle / 2) 
        qz = axis[2] * math.sin(angle / 2) 
        qw = math.cos(angle / 2) 
        return np.array([qx, qy, qz, qw])

    def isUseful(self): 
        """ The transform has an ID """
        return self.item_id != None 

    def deepItemIDMapping(self, id_map):
        """ Maps each transform with its item_ID object """
        if self.id_string in id_map: 
            self.item_id = id_map[self.id_string] 
        for transform in self.children_transforms: 
            transform.deepItemIDMapping(id_map)

    def getID(self): 
        return self.item_id

    def deepSearch(self, transform_list): 
        """ Search for all useful Transform recursively """ 
        if self.isUseful(): 
            transform_list.append(self) 
        for transform in self.children_transforms: 
            transform.deepSearch(transform_list)

    def getRoot(self): 
        """ Iterative search for Root Transform """ 
        if self.parent != None: 
            if self.parent.isUseful(): 
                return self.parent.getRoot() 
            else: 
                return self 
        else: 
            return self

    def getName(self): 
        if self.isUseful(): 
            return self.item_id.getCadID() + self.file_name
        else: 
            return "NONAME"

    def getCommercial(self): 
        if self.isUseful(): 
            return self.item_id.getCommercialID() 
        else: 
            return "NOCOMMERCIAL"

    def getLabel(self): 
        if self.isUseful(): 
            return self.item_id.getLabel()
        else: 
            return "NOLABEL"


class Scene(Transform):
    def __init__(self, node, id_map, file_name):
        self.id_map = id_map
        self.scale = np.array([1.0, 1.0, 1.0])
        super(Scene, self).__init__(node, self.scale, file_name, None) 
        self.deepItemIDMapping(id_map)

    @staticmethod
    def buildScene(x3d_file, wri_file, file_name):
        tree = ET.parse(x3d_file)
        root = tree.getroot()
        id_map = {}
        with open(wri_file) as rfile:
            content = rfile.readlines()
            for line in content:
                chunks = line.split('\t')
                print(chunks)
                for line in content:
                    id_map[chunks[0].strip()] = ItemID(chunks[0].strip(), chunks[1].strip(), chunks[2].strip())
        for child in root:
            if child.tag.lower() == 'scene':
                return Scene(child, id_map, file_name)


class Platform(object):
    def __init__(self, name, folder):
        self.name = name
        self.folder = folder
        self.files = self.retrieveFilesNames()
	#The following line creates an scene and all the transforms (one for each component of the ELVEZ platform)
        self.scene = Scene.buildScene(self.files['cad'], self.files['ids'], name)
        self.useful_transforms = []
        self.scene.deepSearch(self.useful_transforms)

    def retrieveFilesNames(self):
        file_map = { 
            "cad": os.path.join(self.folder, self.getFileNameForCad()), 
            "ids": os.path.join(self.folder, self.getFileNameForIds()),
        }
        return file_map

    def getFileNameForCad(self): 
        return self.name + "_cad.x3d"   #self.name = combs or platform
 
    def getFileNameForIds(self): 
        return self.name + "_ids.wri"    #combs = combs or platform

"""
############### MAIN PRUEBAS ########
plat_ELVEZ = Platform('platform', '/home/rmpama/files_python/')
transforms = plat_ELVEZ.useful_transforms
for trans in transforms:
    if trans.getID().getType() == 0:
        print("TABLE")
        print(trans.getID().getLabel())
        print(trans.getID().getCommercialID())
        print(trans.scale)
        print(trans.scale_current)
        print(trans.p)
        print(trans.p_mm)
    if trans.getID().getType() == 1:
        print("JIG")
        print(trans.getID().getLabel())
        print(trans.getID().getCommercialID())
        print(trans.scale)
        print(trans.scale_current)
        print(trans.p)
        print(trans.p_mm)
    if trans.getID().getType() == 2:
        print("BOX")
        print(trans.getID().getLabel())
        print(trans.getID().getCommercialID())
        print(trans.scale)
        print(trans.scale_current)
        print(trans.p)
        print(trans.p_mm)
"""
