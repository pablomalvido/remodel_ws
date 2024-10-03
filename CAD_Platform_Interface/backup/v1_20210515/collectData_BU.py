import os
import xml.etree.ElementTree as ET
import xmltodict
import PyKDL
import numpy as np
import csv

class InputFilesDataCollector(object): 

  def __init__(self, folder_path, wri_file, jigs_file, components_file, WH_file, seq_file):
    self.folder_path = folder_path 
    self.platform_file_path = os.path.join(folder_path, wri_file)
    self.dict_platform = self._createPlatformDict(self.platform_file_path)
     
    self.jigs_file_path = os.path.join(folder_path, jigs_file)
    self.dict_jigs = self._createJigsDict(self.jigs_file_path)

    self.components_file_path = os.path.join(folder_path, components_file)
    self.dict_components = self._createComponentsDict(self.components_file_path)

    self.WH_file_path = os.path.join(folder_path, WH_file)
    self.dict_WH = self._createWHDict(self.WH_file_path)

    self.seq_file_path = os.path.join(folder_path, seq_file)
    self.list_seq = self._createSequenceList(self.seq_file_path)

    #print("Working...") 

  def _createPlatformDict(self, path):
    f = open(path,'r')
    row = len(f.readlines())
    f.seek(0)
    dic = {}
    for index in range(1, row + 1):
      #print(f.readline())
      cad, label, product = map(str, f.readline().split('\t'))
      dic[cad] = {"label": label, "product": product.rstrip()}
    #print(dic)
    return(dic)

  def _createJigsDict(self, path):
    with open(path) as fd: 
      doc = xmltodict.parse(fd.read())
    tree = ET.parse(path)
    root = tree.getroot()
    main_dic = {} 

    for code in root: 
      if code.tag=="jig":
        dic = {} 
        couples = {}
        dic["type"] = "jig"
        dic["xdim"] = float(code.get('xdim'))/1000
        dic["ydim"] = float(code.get('ydim'))/1000
        dic["zdim"] = float(code.get('zdim'))/1000

        for guide in code.findall('guide'): 
          key_dict = {}
          colli_dict={}
          tape_spots = {}
          for key in guide.findall('key'):
            for pos in key.findall('pos'):
              if pos.get('name') == 'x': 
                x = float(pos.text)/1000
              if pos.get('name') == 'y': 
                y = float(pos.text)/1000
              if pos.get('name') == 'z': 
                z = float(pos.text)/1000
            for rot in key.findall('rot'): 
              if rot.get('name') == 'R': 
                R = float(rot.text) 
              if rot.get('name') == 'P': 
                P = float(rot.text) 
              if rot.get('name') == 'Y': 
                Y = float(rot.text) 
            tf = PyKDL.Frame() 
            tf.p = PyKDL.Vector(x, y, z) 
            tf.M.DoRotX(R) 
            tf.M.DoRotY(P) 
            tf.M.DoRotZ(Y)
            size = []
            size = [float(key.get('length'))/1000, float(key.get('gap'))/1000, float(key.get('height'))/1000]
            tf2 = PyKDL.Frame()
            tf2.p = PyKDL.Vector((size[0])/2, (size[1])/2, (size[2])/2)
            tf2.M.DoRotX(0) 
            tf2.M.DoRotY(0) 
            tf2.M.DoRotZ(0)
            center_pose = tf*tf2 #Frame in the center of the gap of the guide 
            key_dict = {'frame':tf, 'center_pose':center_pose, 'length': size[0], 'gap': size[1], 'height': size[2]}

          for colli in guide.findall('collision'):
            for pos in colli.findall('pos'):
              if pos.get('name') == 'x': 
                x = float(pos.text)/1000
              if pos.get('name') == 'y': 
                y = float(pos.text)/1000
              if pos.get('name') == 'z': 
                z = float(pos.text)/1000
            for rot in colli.findall('rot'): 
              if rot.get('name') == 'R': 
                R = float(rot.text) 
              if rot.get('name') == 'P': 
                P = float(rot.text) 
              if rot.get('name') == 'Y': 
                Y = float(rot.text) 
            tf = PyKDL.Frame() 
            tf.p = PyKDL.Vector(x, y, z) 
            tf.M.DoRotX(R) 
            tf.M.DoRotY(P) 
            tf.M.DoRotZ(Y)
            colli_dict = {'frame':tf, 'xdim': float(colli.get('xdim'))/1000, 'ydim': float(colli.get('ydim'))/1000, 'zdim': float(colli.get('zdim'))/1000}

          couples[guide.get('couple')] = {'key': key_dict, 'collision': colli_dict}
        
        dic["guides"] = couples

        for tape in code.findall('tape_spot'): 
          for pos in tape.findall('pos'):
            if pos.get('name') == 'x': 
              x = float(pos.text)/1000
            if pos.get('name') == 'y': 
              y = float(pos.text)/1000
            if pos.get('name') == 'z': 
              z = float(pos.text)/1000
          for rot in tape.findall('rot'): 
            if rot.get('name') == 'R': 
              R = float(rot.text) 
            if rot.get('name') == 'P': 
              P = float(rot.text) 
            if rot.get('name') == 'Y': 
              Y = float(rot.text) 
          tf = PyKDL.Frame() 
          tf.p = PyKDL.Vector(x, y, z) 
          tf.M.DoRotX(R) 
          tf.M.DoRotY(P) 
          tf.M.DoRotZ(Y)
    #Frame of the guide that is going to align the cables in this taping spot
          tf_guide = PyKDL.Frame()
          tf_guide = dic['guides'][tape.get('guide')]['key']['center_pose']
          #Calculations for getting the position of the cables in the taping spot in Y axis
          dist = (((tf_guide.p[0]-tf.p[0])**2)+((tf_guide.p[1]-tf.p[1])**2))**0.5
          angle = np.arctan((abs(tf_guide.p[1]-tf.p[1]))/(abs(tf_guide.p[0]-tf.p[0])))
          angle_dif = 1.57 - Y - angle
          min_dist = dist*np.cos(angle_dif)
          tf2 = PyKDL.Frame()
          tf2.p = PyKDL.Vector(((float(tape.get('xdim')))/2)/1000, min_dist, (tf_guide.p[2])-z)
          tf2.M.DoRotX(0) 
          tf2.M.DoRotY(0) 
          tf2.M.DoRotZ(0)
          center_pose = tf*tf2 #Frame in the middle of the taping spot (x axis) aligned with the cables (y axis)
          tape_spots[tape.get('id')] = {'frame':tf, 'center_pose':center_pose, 'xdim': float(tape.get('xdim'))/1000, 'ydim': float(tape.get('ydim'))/1000, 'zdim': float(tape.get('zdim'))/1000}

        dic["tape_spots"] = tape_spots
        
        main_dic[code.get("model")] = dic

      elif code.tag=="box":
        dic = {} 
        dic["type"] = "box"
        dic["xdim"] = float(code.get('xdim'))/1000
        dic["ydim"] = float(code.get('ydim'))/1000
        dic["zdim"] = float(code.get('zdim'))/1000
	dic["trays"] = {}

        for tray in code.findall('tray'): 
          tray_dict = {}
          for pos in tray.findall('pos'):
            if pos.get('name') == 'x': 
              x = float(pos.text)/1000
            if pos.get('name') == 'y': 
              y = float(pos.text)/1000
            if pos.get('name') == 'z': 
              z = float(pos.text)/1000
          for rot in tray.findall('rot'): 
            if rot.get('name') == 'R': 
              R = float(rot.text) 
            if rot.get('name') == 'P': 
              P = float(rot.text) 
            if rot.get('name') == 'Y': 
              Y = float(rot.text) 
          tf = PyKDL.Frame() 
          tf.p = PyKDL.Vector(x, y, z) 
          tf.M.DoRotX(R) 
          tf.M.DoRotY(P) 
          tf.M.DoRotZ(Y)
          size = []
          size = [float(tray.get('xdim'))/1000, float(tray.get('ydim'))/1000, float(tray.get('zdim'))/1000]
          tf2 = PyKDL.Frame()
          tf2.p = PyKDL.Vector((size[0])/2, (size[1])/2, 0)
          tf2.M.DoRotX(0) 
          tf2.M.DoRotY(0) 
          tf2.M.DoRotZ(0)
          center_pose = tf*tf2 #Frame in the center of the tray 
          tray_dict = {'frame':tf, 'center_pose':center_pose, 'xdim': size[0], 'ydim': size[1], 'zdim': size[2]}
          dic["trays"][tray.get('id')] = tray_dict

        main_dic[code.get("model")] = dic

    return main_dic

  def _createComponentsDict(self, path):
    main_dict = {}
    con_dict = {}
    cab_dict = {}
    dev_dict = {}

    csv.register_dialect('components', delimiter=';') 

    with open(path, 'rb') as f: 
      reader = csv.reader(f, dialect='components') 

      for row in reader: 
        if ((row[0].upper()) == "CONNECTOR"):
          temp_dict = {}
          temp_dict['model'] = row[1]
          temp_dict['reference'] = row[2]
          temp_dict['color'] = row[4] #RGB?
          temp_dict['xdim'] = float(row[7])
          temp_dict['ydim'] = float(row[8])
          temp_dict['zdim'] = float(row[9])
          con_dict[row[3]] = temp_dict
        
        elif ((row[0].upper())=="CABLE"):
          temp_dict = {}
          temp_dict['color'] = row[4] #RGB?
          temp_dict['length'] = float(row[5])
          temp_dict['diameter'] = float(row[6])
          cab_dict[row[3]] = temp_dict
        
        elif ((row[0].upper())=="DEVICE"):
          temp_dict = {}
          temp_dict['type'] = row[10]
          temp_dict['model'] = row[1]
          temp_dict['color'] = row[4] #RGB?
          dev_dict[row[3]] = temp_dict
      
      main_dict['connector'] = con_dict
      main_dict['cable'] = cab_dict
      main_dict['device'] = dev_dict
    
    return main_dict

  def _createWHDict(self, path):
    with open(path) as fd: 
      doc = xmltodict.parse(fd.read())
    tree = ET.parse(path)
    root = tree.getroot()
    main_dict = {} 

    for WH in root: 
      WH_dict = {} 
      box_tray = []
      box_tray = WH.get("tray").split(".")
      box = box_tray[0]
      if len(box_tray)>1:
        tray = box_tray[1]
      else:
        tray = "1"

      WH_dict['box'] = box
      WH_dict['tray'] = tray
      WH_dict['first_con'] = WH.get('first_connector')
      branch_dict = {}

      for branch in WH.findall('end_connector'):
        cables_dict = {}
        for cable in branch.findall('cable'): 
          first_pins=[]
          end_pins=[]
          if 'Row1' in cable.attrib and 'Col1' in cable.attrib: 
            first_pins = [cable.get('Row1'), cable.get('Col1')]
          if 'Row2' in cable.attrib and 'Col2' in cable.attrib: 
            end_pins = [cable.get('Row2'), cable.get('Col2')]  
          cables_dict[cable.text] = {'first_pins':first_pins, 'end_pins':end_pins}
        branch_dict[branch.get('label')] = cables_dict
      WH_dict['end_con'] = branch_dict
      main_dict[WH.get('id')] = WH_dict
    return main_dict

  def _createSequenceList(self, path):
    main_list = []

    csv.register_dialect('sequence', delimiter=';') 

    with open(path, 'rb') as f: 
      reader = csv.reader(f, dialect='sequence') 

      for row in reader: 
        temp_dict = {}
        
        if ((row[1].upper()) == "RC"): #Routing Cables
          temp_dict['operation'] = row[1]
          temp_dict['label'] = row[2].split("-")
          temp_spot = row[3].split("-")
          #temp_couple_dict = {}
          temp_couple_list = []
          for spots in temp_spot:
            jig, couple = map(str, spots.split("."))
            #if(!(jig in temp_couple_dict):
              #temp_couple_dict[jig] = []
            #temp_couple_dict[jig].append(couple)
            temp_couple_list.append({'jig': jig, 'couple': couple})
          #temp_dict['spot'] = temp_couple_dict
          temp_dict['spot'] = temp_couple_list
          main_list.append(temp_dict)

        elif ((row[1].upper()) == "PC"): #Placing Connector
          temp_dict['operation'] = row[1]
          temp_dict['label'] = row[2].split("-")
          jig, couple, side = map(str, row[3].split("."))
	  temp_dict['spot'] = []
          temp_dict['spot'].append({'jig': jig, 'couple': couple, 'side': side})
          main_list.append(temp_dict)

        elif ((row[1].upper()) == "T"): #Taping
          temp_dict['operation'] = row[1]
          temp_dict['label'] = row[2].split("-")
          jig, tape_spot = map(str, row[3].split("."))
	  temp_dict['spot'] = []
          temp_dict['spot'].append({'jig': jig, 'tape_spot': tape_spot[2:]})
          main_list.append(temp_dict)
      
    return main_list

  def showInfo(self):
    """This method prints all the information extracted from the input files"""
    print(self.dict_platform)
    print('')
    print(self.dict_jigs)
    print('')
    print(self.dict_components)
    print('')
    print(self.dict_WH)
    print('')
    print(self.list_seq)

"""
dict_ELVEZ = InputFilesDataCollector('/home/rmpama/files_python/','platform_ids.wri', 'Jigs_definition_v2.xml', 'Components_definition.csv', 'WH_configuration.xml', 'Assembly_sequence.csv')
print(dict_ELVEZ.dict_platform)
print('')
print(dict_ELVEZ.dict_jigs)
#print('')
#print(dict_ELVEZ.dict_components)
#print('')
#print(dict_ELVEZ.dict_WH)
#print('')
#print(dict_ELVEZ.list_seq)
"""

