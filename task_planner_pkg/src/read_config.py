import csv

def read_config(real_robot, config_file_name):
    config1 = {}
    config2 = {}
    try:
            with open(config_file_name) as csvfile:
                    reader = csv.DictReader(csvfile)
                    for row in reader:
                            config1[row['prop']] = row['value']
    except:
            print("Cannot access to the configuration params")
            exit()

    print(config1)
    execute_grippers=False
    use_camera=False

    try:
            if real_robot:
                    print('REAL ROBOT')
                    if config1['grippers_control_real'] == 'Y' or config1['grippers_control_real'] == 'y':
                            config2['execute_grippers'] = True #True
                    else:
                            config2['execute_grippers'] = False
                    if config1['gun_control_real'] == 'Y' or config1['gun_control_real'] == 'y':
                            config2['execute_gun'] = True #True
                    else:
                            config2['execute_gun'] = False
                    if config1['force_control_real'] == 'Y' or config1['force_control_real'] == 'y':
                            config2['force_control_active'] = True #True
                    else:
                            config2['force_control_active'] = False
                    if config1['use_camera_real'] == 'Y' or config1['use_camera_real'] == 'y':
                            config2['use_camera'] = True #True
                    else:
                            config2['use_camera'] = False
                            print(execute_grippers)
                            print(use_camera)
                    config2['speed_limit'] = min(float(config1['speed_real_per']),0.3) #0.05
                    config2['fast_speed_execution'] = min(float(config1['speed_fast_real_mms']),100) #40mm/s
                    config2['speed_execution'] = min(float(config1['speed_real_mms']),80) #20mm/s
                    config2['slow_speed_execution'] = min(float(config1['speed_slow_real_mms']),40) #10mm/s
                    config2['speed_tension'] = min(float(config1['speed_tension_real_mms']),40) #10mm/s
            else:
                    print('SIMULATED ROBOT')
                    if config1['grippers_control_demo'] == 'Y' or config1['grippers_control_demo'] == 'y':
                            config2['execute_grippers'] = True #False
                    else:
                            config2['execute_grippers'] = False
                    if config1['use_camera_demo'] == 'Y' or config1['use_camera_demo'] == 'y':
                            config2['use_camera'] = True #False
                    else:
                            config2['use_camera'] = False
                    config2['force_control_active'] = False
                    config2['execute_gun'] = False
                    config2['speed_limit'] = float(config1['speed_demo_per']) #1
                    config2['fast_speed_execution'] = float(config1['speed_fast_demo_mms']) #100mm/s
                    config2['speed_execution'] = float(config1['speed_demo_mms']) #50mm/s
                    config2['slow_speed_execution'] = float(config1['speed_slow_demo_mms']) #30mm/s
                    config2['speed_tension'] = float(config1['speed_tension_demo_mms']) #20mm/s
            
            #Offsets
            config2['z_offset'] = float(config1['offset_z'])/1000 #0.02
            config2['x_offset'] = float(config1['offset_x'])/1000 #0.02
            config2['grasp_offset'] = float(config1['offset_grasp'])/1000 #0.005
            config2['force_offset'] = float(config1['offset_force'])/1000 #0.01
            #pick_grasp_offset = float(config1['offset_pick_grasp'])/1000 #0.015
            pick_grasp_offset = {}
            pick_grasp_offset['WH1'] = 0.015
            pick_grasp_offset['WH2'] = 0.015
            pick_grasp_offset['WH3'] = 0.0255
            config2['pick_grasp_offset'] = pick_grasp_offset
            config2['z_offset_pick'] = float(config1['offset_pick_z'])/1000 #0.05
            config2['z_offset_photo'] = float(config1['offset_photo_z'])/1000 #0.1
            config2['gun_nozzle_offset'] = 0.1

            #Force sensor
            force_cable = {}
            force_connector = {}
            force_cable['WH1'] = float(config1['cable_tension_wh1']) #3.5N
            force_cable['WH2'] = float(config1['cable_tension_wh1']) #3.5N
            force_cable['WH3'] = float(config1['cable_tension_wh3']) #3.5N
            force_connector['WH1'] = float(config1['connector_tension_wh1']) #5N
            force_connector['WH2'] = float(config1['connector_tension_wh1']) #5N
            force_connector['WH3'] = float(config1['connector_tension_wh3']) #5N
            config2['force_cable'] = force_cable
            config2['force_connector'] = force_connector
            force_limit_connector = 0
            force_limit_cable = 0

            #Gripper parameters
            config2['open_distance'] = float(config1['gripper_open_dist']) #105
            config2['slide_distance'] = float(config1['gripper_slide_dist']) #36
            config2['grasp_distance'] = float(config1['gripper_grasp_dist']) #30
            config2['gripper_speed'] = float(config1['gripper_speed']) #30
            config2['gripper_speed_slow'] = float(config1['gripper_speed_slow']) #20
    except:
            print("Error defining config robot values")
            exit()
    return config2