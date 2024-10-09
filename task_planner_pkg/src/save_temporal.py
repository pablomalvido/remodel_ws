
def separate_cables(op, step2=0, config=[], route_group="", route_arm=""):
        global grasp_point_global
        global confirmation_received
        global confirmation_msg
        #global process_actionserver
        global holding_cables
        global force_limit_cable
        global holding_comeback_pose_corrected
        global grasping_cables
        global separated_global
        
        print("SEPARATE CABLES")

        if route_arm == "right":
                separation_arm = "left"
                separation_group = 'arm_left'
        else:
                separation_arm = "right"
                separation_group = 'arm_right'

        fingers_size = get_fingers_size(route_arm)
        fingers_size_sep = get_fingers_size(separation_arm)
        grasping_cables = False

        if step2 == 0:
                if not grasping_cables:
                        retract_arm(route_arm, op["spot"][0], config)
                        init_pose = get_current_pose(route_group).pose
                        mold_up_forward = get_shifted_pose(op["spot"][0]["pose_corner"], [op["spot"][0]['width']+config['grasp_offset']+fingers_size[0]/2, (op["spot"][0]['gap']/2), op["spot"][0]["height"] + 2*config['z_offset'], 0, 0, 0])
                        mold_forward = get_shifted_pose(op["spot"][0]["pose_corner"], [op["spot"][0]['width']+config['grasp_offset']+fingers_size[0]/2, (op["spot"][0]['gap']/2), 0, 0, 0, 0])
                        mold_up_forward_corrected = correctPose(mold_up_forward, route_arm, rotate = True, routing_app = True, ATC_sign = -1, secondary_frame = True)
                        mold_forward_corrected = correctPose(mold_forward, route_arm, rotate = True, routing_app = True, ATC_sign = -1, secondary_frame = True)
                        grasping_cables = True
                        waypoints0 = [init_pose, mold_up_forward_corrected, mold_forward_corrected]
                        plan, success, motion_group_plan = compute_cartesian_path_velocity_control_arms_occlusions([waypoints0], [config['speed_execution']], arm_side=route_arm)
                        print(success)
                        if success:
                                step2 += execute_plan_async(motion_group_plan, plan)
                                print(step2)
                else:
                        step2=1

        if step2 ==1:
                print("STEP2=1")
                actuate_grippers(config['slide_distance'], config['gripper_speed'], route_arm, config, grasp=False)
                init_pose = get_current_pose(route_group).pose
                mold_forward_slide = get_shifted_pose(op["spot"][0]["pose_corner"], [op["spot"][0]['width']+0.09, (op["spot"][0]['gap']/2), 0.005, 0, 0, 0])
                mold_forward_slide_corrected = correctPose(mold_forward_slide, route_arm, rotate = True, routing_app = True, ATC_sign = -1, secondary_frame = True)
                waypoints1 = [init_pose, mold_forward_slide_corrected]
                plan, success = compute_cartesian_path_velocity_control([waypoints1], [config['speed_execution']], arm_side=route_arm)
                if success:
                        step2 += execute_plan_async(route_group, plan)                                 


        if step2 == 2:
                os.system('cp ' + str(rospack.get_path('vision_pkg_full_demo')) + '/imgs/error_grasp_1.jpg /home/remodel/UI-REMODEL/src/assets/img/grasp_image.jpg')
                repeat = True
                pixel_D_param = 5
                while repeat:
                        if config['use_camera']:
                                img_cables_path = capture_img()
                                if not capture_success:
                                        stop_function("Failed to take the cables image")
                        else:
                                img_cables_path = str(rospack.get_path('vision_pkg_full_demo')) + '/imgs/Image_cables_wh1_386.jpg' #Image_cables_43

                        msg_log = String()
                        msg_log.data = "Calculating grasp point..."
                        logs_publisher.publish(msg_log)
                        separation_point, separation_point_success = compute_separation_pose(img_cables_path, op['WH'], op['label'], pixel_D_param, dist_next=30, forward=False)
                        grasp_point_global = [float(separation_point[0])/1000, float(separation_point[1])/1000]
                        msg_log.data = "Grasp point calculated"
                        logs_publisher.publish(msg_log)

                        repeat = False
                        img_name = img_cables_path.split('/')[-1]
                        confirm_msg = String()
                        confirm_msg.data = img_cables_path[:-len(img_name)]+'Grasp_point_'+img_name
                        confirmation_publisher.publish(confirm_msg)
                        while not confirmation_received: #Wait until the UI accepts
                                rospy.sleep(0.1)
                        
                        os.system('cp ' + str(rospack.get_path('vision_pkg_full_demo')) + '/imgs/error_grasp_1.jpg /home/remodel/UI-REMODEL/src/assets/img/grasp_image.jpg')
                        confirmation_received = False
                        if confirmation_msg == "N":
                                stop_function("Grasp canceled")
                        elif confirmation_msg == "R":
                                pixel_D_param += 1
                                repeat = True

                if separation_point_success:
                        actuate_grippers(config['open_distance'], config['gripper_speed'], separation_arm, config, grasp=False)
                        print(grasp_point_global)
                        init_pose = get_current_pose(separation_group).pose

                        # test_point = get_shifted_pose(op["spot"][0]["pose_corner"], [0.03, op["spot"][0]['gap']/2, op["spot"][0]["height"], 0, 0, 0])
                        # test_point_corrected = ATC1.correctPose(test_point, separation_arm, rotate = True, routing_app = False, ATC_sign = -1, secondary_frame = True)
                        # waypoints_test = [init_pose, test_point_corrected]
                        # plan, success, motion_group_plan = compute_cartesian_path_velocity_control_arms_occlusions([waypoints_test], [speed_execution], arm_side=separation_arm)
                        # if success:
                        #         execute_plan_async(motion_group_plan, plan)
                        # exit()

                        grasp_point = get_shifted_pose(op["spot"][0]["pose_corner"], [grasp_point_global[0], op["spot"][0]['gap']/2, op["spot"][0]["height"] + grasp_point_global[1], 0, 0, 0])
                        grasp_point_offset = get_shifted_pose(grasp_point, [0, 0, (config['z_offset']*2) - grasp_point_global[1], 0, 0, 0])
                        grasp_point_corrected = correctPose(grasp_point, separation_arm, rotate = True, routing_app = False, ATC_sign = -1, secondary_frame = True)
                        grasp_point_offset_corrected = correctPose(grasp_point_offset, separation_arm, rotate = True, routing_app = False, ATC_sign = -1, secondary_frame = True)                        
                        waypoints2 = [init_pose, grasp_point_offset_corrected, grasp_point_corrected]
                        plan, success = compute_cartesian_path_velocity_control([waypoints2], [config['speed_execution']], arm_side=separation_arm)
                if success:
                        step2 += execute_plan_async(separation_group, plan)  
                else:
                        stop_function("Grasp point determination failed")
                #exit()
                                
        if step2 == 3:
                actuate_grippers(config['slide_distance'], config['gripper_speed'], separation_arm, config, grasp=False)

                rospy.sleep(1)
                print("AA")
                retract_arm(route_arm, op["spot"][0], config, special=True)
                print("BB")
                grasping_cables = False
                rospy.sleep(0.5)

                init_pose = get_current_pose(separation_group).pose
                grasp_point_lift = get_shifted_pose(grasp_point_offset, [config['x_offset']+0.01, 0, config['z_offset']/2, 0, 0, 0])
                grasp_point_lift_corrected = correctPose(grasp_point_lift, separation_arm, rotate = True, routing_app = False, ATC_sign = -1, secondary_frame = True)
                waypoints3 = [init_pose, grasp_point_lift_corrected]
                plan, success, motion_group_plan = compute_cartesian_path_velocity_control_arms_occlusions([waypoints3], [config['speed_execution']], arm_side=separation_arm)
                print(success)
                if success:
                        step2 += execute_plan_async(motion_group_plan, plan)
                        print("CC")
                        print(step2)
                rospy.sleep(0.5)

        if step2 == 4:
                if config['use_camera']:
                        img_cables_path, capture_success = capture_img()
                        if not capture_success:
                                stop_function("Failed to take the cables image")
                else:
                        img_cables_path = str(rospack.get_path('vision_pkg_full_demo')) + '/imgs/Image_cables_wh1_387.jpg' #Image_cables_1

                msg_log = String()
                msg_log.data = "Evaluating cable separation..."
                logs_publisher.publish(msg_log)
                grasp_point_eval_mm = [int((2*config['z_offset'])*1000), int((config['x_offset']+grasp_point_global[0])*1000)]
                result_msg, separation_success, eval_success = check_cable_separation(img_path=img_cables_path, wh=op['WH'], sep_index=op['label'], pixel_D=8, grasp_point_eval_mm=grasp_point_eval_mm)
                msg_log.data = result_msg
                logs_publisher.publish(msg_log)
                print(result_msg)

                if eval_success and separation_success:
                        #step1 += 1
                        #step2 = 0
                        print("Successful cable separation")
                        #process_actionserver.publish_feedback()
                else:
                        if eval_success:
                                #stop_function("Cable separation was not succesful")
                                print("Cable separation was not succesful")
                        else:
                                #stop_function("Grasp evaluation failed")
                                print("Grasp evaluation failed")
                step2 = 5

        if step2 == 5:
                waypoints_SC4 = []
                init_pose = get_current_pose(separation_group).pose
                waypoints_SC4.append(init_pose)
                sep_guide_forward_up = get_shifted_pose(op["spot"][1]["pose_corner"], [op["spot"][1]["width"] + fingers_size[0], op["spot"][1]["gap"]/2, op["spot"][1]["height"] + config['z_offset'] + fingers_size[2]/2, 0, 0, 0])
                waypoints_SC4.append(correctPose(sep_guide_forward_up, separation_arm, rotate = True, routing_app = False, ATC_sign = -1))
                #plan, success = compute_cartesian_path_velocity_control([waypoints_SC4], [config['speed_execution']], arm_side=separation_arm)
                #if success:
                #        execute_plan_async(separation_group, plan)
                plan, success, motion_group_plan = compute_cartesian_path_velocity_control_arms_occlusions([waypoints_SC4], [config['speed_execution']], arm_side=separation_arm)
                if success:
                        step2 += execute_plan_async(motion_group_plan, plan)

        if step2 == 6:
                #Apply tension
                actuate_grippers(config['grasp_distance'], config['gripper_speed'], separation_arm, config, grasp=True)

                waypoints_SC5 = []
                init_pose = get_current_pose(separation_group).pose
                waypoints_SC5.append(init_pose)
                sep_guide_tension = get_shifted_pose(op["spot"][1]["pose_corner"], [op["spot"][1]["width"]  + fingers_size[0] + config['force_offset'], op["spot"][1]["gap"]/2, op["spot"][1]["height"] + config['z_offset'] + fingers_size[2]/2, 0, 0, 0])
                waypoints_SC5.append(correctPose(sep_guide_tension, separation_arm, rotate = True, routing_app = False, ATC_sign = -1))
                #Move with force_control
                plan, success = compute_cartesian_path_velocity_control([waypoints_SC5], [config['speed_tension']], arm_side=separation_arm)
                if success:
                        step2 += execute_force_control(group = separation_group, plan = plan, limit = force_limit_cable, force_active=config['force_control_active'])
                rospy.sleep(0.5)

        if step2 == 7:
                print("###########TEST CIRC###########")
                #Circ motion
                waypoints_SC6 = []
                init_pose = get_current_pose(separation_group).pose
                waypoints_SC6.append(copy.deepcopy(init_pose))
                init_pose_guides = antiCorrectPose(init_pose, separation_arm, routing_app = False)
                rot_center = get_shifted_pose(op["spot"][0]["pose_corner"],[op["spot"][0]['width']/2, op["spot"][0]['gap']/2, op["spot"][0]['height']/2, 0, 0, 0])
                insert_guide_center_sep = get_shifted_pose(op["spot"][1]["pose_corner"],[op["spot"][1]['width']/2, op["spot"][1]['gap']/2, op["spot"][1]['height']/2, 0, 0, 0])
                radius_rot = compute_distance(rot_center, init_pose_guides)
                #final_point = in the direction rot_center --> guide a distance of radius_rot
                final_cable_direction = get_axis(insert_guide_center_sep, rot_center)
                final_point = copy.deepcopy(insert_guide_center_sep) #Same orientation
                final_point.position.x = rot_center.position.x + final_cable_direction[0]*radius_rot
                final_point.position.y = rot_center.position.y + final_cable_direction[1]*radius_rot
                final_point.position.z = rot_center.position.z + final_cable_direction[2]*radius_rot #Maybe delete?

                #Evaluate collisions in final_point, if there are, there slide a bit more distance and evaluate again. Check also when the distance to the next guide is already too small, so then it will just do a slide_top
                #With the correct final_point, calculates the rotation knowing, center, initial and final arc points.

                guide_yaw_axis = get_axis_from_RM(pose_to_frame(rot_center).M, "Y")
                rot_axis = get_ort_axis(guide_yaw_axis, final_cable_direction)[2]
                rot_axis = [-element for element in rot_axis]
                dist_z = compute_distance_relative(final_point, init_pose_guides, guide_yaw_axis) #from circle to guide in z axis
                print(dist_z)
                print(radius_rot)
                print(init_pose)
                print(init_pose_guides)
                rot_degree = (math.asin(-dist_z/radius_rot))*180.0/math.pi

                center_waypoints = []
                circle_waypoints = []
                success, center_waypoints, circle_waypoints = circular_trajectory(rot_center, init_pose_guides, rot_degree, rot_axis, center_waypoints, circle_waypoints, step = 2, rot_gripper = False)
                for wp in circle_waypoints:
                        waypoints_SC6.append(correctPose(wp, separation_arm, rotate = True, ATC_sign = -1, routing_app = False))

                plan, success = compute_cartesian_path_velocity_control([waypoints_SC6], [config['speed_execution']], arm_side=separation_arm)
                if success:
                        step2 += execute_plan_async(separation_group, plan)
                print("###########TEST CIRC END###########")

        if step2 == 8:
                actuate_grippers(config['slide_distance'], config['gripper_speed'], separation_arm, config, grasp=False)
                waypoints_SC7 = []
                init_pose = get_current_pose(separation_group).pose
                waypoints_SC7.append(init_pose)
                if separation_arm=="right":
                        sign_side = 1
                else:
                        sign_side = -1
                side_pose = get_shifted_pose(op["spot"][1]["pose_corner"], [op["spot"][1]["width"] + fingers_size[0] + config['x_offset'], sign_side*0.1, 0, 0, 0, 0])
                waypoints_SC7.append(correctPose(side_pose, separation_arm, rotate = True, routing_app = False, ATC_sign = -1))
                plan, success, motion_group_plan = compute_cartesian_path_velocity_control_arms_occlusions([waypoints_SC7], [config['slow_speed_execution']], arm_side=separation_arm)
                if success:
                        step2 += execute_plan_async(motion_group_plan, plan)

        if step2 == 9:
                actuate_grippers(config['grasp_distance'], config['gripper_speed'], separation_arm, config, grasp=True)
                waypoints_SC8 = []
                init_pose = get_current_pose(separation_group).pose
                waypoints_SC8.append(init_pose)
                if separation_arm=="right":
                        sign_side = 1
                else:
                        sign_side = -1
                side_pose_tension = get_shifted_pose(op["spot"][1]["pose_corner"], [op["spot"][1]["width"] + fingers_size[0] + config['x_offset'], sign_side*(0.1 + config['force_offset']/2), 0, 0, 0, 0])
                waypoints_SC8.append(correctPose(side_pose_tension, separation_arm, rotate = True, routing_app = False, ATC_sign = -1))
                #Move with force_control
                plan, success = compute_cartesian_path_velocity_control([waypoints_SC8], [config['speed_tension']], arm_side=separation_arm)
                if success:
                        step2+=execute_force_control(group = separation_group, plan = plan, limit = force_limit_cable, force_active=config['force_control_active'])
                rospy.sleep(0.5)
                holding_cables = True
                holding_comeback_pose = get_shifted_pose(op["spot"][1]["pose_corner"],[op["spot"][1]["width"] + fingers_size[0] + config['x_offset'] + 0.05, sign_side*0.05, op["spot"][1]['height']/2, 0, 0, 1.57])
                holding_comeback_pose_corrected = correctPose(holding_comeback_pose, separation_arm, rotate = True, routing_app = False, ATC_sign = -1)

        if step2 == 10:
                #retract_arm(separation_arm, op["spot"][1], config)
                step2 = 0
                #process_actionserver.publish_feedback()

        return step2



        if ops.label[0][:2]=='WH':
                        #route cable group
                        ops_temp['label'] = ops.label[0]
                else:
                        conReq = connector_infoRequest()
                        conReq.label = ops.label[0]
                        conResult = my_service_con(conReq)
                        cablesGroup = []
                        for cable in conResult.cables:
                                cablesGroup.append(cable.label)
                        conReq.label = 'WH'+str(conResult.WH)
                        conResult = my_service_con(conReq)
                        cablesWH = {}
                        cablesGroupIndex = []
                        for cable in conResult.cables:
                                if cable.label in cablesGroup:
                                        cablesGroupIndex.append(int(cable.pins[0]))
                                cablesWH[cable.label]=int(cable.pins[0])
                        if max(cablesGroupIndex) < (len(cablesWH)-1):
                                #separate upper cables and place them in another guide
                                sub_op_temp = {}
                                sub_op_temp['type'] = 'SC'
                                sub_op_temp['WH'] = str(conResult.WH)
                                sub_op_temp['label'] = list(range(max(cablesGroupIndex)+1, len(cablesWH)))
                                sub_op_temp['spot'] = []
                                sub_op_temp['spot'].append(last_PC_spot)
                                guideReq = guide_infoRequest()
                                guideReq.jig = 'J'+str(conResult.WH)+'S' #DEFINE THIS ELEMENT
                                guideReq.guide = '1'
                                guideResult = my_service(guideReq)
                                guide = guideResult.data
                                sub_op_temp['spot'].append({'pose_corner': guide.key_corner_frame, 'gap': guide.key_gap, 'width': guide.key_length, 'height': guide.key_height, 'height_corner': guide.key_height_corner, 'collisions': guide.collisions, 'dimensions': guide.dimensions})
                                ops_info.append(sub_op_temp)
                                ops_info_text.append('Separate cables: ' + str(sub_op_temp['label']) + " in J" + str(conResult.WH) + "S")
                                #grasp cable group from the first route position
                                sub_op_temp = {}
                                sub_op_temp['type'] = 'GC'
                                sub_op_temp['label'] = ops.label[0]
                                #sub_op_temp['spot'] = ops_temp['spot'][0]
                                sub_op_temp['spot'] = last_PC_spot
                                ops_info.append(sub_op_temp)
                                ops_info_text.append('Grasp cable group ' + str(ops.label[0]) + " from " + str(sub_op_temp['spot']['jig']))
                                #route cable group
                                ops_temp['label'] = ops.label[0]
                        else:
                                #grasp cable group from the separation guide
                                sub_op_temp = {}
                                sub_op_temp['type'] = 'GCS'
                                sub_op_temp['label'] = ops.label[0]
                                guideReq = guide_infoRequest()
                                guideReq.jig = 'J'+str(conResult.WH)+'S'
                                guideReq.guide = '1'
                                guideResult = my_service(guideReq)
                                guide = guideResult.data
                                sub_op_temp['spot']={'pose_corner': guide.key_corner_frame, 'gap': guide.key_gap, 'width': guide.key_length, 'height': guide.key_height, 'height_corner': guide.key_height_corner, 'collisions': guide.collisions, 'dimensions': guide.dimensions}
                                ops_info.append(sub_op_temp)
                                ops_info_text.append('Grasp cable group ' + str(ops.label[0]) + " from J" + str(conResult.WH) + "S")
                                #route cable group
                                ops_temp['label'] = ops.label[0]