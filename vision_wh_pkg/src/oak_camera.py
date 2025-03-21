#!/usr/bin/env python3

import cv2
import depthai as dai
import time
import sys
import os
from os import listdir
import numpy as np
import rospy
from std_srvs.srv import Trigger, TriggerResponse
from sensor_msgs.msg import CompressedImage

rospy.init_node('service_OAK_capture', anonymous=True)
service_name = '/OAK/capture_img'
image_pub = rospy.Publisher("/OAK/stream_compressed", CompressedImage)
flip = True

# Create pipeline
pipeline = dai.Pipeline()

# Define source and output
camRgb = pipeline.create(dai.node.ColorCamera)
xoutVideo = pipeline.create(dai.node.XLinkOut)

camRgb.initialControl.setManualFocus(155) #155
# This may be redundant when setManualFocus is used
camRgb.initialControl.setAutoFocusMode(dai.RawCameraControl.AutoFocusMode.OFF)

xoutVideo.setStreamName("video")

# Properties
camRgb.setBoardSocket(dai.CameraBoardSocket.RGB)
camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
camRgb.setVideoSize(1920, 1080)
#print("FPS: " + str(camRgb.getFps()))
camRgb.setFps(20)

xoutVideo.input.setBlocking(False)
xoutVideo.input.setQueueSize(1)

# Linking
camRgb.video.link(xoutVideo.input)

# Connect to device and start pipeline
with dai.Device(pipeline) as device:

        video = device.getOutputQueue(name="video", maxSize=1, blocking=False)

        def service_callback(req): 
                """
                Service that captures an image
                """
                global flip
                print("Saving img...")
                #time.sleep(1)
                resp = TriggerResponse()
                #try:
                
                dir_path = os.path.join(os.path.dirname(__file__), '../imgs/')
                dir1 = os.listdir(dir_path)
                file_index = [0]
                for file in dir1:
                        file_index.append(int((file.split(".")[0]).split("_")[-1]))
                new_index = str(max(file_index)+1)

                img_name = "Image_cables_"+new_index+".jpg"
                img_path = os.path.join(os.path.dirname(__file__), '../imgs/'+img_name)
                videoIn = video.get()
                if flip:
                        image = cv2.flip(videoIn.getCvFrame(), -1)
                        #image = cv2.flip(image, 1)
                else:
                        image = videoIn.getCvFrame()
                cv2.imwrite(img_path, image)
                resp.success = True
                resp.message = img_path
                print("Done")
                #except:
                #        resp.success = False
                #        print("Error")
                return resp        

        rospy.Service(service_name, Trigger, service_callback)
        print("OAK capture image service available")

        
        def video_callback(req):
                fps=10
                duration=15
                i=0
                writer= cv2.VideoWriter('/home/remodel/remodel_demos_ws/src/vision_wh_pkg/videos/basicvideo.mp4', cv2.VideoWriter_fourcc('m', 'p', '4', 'v'), fps, (1920,1080))
                while i<(duration*fps):
                        #print(i)
                        videoIn = video.get()
                        frame = videoIn.getCvFrame()
                        writer.write(frame)
                        i+=1

                writer.release()
        
        rospy.Service('/OAK/record_video', Trigger, video_callback)
        print("OAK record video service available")

        while not rospy.is_shutdown():
                try:
                        videoIn = video.get()
                        # Get BGR frame from NV12 encoded video frame to show with opencv
                        # Visualizing the frame on slower hosts might have overhead
                        cv2.imshow("video", videoIn.getCvFrame())
                        #### Create CompressedImage ####
                        msg = CompressedImage()
                        msg.header.stamp = rospy.Time.now()
                        msg.format = "jpeg"
                        if flip:
                                msg.data = np.array(cv2.imencode('.jpg', cv2.flip(videoIn.getCvFrame(), -1))[1]).tostring()
                        else:
                                msg.data = np.array(cv2.imencode('.jpg', videoIn.getCvFrame())[1]).tostring()                       
                        image_pub.publish(msg)

                        if cv2.waitKey(1) == ord('q'):
                                break
                except:
                        pass
