from TMMC_Wrapper import *
import rclpy
import numpy as np
import math
import time
from ultralytics import YOLO
import matplotlib.pyplot as plt

# Variable for controlling which level of the challenge to test -- set to 0 for pure keyboard control
challengeLevel = 3

# Set to True if you want to run the simulation, False if you want to run on the real robot
is_SIM = True

# Set to True if you want to run in debug mode with extra print statements, False otherwise
Debug = False

# Initialization    
if not "robot" in globals():
    robot = Robot(IS_SIM=is_SIM, DEBUG=Debug)
    
control = Control(robot)
camera = Camera(robot)
imu = IMU(robot)
logging = Logging(robot)
lidar = Lidar(robot)

def collisionCheck():
    while (True):
        lidarData = lidar.checkScan()
        frontCollisionDist = 0.2
        sideCollisionDist = 0.25
        frontCollisionData = lidar.detect_obstacle_in_cone(lidarData, frontCollisionDist, 0, 20)
        leftCollisionData = lidar.detect_obstacle_in_cone(lidarData, sideCollisionDist, 60, 20)
        rightCollisionData = lidar.detect_obstacle_in_cone(lidarData, sideCollisionDist, 300, 20)
        
        if (frontCollisionData==(-1,-1) and leftCollisionData==(-1,-1) and rightCollisionData==(-1,-1)):
            control.start_keyboard_input()
            break
        else:
            control.stop_keyboard_input()
        
        if (frontCollisionData != (-1,-1)):
            print("Front Collision:", frontCollisionData)
            control.set_cmd_vel(-0.5, 0, frontCollisionData[0]*2)

            '''
            # angle = collisionData[1]
            # if angle < 0:
            #     angle += 360
            
            # if angle >= 360:
            #     angle -= 360
            # print("Front Collision Angle:", angle)

            # if (angle < 90):
            #     control.rotate(angle, 1)
            #     print("rotating")
            #     time.sleep(0.1)
            #     control.set_cmd_vel(-0.5, 0, collisionData[0]*2)
            #     print("moving")
            #     time.sleep(0.1)
            
            # elif (angle >= 90 and angle < 180):
            #     control.rotate(abs(angle-180), -1)
            #     print("rotating")
            #     time.sleep(0.1)
            #     control.set_cmd_vel(0.5, 0, collisionData[0]*2)
            #     print("moving")
            #     time.sleep(0.1)

            # elif (angle >= 180 and angle < 270):
            #     control.rotate(angle-180, 1)
            #     print("rotating")
            #     time.sleep(0.1)
            #     control.set_cmd_vel(0.5, 0, collisionData[0]*2)
            #     print("moving")
            #     time.sleep(0.1)

            # elif (angle >= 270):
            #     control.rotate(abs(angle-360), -1)
            #     print("rotating")
            #     time.sleep(0.1)
            #     control.set_cmd_vel(-0.5, 0, collisionData[0]*2)
            #     print("moving")
            #     time.sleep(0.1)
            '''
            
        if (leftCollisionData!=(-1,-1)):
            print("Left Collision:", leftCollisionData)
            control.rotate(30, -1)
            control.set_cmd_vel(0.2, 0, 0.5)
            control.rotate(30, 1)

        if (rightCollisionData!=(-1,-1)):
            print("Right Collision:", rightCollisionData)
            control.rotate(30, 1)
            control.set_cmd_vel(0.2, 0, 0.5)
            control.rotate(30, -1)


def stopSignCheck(seenStopSign):
    cameraData = camera.rosImg_to_cv2()

    isStopSign = camera.ML_predict_stop_sign(cameraData)
    print(isStopSign)
    boundingBoxSize = abs( (isStopSign[1]-isStopSign[3]) * (isStopSign[2]-isStopSign[4]) )
    print(boundingBoxSize)
    if (isStopSign[0] and (not seenStopSign) and (boundingBoxSize > 5000)):
        seenStopSign = True
        print("STOP sign detected:", isStopSign)
        
        control.stop_keyboard_input()
        print("stopping")
        time.sleep(2)
        print("go now")
        control.start_keyboard_input()

    if (not(isStopSign[0])):
        seenStopSign = False

    return seenStopSign


if challengeLevel <= 2:
    control.start_keyboard_control()
    rclpy.spin_once(robot, timeout_sec=0.1)


try:
    if challengeLevel == 0:
        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.1)
            time.sleep(0.1)
            # Challenge 0 is pure keyboard control, you do not need to change this it is just for your own testing

    if challengeLevel == 1:
        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.1)
            time.sleep(0.1)
            
            # Write your solution here for challenge level 1
            # It is recommended you use functions for aspects of the challenge that will be resused in later challenges
            # For example, create a function that will detect if the robot is too close to a wall

            # collisionCheck()

            lidarData = lidar.checkScan()

            collisionData = lidar.detect_obstacle_in_cone(lidarData, 0.25, 0, 20)
            print(collisionData)
            if (collisionData != (-1,-1)):
                print("collision:", collisionData)
                control.stop_keyboard_input()
                control.set_cmd_vel(-1, 0, 0.25*(collisionData[0]/0.25))
                
            else:
                control.start_keyboard_input()

    if challengeLevel == 2:
        seenStopSign = False
        
        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.1)
            time.sleep(0.1)
            # Write your solution here for challenge level 2
            
            cameraData = camera.rosImg_to_cv2()

            isStopSign = camera.ML_predict_stop_sign(cameraData)
            print(isStopSign)
            boundingBoxSize = abs( (isStopSign[1]-isStopSign[3]) * (isStopSign[2]-isStopSign[4]) )

            if (isStopSign[0] and (not seenStopSign) and (boundingBoxSize > 10)):
                seenStopSign = True
                print("STOP sign detected:", isStopSign)
                print("stopping")
                control.stop_keyboard_input()
                time.sleep(2)
                print("go now")
                control.start_keyboard_input()

            if (not(isStopSign[0])):
                seenStopSign = False

        
    if challengeLevel == 3:
        seenStopSign = False
        visitedPaths = []
        loop = "B"

        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.1)
            time.sleep(0.1)
            # Write your solution here for challenge level 3 (or 3.5)

            collisionCheck()
            seenStopSign = stopSignCheck(seenStopSign)

            cameraData = camera.rosImg_to_cv2()
            aprilTagData = camera.estimate_apriltag_pose(cameraData)
            print(aprilTagData)

            loopB_aprilTagPath = [6, 7, 3, 5]
            loopA_aprilTagPath = [8, 4, 1, 2, 5] # 8 is the straight hallway, stop sign at 2
            

            if (loop == "B"):
                reachedTagIndex = 0
                foundTag = False

                foundIndex = 0
                for tags in aprilTagData:
                    if ((tags[0] in loopB_aprilTagPath) and not(tags[0] in visitedPaths)):
                        foundIndex = aprilTagData.index(tags)
                        reachedTagIndex = loopB_aprilTagPath.index(tags[0])
                        break
                print("Found index:", foundIndex)

                if (aprilTagData != [] and (aprilTagData[foundIndex][0] == loopB_aprilTagPath[reachedTagIndex])):
                    if (aprilTagData[foundIndex][2] > 2):                    
                        control.rotate(0.5, -1)
                    elif (aprilTagData[foundIndex][2] < -2):
                        pass
                        control.rotate(0.5, 1)
                    else:
                        foundTag = True
                        if (not(aprilTagData[0][0] in visitedPaths)):
                            pass

                        print("atag elevation:", aprilTagData[foundIndex][3])
                        angleLimit = -17
                        if (aprilTagData[foundIndex][3] > angleLimit):
                            control.set_cmd_vel(0.1, 0, 1)
                        else:
                            control.set_cmd_vel(0.5, 0, 1)
                            reachedTagIndex = (reachedTagIndex + 1)%len(loopB_aprilTagPath)
                            foundTag = False
                            visitedPaths.append(aprilTagData[foundIndex][0])

                            if (len(visitedPaths) == len(loopB_aprilTagPath)):
                                visitedPaths = []
                        
                    print("Reached Tag Index:", reachedTagIndex)

                else:
                    if (not foundTag):
                        control.rotate(5, 1)


    if challengeLevel == 4:
        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.1)
            time.sleep(0.1)
            # Write your solution here for challenge level 4

    if challengeLevel == 5:
        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.1)
            time.sleep(0.1)
            # Write your solution here for challenge level 5
            

except KeyboardInterrupt:
    print("Keyboard interrupt received. Stopping...")

finally:
    control.stop_keyboard_control()
    robot.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
