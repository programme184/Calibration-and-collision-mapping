#!/usr/bin/env python
from typing import Any
import pyrealsense2 as rs
import numpy as np
import cv2
import imutils
import cv2.aruco as aruco
from Translation import Transformation
from jaka_move import ManiControl
import time
import threading
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import pandas as pd
from scipy.spatial.transform import Rotation
import os
from realsense_img import ros_camera
from aruco_detec_pose_estimate import aruco_pose
from moveit_python.planning_scene_interface import PlanningSceneInterface
from general_fun import table_cal

# Dist = np.array([0.14574457705020905, -0.49072766304016113, 0.0002240741887362674, -0.00014576673856936395, 0.4482966661453247])  # system given

# mtx=np.array([[901.964599609375, 0.0, 652.5621337890625],
#  [  0.       ,  902.0592651367188, 366.7032165527344],
#  [  0.,           0.,           1.        ]])


class ArucoMapping:

    def table_geometry(self, tr_f, tr_b):
        table_calculate = table_cal()
        # get lines
        fl, fr = table_calculate.corner_points(tr_f, 50)
        blm, br = table_calculate.corner_points(tr_b, 140)
        
        P_f, V_f = table_calculate.line_equation(fl,fr)            # front line
        
        P_b, V_b = br, V_f                              # back line
        
        P_r, V_r = table_calculate.line_equation(fr, br)           # right line
        P_l, V_l = fl, V_r                              # left line
        
        line_back = np.array([P_b, V_b])
        line_left = np.array([P_l, V_l])

        junction_point = table_calculate.calculate_junction_point(line_back, line_left)
        
        
        wide1 = table_calculate.calculate_distance(fl, fr)
        wide2 = table_calculate.calculate_distance(br, junction_point)
        width = (wide1+wide2)/2
        
        len1 = table_calculate.calculate_distance(fr, br)
        len2 = table_calculate.calculate_distance(fl, junction_point)
        length = (len1+len2)/2
        points = [fl, fr, br, junction_point]
        center = table_calculate.calculate_center(points)
        
        height = 0.04       #defualt
        # calculate the orientation of the table
        normal_vector = np.cross(V_f, V_l)
        rotation_matrix = np.column_stack((V_f, V_l, normal_vector)).T

        # Convert the rotation matrix to a quaternion
        rotation = Rotation.from_matrix(rotation_matrix)
        quaternion = rotation.as_quat()
        quat_t = np.roll(quaternion, -1)                 # xyzw
        
        print('wide:', width, 'length:', length, 'center on the surface:', center)
        # filename = "object_data.csv"
        row_name = 'table'
        
        center[2] -= height/2
        csv_row = {}

        csv_row[row_name] = [row_name, width, length, height, center[0], center[1], center[2]]
        
        ArucoMapping.csv_save(csv_row)
        
        # #===== add collision object====
        table_size = [width, length, 0.01]
        ArucoMapping.collision_obj(center, quat_t, table_size, row_name)
        
        return fl, fr, br, junction_point
    
    @staticmethod
    def csv_save(csv_row):
        filename = "object_data.csv"
        collision_path = './Collision_object/'
        column_name = ['name', 'width', 'length', 'height', 'center x', 'center y', 'center z']
        if os.path.isfile(collision_path+filename):
            df_old = pd.read_csv(collision_path+filename)

            for idx in csv_row.keys():
                # print('idx:', idx)
                # print('df_old:', df_old['name'])
                if idx in df_old['name'].values:
                    df_old.loc[df_old['name'] == idx] = csv_row[idx]
                else:

                    df = pd.DataFrame([csv_row[idx]], columns=column_name)
                    df_old = pd.concat([df_old, df])
            df = df_old
        else:
            df = pd.DataFrame(csv_row.values(), columns=column_name)

        df.to_csv(collision_path+filename, index=False)
        

    @staticmethod
    def collision_obj(point, orien, obj_size, obj_name, obj_color=[1.0, 0.0, 0.0]):
        scene2= PlanningSceneInterface("dummy")
        obj_pose = {}
        obj_pose['position']=point
        obj_pose['orientation']=orien  #xyzw
        robot_control.create_object(obj_size, obj_pose, obj_name)
        [obj_r, obj_g, obj_b] = obj_color
        scene2.setColor(obj_name, obj_r, obj_g, obj_b)
        scene2.sendColors()

    
    @staticmethod
    def world_pose(image, aruco_dict, ar_size=0.05):
        if image is None:
            print('video caputre failed')
        print('start estimate pose')
        arucopose.PoseEstimate(image, aruco_dict, aruco_size=ar_size)
        end_position, end_orientation = robot_control.info_get()
        translation = arucopose.to_wordcoord(end_position, end_orientation)
        # print('translation:', translation)
        return translation
        
    def table_mapping(self):
        # global color_image
        aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)

        #===== control robot reach front point
        robot_control.set_joints_values(joints_front)
        print('Arrived')

        imag = rs_image.get_image()
        
        tr_f = ArucoMapping.world_pose(imag, aruco_dict)
        # print(tr_f)

        #===== split the process of control robot move to the back point
        robot_control.set_joints_values(joints_back)
        print('Arrived')
      
        
        imag = rs_image.get_image()
        # print('imag')
        # time.sleep(3)
        tr_b = ArucoMapping.world_pose(imag, aruco_dict)
        # print('tr_b:', tr_b)
        end_position, _ = robot_control.info_get()
    
        pfl, pfr, pbr, pbl = self.table_geometry(tr_f, tr_b)
        print('point1:', pfl, '\npoint2:', pfr, '\npoint3:', pbr)
        print('point4', pbl)
            
        # plot scatter points
        X = [pfl[0], pfr[0], pbr[0], pbl[0], end_position[0]]
        Y = [pfl[1], pfr[1], pbr[1], pbl[1], end_position[1]]
        Z = [pfl[2], pfr[2], pbr[2], pbl[2], end_position[2]]
            
        # Plot the scatter points
        # Create a 3D plot
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        # Plot the scatter points
        ax.scatter(X[:-1], Y[:-1], Z[:-1], color='red')  # Plot all points except the last one
        ax.scatter(X[-1], Y[-1], Z[-1], color='blue')    # Plot the last point with a different color
        # Add labels and title to the plot
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title('3D Scatter Plot')

        # Save the plot as an image file
        plt.savefig(collision_path +'corner of table.png')
        # Display the plot
        plt.show()
    
    
    def single_collision_mapping(self):
        # global color_image
        filename = "object_data.csv"
        
        filepath = os.path.join(collision_path, filename)

        aruco_square = 0.022
        aruco_color = [0.0, 0.0, 1.0]
        column_name = ['name', 'width', 'length','height', 'center x', 'center y', 'center z']
        aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        time.sleep(2)
        
        imag = rs_image.get_image()
        # imag = cv2.imread('1.jpg')
        # imag = np.array(imag)
        aruco_pose = ArucoMapping.world_pose(imag, aruco_dict, ar_size=aruco_square)

        csv_row = {}
        obj_size = [aruco_square, aruco_square, aruco_square]
        for id in aruco_pose.keys():
            # id = ids[i]
            point = aruco_pose[id][0]
            orien = aruco_pose[id][1]
            # print('id:', id)
            row_name = 'collision' + str(id) 
            point[2] += aruco_square/2                  # only suitable for aruco markers are on the surface; this should be point[2] -= aruco_square/2 for aruco are attached on objects
            csv_row[row_name] = [row_name, aruco_square, aruco_square, aruco_square, point[0], point[1], point[2]]

            
            # #===== add collision object====
            ArucoMapping.collision_obj(point, orien, obj_size, row_name, obj_color=aruco_color)
            
        ArucoMapping.csv_save(csv_row)
    

joints_front=  [-1.8464380, 1.08393180, -0.78396, 1.583143115, 1.55532, 2.71818]


joints_back = [-2.511842966, 1.553446, -1.16143035, 0.7067855, 1.455289006, 2.6161327]

real_camera_img_aligned = None

if __name__ == "__main__":
    collision_path = './Collision_object/'

    robot_control = ManiControl()
    arucopose = aruco_pose()
    code_mapping = ArucoMapping()
    rs_image = ros_camera()
    
    # code_mapping.table_mapping()                  # add table as collision object
    
    
    code_mapping.single_collision_mapping()         # add aruco markers as collision object
