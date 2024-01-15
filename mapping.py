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

from general_fun import table_cal

Dist = np.array([0.14574457705020905, -0.49072766304016113, 0.0002240741887362674, -0.00014576673856936395, 0.4482966661453247])  # system given

mtx=np.array([[901.964599609375, 0.0, 652.5621337890625],
 [  0.       ,  902.0592651367188, 366.7032165527344],
 [  0.,           0.,           1.        ]])


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
    def collision_obj(point, orien, obj_size, obj_name):
        obj_pose = {}
        obj_pose['position']=point
        obj_pose['orientation']=orien  #xyzw
        robot_control.create_object(obj_size, obj_pose, obj_name)
    
    @staticmethod
    def world_pose(image, aruco_dict):
        if image is None:
            print('video caputre failed')
        arucopose.PoseEstimate(image, aruco_dict)
        end_position, end_orientation = robot_control.info_get()
        translation = arucopose.to_wordcoord(end_position, end_orientation)
        
        return translation
        
    def table_mapping(self):
        # global color_image
        aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)

        #===== control robot reach front point
        robot_control.pos_assume(p_front, o_front)
        time.sleep(15)
        robot_control.pos_assume(p_front, o_front)
        time.sleep(15)

        imag = rs_image.get_image()
        
        tr_f = ArucoMapping.world_pose(imag, aruco_dict)
        # print(tr_f)

        #===== split the process of control robot move to the back point
        robot_control.pos_assume(p_mid, o_mid)
        time.sleep(9)
        robot_control.pos_assume(p_mid2, o_mid2)
        time.sleep(9)

        robot_control.pos_assume(p_back, o_back)
        time.sleep(10)
        robot_control.pos_assume(p_back, o_back)
        time.sleep(10)
        
        imag = rs_image.get_image()
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
        column_name = ['name', 'width', 'length','height', 'center x', 'center y', 'center z']
        aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        time.sleep(2)
        
        imag = rs_image.get_image()
        aruco_pose = ArucoMapping.world_pose(imag, aruco_dict)

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
            ArucoMapping.collision_obj(point, orien, obj_size, row_name)
            
        ArucoMapping.csv_save(csv_row)
    

p_front = [-0.08115254138234, -0.6977417850855295, 0.5424513589271386]#[-0.00012388227045280475, -0.7117525244097773, 0.5372345966719493]
o_front = [0.9846803528520798, 0.06827924866750057, -0.062458337588609754, 0.147788710577543]#[0.9764532946278757, 0.10590385755422099, -0.13184847352664314, 0.13393773327257946]  # xyzw
    
p_back = [-0.26334396408811883, -0.31816837232143014, 0.5439693594084783]#[-0.031532668333874976, -0.5041097953656779, 0.6133438658758624]
o_back = [-0.9543479651433987, 0.18297553888233284, -0.11195362619907073, 0.20786124982365767]#[-0.9967489144080584, -0.051729046796425246, -0.008115452927891325, 0.0612359924200282]


p_mid = [-0.12689171249921802, -0.5588047858817902, 0.5350175720820082] 
o_mid =  [0.9907748164918035, 0.03536107217392759, 0.10713373323048574, 0.07508142769315632]

p_mid2 = [-0.09842332376164868, -0.46549709461842853, 0.5687013527365182] 
o_mid2 = [-0.9888968946187255, -0.049427740304110256, 0.0003461591758668705, 0.14014175136516538]


real_camera_img_aligned = None

if __name__ == "__main__":
    collision_path = './Collision_object/'

    robot_control = ManiControl()
    arucopose = aruco_pose()
    code_mapping = ArucoMapping()
    rs_image = ros_camera()
    
    # code_mapping.table_mapping()                  # add table as collision object
    
    
    code_mapping.single_collision_mapping()         # add aruco markers as collision object
