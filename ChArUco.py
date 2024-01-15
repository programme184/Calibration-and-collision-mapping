import os
import numpy as np
import cv2
import cv2.aruco as aruco
import matplotlib.pyplot as plt
import pandas as pd
import json
from Translation import Transformation
from PIL import Image
import transforms3d as tfs
# create_and_save_new_board()

# define names of each possible ArUco tag OpenCV supports
ARUCO_DICT = {
	"DICT_4X4_50": cv2.aruco.DICT_4X4_50,
	"DICT_4X4_100": cv2.aruco.DICT_4X4_100,
	"DICT_4X4_250": cv2.aruco.DICT_4X4_250,
	"DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
	"DICT_5X5_50": cv2.aruco.DICT_5X5_50,
	"DICT_5X5_100": cv2.aruco.DICT_5X5_100,
	"DICT_5X5_250": cv2.aruco.DICT_5X5_250,
	"DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
	"DICT_6X6_50": cv2.aruco.DICT_6X6_50,
	"DICT_6X6_100": cv2.aruco.DICT_6X6_100,
	"DICT_6X6_250": cv2.aruco.DICT_6X6_250,
	"DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
	"DICT_7X7_50": cv2.aruco.DICT_7X7_50,
	"DICT_7X7_100": cv2.aruco.DICT_7X7_100,
	"DICT_7X7_250": cv2.aruco.DICT_7X7_250,
	"DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
	"DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
	"DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
	"DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
	"DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
	"DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}
Dist = np.array([0.14574457705020905, -0.49072766304016113, 0.0002240741887362674, -0.00014576673856936395, 0.4482966661453247])  # system given


mtx=np.array([[901.964599609375, 0.0, 652.5621337890625],
 [  0.       ,  902.0592651367188, 366.7032165527344],
 [  0.,           0.,           1.        ]])

size_of_marker =  0.022 # side lenght of the marker in meter


aruco_dict = aruco.Dictionary_get(ARUCO_DICT['DICT_6X6_250'])
board = aruco.CharucoBoard_create(5, 7, 0.035, 0.021, aruco_dict)       # not sure, roughly measured

trans = Transformation()
    

def charuco_pose_cam(img, charuco=True, draw=False):
    
    frame = cv2.imread(img)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    rvec, tvec = None, None
    if charuco is True:
    
        aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        parameters =  aruco.DetectorParameters_create()
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, 
                                                      parameters=parameters)
        
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.0001)
        
        if len(corners)>0:
            # SUB PIXEL DETECTION
            for corner in corners:
                cv2.cornerSubPix(gray, corner,
                                 winSize = (3,3),
                                 zeroZone = (-1,-1),
                                 criteria = criteria)
            ret, c_corners, c_ids = cv2.aruco.interpolateCornersCharuco(corners, ids, gray, board)

        else:
            print('Failed!')
        # Estimate charuco board pose       rvec is the Rodrigues() vector, shape is 3x1
        retval, rvec, tvec = aruco.estimatePoseCharucoBoard(c_corners, c_ids, board, mtx, Dist, np.empty(1), np.empty(1))
        
        if draw is True:
            length_of_axis = 0.01
            # imaxis = aruco.drawDetectedMarkers(frame.copy(), c_corners, c_ids)
            imaxis = aruco.drawDetectedCornersCharuco(frame.copy(), c_corners, c_ids)
            cv2.drawFrameAxes(imaxis, mtx, Dist, rvec, tvec, length_of_axis)

    else:   # detect aruco markers
        aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
        parameters =  aruco.DetectorParameters_create()

        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict,parameters=parameters)

        if ids is not None:

            rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, 0.05, mtx, Dist)
            print('tvec:', tvec)
            rvec = rvec.reshape(3,-1)
            tvec = tvec.reshape(3,-1)
            print('tvec after:', tvec)

    return rvec,tvec            # 3x1


def charuco_dataset(images, charuco=True):
    if charuco is True:
        filename = "charuco_board_pose_cam.csv"
    else:
        filename = "Aruco_pose_cam.csv"
    for i in range(0, len(images)):
        rvec,tvec = charuco_pose_cam(images[i], charuco)
        df = pd.DataFrame(np.concatenate((rvec.T, tvec.T), axis =1), columns=['Rodrigues 1', 'Rodrigues 2', 'Rodrigues 3', 'position x', 'position y', 'position z'])     # np.concatenate((list_tr, list_rot), axis=1)
        if i != 0:
   
            df_old = pd.read_csv(datadir+filename)
            # Concatenate the existing DataFrame with the new DataFrame
            df = pd.concat([df_old, df], ignore_index=True)

        df.to_csv(datadir+filename, index=False)

def csv_read(charuco=True):
    if charuco is True:
        file_aruco = "charuco_board_pose_cam.csv"
        file_end_aligned = "endpose_data.csv"
    else:
        file_aruco = "Aruco_pose_cam.csv"
        file_end_aligned = "endpose_aruco.csv"
        
    df_aruco = pd.read_csv(datadir+file_aruco)
    df_end = pd.read_csv(datadir+file_end_aligned)

    hand_world_tr, hand_world_rot = df_end.iloc[:, 0:3].values, df_end.iloc[:, 3:7].values        #hand_world_rot is quaternion
    # print()
    marker_camera_rot, marker_camera_tr = df_aruco.iloc[:, 0:3].values, df_aruco.iloc[:, 3:6].values
    
    return hand_world_tr, hand_world_rot, marker_camera_rot, marker_camera_tr

def calibration_ex(algorithm):
    hand_world_tr, hand_world_rot, marker_camera_rot, marker_camera_tr = csv_read(charuco=True)

    if len(hand_world_rot) != len(marker_camera_rot):
        print("Different numbers of hand-world and camera-marker samples!")
        raise AssertionError
    
    method = AVAILABLE_ALGORITHMS[algorithm]

        
    hand_world_rotMatrix=[]
    marker_camera_rotMtrix= []
    for i in range(len(hand_world_rot)):
        rotmatrix = trans.RotMatrix(hand_world_rot[i])

        rotmatrix_char = trans.RotMatrix(marker_camera_rot[i], rodrigues=True)
        hand_world_rotMatrix.append(rotmatrix)
        marker_camera_rotMtrix.append(rotmatrix_char)

    hand_camera_rot, hand_camera_tr = cv2.calibrateHandEye(hand_world_rotMatrix, hand_world_tr, marker_camera_rotMtrix,
                                                          marker_camera_tr, method=method)
    
    
    (qw, qx, qy, qz) = [float(i) for i in tfs.quaternions.mat2quat(hand_camera_rot)]
    (hctx, hcty, hctz) = [float(i) for i in hand_camera_tr]
    # print('\nhand_camera_rot:', hand_camera_rot, '\nhand_camera_tr', hand_camera_tr)


    hand_camera_quat = [qx, qy, qz, qw]
    hand_camera_tran = [hctx, hcty, hctz]
    
    return hand_camera_quat, hand_camera_tran

def result_analysis():
    position = []
    orientation =[]
    for algorithm in AVAILABLE_ALGORITHMS:
        hand_camera_quat, hand_camera_tran = calibration_ex(algorithm)
        print('\n===algorithm:', algorithm,'===')
        print('hand_camera_quat:', hand_camera_quat)
        print('hand_camera_translation:', hand_camera_tran)
        orientation.append(hand_camera_quat)
        position.append(hand_camera_tran)
    tr_ana = [np.mean(position, axis=0), np.var(position, axis=0), np.std(position, axis=0)]
    or_ana = [np.mean(orientation, axis=0), np.var(orientation, axis=0), np.std(orientation, axis=0)]
    print('\nMean:', '\ntranslation', tr_ana[0], 'quaternion', or_ana[0])
    print('\nVariance:', '\ntranslation', tr_ana[1], 'quaternion', or_ana[1])
    # get world pose from test dataset
    tran = Transformation(hand_camera_tran, hand_camera_quat)#hand_camera_tran, hand_camera_quat
    hand_world_tr, hand_world_rot, marker_camera_rot, marker_camera_tr = csv_read()

    
    position_test =[]
    orientation_test=[]
    for i in range(0, len(hand_world_rot)):     
        rotmatrix_char = trans.RotMatrix(marker_camera_rot[i], rodrigues=True)
        T_obj = marker_camera_tr[i]
        R_obj = rotmatrix_char    # rotation matrix
        end_pos = hand_world_tr[i]
        end_ori = hand_world_rot[i]
        tr, quat_t = tran.express_transform(T_obj, R_obj, end_pos, end_ori)
        position_test.append(tr)
        orientation_test.append(quat_t)
        # print('tr world:', tr)

    tr_test_ana = [np.mean(position_test, axis=0), np.var(position_test, axis=0), np.std(position_test, axis=0)]
    or_test_ana = [np.mean(orientation_test, axis=0), np.var(orientation_test, axis=0), np.std(orientation_test, axis=0)]
    
    #save data into csv file
    filename = "calibration_result.csv"
    row_name = ['Tsai-Lenz', 'Park','Horaud', 'Daniilidis']
    row_name2 = ['mean', 'variance', 'std']
    column_name=['position x', 'position y', 'position z', 'Rotdata x', 'Rotdata y', 'Rotdata z', 'Rotdata w']
    df_title = pd.DataFrame(['Eye in hand calibration results'])
    df = pd.DataFrame(np.concatenate([np.array(position), np.array(orientation)], axis=1), index=row_name, columns=column_name)
    train_ana = np.concatenate([np.array(tr_ana), np.array(or_ana)], axis=1)
    df_ana_train = pd.DataFrame(train_ana, index=row_name2, columns=column_name)
    
    df_train = pd.concat([df_title, df, df_ana_train])

    df_title = pd.DataFrame(['Test results: estimated world pose of Charuco board'])
    df_test = pd.DataFrame(np.concatenate([np.array(position_test), np.array(orientation_test)], axis=1), columns=column_name)
    test_ana = np.concatenate([np.array(tr_test_ana), np.array(or_test_ana)], axis=1)
    df_ana_test = pd.DataFrame(test_ana, index=row_name2, columns=column_name)
    df_test = pd.concat([df_title, df_test, df_ana_test])

    df= pd.concat([df_train, df_test])

    df.to_csv(testdatadir+filename)
        

def image_read(dir):
    images = np.array([datadir + f for f in os.listdir(datadir) if f.endswith(".jpg") ])
    order = np.argsort([int(p.split(".")[-2].split("/")[-1]) for p in images])
    images = images[order]
    print(len(images))

    return images

if __name__=='__main__':
    # for charuco:
    datadir = './checkerboard_img/'
    testdatadir = './charuco_test/'
    
    AVAILABLE_ALGORITHMS = {
        'Tsai-Lenz': cv2.CALIB_HAND_EYE_TSAI,
        'Park': cv2.CALIB_HAND_EYE_PARK,
        'Horaud': cv2.CALIB_HAND_EYE_HORAUD,
       # 'Andreff': cv2.CALIB_HAND_EYE_ANDREFF,
        'Daniilidis': cv2.CALIB_HAND_EYE_DANIILIDIS,
    }
    # calibration
    images = image_read(datadir)
    charuco_dataset(images, charuco=True)
    
    # test calibration result
    images_test = image_read(testdatadir)
    charuco_dataset(images_test, charuco=True)
    result_analysis()