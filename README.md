# Calibration-and-collision-mapping

## Charuco calibration
```shell
Dist = np.array([0.14574457705020905, -0.49072766304016113, 0.0002240741887362674, -0.00014576673856936395, 0.4482966661453247])  # system given


mtx=np.array([[901.964599609375, 0.0, 652.5621337890625],
 [  0.       ,  902.0592651367188, 366.7032165527344],
 [  0.,           0.,           1.        ]])

size_of_marker =  0.022 # side lenght of the marker in meter


aruco_dict = aruco.Dictionary_get(ARUCO_DICT['DICT_6X6_250'])
board = aruco.CharucoBoard_create(5, 7, 0.035, 0.021, aruco_dict)       # not sure, roughly measured
```
其中Dist, mtx是由相机内参构成的，可由rostopic list中的/camera/depth/camera_info得到
