# Calibration-and-collision-mapping

## Charuco calibration （ChArUco.py + realsense_img.py)
### global variables
```shell
Dist = np.array([0.14574457705020905, -0.49072766304016113, 0.0002240741887362674, -0.00014576673856936395, 0.4482966661453247])  # system given


mtx=np.array([[901.964599609375, 0.0, 652.5621337890625],
 [  0.       ,  902.0592651367188, 366.7032165527344],
 [  0.,           0.,           1.        ]])

size_of_marker =  0.022 # side lenght of the marker in meter         


aruco_dict = aruco.Dictionary_get(ARUCO_DICT['DICT_6X6_250'])
board = aruco.CharucoBoard_create(5, 7, 0.035, 0.021, aruco_dict)       # charuco board parameter
```
其中Dist, mtx是由相机内参，可由rostopic list中的/camera/depth/camera_info得到。
![image](https://github.com/programme184/Notes/assets/118700233/7f9f40f2-71a6-47ae-a3a7-b4adcd6189a3)
结合上图和rostopic echo结果更改Dist, mtx

size_of_marker由尺子测量得出
### function/method
```shell
def charuco_pose_cam(img, charuco=True, draw=False):  # charuco=True, calibration by charuco board, otherwise: aruco markers
def charuco_dataset(images, charuco=True)
def csv_read(charuco=True)
```
若使用aruco标定，需在函数引用时输入：charuco=False

### realsense_img.py
1. 可以采集相机标定时需要的图像数据
（运行是在终端输入1后，当机械臂到达符合的位置时键入's'保存图片和相机末端的位姿）

2. 相机图像显示
可以显示实时图像和检测出特定aruco类型的ID和位置
（display()中可更改待检测的aruco码类型）
  ```shell
        aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
        aruco_dict2 = aruco.Dictionary_get(aruco.DICT_6X6_250)
```
### calib.launch
1. 在CMakeList.txt中添加calib.launch运行的calib.cpp的可执行性
```shell
add_executable(jaka_calib src/calib.cpp)
target_link_libraries(jaka_calib  ${catkin_LIBRARIES} ${PROJECT_SOURCE_DIR}/libs/libjakaAPI.so)
add_dependencies(jaka_calib robot_msgs)
```
2. 修改main()函数：
```shell
    double init_x=-0.14;
    double init_y=-0.5;
    double init_z=0.40;
```
（444行）这里定义了机械臂标定的初始位置，后续的机械臂移动范围在此基础上更改。
```shell
int points_num =10; 
```
定义了相机标定的样本数，可根据需求更改

修改完后catkin_make工作空间编译C++文件

## collision object adding (mapping.py + realsense_img.py)
### mapping.py
```
p_front = [-0.08115254138234, -0.6977417850855295, 0.5424513589271386]#[-0.00012388227045280475, -0.7117525244097773, 0.5372345966719493]
o_front = [0.9846803528520798, 0.06827924866750057, -0.062458337588609754, 0.147788710577543]#[0.9764532946278757, 0.10590385755422099, -0.13184847352664314, 0.13393773327257946]  # xyzw
    
p_back = [-0.26334396408811883, -0.31816837232143014, 0.5439693594084783]#[-0.031532668333874976, -0.5041097953656779, 0.6133438658758624]
o_back = [-0.9543479651433987, 0.18297553888233284, -0.11195362619907073, 0.20786124982365767]#[-0.9967489144080584, -0.051729046796425246, -0.008115452927891325, 0.0612359924200282]


p_mid = [-0.12689171249921802, -0.5588047858817902, 0.5350175720820082] 
o_mid =  [0.9907748164918035, 0.03536107217392759, 0.10713373323048574, 0.07508142769315632]

p_mid2 = [-0.09842332376164868, -0.46549709461842853, 0.5687013527365182] 
o_mid2 = [-0.9888968946187255, -0.049427740304110256, 0.0003461591758668705, 0.14014175136516538]
```
这部分是有示教确定机械臂位置后有jaka_move.py中的get_info()函数读出的机械臂末端位姿。
（p_front为相机能检测到前半段桌的机械臂坐标，o_front为其旋转四元数）





