#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/RobotState.h>
#include <moveit_msgs/Constraints.h>
#include <moveit_msgs/JointConstraint.h>

#include <moveit/robot_state/conversions.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/kinematics_metrics/kinematics_metrics.h>
#include <moveit/kinematics_plugin_loader/kinematics_plugin_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
// #include <moveit/robot_state/joint_state_group.h>
#include <moveit/robot_state/attached_body.h>

#include "robot_msgs/RobotMsg.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


#include <math.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include "sensor_msgs/JointState.h"
// #include "../include/jaka_jog_panel/main_window.hpp"
#define PI 3.1515926

geometry_msgs::PoseStamped cam_pose;
sensor_msgs::JointState arm_pose_joint;
sensor_msgs::JointState arm_pose_joint_last;
robot_msgs::RobotMsg robot_state_msg;
ros::Publisher pose_pub;
Eigen::Isometry3d ee_to_camera;
void robot_states_cb(const robot_msgs::RobotMsg::ConstPtr& msg)
{
     robot_state_msg=*msg;
}

void aruco_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
     cam_pose=*msg;
}

void arm_pose_joint_cb(const sensor_msgs::JointState::ConstPtr& msg)
{
    arm_pose_joint=*msg;
}

void pub_curr_pose(moveit::planning_interface::MoveGroupInterface& move_group){
    std::string end_effector_link = move_group.getEndEffectorLink();
    geometry_msgs::Pose current_pose =  move_group.getCurrentPose(end_effector_link).pose;
    std::cout << "current_pose:" << current_pose.position.x << ", "  << current_pose.position.y << ", " << current_pose.position.z << ", " 
     << current_pose.orientation.w << ", "  << current_pose.orientation.x << ", " << current_pose.orientation.y << ", " << current_pose.orientation.z
     <<std::endl;
    // ee_to_camera

    geometry_msgs::PoseStamped pose;
    pose.pose = current_pose;

    Eigen::Vector3d translation(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);

    // 将geometry_msgs::Pose中的旋转部分转换为Eigen中的四元数
    Eigen::Quaterniond quaternion(pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z);

    // 创建转移矩阵
    Eigen::Isometry3d transformationMatrix = Eigen::Isometry3d::Identity();
    transformationMatrix.translation() = translation;
    transformationMatrix.linear() = quaternion.toRotationMatrix();

    Eigen::Isometry3d final_tf =transformationMatrix*ee_to_camera;
    // Eigen::Isometry3d final_tf =transformationMatrix;

    // geometry_msgs::Pose pose;
    pose.pose.position.x = final_tf.translation().x();
    pose.pose.position.y = final_tf.translation().y();
    pose.pose.position.z = final_tf.translation().z();
    Eigen::Quaterniond quaternion1(final_tf.linear());
    pose.pose.orientation.w = quaternion1.w();
    pose.pose.orientation.x = quaternion1.x();
    pose.pose.orientation.y = quaternion1.y();
    pose.pose.orientation.z = quaternion1.z();

    pose_pub.publish(pose);
}
void arm_move(moveit::planning_interface::MoveGroupInterface& move_group,moveit::planning_interface::MoveGroupInterface::Plan& plan){
    bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success)
    {
        move_group.move();
        ROS_INFO("move cmd send");
        // sleep(3);
    }
    else{ROS_ERROR("move fail");}
    // 1232134
    // while(1){
    //     if(robot_state_msg.state == 0){
    //         break;
    //     }
    // }
    sleep(15);
    pub_curr_pose(move_group);
    std::cout << "move done" <<std::endl;
}

void set_ee_to_camera(){
    ee_to_camera = Eigen::Isometry3d::Identity();
    Eigen::Vector3d translation(-0.0368686, -0.033816, -0.0333575);
    // w x y z
    Eigen::Quaterniond quaternion(0.008354861112946355, -0.9200639780864388, 0.3916255267279043, -0.006474514547970843);
    ee_to_camera.translation() = translation;
    ee_to_camera.rotate(quaternion);
}

geometry_msgs::Pose arm2cam(geometry_msgs::Pose pose, Eigen::Isometry3d ee_to_camera_){
    geometry_msgs::Pose pose1;

    Eigen::Vector3d translation(pose.position.x, pose.position.y, pose.position.z);

    // 将geometry_msgs::Pose中的旋转部分转换为Eigen中的四元数
    Eigen::Quaterniond quaternion(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);

    // 创建转移矩阵
    Eigen::Isometry3d transformationMatrix = Eigen::Isometry3d::Identity();
    transformationMatrix.translation() = translation;
    transformationMatrix.linear() = quaternion.toRotationMatrix();

    Eigen::Isometry3d final_tf =transformationMatrix*ee_to_camera;

    pose1.position.x = final_tf.translation().x();
    pose1.position.y = final_tf.translation().y();
    pose1.position.z = final_tf.translation().z();
    Eigen::Quaterniond quaternion1(final_tf.linear());
    pose1.orientation.w = quaternion1.w();
    pose1.orientation.x = quaternion1.x();
    pose1.orientation.y = quaternion1.y();
    pose1.orientation.z = quaternion1.z();

}

geometry_msgs::Pose cam2arm(geometry_msgs::Pose pose, Eigen::Isometry3d ee_to_camera_){
    geometry_msgs::Pose pose1;

    Eigen::Vector3d translation(pose.position.x, pose.position.y, pose.position.z);

    // 将geometry_msgs::Pose中的旋转部分转换为Eigen中的四元数
    Eigen::Quaterniond quaternion(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);

    // 创建转移矩阵
    Eigen::Isometry3d transformationMatrix = Eigen::Isometry3d::Identity();
    transformationMatrix.translation() = translation;
    transformationMatrix.linear() = quaternion.toRotationMatrix();

    Eigen::Isometry3d final_tf =transformationMatrix*ee_to_camera.inverse();

    pose1.position.x = final_tf.translation().x();
    pose1.position.y = final_tf.translation().y();
    pose1.position.z = final_tf.translation().z();
    Eigen::Quaterniond quaternion1(final_tf.linear());
    pose1.orientation.w = quaternion1.w();
    pose1.orientation.x = quaternion1.x();
    pose1.orientation.y = quaternion1.y();
    pose1.orientation.z = quaternion1.z();

}
Eigen::Quaterniond  rpy2qua(double rx, double ry, double rz){
    geometry_msgs::Pose pose1;
    // 创建 AngleAxis 结构
    Eigen::AngleAxisd rotation_x(rx, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd rotation_y(ry, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd rotation_z(rz, Eigen::Vector3d::UnitZ());
    
    // 将 AngleAxis 转换为四元数
    // Eigen::Quaterniond quaternion = rotation_z * rotation_y * rotation_x;
#if 1
    Eigen::Quaterniond quaternion = rotation_x * rotation_y * rotation_z;//动轴旋转
#else
    Eigen::Quaterniond quaternion = rotation_z * rotation_y * rotation_x;//定轴旋转
#endif
    quaternion.normalize();
    return quaternion;
}


void dire2rpy(Eigen::Vector3d direction, double& rx, double& ry, double& rz)
{
// rx = atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy))
// ry = asin(2 * (qw * qy - qx * qz))
// rz = atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
// rz = std::acos(direction.norm());
rz = direction.norm()-1;
direction.normalize();
std::cout<<"debug direction: " << direction.transpose()<< std::endl;
#if 1
    // double angle =0.314;
    // direction.normalize();
    // // 创建 AngleAxis 结构
    // Eigen::AngleAxisd rotation_axis_angle(angle, direction);

    // // 将 AngleAxis 转换为旋转矩阵
    // Eigen::Matrix3d rotation_matrix = rotation_axis_angle.toRotationMatrix();

    // // 计算 RX-RY-RZ 旋转顺序下的欧拉角
    // double roll, pitch, yaw;
    // Eigen::Vector3d euler_angles = rotation_matrix.eulerAngles(0, 1, 2); 
    // rx=euler_angles[0];
    // ry=euler_angles[1];
    // rz=euler_angles[2];
    // // roll = euler_angles[2];
    // // pitch = euler_angles[1];
    // // yaw = euler_angles[0];
    rx = std::atan2(-direction[1],direction[2]); // 0-pi
    ry = std::atan2(direction[0],sqrt(direction[1]*direction[1]+direction[2]*direction[2])); // 0-pi
    // rz = 0.0;
    // rz = std::atan(direction.norm());
#else
    // direction.normalize();
    std::cout<<"debug direction: " << direction.transpose()<< std::endl;
    // 计算角度 rx
    // rx = std::atan2(-direction[0], direction[2]);
    rx = std::atan2(-sqrt(direction[0]*direction[0]+direction[1]*direction[1]),direction[2]); // 0-pi
    // if(rx<0){
        // rx+=M_PI;
    // }
    rx=-rx;
    if(abs(rx-M_PI)< 0.1 || abs(rx+M_PI)< 0.1){
        rx=M_PI/2;
    }


    // 计算角度 ry
    ry = std::atan2(direction[0], -direction[1]);
    if(abs(ry-M_PI)< 0.1|| abs(ry+M_PI)< 0.1){
        ry=0;
    }
    if(abs(ry)==M_PI/2){
        ry+=0.05;
    }

    
    // ry = 0.0;
    // // 计算角度 rz
    rz = 0.0;  // 这里假设方向向量在 xy 平面上，即 rz 为 0
#endif
    std::cout <<"rx ry rz: "<< rx <<", "<< ry <<", "<< rz << std::endl;
    // rx = -rx;
    // ry = -ry;
    // rz = -rz;
    // // 将弧度转换为角度
    // rx = rx * 180.0 / M_PI;
    // ry = ry * 180.0 / M_PI;
    // rz = rz * 180.0 / M_PI;
}
geometry_msgs::Pose set_pose(double x, double y, double z){
    geometry_msgs::Pose out_;

    out_.position.x=x;
    out_.position.y=y;
    out_.position.z=z;    

    std::cout <<"warn: don't use it!"<< std::endl;

    // std::cout <<"rx ry rz: "<< rx <<", "<< ry <<", "<< rz << std::endl;
    std::cout << "ori:" << out_.orientation.w << ", "<< out_.orientation.x << ", "<< out_.orientation.y << ", "<< out_.orientation.z << std::endl;
    return out_;
}
geometry_msgs::Pose set_pose(double x, double y, double z, double qw, double qx, double qy, double qz){
    geometry_msgs::Pose out_;

    out_.position.x=x;
    out_.position.y=y;
    out_.position.z=z;    

#if 1
    Eigen::Quaterniond quaternion(qw,qx,qy,qz);
    quaternion.normalized();
    out_.orientation.w = quaternion.w();
    out_.orientation.x = quaternion.x();
    out_.orientation.y = quaternion.y();
    out_.orientation.z = quaternion.z();
#else

    tf2::Quaternion orientation;
    orientation.setRPY(rx, ry, rz);  // 使用 roll, pitch, yaw 来设置末端姿态的方向
    out_.orientation = tf2::toMsg(orientation);
#endif
    // std::cout <<"rx ry rz: "<< rx <<", "<< ry <<", "<< rz << std::endl;
    std::cout << "ori:" << out_.orientation.w << ", "<< out_.orientation.x << ", "<< out_.orientation.y << ", "<< out_.orientation.z << std::endl;
    return out_;
}

geometry_msgs::Pose set_pose(double x, double y, double z, double rx, double ry, double rz){
    geometry_msgs::Pose out_;

    out_.position.x=x;
    out_.position.y=y;
    out_.position.z=z;    

#if 1
    Eigen::Quaterniond quaternion = rpy2qua(rx, ry, rz);  //这里是基于机械臂的动轴
    quaternion.normalized();
    out_.orientation.w = quaternion.w();
    out_.orientation.x = quaternion.x();
    out_.orientation.y = quaternion.y();
    out_.orientation.z = quaternion.z();
#else

    tf2::Quaternion orientation;
    orientation.setRPY(rx, ry, rz);  // 使用 roll, pitch, yaw 来设置末端姿态的方向
    out_.orientation = tf2::toMsg(orientation);
#endif
    std::cout <<"rx ry rz: "<< rx <<", "<< ry <<", "<< rz << std::endl;
    std::cout << "ori:" << out_.orientation.w << ", "<< out_.orientation.x << ", "<< out_.orientation.y << ", "<< out_.orientation.z << std::endl;
    return out_;
}

geometry_msgs::Pose set_pose(double x, double y, double z, Eigen::Vector3d direction){
    // geometry_msgs::Pose out_;

    // out_.position.x=x;
    // out_.position.y=y;
    // out_.position.z=z;    

    double rx,ry,rz;
    // Eigen::Vector3d direction(0, -1, 0);
    dire2rpy(direction, rx, ry, rz);
    return set_pose(x, y, z, rx, ry, rz);
    // Eigen::Quaterniond quaternion = rpy2qua(rx, ry, rz);
    // quaternion.normalized();
    // std::cout <<"rx ry rz: "<< rx <<", "<< ry <<", "<< rz << std::endl;
    // out_.orientation.w = quaternion.w();
    // out_.orientation.x = quaternion.x();
    // out_.orientation.y = quaternion.y();
    // out_.orientation.z = quaternion.z();

    // return out_;
}



int main(int argc, char **argv)
{
    //初始化节点
	ros::init(argc, argv, "moveit_cartesian_demo");
    ros::NodeHandle nh;
    //引入多线程
	ros::AsyncSpinner spinner(1);
    //开启多线程
	spinner.start();
    
    ros::Subscriber sub_cam_pose = nh.subscribe("/aruco_single/pose", 1, &aruco_pose_cb);
    ros::Subscriber sub_robot_state_pose = nh.subscribe("/l_arm_controller/robot_driver/robot_states", 10, &robot_states_cb);
    ros::Subscriber sub_arm_pose_joint = nh.subscribe("/joint_states", 1, &arm_pose_joint_cb);
    ros::Publisher calib_cmd_pub = nh.advertise<sensor_msgs::JointState>("/calib_cmd", 1);
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/pose_moveit", 1);

    //初始化需要使用move group控制的机械臂中的move_group group
    moveit::planning_interface::MoveGroupInterface move_group("manipulator");
    move_group.setPlannerId("EST"); // 选择运动规划器

    // // 存储关节角度的向量
    // std::vector<double> joint_values ={-1.574390172958374, 1.5622552633285522, -1.565582036972046, 3.1364462375640874, 1.5742380619049072, -0.7827801704406738} ;
    geometry_msgs::Pose  curr_pose;
    geometry_msgs::Pose  start_pose;
    sensor_msgs::JointState calib_cmd; 
    
    bool success =false;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;


    for(int i = 0; i < 6; i++)
    {
        calib_cmd.position.push_back(0); // write data into standard ros msg
    }

    //获取终端link的名称
    std::string end_effector_link = move_group.getEndEffectorLink();
    std::cout << "end_effector_link:" << end_effector_link << std::endl;
    //设置目标位置所使用的参考坐标系
    // std::string reference_frame = "base_link";
    std::string reference_frame = "dummy";
    move_group.setPoseReferenceFrame(reference_frame);
    //当运动规划失败后，允许重新规划
    move_group.allowReplanning(true);
    //设置位置(单位：米)和姿态（单位：弧度）的允许误差
    move_group.setGoalPositionTolerance(0.01);
    move_group.setGoalOrientationTolerance(0.01);
    move_group.setStartStateToCurrentState();
    //设置允许的最大速度和加速度
    move_group.setMaxAccelerationScalingFactor(0.5);
    move_group.setMaxVelocityScalingFactor(0.2);
 
    // 控制机械臂先回到初始化位置
    move_group.setNamedTarget("home");
    arm_move(move_group, my_plan);

    set_ee_to_camera();

    
{
    //test
    double rx,ry,rz;
    Eigen::Vector3d direction(1, -0.1, -1);
    dire2rpy(direction, rx, ry, rz);
    std::cout <<"TEST: rx ry rz: "<< rx <<", "<< ry <<", "<< rz << std::endl;

    // Eigen::Vector3d direction(1, -0.1, -1);
    // dire2rpy(direction, rx, ry, rz);
    // std::cout <<"TEST: rx ry rz: "<< rx <<", "<< ry <<", "<< rz << std::endl;

}
    ROS_INFO("next move");
    float bias_pre_x=-0.20;
    float bias_pre_y=0;
    float bias_pre_z=-0.40;

    // geometry_msgs::Pose target_pose; // 设置目标姿势

    // // 设置目标姿势的位置和朝向

    // 获取当前位姿数据最为机械臂运动的起始位姿
    start_pose = move_group.getCurrentPose(end_effector_link).pose;
    
    // float joint_bias = 0.02;
    // if(abs(start_pose.position.x-1.574390172958374)<joint_bias && )

    // 创建Eigen向量表示中心点的位置
    Eigen::Vector3d target_position(-0.15, -0.6, 0.05);

    std::cout << "start_pose:" << start_pose.position.x << ", "  << start_pose.position.y << ", " << start_pose.position.z << ", " 
     << start_pose.orientation.w << ", "  << start_pose.orientation.x << ", " << start_pose.orientation.y << ", " << start_pose.orientation.z
     <<std::endl;

    double init_x=-0.14;
    double init_y=-0.5;
    double init_z=0.40;

    float r = 0.10;
    // for training dataset
    // int points_num =40;
    int points_num =10;     //for test dataset
    geometry_msgs::Pose tmp;
    // geometry_msgs::Pose current_pose;
    // tmp=start_pose;
    tmp=start_pose;
    // std::cout <<"show: "<< tmp.position.x-0.08 << tmp.position.y-0.35 << tmp.position.z << std::endl;

    // tmp=set_pose(init_x,init_y,init_z,PI*0.75, PI*0.25, PI*-0.25);
    tmp=set_pose(init_x,init_y,init_z,PI*1, PI*0.0, PI*0.0);
    move_group.setPoseTarget(tmp);
    arm_move(move_group, my_plan);
    std::cout << "init done"<< std::endl;
    sleep(5);


    for(int i=0; i<points_num; ++i){
        geometry_msgs::Pose current_pose =  move_group.getCurrentPose(end_effector_link).pose;

        double set_x,set_y, set_z;
        // dfaderror
        set_x = init_x + r * cos(2*PI/points_num*i);
        set_y = init_y + r * sin(2*PI/points_num*i);
        // tmp.position.z += -0.05;
        set_z=init_z;
        Eigen::Vector3d direction(target_position[0]-set_x, target_position[1]-set_y, target_position[2]-set_z);
        
        
        tmp=set_pose(set_x,set_y,set_z, direction);
        move_group.setPoseTarget(tmp);
        arm_move(move_group, my_plan);
        sleep(1);
    }


   
    sleep(5);
    ros::Rate loop_rate(1);
    while (ros::ok())
    {


        while(1){
            if(robot_state_msg.state == 0){
                break;
            }
        }

        std::string end_effector_link = move_group.getEndEffectorLink();
        geometry_msgs::Pose current_pose =  move_group.getCurrentPose(end_effector_link).pose;
        std::cout << "current_pose:" << current_pose.position.x << ", "  << current_pose.position.y << ", " << current_pose.position.z << ", " 
        << current_pose.orientation.w << ", "  << current_pose.orientation.x << ", " << current_pose.orientation.y << ", " << current_pose.orientation.z
        <<std::endl;
        Eigen::Quaterniond quat(current_pose.orientation.w,current_pose.orientation.x,current_pose.orientation.y,current_pose.orientation.z);
        Eigen::Matrix3d rotation_matrix = quat.toRotationMatrix();
        double roll, pitch, yaw;
        Eigen::Vector3d euler_angles = rotation_matrix.eulerAngles(0, 1, 2); // ZYX顺序
        roll = euler_angles[2];
        pitch = euler_angles[1];
        yaw = euler_angles[0];

        loop_rate.sleep();
    }
	ros::shutdown(); 
	return 0;
}