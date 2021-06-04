#include <string>
#include <ros/ros.h> // 包含ROS的头文件
#include <serial/serial.h> //ROS已经内置了的串口包
#include <iostream>
#include <cstring>
#include "std_msgs/String.h" //ros定义的String数据类型
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"//转换函数头文件

// mavlink
#include "mavlink1/v1.0/mavlink_types.h"
#include "mavlink1/v1.0/common/mavlink.h"
// #include "MAVLINK/mavlink_avoid_errors.h"

using namespace std;
using namespace geometry_msgs;

Pose trackPose;

mavlink_message_t mav_msg;
mavlink_message_t mav_rcv;
mavlink_status_t mav_rx_status;


/*
void chatterCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  PoseStamped submsg=*msg;
  Point position_opti = msg->pose.position;
  Quaternion quat_opti = msg->pose.orientation;
  trackPose = msg->pose;

  // convert quat 2 yaw
  tf::Quaternion quat;
  tf::quaternionMsgToTF(quat_opti, quat);

  double roll, pitch, yaw;//定义存储r\p\y的容器
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换  
  //cout<<"find position"<<endl;
  //ROS_INFO("pos.x:%lf",position_opti.x); //打印接受到的字符串
}
*/
// 发送pose函数，将pose信息加入u8数组发送
// return: buffer中装载的byte数目：50
// 
//buf[0] = 0xff,buf[1]=0xfe
//buf[2]~buf[2+sizeof(double)-1] = x
//buf[2+sizeof(double)]~buf[2+2*sizeof(double)-1] = y
//buf[2+2*sizeof(double)]~buf[2+3*sizeof(double)-1] = z
//buf[2+3*sizeof(double)]~buf[2+4*sizeof(double)-1] = r
//buf[2+4sizeof(double)]~buf[2+5*sizeof(double)-1] = p
//buf[2+5sizeof(double)]~buf[2+6*sizeof(double)-1] = y
size_t PoseXYZRPY2buffer(uint8_t* buffer)
{
  tf::Quaternion quat;
  trackPose.orientation.x =1;
  trackPose.orientation.y = 0;
  trackPose.orientation.z = 0;
  trackPose.orientation.w = 0;
  tf::quaternionMsgToTF(trackPose.orientation, quat);

  double roll, pitch, yaw;//定义存储r\p\y的容器
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换
  roll = 0.44;
  pitch = 0.27;
  yaw = 0.62;  

  buffer[0] = 0xff;
  buffer[1] = 0xfe;
  memcpy(buffer+2,&(trackPose.position.x),sizeof(double));
  memcpy(buffer+2+8,&(trackPose.position.y),sizeof(double));
  memcpy(buffer+2+16,&(trackPose.position.z),sizeof(double));
  memcpy(buffer+2+24,&(roll),sizeof(double));
  memcpy(buffer+2+32,&(pitch),sizeof(double));
  memcpy(buffer+2+40,&(yaw),sizeof(double));
  buffer[50] = 0x0d;
  buffer[51] = 0x0a;
  return 52;
}

// 发送pose函数，将pose信息加入u8数组发送
//// return: buffer中装载的byte数目：58
//buf[0] = 0xff,buf[1]=0xfe
//buf[2]~buf[2+sizeof(double)-1] = x
//buf[2+sizeof(double)]~buf[2+2*sizeof(double)-1] = y
//buf[2+2*sizeof(double)]~buf[2+3*sizeof(double)-1] = z
//buf[2+3*sizeof(double)]~buf[2+4*sizeof(double)-1] = x
//buf[2+4*sizeof(double)]~buf[2+5*sizeof(double)-1] = y
//buf[2+5*sizeof(double)]~buf[2+6*sizeof(double)-1] = z
//buf[2+6*sizeof(double)]~buf[2+7*sizeof(double)-1] = w
size_t PoseXYZQuat2buffer(uint8_t* buffer)
{
      

    buffer[0] = 0xff;
    buffer[1] = 0xfe;
    memcpy(buffer+2,&(trackPose.position.x),sizeof(double));
    memcpy(buffer+2+8,&(trackPose.position.y),sizeof(double));
    memcpy(buffer+2+16,&(trackPose.position.z),sizeof(double));
    memcpy(buffer+2+24,&(trackPose.orientation.x),sizeof(double));
    memcpy(buffer+2+32,&(trackPose.orientation.y),sizeof(double));
    memcpy(buffer+2+40,&(trackPose.orientation.z),sizeof(double));
    memcpy(buffer+2+48,&(trackPose.orientation.w),sizeof(double));
    buffer[58] = 0x0d;
    buffer[59] = 0x0a;

    return 60;

}

static void handleMessage(mavlink_message_t *msg)
{
 //command for telemetry
    switch (msg->msgid) {
        
        case MAVLINK_MSG_ID_HEARTBEAT: {
            //mavlink_msg_global_position_int_decode(msg, &position);
            
            ROS_INFO("HeartBeat");
            
            break;
        }
       
        
       
        
        default:
            break;
    }     // end switch
    
} // end handle mavlink

int main(int argc, char** argv)
{

    ros::init(argc, argv, "serial_nrf");
    //创建句柄（虽然后面没用到这个句柄，但如果不创建，运行时进程会出错）
    ros::NodeHandle n;
    
    //创建一个serial类
    serial::Serial sp;

    trackPose.position.x = 0.33;
    trackPose.position.y = 0.43;
    trackPose.position.z = 0.56;

    uint8_t buffer[1024];
    uint8_t buf_pose[100];

    int bbs = 0; //pose send test flag. need delet  

    mav_msg.compid = 1;
    mav_msg.sysid = 2;

    //dingyue
    //ros::Subscriber sub = n.subscribe("/vrpn_client_node/RigidBody002/pose", 1000, chatterCallback);
    //创建timeout
    serial::Timeout to = serial::Timeout::simpleTimeout(100);
    //设置要打开的串口名称
    sp.setPort("/dev/ttyUSB0");
    //设置串口通信的波特率
    sp.setBaudrate(57600);
    //串口设置timeout
    sp.setTimeout(to);
    
    try
    {
        //打开串口
        sp.open();
    }
    catch(serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port.");
        return -1;
    }
    
    //判断串口是否打开成功
    if(sp.isOpen())
    {
        ROS_INFO_STREAM("/dev/ttyUSB0 is opened.");
    }
    else
    {
        return -1;
    }
    
    //mavlink_msg_heartbeat_pack(0, 200, mav_msg,0,3, 1, 2, 0); 
    ROS_INFO_STREAM("1.");
    // mavlink_heartbeat_t heartbeat_send;
	// heartbeat_send.base_mode = 1;
	// heartbeat_send.custom_mode = 2;
	// heartbeat_send.autopilot = 3;
	// heartbeat_send.type = 4;
	// heartbeat_send.system_status = 5;
	// mavlink_msg_heartbeat_encode(9,201,&mav_msg,&heartbeat_send);
    //size_t len = mavlink_msg_to_send_buffer(buf_pose, &mav_msg);

    mavlink_vicon_position_estimate_t mocap_pos_send;
    mocap_pos_send.x = 0.2;
    mocap_pos_send.y = 0.3;
    mocap_pos_send.z = 0.4;
    mocap_pos_send.roll = 0.6;
    mocap_pos_send.pitch = 0.7;
    mocap_pos_send.yaw = 0.8;
    mavlink_msg_vicon_position_estimate_encode(9,201,&mav_msg,&mocap_pos_send);
    size_t len = mavlink_msg_to_send_buffer(buf_pose, &mav_msg);

    ros::Rate loop_rate(20);
    int cnt = 0;
    while(ros::ok())
    {
        //获取缓冲区内的字节数
        size_t n = sp.available();
        if(n!=0)
        {
            //uint8_t buffer[1024];
            //读出数据
            n = sp.read(buffer, n);
            for(int i=0; i<n; i++)
            {
                if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &mav_rcv, &mav_rx_status))
                {
                    //Serial.println(msg.msgid);
                    ROS_INFO("msgid is: %d \n",mav_rcv.msgid);
                    handleMessage(&mav_rcv);
        
                }
            }

            // if(buffer[0]!=0xff||buffer[1]!=0xfe)
            // {
            //     //std::cout<<"wrong header"<<std::endl;
            //     ROS_WARN("wrong header");
            //     continue;
            // }
            // float test;
            // memcpy(&test,buffer+2,sizeof(test));
            // std::cout<<"float is: "<<test<<endl;
            // for(int i=0; i<n; i++)
            // {
            //     //16进制的方式打印到屏幕
            //     std::cout << std::hex << (buffer[i] & 0xff) << " ";
            // }
            // std::cout << std::endl;
            //把数据发送回去
            //sp.write(buffer, n);
        }
        if(cnt>=20)
            {
                sp.write(buf_pose,len);
                ROS_INFO("SEND");
                cnt=0;
            }
            cnt++;
        //////////////////////////// send pose ////////////////////////////////
        //size_t bytenum_pose = PoseXYZRPY2buffer(buf_pose);
        /*
        if(bbs==20)
        {
            bbs = 0;
            for(int i=0; i<50; i++)
            {
                //16进制的方式打印到屏幕
                std::cout << std::hex << (buf_pose[i] & 0xff) << " ";
            }
            std::cout << std::endl;
        }
        
        bbs++;
        */
        //sp.write(buf_pose,bytenum_pose);
        //sp.write(&mav_msg->magic,msg_length);
        ///////////////////////////////////////send pose end ////////////////////

        ros::spinOnce();
        loop_rate.sleep();
    }
    
    //关闭串口
    sp.close();
 
    return 0;
}


