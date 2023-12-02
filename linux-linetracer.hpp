/*******************************************************************************
* Copyright 2017 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/
 
#ifndef _ROS_TRACER_HPP_
#define _ROS_TRACER_HPP_
#define STDIN_FILENO 0
#include <fcntl.h>
#include <termios.h>
#include "dynamixel_sdk.h"              // Uses Dynamixel SDK library
#include <stdlib.h>
#include <sstream> 
#include <stdio.h>
#include <chrono>
#include <functional>
#include <memory> 
#include <string> 
#include <cstdio> 
#include <iostream> 
#include <utility> 
#include "opencv2/opencv.hpp" 
#include "rclcpp/rclcpp.hpp" 
#include "sensor_msgs/msg/image.hpp" 
#include "geometry_msgs/msg/twist.hpp"

// Control table address for AX-12W and MX-12W
//AX-12W와  MX-12W의 Control table 주소
//데이터 길이(Byte)를 Instruction Packet 의 파라미터로 보내주기 위한 매크로 정의
#define ADDR_MX_TORQUE_ENABLE           24                  
#define ADDR_MX_GOAL_POSITION           30
#define ADDR_MX_PRESENT_POSITION        36
#define ADDR_MX_MOVING_SPEED            32
 

// Data Byte Length for AX-12W and MX-12W
//AX-12W와 MX-12W의 데이터 길이
#define LEN_MX_GOAL_POSITION            2
#define LEN_MX_PRESENT_POSITION         2
#define LEN_MX_MOVING_SPEED             2
 

 
// Protocol version for AX-12W and MX-12W
#define PROTOCOL_VERSION                1.0//프로토콜 버전                  
// Protocol version for XL and XC model
//#define PROTOCOL_VERSION                2.0                 
 
// Default setting
#define DXL1_ID                         1                
// Dynamixel#1 ID: 1 //왼쪽바퀴
#define DXL2_ID                         2               
// Dynamixel#2 ID: 2 //오른쪽 바퀴
#define BAUDRATE                        2000000         
// for AX-12W and MX-12W model //MX,AX모델의 보드레이트 속도
//#define BAUDRATE                        4000000       
// for XL and XC model
#define DEVICENAME                      "/dev/ttyUSB0"  
// Check which port is being used on your controller //사용할 컨트롤러         
// ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"
#define TORQUE_ENABLE                   1               
// Value for enabling the torque //토크활성화 값
#define TORQUE_DISABLE                  0               
// Value for disabling the torque //토크 비활성화 값
#define OPMODE_XL_VELOCITY              1               
#define OPMODE_XL_POSITION              3               
#define OPMODE_XL_PWM                    16              
#define ESC_ASCII_VALUE                 0x1b

using std::placeholders::_1; 
//namespace(이름공간)사용
using namespace std::chrono_literals;
using namespace std;
using namespace cv;
class LineTracer
{
private:
//멤버변수 정의
    int port_num;
    int group_num;
    int dxl_comm_result;             // Communication result
    uint8_t dxl_addparam_result;     // AddParam result
    uint8_t dxl_error;               // Dynamixel error
    dynamixel::PortHandler* portHandler;
    dynamixel::PacketHandler* packetHandler;
public:
//멤버 함수 선언
    LineTracer();
    bool dxl_open(void);
    void dxl_close(void);
    bool dxl_set_velocity(int goal_velocity1, int goal_velocity2);
    unsigned int vel_convert(int speed);
    void vel_convert_rpm(double linear_speed, double angular_speed, int* left_rpm, int* right_rpm);
    std::string mat_type2encoding(int mat_type);
    int getch(void);
    void convert_frame_to_message(const cv::Mat& frame, sensor_msgs::msg::Image& msg);
};
 
#endif 
