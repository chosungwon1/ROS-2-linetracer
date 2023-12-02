
 
#include "linetracer/linetracer.hpp"//헤더파일 경로

int LineTracer::getch(void)
{
    struct termios oldt, newt;
    int ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
}

//생성자 정의
LineTracer::LineTracer(void)
{
    port_num = 0;
    group_num = 0;
    dxl_comm_result = COMM_TX_FAIL;             // Communication result
    dxl_addparam_result = false;            // AddParam result
    dxl_error = 0;                          // Dynamixel error
    // Initialize PortHandler instance
    // Set the port path
    // Get methods and members of PortHandlerLinux or PortHandlerWindows
    portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
 
    // Initialize PacketHandler instance
    // Set the protocol version
    // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
    packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
    
}
bool LineTracer::dxl_open(void)
{    
    // Open port
    if (portHandler->openPort())
    {
        printf("Succeeded to open the port!\n");
    }
    else
    {
        printf("Failed to open the port!\n");
        printf("Press any key to terminate...\n");
        getch();
        return false;
    }
 
    // Set port baudrate
    if (portHandler->setBaudRate(BAUDRATE))
    {
        printf("Succeeded to change the baudrate!\n");
    }
    else
    {
        printf("Failed to change the baudrate!\n");
        printf("Press any key to terminate...\n");
        getch();
        return false;
    }
 
    // Enable Dynamixel#1 Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL1_ID, 
                                       ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        return false;
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        return false;
    }
    else
    {
        printf("Dynamixel#%d has been successfully connected \n", DXL1_ID);
    }
 
    // Enable Dynamixel#2 Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL2_ID, 
                                      ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        return false;
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        return false;
    }
    else
    {
        printf("Dynamixel#%d has been successfully connected \n", DXL2_ID);
    }
    return true;
}
void LineTracer::dxl_close(void)
{
    // stop motor
    dxl_set_velocity(0, 0);
    
    // Disable Dynamixel#1 Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL1_ID, 
                                         ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }
 
    // Disable Dynamixel#2 Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL2_ID, 
                                     ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }
 
    // Close port
    portHandler->closePort();
}
bool LineTracer::dxl_set_velocity(int goal_velocity1, int goal_velocity2)
{
    dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, 
                                            ADDR_MX_MOVING_SPEED, LEN_MX_MOVING_SPEED);
    uint8_t param_goal_position[2];
    
    // Allocate goal position value into byte array
    param_goal_position[0] = DXL_LOBYTE(vel_convert(goal_velocity1));
    param_goal_position[1] = DXL_HIBYTE(vel_convert(goal_velocity1));
    
    // Add Dynamixel#1 goal position value to the Syncwrite storage
    dxl_addparam_result = groupSyncWrite.addParam(DXL1_ID, param_goal_position);
    if (dxl_addparam_result != true)
    {
        fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL1_ID);
        return false;
    }
   
    param_goal_position[0] = DXL_LOBYTE(vel_convert(goal_velocity2));
    param_goal_position[1] = DXL_HIBYTE(vel_convert(goal_velocity2));
   
    // Add Dynamixel#2 goal position value to the Syncwrite parameter storage
    dxl_addparam_result = groupSyncWrite.addParam(DXL2_ID, param_goal_position);
    if (dxl_addparam_result != true)
    {
        fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL2_ID);
        return false;
    }    
    // Syncwrite goal position
    dxl_comm_result = groupSyncWrite.txPacket();
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        return false;
    }
 
    // Clear syncwrite parameter storage
    groupSyncWrite.clearParam();
    return true;
}
 
 
//-1023 <= speed <= 1023 -> +(CCW) 0~1023, -(CW) 1024~2047
unsigned int LineTracer::vel_convert(int speed)
{
    unsigned int temp;
    if (speed > 1023) speed = 1023;
    else if (speed < -1023) speed = -1023;
 
    if (speed >= 0) temp = (unsigned int)speed;
    else temp = (unsigned int)(-speed + 1023);
 
    return temp;
}
void LineTracer::vel_convert_rpm(double linear_speed, double angular_speed, int* left_rpm, int* right_rpm)
{
    double velocity_constant_value = 1 / (0.03 * 0.916 * 0.10472);
    double left_velocity = linear_speed - (0.162 * angular_speed / 2);
    *left_rpm = (int)(left_velocity * velocity_constant_value);
    double right_velocity = linear_speed + (0.162 * angular_speed / 2);
    *right_rpm = (int)(right_velocity * velocity_constant_value);
}
std::string LineTracer::mat_type2encoding(int mat_type)
{//카메라 모듈으로 영상을 확인하기위해 Display Image Format에 맞게 Image Format 설정
    switch (mat_type) {
    case CV_8UC1:
        return "mono8";
    case CV_8UC3:
        return "bgr8";
    case CV_16SC1:
        return "mono16";
    case CV_8UC4:
        return "rgba8";
    default:
        throw std::runtime_error("Unsupported encoding type");//에러메시지 출력
    }
}
void LineTracer::convert_frame_to_message(const cv::Mat& frame, sensor_msgs::msg::Image& msg)
{
    // copy cv information into ros message
    msg.height = frame.rows;//멤버 변수에 영상의 세로 넓이 저장
    msg.width = frame.cols;//멤버 변수에 영상의 가로 넓이 저장
    msg.encoding = mat_type2encoding(frame.type());//영상 타입 변환 ->encoding
    msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
    //영상의 _step_type을 msg의 멤버 변수에 저장
    size_t size = frame.step * frame.rows;//각행의 스텝 크기 저장
    msg.data.resize(size);//resize함수 
    memcpy(&msg.data[0], frame.data, size);//(memory + copy)
    //인자 1:복사 받을 메모리의 주소
    //인자 2:복사 할 메모리
    //인자 3:복사 할 데이터의 길이
    //msg.header.frame_id = std::to_string(frame_id);
    msg.header.frame_id = "camera";//frame_id 지정
}
