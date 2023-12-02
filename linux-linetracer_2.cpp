
#include "linetracer/linetracer.hpp"//헤더파일 경로



class Linux : public rclcpp::Node{
    //Node 클래스를 상속받는 Linux 클래스 정의
public:
    Linux():Node("linetracer"), count_(1), width_(320), height_(240)
    {//생성자 정의 및 멤버 변수 초기화
        size_t depth = rmw_qos_profile_default.depth;
        //depth크기를 default값인 10으로 저장(Qos설정)   
        rmw_qos_reliability_policy_t reliability_policy = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
           //Reliadility의 Best_effort 사용(퍼블리시 할때 호환성 설정)
        rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history;
        //history default값인 KEEP_LAST 저장(Qos설정)
        auto qos_profile = rclcpp::QoS(rclcpp::QoSInitialization(history_policy,depth));
        //history_policy값과 depth 값을 생성자 인수 전달 및 QOs구성
        qos_profile.reliability(reliability_policy);
        //위에서 설정한 qos의 reliability를 best_effort로 지정
        auto dynamix_qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
        //다이나믹셀 관련 Qos설정
 
 
        camera_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image", qos_profile);
        //이름 Image 및 qos_profile(Qos) 퍼블리셔 초기화
        dynamixel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
             "cmd_vel", dynamix_qos_profile, std::bind(&Linux::subscribe_topic_message, this, _1));
        //이름 cmd_vel (Qos) 서브스크라이브 초기화
        timer_ = this->create_wall_timer(20ms, std::bind(&Linux::publish_image, this));
         //20ms 주기마다 publish_image 함수 호출
        cap_.open(src_, cv::CAP_GSTREAMER);
        //GSTREAMER을 이용하여 캠 열어주는 함수

       
        if (!cap_.isOpened()) { 
            RCLCPP_ERROR(this->get_logger(), "Could not open video stream");        
        }
        if(!dxl.dxl_open()) RCLCPP_ERROR(this->get_logger(), "dynamixel open error");
    }
    ~Linux() 
    {
        dxl.dxl_close(); 
    }
private: 
  void publish_image() 
  {
    TickMeter tm;
	tm.start();
 
    auto msg = std::make_unique<sensor_msgs::msg::Image>();
    //Image 타입의 스마트 포인터
    msg->is_bigendian = false;
    //인수 지정->flase(데이터 저장 방식)
    
    cap_ >> frame_;//캠으로부터 영상 받아와 객체에 저장
 
    //멤버 변수 cap_을 frame_에 대입
    if (!frame_.empty())//프레임 비어 있지 않으면
    {
        dxl.convert_frame_to_message(frame_, *msg); //프레임을 메시지로 변환
 
        camera_publisher_->publish(std::move(msg)); //메시지 형태로 퍼블리셔    
    }
    else { 
        RCLCPP_INFO(this->get_logger(), "frame empty!"); 
    }
    tm.stop();
	cout << "Image Pub_process : " << tm.getTimeMilli() << "ms" << endl;
  }
  void subscribe_topic_message(const geometry_msgs::msg::Twist::SharedPtr msg)
    //함수정의 
    {
        
         RCLCPP_INFO(this->get_logger(), "Received message: %lf,%lf", msg->linear.x,msg->angular.z);//정보 전달
        int get_rpm1=0,get_rpm2=0; 
        dxl.vel_convert_rpm(msg->linear.x,msg->angular.z,&get_rpm1,&get_rpm2);
        bool result = dxl.dxl_set_velocity(get_rpm1,-get_rpm2);
        cout<<get_rpm1<<" : "<<-get_rpm2<<endl;
        //bool result = dxl.dxl_set_velocity(msg->linear.x, -(msg->linear.y));
        if(result == false) 
        {
            RCLCPP_ERROR(this->get_logger(), "dxl_set_velocity error"); 
        }
      
    }
    //멤버 변수
    LineTracer dxl;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr dynamixel_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_; 
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr camera_publisher_;
    size_t count_; 
    cv::VideoCapture cap_; 
 
    std::string src_ = "nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width=(int)320, height=(int)240, \
                        format=(string)NV12, framerate=(fraction)30/1 ! nvvidconv flip-method=0 ! video/x-raw, \
                        width=(int)320,height=(int)240, format=(string)BGRx ! \
                       videoconvert ! video/x-raw, format=(string)BGR ! appsink";
    
    cv::Mat frame_;
    size_t width_; 
    size_t height_; 
};
int main(int argc, char* argv[]) 
{
    rclcpp::init(argc, argv);
    //초기화 
    auto node = std::make_shared<Linux>();
   //작성 한 클래스(스마트 포인터->할당 메모리 자동 해제) 노드 변수에 생성
    rclcpp::spin(node); 
    //노드 spin시켜 콜백 함수 실행 유지
    rclcpp::shutdown(); 
    //종료(Ctrl +c)와 같은 인터럽트 시그널 예외 상황 ->노드 소멸 및 프로세스 종료 
 
    return 0; 
}
