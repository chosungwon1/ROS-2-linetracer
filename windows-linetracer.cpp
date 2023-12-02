#include "linetracer/linetracer.hpp"

int encoding2mat_type(const std::string& encoding)
{//메시지 형태를 Opencv에서 사용 가능한 Mat객체로 변경하기 위함
    if (encoding == "mono8") {
        return CV_8UC1;
    }
    else if (encoding == "bgr8") {
        return CV_8UC3;
    }
    else if (encoding == "mono16") {
        return CV_16SC1;
    }
    else if (encoding == "rgba8") {
        return CV_8UC4;
    }
    else if (encoding == "bgra8") {
        return CV_8UC4;
    }
    else if (encoding == "32FC1") {
        return CV_32FC1;
    }
    else if (encoding == "rgb8") {
        return CV_8UC3;
    }
    else {
        throw std::runtime_error("Unsupported encoding type");
    }
}
class Window : public rclcpp::Node 
{//메인 클래스로 rclcpp의 Node클래스를 상속해 사용
	public: 
	Window() : Node("Camera_subscriber")
	{//클래스 생성자의 정의로 부모 클래스(Node)생성자 호출 및 노드 이름 초기화
		
		//Qos관련 설정을 하기위함 ->publisher설정 과 동일
		size_t depth = rmw_qos_profile_default.depth;
		rmw_qos_reliability_policy_t reliability_policy = rmw_qos_profile_default.reliability;
		rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history;
		auto qos_profile = rclcpp::QoS(rclcpp::QoSInitialization(history_policy, depth));
		reliability_policy = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
		qos_profile.reliability(reliability_policy);

		camera_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>("image", qos_profile,
		std::bind(&Window::show_image, this, _1));
		//(_1) ->함수의 인자값을 계속 변경해서 받도록 하겠다는 의미

		auto dynamix_qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
		dynamixel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", dynamix_qos_profile);
		timer_ = this->create_wall_timer(10ms, std::bind(&Window::publish_velcmd_msg, this));
		namedWindow("src", cv::WINDOW_AUTOSIZE); //창 생성
	}
	
	private: 
	void show_image(const sensor_msgs::msg::Image::SharedPtr msg)
	//이미지 출력 해주는 함수
	{
		TickMeter tm;
		tm.start();
		
		RCLCPP_INFO(this->get_logger(), "Received image #%s", msg->header.frame_id.c_str());
		//정보 전달 
		
		Mat frame(msg->height, msg->width, encoding2mat_type(msg->encoding),
		const_cast<unsigned char*>(msg->data.data()), msg->step);
		//frame객체에 서브스크라이브 한 메세지를 Opencv에서 사용 가능한 Mat객체로 생성
		if (msg->encoding == "rgb8") //RGB형태이면
		{ 
			cvtColor(frame, frame, cv::COLOR_RGB2BGR);
			//BGR형태로 변환
		}
		Mat cvframe = frame;
		Mat gray,dst; 
		
		cvtColor(cvframe, gray, 6);
		//GRAY로 변환
		GaussianBlur(gray, gray, Size(), 1);
		//블러링
		threshold(gray, gray, 170, 255, THRESH_BINARY);
		//이진화
		
		dst=gray(Rect(gray.cols/4,gray.rows/2,gray.cols/2,gray.rows/2));
		//관심 영역 추출
		
		Mat labels,status,centroids;
		int cnt=connectedComponentsWithStats(dst,labels,status,centroids);
		//객체 검출 및 정보를 알기위함
		
		int num=0;
		int get_max_labels=0;
		for(int i=1;i<cnt;i++)//영상에서 면적이 가장 큰 객체를 찾기위함
		{
			int *p=status.ptr<int>(i);
			if(p[4]>get_max_labels)
			{
				get_max_labels=p[4];
				num=i;
			}
		}
		double *pa=centroids.ptr<double>(num); 
		//가장 큰 객체의 중심좌표를 저장하는 포인터 변수
		cvtColor(dst,dst,COLOR_GRAY2BGR);
		circle(dst,Point2d(pa[0],pa[1]),3,Scalar(0,0,255),-1);
		error=(gray.cols/2-(pa[0]+gray.cols/4))/2.2;
		//(영상의 중심(x)좌표에서 객체의 중심좌표(x)값를 뺀값+이동한x좌표)/2.2
		
		if(abs(prev_error-error)>20)error=prev_error;
		//에러값의 차이가 클 때 이전 에러값 불러와줌
		tm.stop();
		cout << "Image Sub_process : " << tm.getTimeMilli() << "ms" << endl;
		imshow("dst", dst);
		imshow("src", cvframe); 
		
		
		waitKey(1); 
	}
	void publish_velcmd_msg()
	{//메시지를 전송하기 위한 함수
		msg.linear.x = 0.25 ; //left(0.1일때 선속소 34 0.25 =대략80)
		msg.angular.z = error/28;
		RCLCPP_INFO(this->get_logger(), "Published message: %lf, error : %lf",
		msg.linear.x, msg.angular.z);
		//정보 전달
		dynamixel_publisher_->publish(msg);//메시지 퍼블리시
		prev_error=error;//현재 에러값을 과거 에러 값 저장하는 변수 대입
	}
	//멤버 변수 선언
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_subscriber_;
	double error,prev_error=0.0;
	geometry_msgs::msg::Twist msg;
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr dynamixel_publisher_;
};


int main(int argc, char* argv[]) 
{
	rclcpp::init(argc, argv); 
	//초기화
	auto node = std::make_shared<Window>(); 
	//작성 한 클래스(스마트 포인터->할당 메모리 자동 해제) 노드 변수에 생성
	rclcpp::spin(node);
	//노드 spin시켜 콜백 함수 실행 유지  
	rclcpp::shutdown();
	//종료(Ctrl +c)와 같은 인터럽트 시그널 예외 상황 ->노드 소멸 및 프로세스 종료 
	return 0; 
}
