
#ifndef LINETRACER_HPP_
#define LINETRACER_HPP_

#include <functional>
#include <memory>
#include <cstdio>
#include <iostream>
#include <string>
#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "linetracer/linetracer.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <conio.h>
#include <utility>
#include <sstream>

using namespace std;
using namespace cv;
using namespace std::chrono_literals;
using std::placeholders::_1;
//bind 함수 호출시 인자를 사용해야 하는데 이는 bind 함수가 호출될 때로 초기화->변경 불가
//std:placeholdse ->인자값을 새로운 함수의 인자로 받을 수 있도록 해준다


int encoding2mat_type(const std::string& encoding);

#endif  

