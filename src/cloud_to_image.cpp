#include "cloud_to_image.h" // cloud_to_image 헤더 파일을 포함합니다.
#include "projection_params.h" // projection_params 헤더 파일을 포함합니다.
#include "cloud_projection.h" // cloud_projection 헤더 파일을 포함합니다.
#include <iostream> // 표준 입출력 스트림 헤더 파일을 포함합니다.
#include <fstream> // 파일 입출력 헤더 파일을 포함합니다.

#include <ros/ros.h> // ROS 관련 헤더 파일을 포함합니다.
#include <sensor_msgs/image_encodings.h> // 센서 메시지 인코딩 헤더 파일을 포함합니다.
#include <std_msgs/Header.h> // 표준 메시지 헤더 헤더 파일을 포함합니다.

#include <opencv2/core/core.hpp> // OpenCV의 핵심 기능 헤더 파일을 포함합니다.
#include <image_transport/image_transport.h> // 이미지 전송 관련 헤더 파일을 포함합니다.
#include <opencv2/highgui/highgui.hpp> // OpenCV의 고수준 GUI 기능 헤더 파일을 포함합니다.
#include <cv_bridge/cv_bridge.h> // OpenCV와 ROS 이미지 포맷 변환을 위한 헤더 파일을 포함합니다.

#include <boost/algorithm/string.hpp> // 문자열 조작을 위한 부스트 헤더 파일을 포함합니다.


namespace cloud_to_image
{

// CloudToImage 클래스의 생성자를 정의합니다.
CloudToImage::CloudToImage():
    _has_depth_image(false), // 깊이 이미지 여부를 나타내는 플래그를 초기화합니다.
    _has_intensity_image(false), // 강도 이미지 여부를 나타내는 플래그를 초기화합니다.
    _has_reflectance_image(false), // 반사 이미지 여부를 나타내는 플래그를 초기화합니다.
    _has_noise_image(false), // 잡음 이미지 여부를 나타내는 플래그를 초기화합니다.
    _save_images(false), // 이미지 저장 여부를 나타내는 플래그를 초기화합니다.
    _8bpp(false), // 8비트 이미지 여부를 나타내는 플래그를 초기화합니다.
    _equalize(false), // 히스토그램 평활화 여부를 나타내는 플래그를 초기화합니다.
    _flip(false), // 이미지 뒤집기 여부를 나타내는 플래그를 초기화합니다.
    _overlapping(0), // 이미지 중첩 정도를 나타내는 변수를 초기화합니다.
    _output_mode(ImageOutputMode::SINGLE) // 출력 모드를 나타내는 변수를 초기화합니다.
{
    // 깊이 이미지를 나타내는 OpenCV Mat 객체를 초기화합니다.
    _depth_image = cv::Mat::zeros(1, 1, CV_32FC1);
    // 강도 이미지를 나타내는 OpenCV Mat 객체를 초기화합니다.
    _intensity_image = cv::Mat::zeros(1, 1, CV_16UC1);
    // 반사 이미지를 나타내는 OpenCV Mat 객체를 초기화합니다.
    _reflectance_image = cv::Mat::zeros(1, 1, CV_16UC1);
    // 잡음 이미지를 나타내는 OpenCV Mat 객체를 초기화합니다.
    _noise_image = cv::Mat::zeros(1, 1, CV_16UC1);
    // 그룹 이미지를 나타내는 OpenCV Mat 객체를 초기화합니다.
    _group_image = cv::Mat::zeros(1, 1, CV_16UC1);
    // 스택 이미지를 나타내는 OpenCV Mat 객체를 초기화합니다.
    _stack_image = cv::Mat::zeros(1, 1, CV_16UC3);
}

CloudToImage::~CloudToImage() 
{
    // 클라우드 프로젝션 객체가 할당되어 있는지 확인하고 메모리를 해제합니다.
    if (_cloud_proj) {
        delete _cloud_proj;
    }
    // ROS 매개변수 서버에서 해당 매개변수들을 삭제합니다.
    _nodehandle.deleteParam("/cloud2image/cloud_topic");
    _nodehandle.deleteParam("/cloud2image/proj_params");
    _nodehandle.deleteParam("/cloud2image/sensor_model");
    _nodehandle.deleteParam("/cloud2image/point_type");

    _nodehandle.deleteParam("/cloud2image/depth_image_topic");
    _nodehandle.deleteParam("/cloud2image/intensity_image_topic");
    _nodehandle.deleteParam("/cloud2image/reflectance_image_topic");
    _nodehandle.deleteParam("/cloud2image/noise_image_topic");
    
    _nodehandle.deleteParam("/cloud2image/equalize"); // 히스토그램 평활화에 대한 매개변수를 삭제합니다.
}


template <typename T>
T CloudToImage::getParam(const ros::NodeHandle& nh, const std::string& param_name, T default_value)
{
    // 반환할 값의 변수를 선언합니다.
    T value;
    // 노드 핸들러가 주어진 매개변수 이름을 가지고 있는지 확인합니다.
    if (nh.hasParam(param_name))
    {
        // 매개변수가 존재하면 그 값을 가져옵니다.
        nh.getParam(param_name, value);
    }
    else
    {
        // 매개변수가 존재하지 않을 경우 경고 메시지를 출력하고 기본값으로 설정합니다.
        ROS_WARN_STREAM("Parameter '" << param_name << "' not found, defaults to '" << default_value << "'");
        value = default_value;
    }
    // 설정된 값을 반환합니다.
    return value;
}

void CloudToImage::init(int argc, char* argv[])
{
    // 인수 argc와 argv는 사용하지 않으므로 무시합니다.
    (void)argc;
    (void)argv;
    std::string config_filename; // 설정 파일 이름을 저장할 변수를 선언합니다.
    _nodehandle = ros::NodeHandle("~"); // 프라이빗 네임스페이스로 노드 핸들러를 초기화합니다.

    // 모든 매개변수를 읽어옵니다.
    config_filename = getParam(_nodehandle, "/cloud2image/proj_params", std::string("")); // 프로젝션 매개변수를 읽어옵니다.

    _cloud_topic = getParam(_nodehandle, "/cloud2image/cloud_topic", std::string("/points_raw")); // 클라우드 토픽을 읽어옵니다.
    _sensor_model = getParam(_nodehandle, "/cloud2image/sensor_model", std::string("VLP-16")); // 센서 모델을 읽어옵니다.
    _point_type = getParam(_nodehandle, "/cloud2image/point_type", std::string("XYZI")); // 포인트 유형을 읽어옵니다.

    _depth_image_topic = getParam(_nodehandle, "/cloud2image/depth_image_topic", std::string("/c2i_depth_image")); // 깊이 이미지 토픽을 읽어옵니다.
    _intensity_image_topic = getParam(_nodehandle, "/cloud2image/intensity_image_topic", std::string("/c2i_intensity_image")); // 강도 이미지 토픽을 읽어옵니다.
    _reflectance_image_topic = getParam(_nodehandle, "/cloud2image/reflectance_image_topic", std::string("/c2i_reflectance_image")); // 반사 이미지 토픽을 읽어옵니다.
    _noise_image_topic = getParam(_nodehandle, "/cloud2image/noise_image_topic", std::string("/c2i_noise_image")); // 잡음 이미지 토픽을 읽어옵니다.

    _horizontal_scale = getParam(_nodehandle, "/cloud2image/h_scale", 1.0); // 수평 스케일을 읽어옵니다.
    _vertical_scale = getParam(_nodehandle, "/cloud2image/v_scale", 1.0); // 수직 스케일을 읽어옵니다.
    std::string output_mode = getParam(_nodehandle, "/cloud2image/output_mode", std::string("SINGLE")); // 출력 모드를 읽어옵니다.

    _save_images = getParam(_nodehandle, "/cloud2image/save_images", false); // 이미지 저장 여부를 읽어옵니다.
    _overlapping = getParam(_nodehandle, "/cloud2image/overlapping", 0); // 이미지 중첩 정도를 읽어옵니다.

    _8bpp = getParam(_nodehandle, "/cloud2image/8bpp", false); // 8비트 이미지 여부를 읽어옵니다.
    _equalize = getParam(_nodehandle, "/cloud2image/equalize", false); // 히스토그램 평활화 여부를 읽어옵니다.
    _flip = getParam(_nodehandle, "/cloud2image/flip", false); // 이미지 뒤집기 여부를 읽어옵니다.

    _equalize = _equalize && _8bpp; // 8비트 이미지가 아닐 경우 히스토그램 평활화를 사용하지 않습니다.

	if (boost::iequals(output_mode, "SINGLE")) { // 출력 모드가 "SINGLE"인 경우
	    _output_mode = ImageOutputMode::SINGLE; // 출력 모드를 단일로 설정합니다.
	} else if (boost::iequals(output_mode, "GROUP")) { // 출력 모드가 "GROUP"인 경우
	    _output_mode = ImageOutputMode::GROUP; // 출력 모드를 그룹으로 설정합니다.
	} else if (boost::iequals(output_mode, "STACK")) { // 출력 모드가 "STACK"인 경우
	    _output_mode = ImageOutputMode::STACK; // 출력 모드를 스택으로 설정합니다.
	} else if (boost::iequals(output_mode, "ALL")) { // 출력 모드가 "ALL"인 경우
	    _output_mode = ImageOutputMode::ALL; // 출력 모드를 모두로 설정합니다.
	} else {
	    // 지원되지 않는 이미지 출력 모드일 경우 예외를 던집니다.
	    throw std::runtime_error("image output mode \"" + output_mode + "\" not supported");
	}


	if (boost::iequals(_point_type, "XYZ")) {  // 강도가 없는 경우 (예: Velodyne, Ouster)
	    _has_depth_image = true; // 깊이 이미지가 있음을 설정합니다.
	    _output_mode = ImageOutputMode::SINGLE; // 깊이 이미지만 있으므로 출력 모드를 단일로 설정합니다.
	} else if (boost::iequals(_point_type, "XYZI")) { // 강도가 있는 경우 (예: Velodyne, Ouster)
	    _has_depth_image = true; // 깊이 이미지가 있음을 설정합니다.
	    _has_intensity_image = true; // 강도 이미지가 있음을 설정합니다.
	} else if (boost::iequals(_point_type, "XYZIR")) { // 강도가 있는 경우 (예: Velodyne, Ouster)
	    _has_depth_image = true; // 깊이 이미지가 있음을 설정합니다.
	    _has_intensity_image = true; // 강도 이미지가 있음을 설정합니다.
	} else if (boost::iequals(_point_type, "XYZIF")) { // 강도와 반사율이 있는 경우 (예: Ouster)
	    _has_depth_image = true; // 깊이 이미지가 있음을 설정합니다.
	    _has_intensity_image = true; // 강도 이미지가 있음을 설정합니다.
	    _has_reflectance_image = true; // 반사 이미지가 있음을 설정합니다.
	} else if (boost::iequals(_point_type, "XYZIFN")) { // 강도와 반사율 및 잡음이 있는 경우 (예: Ouster)
	    _has_depth_image = true; // 깊이 이미지가 있음을 설정합니다.
	    _has_intensity_image = true; // 강도 이미지가 있음을 설정합니다.
	    _has_reflectance_image = true; // 반사 이미지가 있음을 설정합니다.
	    _has_noise_image = true; // 잡음 이미지가 있음을 설정합니다.
	} else {
	    // 지원되지 않는 포인트 유형일 경우 예외를 던집니다.
	    throw std::runtime_error("point type \"" + _point_type + "\" not supported");
	}

	// 구성 파일을 확인합니다.
	std::ifstream config(config_filename.c_str());
	if (!config.good()) {
	    throw std::runtime_error("Projection parameters file \"" + config_filename + "\" does not exist or invalid path");	
	}
	
	// 구성 파일에서 투영 매개변수를 로드합니다.
	ProjectionParams proj_params;
	proj_params.loadFromFile(config_filename);
	
	// 센서 모델을 확인합니다.
	if (!proj_params.sensorExists(_sensor_model)) {
	    throw std::runtime_error("Sensor model \"" + _sensor_model + "\" does not exist in configuration file");	
	}
	
	// 센서 모델에 대한 클라우드 투영을 생성합니다.
	_cloud_proj = new CloudProjection(*proj_params[_sensor_model]);
	
	// Velodyne에 대해 Mossman 보정이 도움이 되는 것으로 보입니다.
	if (boost::iequals(_sensor_model, "HDL-64") || boost::iequals(_sensor_model, "HDL-32") || boost::iequals(_sensor_model, "VLP-16")) {
	    _cloud_proj->loadMossmanCorrections();
	}
	
	// 구독자와 발행자를 생성합니다.
	_sub_PointCloud = _nodehandle.subscribe(_cloud_topic, 10, &CloudToImage::pointCloudCallback, this);

	
	// 발행자를 생성합니다.
	// 출력 모드가 그룹 또는 모든 경우인 경우
	if (_output_mode == ImageOutputMode::GROUP || _output_mode == ImageOutputMode::ALL) {
	    // 그룹 이미지를 발행합니다.
	    _pub_GroupImage = _nodehandle.advertise<sensor_msgs::Image>(std::string("/c2i_group_image"), 10);
	} 
	
	// 출력 모드가 스택 또는 모든 경우인 경우
	if (_output_mode == ImageOutputMode::STACK || _output_mode == ImageOutputMode::ALL) {
	    // 스택 이미지를 발행합니다.
	    _pub_StackImage = _nodehandle.advertise<sensor_msgs::Image>(std::string("/c2i_stack_image"), 10);
	} 
	
	// 출력 모드가 단일 또는 모든 경우인 경우
	if (_output_mode == ImageOutputMode::SINGLE || _output_mode == ImageOutputMode::ALL) {
	    // 깊이 이미지를 발행합니다.
	    _pub_DepthImage = _nodehandle.advertise<sensor_msgs::Image>(_depth_image_topic, 10);
	    // 강도 이미지를 발행합니다.
	    _pub_IntensityImage = _nodehandle.advertise<sensor_msgs::Image>(_intensity_image_topic, 10);
	    // 반사 이미지를 발행합니다.
	    _pub_ReflectanceImage = _nodehandle.advertise<sensor_msgs::Image>(_reflectance_image_topic, 10);
	    // 잡음 이미지를 발행합니다.
	    _pub_NoiseImage = _nodehandle.advertise<sensor_msgs::Image>(_noise_image_topic, 10);
	} 


void CloudToImage::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
    // 이전 투영 데이터를 지웁니다.
    _cloud_proj->clearData();

    // 포인트 유형에 따라 투영을 초기화합니다.

    // 포인트 유형이 "XYZ"인 경우
    if (boost::iequals(_point_type, "XYZ")) {
        // 포인트 클라우드를 XYZ 형식으로 변환합니다.
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*input, *cloud_ptr);
        // 변환된 클라우드를 상수 포인터로 전달하며 초기화합니다.
        const pcl::PointCloud<pcl::PointXYZ>::ConstPtr c_cloud_ptr(&(*cloud_ptr), &CloudToImage::DoNotFree< pcl::PointCloud<pcl::PointXYZ> >);
        _cloud_proj->initFromPoints(c_cloud_ptr);
    } 
    // 포인트 유형이 "XYZIR"인 경우
    else if (boost::iequals(_point_type, "XYZIR")) {
        // 포인트 클라우드를 XYZIR 형식으로 변환합니다.
        pcl::PointCloud<pcl::PointXYZIR>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZIR>);
        pcl::fromROSMsg(*input, *cloud_ptr);
        // 변환된 클라우드를 상수 포인터로 전달하며 초기화합니다.
        const pcl::PointCloud<pcl::PointXYZIR>::ConstPtr c_cloud_ptr(&(*cloud_ptr), &CloudToImage::DoNotFree< pcl::PointCloud<pcl::PointXYZIR> >);
        _cloud_proj->initFromPoints(c_cloud_ptr);
    } 
    // 포인트 유형이 "XYZI"인 경우
    else if (boost::iequals(_point_type, "XYZI")) {
        // 포인트 클라우드를 XYZI 형식으로 변환합니다.
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*input, *cloud_ptr);
        // 변환된 클라우드를 상수 포인터로 전달하며 초기화합니다.
        const pcl::PointCloud<pcl::PointXYZI>::ConstPtr c_cloud_ptr(&(*cloud_ptr), &CloudToImage::DoNotFree< pcl::PointCloud<pcl::PointXYZI> >);
        _cloud_proj->initFromPoints(c_cloud_ptr);
    } 
    // 포인트 유형이 "XYZIF"인 경우
    else if (boost::iequals(_point_type, "XYZIF")) {
        // 포인트 클라우드를 XYZIF 형식으로 변환합니다.
        pcl::PointCloud<pcl::PointXYZIF>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZIF>);
        pcl::fromROSMsg(*input, *cloud_ptr);
        // 변환된 클라우드를 상수 포인터로 전달하며 초기화합니다.
        const pcl::PointCloud<pcl::PointXYZIF>::ConstPtr c_cloud_ptr(&(*cloud_ptr), &CloudToImage::DoNotFree< pcl::PointCloud<pcl::PointXYZIF> >);
        _cloud_proj->initFromPoints(c_cloud_ptr);
    } 
    // 포인트 유형이 "XYZIFN"인 경우
    else if (boost::iequals(_point_type, "XYZIFN")) {
        // 포인트 클라우드를 XYZIFN 형식으로 변환합니다.
        pcl::PointCloud<pcl::PointXYZIFN>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZIFN>);
        pcl::fromROSMsg(*input, *cloud_ptr);
        // 변환된 클라우드를 상수 포인터로 전달하며 초기화합니다.
        const pcl::PointCloud<pcl::PointXYZIFN>::ConstPtr c_cloud_ptr(&(*cloud_ptr), &CloudToImage::DoNotFree< pcl::PointCloud<pcl::PointXYZIFN> >);
        _cloud_proj->initFromPoints(c_cloud_ptr);
    }
}


	// 각 이미지의 로컬 복사본을 가져옵니다.
	_depth_image = _cloud_proj->depth_image(); // 깊이 이미지의 로컬 복사본을 가져옵니다.
	_intensity_image = _cloud_proj->intensity_image(); // 강도 이미지의 로컬 복사본을 가져옵니다.
	_reflectance_image = _cloud_proj->reflectance_image(); // 반사 이미지의 로컬 복사본을 가져옵니다.
	_noise_image = _cloud_proj->noise_image(); // 잡음 이미지의 로컬 복사본을 가져옵니다.
	
	// 중첩이 있으면
	if (_overlapping) {
	    // 각 이미지의 왼쪽 영역을 가져옵니다.
	    cv::Mat left_roi = _depth_image.colRange(cv::Range(0,_overlapping)); // 깊이 이미지의 왼쪽 영역을 가져옵니다.
	    // 왼쪽 영역을 현재 이미지에 연결합니다.
	    cv::hconcat(_depth_image, left_roi, _depth_image); // 현재 깊이 이미지와 왼쪽 영역을 수평으로 결합합니다.
	    // 각 이미지의 왼쪽 영역을 가져옵니다.
	    left_roi = _intensity_image.colRange(cv::Range(0,_overlapping)); // 강도 이미지의 왼쪽 영역을 가져옵니다.
	    // 왼쪽 영역을 현재 이미지에 연결합니다.
	    cv::hconcat(_intensity_image, left_roi, _intensity_image); // 현재 강도 이미지와 왼쪽 영역을 수평으로 결합합니다.
	    // 각 이미지의 왼쪽 영역을 가져옵니다.
	    left_roi = _reflectance_image.colRange(cv::Range(0,_overlapping)); // 반사 이미지의 왼쪽 영역을 가져옵니다.
	    // 왼쪽 영역을 현재 이미지에 연결합니다.
	    cv::hconcat(_reflectance_image, left_roi, _reflectance_image); // 현재 반사 이미지와 왼쪽 영역을 수평으로 결합합니다.
	    // 각 이미지의 왼쪽 영역을 가져옵니다.
	    left_roi = _noise_image.colRange(cv::Range(0,_overlapping)); // 잡음 이미지의 왼쪽 영역을 가져옵니다.
	    // 왼쪽 영역을 현재 이미지에 연결합니다.
	    cv::hconcat(_noise_image, left_roi, _noise_image); // 현재 잡음 이미지와 왼쪽 영역을 수평으로 결합합니다.
	}
	
	// 이미지를 발행합니다.
	publishImages(input->header); // 입력 메시지의 헤더 정보를 이용하여 이미지를 발행합니다.

}


void CloudToImage::publishImages(const std_msgs::Header& header)
{
    auto mode = CV_16UC1; // 이미지의 데이터 타입을 나타내는 변수를 선언하고 기본적으로 16비트 부호 없는 정수로 설정합니다.
    auto min_range = 0.0; // 이미지의 최소 범위를 나타내는 변수를 선언하고 기본값으로 0을 설정합니다.
    auto max_range = 65535.0; // 이미지의 최대 범위를 나타내는 변수를 선언하고 기본값으로 65535를 설정합니다.
    auto encoding = sensor_msgs::image_encodings::MONO16; // 이미지의 인코딩 방식을 나타내는 변수를 선언하고 MONO16으로 설정합니다.
   
    // 만약 8비트 이미지로 설정되어 있다면
    if (_8bpp) {
        mode = CV_8UC1; // 이미지의 데이터 타입을 8비트 부호 없는 정수로 설정합니다.
        max_range = 255; // 이미지의 최대 범위를 255로 설정합니다.
        encoding = sensor_msgs::image_encodings::MONO8; // 이미지의 인코딩 방식을 MONO8로 설정합니다.
    }

	
	if (_output_mode == ImageOutputMode::SINGLE || _output_mode == ImageOutputMode::ALL) {
	    // 단일 출력 모드 또는 모든 출력 모드인 경우
	
	    // 깊이 이미지가 있고 해당 이미지에 구독자가 있는 경우
	    if (_has_depth_image && _pub_DepthImage.getNumSubscribers() > 0) {
	        // 깊이 이미지는 부동 소수점 형식이므로 시각화를 위해 정규화합니다.
	        cv::Mat mono_img = cv::Mat(_depth_image.size(), mode);
	        cv::normalize(_depth_image, mono_img, min_range, max_range, cv::NORM_MINMAX, mode); // 이미지를 정규화합니다.
	        cv::resize(mono_img, mono_img, cv::Size(0,0), _horizontal_scale, _vertical_scale, CV_INTER_LINEAR); // 이미지의 크기를 조정합니다.
		    
	        // 히스토그램 평활화를 수행합니다.
	        if (_equalize) cv::equalizeHist( mono_img, mono_img );
	        // 이미지를 뒤집습니다.
	        if (_flip) cv::flip( mono_img, mono_img, 1);
		    
	        // OpenCV 이미지를 ROS 이미지 메시지로 변환합니다.
	        sensor_msgs::ImagePtr depth_img = cv_bridge::CvImage(header, encoding, mono_img).toImageMsg();
		    
	        // ROS 토픽에 깊이 이미지를 발행합니다.
	        _pub_DepthImage.publish(depth_img);
		    
	        // 만약 8비트 이미지이면, 깊이 이미지를 업데이트합니다.
	        if (_8bpp) _depth_image = mono_img;
	    }
		if (_has_intensity_image && _pub_IntensityImage.getNumSubscribers() > 0) {
	    // 이미지를 단일 채널 형식으로 변환합니다.
	    cv::Mat mono_img = cv::Mat(_intensity_image.size(), mode);
			
	    // 이미지를 정규화합니다.
	    cv::normalize(_intensity_image, mono_img, min_range, max_range, cv::NORM_MINMAX, mode);
			
	    // 이미지의 크기를 조정합니다.
	    cv::resize(mono_img, mono_img, cv::Size(0,0), _horizontal_scale, _vertical_scale, CV_INTER_LINEAR);
			
	    // 히스토그램 평활화를 수행합니다.
	    if (_equalize) cv::equalizeHist( mono_img, mono_img );
			
	    // 이미지를 뒤집습니다.
	    if (_flip) cv::flip( mono_img, mono_img, 1);
			
	    // OpenCV 이미지를 ROS 이미지 메시지로 변환합니다.
	    sensor_msgs::ImagePtr intensity_img = cv_bridge::CvImage(header, encoding, mono_img).toImageMsg();
			
	    // ROS 토픽에 강도 이미지를 발행합니다.
	    _pub_IntensityImage.publish(intensity_img);
			
	    // 만약 8비트 이미지이면, 강도 이미지를 업데이트합니다.
	    if (_8bpp) _intensity_image = mono_img;
	}

		if (_has_reflectance_image && _pub_ReflectanceImage.getNumSubscribers() > 0) {
	    // 반사 이미지가 있고 해당 이미지에 구독자가 있는 경우
	    // 이미지를 단일 채널 형식으로 변환합니다.
	    cv::Mat mono_img = cv::Mat(_reflectance_image.size(), mode);
	    // 이미지를 정규화합니다.
	    cv::normalize(_reflectance_image, mono_img, min_range, max_range, cv::NORM_MINMAX, mode);
	    // 이미지의 크기를 조정합니다.
	    cv::resize(mono_img, mono_img, cv::Size(0,0), _horizontal_scale, _vertical_scale, CV_INTER_LINEAR);
	    // 히스토그램 평활화를 수행합니다.
	    if (_equalize) cv::equalizeHist( mono_img, mono_img );
	    // 이미지를 뒤집습니다.
	    if (_flip) cv::flip( mono_img, mono_img, 1);
	    // OpenCV 이미지를 ROS 이미지 메시지로 변환합니다.
	    sensor_msgs::ImagePtr reflectance_img = cv_bridge::CvImage(header, encoding, mono_img).toImageMsg();
	    // ROS 토픽에 반사 이미지를 발행합니다.
	    _pub_ReflectanceImage.publish(reflectance_img);
	    // 만약 8비트 이미지이면, 반사 이미지를 업데이트합니다.
	    if (_8bpp) _reflectance_image = mono_img;
	}

	if (_has_noise_image && _pub_NoiseImage.getNumSubscribers() > 0) {
	    // 잡음 이미지가 있고 해당 이미지에 구독자가 있는 경우
	
	    // 이미지를 단일 채널 형식으로 변환합니다.
	    cv::Mat mono_img = cv::Mat(_noise_image.size(), mode);
	    // 이미지를 정규화합니다.
	    cv::normalize(_noise_image, mono_img, min_range, max_range, cv::NORM_MINMAX, mode);
	    // 이미지의 크기를 조정합니다.
	    cv::resize(mono_img, mono_img, cv::Size(0,0), _horizontal_scale, _vertical_scale, CV_INTER_LINEAR);
	    // 히스토그램 평활화를 수행합니다.
	    if (_equalize) cv::equalizeHist( mono_img, mono_img );
	    // 이미지를 뒤집습니다.
	    if (_flip) cv::flip( mono_img, mono_img, 1);
	    // OpenCV 이미지를 ROS 이미지 메시지로 변환합니다.
	    sensor_msgs::ImagePtr noise_img = cv_bridge::CvImage(header, encoding, mono_img).toImageMsg();
	    // ROS 토픽에 잡음 이미지를 발행합니다.
	    _pub_NoiseImage.publish(noise_img);
	    // 만약 8비트 이미지이면, 잡음 이미지를 업데이트합니다.
	    if (_8bpp) _noise_image = mono_img;
	}

	} 
	if (_output_mode == ImageOutputMode::GROUP || _output_mode == ImageOutputMode::STACK || _output_mode == ImageOutputMode::ALL) {
		
	    // 깊이 이미지 처리
	    cv::Mat mono_img_depth = cv::Mat(_depth_image.size(), mode);
	    cv::normalize(_depth_image, mono_img_depth, min_range, max_range, cv::NORM_MINMAX, mode);
	    cv::resize(mono_img_depth, mono_img_depth, cv::Size(0,0), _horizontal_scale, _vertical_scale, CV_INTER_LINEAR);
	    if (_equalize) cv::equalizeHist( mono_img_depth, mono_img_depth );
	    if (_flip) cv::flip( mono_img_depth, mono_img_depth, 1 );
	    
	    // 강도 이미지 처리
	    cv::Mat mono_img_intensity = cv::Mat(_intensity_image.size(), mode);
	    cv::normalize(_intensity_image, mono_img_intensity, min_range, max_range, cv::NORM_MINMAX, mode);
	    cv::resize(mono_img_intensity, mono_img_intensity, cv::Size(0,0), _horizontal_scale, _vertical_scale, CV_INTER_LINEAR);
	    if (_equalize) cv::equalizeHist( mono_img_intensity, mono_img_intensity );
	    if (_flip) cv::flip( mono_img_intensity, mono_img_intensity, 1 );
	    
	    // 반사 이미지 처리
	    cv::Mat mono_img_reflectance = cv::Mat(_reflectance_image.size(), mode);
	    cv::normalize(_reflectance_image, mono_img_reflectance, min_range, max_range, cv::NORM_MINMAX, mode);
	    cv::resize(mono_img_reflectance, mono_img_reflectance, cv::Size(0,0), _horizontal_scale, _vertical_scale, CV_INTER_LINEAR);
	    if (_equalize) cv::equalizeHist( mono_img_reflectance, mono_img_reflectance );
	    if (_flip) cv::flip( mono_img_reflectance, mono_img_reflectance, 1 );
	    
	    // 잡음 이미지 처리
	    cv::Mat mono_img_noise = cv::Mat(_noise_image.size(), mode);
	    cv::normalize(_noise_image, mono_img_noise, min_range, max_range, cv::NORM_MINMAX, mode);
	    cv::resize(mono_img_noise, mono_img_noise, cv::Size(0,0), _horizontal_scale, _vertical_scale, CV_INTER_LINEAR);
	    if (_equalize) cv::equalizeHist( mono_img_noise, mono_img_noise );
	    if (_flip) cv::flip( mono_img_noise, mono_img_noise, 1 );
	
	    // 출력할 이미지의 수를 계산합니다.
	    int img_count = 0;
	    img_count += _has_depth_image;
	    img_count += _has_intensity_image;
	    img_count += _has_reflectance_image;
	    img_count += _has_noise_image;


		if (_output_mode == ImageOutputMode::GROUP || _output_mode == ImageOutputMode::ALL) {
	    // 전제 조건: 위의 모든 이미지가 동일한 폭과 높이를 가지고 있어야 합니다.
	    auto group_size = mono_img_depth.size();
	    group_size.height = group_size.height * img_count;
	    cv::Mat mono_img_group = cv::Mat(group_size, mode);
	
	    auto cols = mono_img_depth.cols;
	    auto rows = mono_img_depth.rows;
			
	    img_count = 0;
	    if (_has_depth_image) {
	        // 깊이 이미지를 복사합니다.
	        mono_img_depth.copyTo(mono_img_group(cv::Rect(0, img_count*rows, cols, rows)));
	        img_count++;
	    }
	    if (_has_intensity_image) {
	        // 강도 이미지를 복사합니다.
	        mono_img_intensity.copyTo(mono_img_group(cv::Rect(0, img_count*rows, cols, rows)));
	        img_count++;
	    }
	    if (_has_reflectance_image) {
	        // 반사 이미지를 복사합니다.
	        mono_img_reflectance.copyTo(mono_img_group(cv::Rect(0, img_count*rows, cols, rows)));
	        img_count++;
	    }
	    if (_has_noise_image) {
	        // 잡음 이미지를 복사합니다.
	        mono_img_noise.copyTo(mono_img_group(cv::Rect(0, img_count*rows, cols, rows)));
	        img_count++;
	    }
	    // 로컬 복사본을 가져옵니다.
	    _group_image = mono_img_group;
	
	    // 발행합니다.
	    sensor_msgs::ImagePtr output_img = cv_bridge::CvImage(header, encoding, mono_img_group).toImageMsg();
	    _pub_GroupImage.publish(output_img);
	}

		if (_output_mode == ImageOutputMode::STACK || _output_mode == ImageOutputMode::ALL) {
			// 전제 조건: 위의 모든 이미지가 동일한 폭과 높이를 가지고 있어야 합니다.
			auto format = CV_16UC1;
			auto encoding = sensor_msgs::image_encodings::MONO16;
			if (_8bpp) {
				format = CV_8UC1;
				encoding = sensor_msgs::image_encodings::MONO8;
			}
			if (img_count == 2) {
				format = CV_16UC3;
				encoding = sensor_msgs::image_encodings::BGR16;
				if (_8bpp) {
					format = CV_8UC3;
					encoding = sensor_msgs::image_encodings::BGR8;
				}
			} else if (img_count == 3) {
				format = CV_16UC3;
				encoding = sensor_msgs::image_encodings::BGR16;
				if (_8bpp) {
					format = CV_8UC3;
					encoding = sensor_msgs::image_encodings::BGR8;
				}
			} else if (img_count == 4) {
				format = CV_16UC4;
				encoding = sensor_msgs::image_encodings::BGRA16;
				if (_8bpp) {
					format = CV_8UC4;
					encoding = sensor_msgs::image_encodings::BGRA8;
				}
			}
			cv::Mat mono_img_stack = cv::Mat(mono_img_depth.size(), format); //n channels

			//special case: if only two images, add space for the 3rd channel
			if (img_count == 2) {
				img_count++;
			}
			std::vector<cv::Mat> images(img_count);
			img_count = 0;

			if (_has_depth_image) {
				images.at(img_count++) = mono_img_depth; 
			}
			if (_has_intensity_image) {
				images.at(img_count++) = mono_img_intensity; 
			}
			if (_has_reflectance_image) {
				images.at(img_count++) = mono_img_reflectance;
			}
			if (_has_noise_image) {
				images.at(img_count++) = mono_img_noise;
			}

			// 특수한 경우: 이미지가 두 개일 때, 3번째 채널을 0으로 채웁니다.
			if (img_count == 2) {
			    images.at(img_count) = cv::Mat::zeros(mono_img_depth.rows, mono_img_depth.cols, mode);
			}
			
			// 모든 이미지를 별도의 채널로 결합합니다.
			cv::merge(images, mono_img_stack);
			
			// 로컬 복사본을 가져옵니다.
			_stack_image = mono_img_stack;
			
			// 이미지를 게시합니다.
			sensor_msgs::ImagePtr output_img = cv_bridge::CvImage(header, encoding, mono_img_stack).toImageMsg();
			_pub_StackImage.publish(output_img);

		}
	}

	// 요청된 경우 PNG 파일로 이미지를 출력합니다.
	if (_save_images) {
		saveImages();
	}
}

template<class T>
std::string CloudToImage::timeToStr(T ros_t)
{
    (void)ros_t; // 사용하지 않는 매개변수에 대한 경고를 방지합니다.
    std::stringstream msg; // 문자열을 담을 스트림을 생성합니다.
    // 현재 로컬 시간을 가져옵니다.
    const boost::posix_time::ptime now = boost::posix_time::microsec_clock::local_time();
    // 시간 출력 형식을 설정합니다.
    boost::posix_time::time_facet *const f = new boost::posix_time::time_facet("%Y-%m-%d-%H-%M-%S.%f");
    // 스트림에 출력 형식을 적용합니다.
    msg.imbue(std::locale(msg.getloc(),f));
    // 현재 시간을 스트림에 씁니다.
    msg << now;
    // 스트림에 있는 문자열을 반환합니다.
    return msg.str();
}


void CloudToImage::saveImages(const std::string& base_name)
{
    std::string filename; // 파일 이름을 저장할 변수를 선언합니다.
    
    // 출력 모드가 단일이거나 모든 경우인 경우
    if (_output_mode == ImageOutputMode::SINGLE || _output_mode == ImageOutputMode::ALL) {
        // 깊이 이미지가 있는 경우
        if (_has_depth_image) {
            // 생성된 깊이 이미지를 저장합니다.
            filename = std::string(base_name + "_depth"); // 기본 이름에 "_depth"를 추가합니다.
            filename += std::string("_") + timeToStr(ros::WallTime::now()) + std::string(".png"); // 현재 시간을 포함한 파일 이름을 생성합니다.
            // 8비트 이미지인 경우
            if (_8bpp) {
                // 깊이 이미지를 컬러 PNG 형식으로 저장합니다.
                CloudProjection::cvMatToColorPNG(_depth_image, filename);
            } 
            // 16비트 이미지인 경우
            else {
                // 깊이 이미지를 깊이 PNG 형식으로 저장합니다.
                CloudProjection::cvMatToDepthPNG(_depth_image, filename);
            }
        }

	// 강도 이미지가 있는 경우
	if (_has_intensity_image) {
	    // 생성된 강도 이미지를 저장합니다.
	    filename = std::string(base_name + "_intensity"); // 기본 이름에 "_intensity"를 추가합니다.
	    filename += std::string("_") + timeToStr(ros::WallTime::now()) + std::string(".png"); // 현재 시간을 포함한 파일 이름을 생성합니다.
	    // 8비트 이미지인 경우
	    if (_8bpp) {
		// 강도 이미지를 컬러 PNG 형식으로 저장합니다.
		CloudProjection::cvMatToColorPNG(_intensity_image, filename);
	    } 
	    // 16비트 이미지인 경우
	    else {
		// 강도 이미지를 깊이 PNG 형식으로 저장합니다.
		CloudProjection::cvMatToDepthPNG(_intensity_image, filename);
	    }
	}
	
	// 반사 이미지가 있는 경우
	if (_has_reflectance_image) {
	    // 생성된 반사 이미지를 저장합니다.
	    filename = std::string(base_name + "_reflectance"); // 기본 이름에 "_reflectance"를 추가합니다.
	    filename += std::string("_") + timeToStr(ros::WallTime::now()) + std::string(".png"); // 현재 시간을 포함한 파일 이름을 생성합니다.
	    // 8비트 이미지인 경우
	    if (_8bpp) {
		// 반사 이미지를 컬러 PNG 형식으로 저장합니다.
		CloudProjection::cvMatToColorPNG(_reflectance_image, filename);
	    } 
	    // 16비트 이미지인 경우
	    else {
		// 반사 이미지를 깊이 PNG 형식으로 저장합니다.
		CloudProjection::cvMatToDepthPNG(_reflectance_image, filename);
	    }
	}

		// 잡음 이미지가 있는 경우
		if (_has_noise_image) {
		    // 생성된 잡음 이미지를 저장합니다.
		    filename = std::string(base_name + "_noise"); // 기본 이름에 "_noise"를 추가합니다.
		    filename += std::string("_") + timeToStr(ros::WallTime::now()) + std::string(".png"); // 현재 시간을 포함한 파일 이름을 생성합니다.
		    // 8비트 이미지인 경우
		    if (_8bpp) {
		        // 잡음 이미지를 컬러 PNG 형식으로 저장합니다.
		        CloudProjection::cvMatToColorPNG(_noise_image, filename);
		    } 
		    // 16비트 이미지인 경우
		    else {
		        // 잡음 이미지를 깊이 PNG 형식으로 저장합니다.
		        CloudProjection::cvMatToDepthPNG(_noise_image, filename);
		    }
		}

	} 
	// 출력 모드가 그룹이거나 모든 경우인 경우
	if (_output_mode == ImageOutputMode::GROUP || _output_mode == ImageOutputMode::ALL) {
	    // 생성된 그룹 이미지를 저장합니다.
	    filename = std::string(base_name + "_group"); // 기본 이름에 "_group"을 추가합니다.
	    filename += std::string("_") + timeToStr(ros::WallTime::now()) + std::string(".png"); // 현재 시간을 포함한 파일 이름을 생성합니다.
	    // 그룹 이미지를 컬러 PNG 형식으로 저장합니다.
	    CloudProjection::cvMatToColorPNG(_group_image, filename);
	} 
	
	// 출력 모드가 스택이거나 모든 경우인 경우
	if (_output_mode == ImageOutputMode::STACK || _output_mode == ImageOutputMode::ALL) {
	    // 생성된 스택 이미지를 저장합니다.
	    filename = std::string(base_name + "_stack"); // 기본 이름에 "_stack"을 추가합니다.
	    filename += std::string("_") + timeToStr(ros::WallTime::now()) + std::string(".png"); // 현재 시간을 포함한 파일 이름을 생성합니다.
	    // 스택 이미지를 컬러 PNG 형식으로 저장합니다.
	    CloudProjection::cvMatToColorPNG(_stack_image, filename);
	}

}

}
