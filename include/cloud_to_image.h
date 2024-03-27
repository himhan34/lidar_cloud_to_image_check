#ifndef CLOUD_TO_IMAGE_H_ // 헤더 파일이 중복으로 인클루드되는 것을 방지하기 위한 헤더 가드
#define CLOUD_TO_IMAGE_H_

#include "projection_params.h" // 프로젝션 파라미터 헤더 파일을 인클루드
#include "cloud_projection.h" // 클라우드 프로젝션 헤더 파일을 인클루드

#include <ros/ros.h> // ROS 라이브러리를 사용하기 위한 헤더 파일을 인클루드
#include <std_msgs/Header.h> // ROS의 std_msgs/Header 메시지 타입 헤더 파일을 인클루드
#include <sensor_msgs/PointCloud2.h> // ROS의 sensor_msgs/PointCloud2 메시지 타입 헤더 파일을 인클루드
#include <pcl/point_types.h> // PCL의 포인트 타입 헤더 파일을 인클루드
#include <pcl_ros/point_cloud.h> // PCL과 ROS 간 포인트 클라우드 메시지 변환을 위한 헤더 파일을 인클루드


namespace cloud_to_image {

// 포인트 클라우드 데이터를 이미지로 변환하는 클래스
class CloudToImage
{
    public:
        // 이미지 출력 모드를 나타내는 열거형
        enum class ImageOutputMode { SINGLE, GROUP, STACK, ALL };

        // 생성자
        explicit CloudToImage();
        // 소멸자
        ~CloudToImage();

        // 초기화 함수
        void init(int argc, char* argv[]);
        // 포인트 클라우드 데이터 콜백 함수
        void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& input);
        // 이미지 발행 함수
        void publishImages(const std_msgs::Header& header);
        // 이미지 저장 함수
        void saveImages(const std::string& base_name = std::string("cloud2image"));

    private:
        // ROS 노드 핸들에서 파라미터를 가져오는 템플릿 함수
        template <typename T>
        T getParam(const ros::NodeHandle& nh, const std::string& param_name, T default_value);

        // 메모리 해제 방지를 위한 템플릿 함수
        template <class T>
        static void DoNotFree(T*);

        // ROS 시간을 문자열로 변환하는 템플릿 함수
        template<class T>
        static std::string timeToStr(T ros_t);

private:
        CloudProjection* _cloud_proj; // 클라우드 프로젝션 객체에 대한 포인터

        std::string _cloud_topic; // 포인트 클라우드 토픽의 이름
        std::string _sensor_model; // 센서 모델의 이름
        std::string _point_type; // 포인트 타입의 이름
        std::string _depth_image_topic; // 깊이 이미지 토픽의 이름
        std::string _intensity_image_topic; // 강도 이미지 토픽의 이름
        std::string _reflectance_image_topic; // 반사율 이미지 토픽의 이름
        std::string _noise_image_topic; // 잡음 이미지 토픽의 이름

        bool _has_depth_image; // 깊이 이미지가 있는지 여부
        bool _has_intensity_image; // 강도 이미지가 있는지 여부
        bool _has_reflectance_image; // 반사율 이미지가 있는지 여부
        bool _has_noise_image; // 잡음 이미지가 있는지 여부

        bool _save_images; // 이미지를 저장할지 여부
        bool _8bpp; // 8비트 픽셀 표현 여부
        bool _equalize; // 히스토그램 평활화 여부
        bool _flip; // 이미지 뒤집기 여부

        float _horizontal_scale; // 수평 스케일
        float _vertical_scale; // 수직 스케일

        unsigned int _overlapping; // 중첩

        ImageOutputMode _output_mode; // 이미지 출력 모드

        cv::Mat _depth_image; // 깊이 이미지
        cv::Mat _intensity_image; // 강도 이미지
        cv::Mat _reflectance_image; // 반사율 이미지
        cv::Mat _noise_image; // 잡음 이미지
        cv::Mat _group_image; // 그룹 이미지
        cv::Mat _stack_image; // 스택 이미지

        ros::NodeHandle _nodehandle; // ROS 노드 핸들
        ros::Subscriber _sub_PointCloud; // 포인트 클라우드 구독자
        ros::Publisher _pub_DepthImage; // 깊이 이미지 발행자
        ros::Publisher _pub_IntensityImage; // 강도 이미지 발행자
        ros::Publisher _pub_ReflectanceImage; // 반사율 이미지 발행자
        ros::Publisher _pub_NoiseImage; // 잡음 이미지 발행자
        ros::Publisher _pub_GroupImage; // 그룹 이미지 발행자
        ros::Publisher _pub_StackImage; // 스택 이미지 발행자
};

} // namespace cloud_to_image

#endif // CLOUD_TO_IMAGE_H_
