#include "cloud_to_image.h" // cloud_to_image 헤더 파일을 포함합니다. 이 헤더 파일은 CloudToImage 클래스를 선언합니다.
#include <ros/ros.h> // ROS 관련 기능을 사용하기 위해 ROS 라이브러리를 포함합니다.

using namespace cloud_to_image; // cloud_to_image 네임스페이스를 사용합니다.

int main(int argc, char* argv[]) // main 함수 시작
{
	ros::init(argc, argv, "cloud2image"); // ROS를 초기화하고 노드의 이름을 "cloud2image"로 설정합니다.

	CloudToImage cld2img; // CloudToImage 클래스의 인스턴스를 생성합니다.
	cld2img.init(argc, argv); // 생성된 인스턴스를 초기화합니다.

	ros::spin(); // ROS 이벤트 루프를 시작하고 메시지를 수신하기 위해 대기합니다.

	return 0; // 프로그램을 종료하고 0을 반환합니다.
} // main 함수 종료
