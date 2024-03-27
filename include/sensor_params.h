
#ifndef SENSOR_PARAMS_H_ // SENSOR_PARAMS_H_ 헤더 파일 중복 인클루드 방지를 위한 가드
#define SENSOR_PARAMS_H_

#include <memory> // 메모리 관리를 위한 헤더 파일 인클루드
#include <string> // 문자열 처리를 위한 헤더 파일 인클루드
#include <vector> // 벡터 컨테이너를 위한 헤더 파일 인클루드
#include <map> // 맵 컨테이너를 위한 헤더 파일 인클루드
#include "angles.h" // 각도 계산 관련 헤더 파일 인클루드

#include <boost/shared_ptr.hpp> // Boost 라이브러리의 shared_ptr 헤더 파일 인클루드
#include <boost/make_shared.hpp> // Boost 라이브러리의 make_shared 함수 헤더 파일 인클루드

#include <yaml-cpp/yaml.h> // YAML 파일 파싱을 위한 헤더 파일 인클루드

using boost::shared_ptr; // Boost 라이브러리의 shared_ptr를 사용하기 위한 선언
using boost::make_shared; // Boost 라이브러리의 make_shared 함수를 사용하기 위한 선언


// This work was inspired on projection_params from I. Bogoslavskyi, C. Stachniss, University of Bonn 
// https://github.com/PRBonn/cloud_to_image.git
// The original license copyright is as follows:
//------------------------------------
// Copyright (C) 2017  I. Bogoslavskyi, C. Stachniss, University of Bonn

// This program is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by the Free
// Software Foundation, either version 3 of the License, or (at your option)
// any later version.

// This program is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
// FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
// more details.

// You should have received a copy of the GNU General Public License along
// with this program.  If not, see <http://www.gnu.org/licenses/>.
//------------------------------------

namespace cloud_to_image 
{


/**
 * @brief 센서별 특정 투영 파라미터를 다루는 클래스
 */
class SensorParams 
{
  public:
    enum class ScanDirection { CLOCK_WISE, COUNTER_CLOCK_WISE }; // 스캔 방향을 나타내는 열거형

    using Ptr = shared_ptr<SensorParams>; // 센서 파라미터 포인터 타입의 별칭 정의
    using ConstPtr = const shared_ptr<const SensorParams>; // 상수 센서 파라미터 포인터 타입의 별칭 정의

    enum class Set { COLS, ROWS }; // 열거형 Set 정의

    SensorParams(): _scan_direction(ScanDirection::COUNTER_CLOCK_WISE) {} // 생성자. 기본 스캔 방향은 반시계 방향으로 설정
    ~SensorParams() {} // 소멸자


 /**
 * @brief 특정 방향에서 각도 범위를 설정합니다.
 *
 * @param[in] span_params 각도 범위 매개변수. ::AngularRange 형식으로 패키징됩니다.
 * @param[in] direction 방향. AngularRange::Direction 중 하나여야 합니다.
 */

void setSpan(const AngularRange& span_params, const AngularRange::Direction& direction); // 주어진 방향에 각도 범위를 설정하는 함수 선언

	/**
	 * @brief 특정 방향에서 각도 범위를 설정합니다.
	 *
	 * @param[in] span_params 각도 범위 매개변수. ::AngularRange 형식으로 패키징됩니다.
	 * @param[in] direction 방향. AngularRange::Direction 중 하나여야 합니다.
	 */

void setSpan(const std::vector<AngularRange>& span_params, const AngularRange::Direction& direction); // 주어진 방향에 각도 범위를 설정하는 함수 선언

inline const Angle& v_start_angle() const { return _v_span_params.start_angle(); } // 수직 시작 각도를 반환하는 인라인 함수

inline const Angle& v_end_angle() const { return _v_span_params.end_angle(); } // 수직 종료 각도를 반환하는 인라인 함수

inline const Angle& v_span() const { return _v_span_params.span(); } // 수직 범위를 반환하는 인라인 함수

inline const Angle& h_start_angle() const { return _h_span_params.start_angle(); } // 수평 시작 각도를 반환하는 인라인 함수

inline const Angle& h_end_angle() const { return _h_span_params.end_angle(); } // 수평 종료 각도를 반환하는 인라인 함수

inline const Angle& h_span() const { return _h_span_params.span(); } // 수평 범위를 반환하는 인라인 함수

inline const AngularRange& v_span_params() const { return _v_span_params; } // 수직 각도 범위 매개변수를 반환하는 인라인 함수

inline const AngularRange& h_span_params() const { return _h_span_params; } // 수평 각도 범위 매개변수를 반환하는 인라인 함수

inline size_t rows() const { return _row_angles.size(); } // 행의 수를 반환하는 인라인 함수

inline size_t cols() const { return _col_angles.size(); } // 열의 수를 반환하는 인라인 함수

inline size_t size() const { return rows() * cols(); } // 전체 요소 수를 반환하는 인라인 함수

inline const std::vector<Angle>& rowAngles() const { return _row_angles; } // 행 각도 벡터를 반환하는 인라인 함수
inline const std::vector<Angle>& colAngles() const { return _col_angles; } // 열 각도 벡터를 반환하는 인라인 함수

friend YAML::Emitter& operator << (YAML::Emitter& out, const SensorParams& val)
{
    out << YAML::BeginMap; // YAML 맵 시작
    out << YAML::Key << "vertical_span"; // 키: "vertical_span"
    out << YAML::Value << val.v_span_params(); // 값: 수직 각도 범위 매개변수
    out << YAML::Key << "horizontal_span"; // 키: "horizontal_span"
    out << YAML::Value << val.h_span_params(); // 값: 수평 각도 범위 매개변수
    out << YAML::Key << "scan_direction"; // 키: "scan_direction"
    out << YAML::Value << val.getScanDirectionStr(); // 값: 스캔 방향 문자열
    out << YAML::EndMap; // YAML 맵 종료
    return out;
}


	friend std::ostream& operator << (std::ostream& out, const SensorParams& val)
	{
	    out << "vertical_span: " << std::endl; // 수직 각도 범위 출력
	    out << val.v_span_params(); // 수직 각도 범위 매개변수 출력
	    out << "horizontal_span: " << std::endl; // 수평 각도 범위 출력
	    out << val.h_span_params(); // 수평 각도 범위 매개변수 출력
	    out << "scan_direction: " << std::endl;  // 스캔 방향 출력
	    out << val.getScanDirectionStr(); // 스캔 방향 문자열 출력
	    return out;
	}


   	/**
	 * @brief      행으로부터 각도를 가져옵니다.
	 *
	 * @param[in]  row   행
	 *
	 * @return     라디안 단위의 각도
	 */

    const Angle angleFromRow(int row) const; // 특정 행에 해당하는 각도를 가져오는 함수 선언


  
	/**
	* @brief 해당 col로부터 각도를 얻습니다.
	*
	* @param[in] col col 값
	*
	* @return 라디안 단위의 각도
	*/

    const Angle angleFromCol(int col) const; // 열 번호로부터 각도를 얻는 함수

/**
 * @brief      Get row number from angle
 *
 * @param[in]  angle  The angle
 *
 * @return     Row number
 */
size_t rowFromAngle(const Angle& angle) const; // 각도로부터 행 번호를 얻는 함수


  /**
 * @brief      Get col number from angle
 *
 * @param[in]  angle  The angle
 *
 * @return     Col number
 */
size_t colFromAngle(const Angle& angle) const; // 각도로부터 열 번호를 얻는 함수

const std::vector<float>& rowAngleCosines() const { return _row_angles_cosines; } // 행 각도의 코사인을 반환하는 함수
const std::vector<float>& colAngleCosines() const { return _col_angles_cosines; } // 열 각도의 코사인을 반환하는 함수
const std::vector<float>& rowAngleSines() const { return _row_angles_sines; } // 행 각도의 사인을 반환하는 함수
const std::vector<float>& colAngleSines() const { return _col_angles_sines; } // 열 각도의 사인을 반환하는 함수


    bool valid(); // 유효성을 확인하는 함수

inline void setScanDirection(const ScanDirection& direction) { _scan_direction = direction; } // 스캔 방향을 설정하는 함수
inline void setScanDirection(const std::string& direction) 
{ 
    if (direction == "CW") { // 만약 방향이 시계 방향이라면
        _scan_direction = ScanDirection::CLOCK_WISE; // 스캔 방향을 시계 방향으로 설정
    } else { // 그렇지 않으면
        _scan_direction = ScanDirection::COUNTER_CLOCK_WISE; // 스캔 방향을 반시계 방향으로 설정
}

    }

	std::string getScanDirectionStr() const // 스캔 방향을 문자열로 반환하는 함수
	{
	    std::string dir = std::string("CCW"); // 기본적으로 반시계 방향으로 설정된 문자열
	    if (_scan_direction == ScanDirection::CLOCK_WISE) { // 만약 스캔 방향이 시계 방향이라면
	        dir = std::string("CW"); // 문자열을 시계 방향으로 변경
	    }
	    return dir; // 방향 문자열 반환
	}


     const ScanDirection& getScanDirection() const { return _scan_direction; } // 스캔 방향을 반환하는 함수



	/**
	 * @brief      16 빔 Velodyne의 기본 매개변수
	 *
	 * @return     매개변수에 대한 포인터
	 */
	static std::unique_ptr<SensorParams> VLP_16();
	/**
	 * @brief      32 빔 Velodyne의 기본 매개변수
	 *
	 * @return     매개변수에 대한 포인터
	 */
	static std::unique_ptr<SensorParams> VLP_32();
	/**
	 * @brief      32 빔 Velodyne의 기본 매개변수
	 *
	 * @return     매개변수에 대한 포인터
	 */
	static std::unique_ptr<SensorParams> HDL_32();
	/**
	 * @brief      64 빔 Velodyne의 기본 매개변수
	 *
	 * @return     매개변수에 대한 포인터
	 */
	static std::unique_ptr<SensorParams> HDL_64_S2();
	/**
	 * @brief      64 빔 Velodyne의 기본 매개변수
	 *
	 * @return     매개변수에 대한 포인터
	 */
	static std::unique_ptr<SensorParams> HDL_64_S3();
	/**
	 * @brief      레이저 간의 동일한 간격을 가정한 64 빔 Velodyne의 매개변수
	 *
	 * @return     매개변수에 대한 포인터
	 */
	static std::unique_ptr<SensorParams> HDL_64_EQUAL();
	/**
	 * @brief      일반적인 64 빔 Velodyne의 매개변수
	 *
	 * @return     매개변수에 대한 포인터
	 */
	static std::unique_ptr<SensorParams> HDL_64_GENERAL();
	/**
	 * @brief      16 빔 Ouster의 매개변수
	 *
	 * @return     매개변수에 대한 포인터
	 */
	static std::unique_ptr<SensorParams> OS_1_16_0512();
	/**
	 * @brief      16 빔 Ouster의 매개변수
	 *
	 * @return     매개변수에 대한 포인터
	 */
	static std::unique_ptr<SensorParams> OS_1_16_1024();
	/**
	 * @brief      16 빔 Ouster의 매개변수
	 *
	 * @return     매개변수에 대한 포인터
	 */
	static std::unique_ptr<SensorParams> OS_1_16_2048();
	/**
	 * @brief      64 빔 Ouster의 매개변수
	 *
	 * @return     매개변수에 대한 포인터
	 */
	static std::unique_ptr<SensorParams> OS_1_64_0512();
	/**
	 * @brief      64 빔 Ouster의 매개변수
	 *
	 * @return     매개변수에 대한 포인터
	 */
	static std::unique_ptr<SensorParams> OS_1_64_1024();
	/**
	 * @brief      64 빔 Ouster의 매개변수
	 *
	 * @return     매개변수에 대한 포인터
	 */
	static std::unique_ptr<SensorParams> OS_1_64_2048();
	
	/**    
	 * @brief      전체 구를 포괄하기 위한 기본 매개변수
	 *
	 * @return     매개변수에 대한 포인터
	 */
	static std::unique_ptr<SensorParams> fullSphere(const Angle& discretization = 5_deg);



    private:
    std::vector<Angle> fillVector(const AngularRange& span_params); // 벡터를 채우는 함수(각도 범위로)
    std::vector<Angle> fillVector(const std::vector<AngularRange>& span_params); // 벡터를 채우는 함수(각도 범위 벡터로)

    static size_t findClosest(const std::vector<Angle>& vec, const Angle& val); // 가장 가까운 값을 찾는 함수

    void fillCosSin(); // 코사인과 사인을 채우는 함수

    AngularRange _v_span_params; // 수직 범위 매개변수
    AngularRange _h_span_params; // 수평 범위 매개변수
    ScanDirection _scan_direction; // 스캔 방향

    std::vector<Angle> _col_angles; // 열 각도
    std::vector<Angle> _row_angles; // 행 각도

    std::vector<float> _col_angles_sines; // 열 각도의 사인
    std::vector<float> _col_angles_cosines; // 열 각도의 코사인

    std::vector<float> _row_angles_sines; // 행 각도의 사인
    std::vector<float> _row_angles_cosines; // 행 각도의 코사인
};


}  // namespace cloud_to_image

namespace YAML
{
	template<>
	struct convert<cloud_to_image::SensorParams> 
	{
		static Node encode(const cloud_to_image::SensorParams& rhs)
		{
			Node node;
			node["vertical_span"] = rhs.v_span_params(); // 수직 범위를 인코딩
			node["horizontal_span"] = rhs.h_span_params(); // 수평 범위를 인코딩
            node["scan_direction"] = rhs.getScanDirectionStr(); // 스캔 방향을 인코딩
			return node;
		}

		static bool decode(const Node& node, cloud_to_image::SensorParams& rhs)
		{
			if (!node.IsMap() || node.size() != 3) { // 맵 형식이 아니거나 크기가 3이 아니라면
				return false;
			}
			rhs = cloud_to_image::SensorParams(); // 기본 센서 매개변수 생성
			rhs.setSpan(node["vertical_span"].as<cloud_to_image::AngularRange>(), cloud_to_image::AngularRange::Direction::VERTICAL); // 수직 범위 설정
			rhs.setSpan(node["horizontal_span"].as<cloud_to_image::AngularRange>(), cloud_to_image::AngularRange::Direction::HORIZONTAL); // 수평 범위 설정
            rhs.setScanDirection(node["scan_direction"].as<std::string>()); // 스캔 방향 설정
			return true;
		}
	};
}

#endif  // SENSOR_PARAMS_H_
