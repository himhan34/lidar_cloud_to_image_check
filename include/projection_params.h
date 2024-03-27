#ifndef PROJECTION_PARAMS_H_
#define PROJECTION_PARAMS_H_
#ifndef PROJECTION_PARAMS_H_ // PROJECTION_PARAMS_H_ 헤더 파일 중복 인클루드 방지를 위한 가드
#define PROJECTION_PARAMS_H_

#include <memory> // 메모리 관리를 위한 헤더 파일 인클루드
#include <string> // 문자열 처리를 위한 헤더 파일 인클루드
#include <vector> // 벡터 컨테이너를 위한 헤더 파일 인클루드
#include <map> // 맵 컨테이너를 위한 헤더 파일 인클루드
#include "sensor_params.h" // 센서 파라미터 헤더 파일 인클루드

#include <boost/shared_ptr.hpp> // Boost 라이브러리의 shared_ptr 헤더 파일 인클루드
#include <boost/make_shared.hpp> // Boost 라이브러리의 make_shared 함수 헤더 파일 인클루드

#include <yaml-cpp/yaml.h> // YAML 파일 파싱을 위한 헤더 파일 인클루드

using boost::shared_ptr; // shared_ptr 사용을 위한 네임스페이스 선언
using boost::make_shared; // make_shared 함수 사용을 위한 네임스페이스 선언


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
 * @brief 영상 투영에 대한 파라미터를 다루는 클래스
 */
class ProjectionParams 
{
  public:
    using Ptr = shared_ptr<ProjectionParams>; // 포인터 타입의 별칭 정의
    using ConstPtr = const shared_ptr<const ProjectionParams>; // 상수 포인터 타입의 별칭 정의

    ProjectionParams() {} // 생성자
    ~ProjectionParams() {} // 소멸자

    /**
     * @brief      지정된 센서 모델의 파라미터
     * @param[in]  model 쿼리할 센서 모델
     * @return     파라미터에 대한 포인터
     */
    std::unique_ptr<SensorParams> fromModel(const std::string& model); // 특정 센서 모델의 파라미터 가져오기

    const SensorParams& fromModel(const std::string& model) const; // 지정된 센서 모델에 대한 파라미터를 가져오는 함수 (상수 멤버 함수)

    std::unique_ptr<SensorParams> operator [](const std::string& model) { return fromModel(model); } // 배열 형태로 센서 모델의 파라미터를 가져오는 연산자 오버로딩

    const SensorParams& operator [](const std::string& model) const { return fromModel(model); } // 배열 형태로 센서 모델의 파라미터를 가져오는 연산자 오버로딩 (상수 멤버 함수)

    bool sensorExists(const std::string& model) const { return (_projection_params.size() && _projection_params.find(model) != _projection_params.end()); }; // 센서 모델이 존재하는지 확인하는 함수 (상수 멤버 함수)

  friend YAML::Emitter& operator << (YAML::Emitter& out, ProjectionParams& val)
  {
    // 빈 맵을 저장하지 않도록 스킵
    if (!val._projection_params.size()) {
      return out;
    }
    out << YAML::Comment("LiDARs projection parameters"); // LiDAR의 투영 파라미터에 대한 주석 추가  
    out << YAML::BeginSeq; // 시퀀스 시작
    for (auto& it : val._projection_params) {
      out << YAML::BeginMap; // 맵 시작
      out << YAML::Key << "name"; // 키: "name"
      out << YAML::Value << it.first; // 값: 센서 모델의 이름
      out << YAML::Key << "params"; // 키: "params"
      out << YAML::Value << *(it.second); // 값: 센서 파라미터
      out << YAML::EndMap; // 맵 종료
    }
    out << YAML::EndSeq; // 시퀀스 종료
  
    return out;
  }


  friend std::ostream& operator << (std::ostream& out, ProjectionParams& val)
  {
    // 빈 맵을 저장하지 않도록 스킵
    if (!val._projection_params.size()) {
      return out;
    }
    out << "#LiDARs projection parameters" << std::endl; // LiDAR의 투영 파라미터에 대한 주석 출력
    for (auto& it : val._projection_params) {
      out << "name: "; // 이름 출력
      out << it.first << std::endl; // 센서 모델의 이름 출력
      out << "params: " << std::endl; // 파라미터 출력
      out << *(it.second); // 센서 파라미터 출력
    }
  
    return out;
  }


void loadFromFile(const std::string& filename); // 파일로부터 데이터를 로드하는 함수 선언

void saveToFile(const std::string& filename); // 데이터를 파일에 저장하는 함수 선언

void genSampleSensors(); // 샘플 센서를 생성하는 함수 선언

bool valid(); // 파라미터의 유효성을 확인하는 함수 선언

private:
std::map<std::string, std::unique_ptr<SensorParams> > _projection_params; // 센서 파라미터를 저장하는 맵 컨테이너
};

}  // namespace cloud_to_image // cloud_to_image 네임스페이스 닫기

#endif  // PROJECTION_PARAMS_H // PROJECTION_PARAMS_H 헤더 파일 종료
