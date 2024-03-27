#include "projection_params.h" // 투영 매개변수 헤더 파일을 포함합니다.
#include "sensor_params.h" // 센서 매개변수 헤더 파일을 포함합니다.

#include <boost/algorithm/string.hpp> // boost 알고리즘 라이브러리를 사용하기 위해 포함합니다.
#include <boost/algorithm/string/predicate.hpp> // boost 문자열 알고리즘 라이브러리를 사용하기 위해 포함합니다.

#include <algorithm> // 알고리즘 헤더 파일을 포함합니다.
#include <fstream> // 파일 입출력을 위한 헤더 파일을 포함합니다.
#include <string> // 문자열 처리를 위한 헤더 파일을 포함합니다.
#include <vector> // 벡터 컨테이너를 사용하기 위한 헤더 파일을 포함합니다.
#include <memory> // 스마트 포인터를 사용하기 위한 헤더 파일을 포함합니다.

#include <yaml-cpp/yaml.h> // YAML 파일 파싱을 위한 헤더 파일을 포함합니다.


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

template <typename T, typename... Args>
std::unique_ptr<T> make_unique(Args&&... args) {
  // 인자 팩을 이용하여 T 형식의 객체를 동적으로 생성하고 유일한 포인터로 래핑하여 반환합니다.
  return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}


namespace cloud_to_image 
{

std::unique_ptr<SensorParams> ProjectionParams::fromModel(const std::string& model)
{
  // 리스트가 비어있거나 모델을 사용할 수 없는 경우
  if (!_projection_params.size() || _projection_params.find(model) == _projection_params.end()) {
    // 빈 매개변수를 반환합니다.
    auto params = SensorParams();
    return make_unique<SensorParams>(params);
  } else {
    // 해당 모델의 매개변수를 복제하여 반환합니다.
    return make_unique<SensorParams>(*(_projection_params.at(model)));
  }
}


const SensorParams& ProjectionParams::fromModel(const std::string& model) const 
{
  // 리스트가 비어있거나 모델을 사용할 수 없는 경우
  if (!_projection_params.size() || _projection_params.find(model) == _projection_params.end()) {
    // 빈 매개변수를 반환합니다.
    auto params = SensorParams();
    return *(make_unique<SensorParams>(params));
  } else {
    // 해당 모델의 매개변수를 반환합니다.
    return *(_projection_params.at(model));
  }
}



void ProjectionParams::loadFromFile(const std::string& filename)
{
  // 만약 _projection_params가 비어있다면
  if (_projection_params.size()) {
    // _projection_params를 비웁니다.
    _projection_params.clear();
  }

  // 파일에서 YAML 구성을 로드합니다.
  YAML::Node config = YAML::LoadFile(filename);
  // YAML 구성을 반복하여 처리합니다.
  for(YAML::const_iterator iter=config.begin(); iter != config.end(); ++iter) {
    // 이터레이터를 사용하여 현재 YAML 노드를 가져옵니다.
    auto it = *iter;
    // 모델 이름을 가져옵니다.
    std::string model = it["name"].as<std::string>();
    // 모델에 대한 매개변수를 가져옵니다.
    auto params = it["params"].as<SensorParams>();

    // 유효성을 확인합니다.
    if (!params.valid()) {
      // 유효하지 않은 경우 오류를 출력하고 종료합니다.
      fprintf(stderr, "ERROR: the config read was not valid.\n");
      exit(1);
    }
    // _projection_params에 새로운 매개변수를 추가합니다.
    _projection_params[model] = make_unique<SensorParams>(params);
  }
}


void ProjectionParams::saveToFile(const std::string& filename)
{
  // 빈 맵은 저장하지 않습니다.
  if (!_projection_params.size()) {
    return;
  }

  // YAML 출력 생성기를 생성합니다.
  YAML::Emitter out;
  // 현재 객체를 YAML 출력에 씁니다.
  out << *this;

  // 파일을 열어서 YAML 내용을 씁니다.
  std::ofstream ofs(filename, std::ofstream::out);
  ofs << out.c_str();
  ofs.close();
}


void ProjectionParams::genSampleSensors()
{
  // 이미 맵이 채워져 있는 경우 모두 지웁니다.
  if (_projection_params.size()) {
    _projection_params.clear();
  }

  // Velodyne
  _projection_params["VLP-16"] = SensorParams::VLP_16(); // VLP-16 센서에 대한 매개변수 생성
  _projection_params["VLP-32"] = SensorParams::VLP_32(); // VLP-32 센서에 대한 매개변수 생성
  _projection_params["HDL-32"] = SensorParams::HDL_32(); // HDL-32 센서에 대한 매개변수 생성
  _projection_params["HDL-64-S2"] = SensorParams::HDL_64_S2(); // HDL-64-S2 센서에 대한 매개변수 생성
  _projection_params["HDL-64-S3"] = SensorParams::HDL_64_S3(); // HDL-64-S3 센서에 대한 매개변수 생성

  // Ouster
  _projection_params["OS-1-16-0512"] = SensorParams::OS_1_16_0512(); // OS-1-16-0512 센서에 대한 매개변수 생성
  _projection_params["OS-1-16-1024"] = SensorParams::OS_1_16_1024(); // OS-1-16-1024 센서에 대한 매개변수 생성
  _projection_params["OS-1-16-2048"] = SensorParams::OS_1_16_2048(); // OS-1-16-2048 센서에 대한 매개변수 생성
  _projection_params["OS-1-64-0512"] = SensorParams::OS_1_64_0512(); // OS-1-64-0512 센서에 대한 매개변수 생성
  _projection_params["OS-1-64-1024"] = SensorParams::OS_1_64_1024(); // OS-1-64-1024 센서에 대한 매개변수 생성
  _projection_params["OS-1-64-2048"] = SensorParams::OS_1_64_2048(); // OS-1-64-2048 센서에 대한 매개변수 생성
}

bool ProjectionParams::valid()
{
  bool _valid = true;
  
  // 만약 맵이 비어 있다면 유효하지 않음을 반환합니다.
  if (!_projection_params.size()) {
    return false;
  }
  
  // 모든 센서 매개변수에 대해 유효성을 검사합니다.
  for (auto& it : _projection_params) {
    _valid |= (it.first.size() && it.second->valid()); // 키와 센서 매개변수가 모두 유효한지 확인합니다.
  }
  
  return _valid;
}
}  // namespace cloud_to_image
