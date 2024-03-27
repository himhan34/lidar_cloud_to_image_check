#include "sensor_params.h"

#include <boost/algorithm/string.hpp> // Boost 라이브러리의 문자열 알고리즘 헤더 파일 포함
#include <boost/algorithm/string/predicate.hpp> // Boost 라이브러리의 문자열 알고리즘 헤더 파일 포함

#include <algorithm> // 알고리즘 헤더 파일 포함
#include <fstream> // 파일 입력 및 출력 헤더 파일 포함
#include <string> // 문자열 관련 헤더 파일 포함
#include <vector> // 벡터 헤더 파일 포함
#include <memory> // 스마트 포인터 관련 헤더 파일 포함

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

// 템플릿 함수로, 주어진 매개변수를 사용하여 유니크 포인터를 생성합니다.
// Args&&... args는 가변 개수의 매개변수를 받습니다.
template <typename T, typename... Args>
std::unique_ptr<T> make_unique(Args&&... args) {
  // T 타입의 객체를 생성하고 그것에 대한 유니크 포인터를 반환합니다.
  return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}

namespace cloud_to_image 
{

// AngularRange 객체를 사용하여 센서 파라미터의 범위를 설정하는 함수입니다.
// span_params는 각도 범위를 나타내는 AngularRange 객체입니다.
// direction은 각도의 방향을 지정하는데 사용됩니다.
void SensorParams::setSpan(const AngularRange& span_params, const AngularRange::Direction& direction) 
{
  // 벡터에 AngularRange 객체를 담아서 setSpan 함수를 호출합니다.
  std::vector<AngularRange> params_vec = {{span_params}};
  this->setSpan(params_vec, direction);
}


// AngularRange 벡터를 사용하여 센서 파라미터의 범위를 설정하는 함수입니다.
// span_params는 각도 범위를 나타내는 AngularRange 객체의 벡터입니다.
// direction은 각도의 방향을 지정하는데 사용됩니다.
void SensorParams::setSpan(const std::vector<AngularRange>& span_params, const AngularRange::Direction& direction) 
{
  // 각도의 개수를 초기화합니다.
  int num_beams = 0;
  // 각 AngularRange 객체의 num_beams를 더하여 총 각도의 개수를 계산합니다.
  for (const auto& span : span_params) {
    num_beams += span.num_beams();
  }
  // 방향에 따라 처리합니다.
  switch (direction) {
    // 수평 방향의 경우
    case AngularRange::Direction::HORIZONTAL:
      // 첫 번째와 마지막 AngularRange 객체의 시작 각도와 종료 각도를 사용하여 수평 방향의 span_params를 설정합니다.
      _h_span_params = AngularRange(span_params.front().start_angle(),
                                    span_params.back().end_angle(), 
                                    num_beams, 
                                    span_params.front().getAngleCorrectionTableF());
      // 열 각도 벡터를 채웁니다.
      _col_angles = fillVector(span_params);
      break;
    // 수직 방향의 경우
    case AngularRange::Direction::VERTICAL:
      // 첫 번째와 마지막 AngularRange 객체의 시작 각도와 종료 각도를 사용하여 수직 방향의 span_params를 설정합니다.
      _v_span_params = AngularRange(span_params.front().start_angle(),
                                    span_params.back().end_angle(), 
                                    num_beams,
                                    span_params.front().getAngleCorrectionTableF());
      // 행 각도 벡터를 채웁니다.
      _row_angles = fillVector(span_params);
      break;
  }
  // cos 및 sin 값을 채웁니다.
  fillCosSin();
}


// 주어진 AngularRange 객체를 사용하여 벡터를 채우는 함수입니다.
// span_params는 AngularRange 객체입니다.
std::vector<Angle> SensorParams::fillVector(const AngularRange& span_params) 
{
  // span_params를 포함하는 벡터를 생성합니다.
  std::vector<AngularRange> params_vec = {{span_params}};
  // fillVector 함수를 호출하여 벡터를 채우고 반환합니다.
  return this->fillVector(params_vec);
}


// 주어진 AngularRange 벡터를 사용하여 Angle 벡터를 채우는 함수입니다.
// span_params는 AngularRange 객체의 벡터입니다.
std::vector<Angle> SensorParams::fillVector(const std::vector<AngularRange>& span_params) 
{
  // 결과를 저장할 벡터를 선언합니다.
  std::vector<Angle> res;
  // 각 AngularRange 객체에 대해 반복합니다.
  for (const auto span_param : span_params) {
    // Angle 보정 테이블이 비어있지 않으면
    if (span_param.getAngleCorrectionTable().size()) {
      // 보정 테이블의 각 요소를 결과 벡터에 추가합니다.
      for (auto& it : span_param.getAngleCorrectionTable()) {
        res.push_back(it);
      }
    } else {
      int direction = 1; // 증가 방향
      Angle rad = span_param.start_angle();
      // 시작 각도가 끝 각도보다 큰 경우 감소 방향으로 설정합니다.
      if (span_param.start_angle() > span_param.end_angle()) {
        direction = -1;
      }
      // 각 Beam에 대해 반복하면서 Angle을 추가합니다.
      for (int i = 0; i < span_param.num_beams(); ++i) {
        // 시작 각도가 0인 경우, 각도를 0으로 유지합니다.
        if (rad == 0_deg) {
          rad = 0_deg;
        }
        // 결과 벡터에 Angle을 추가합니다.
        res.push_back(rad);
        // 다음 Angle을 계산합니다.
        rad += (span_param.step() * direction);
      }
    }
  }
  // 채워진 결과 벡터를 반환합니다.
  return res;
}


// SensorParams의 유효성을 검사하는 함수입니다.
bool SensorParams::valid() 
{
  // 수직 및 수평 각도 범위가 모두 유효한지 확인합니다.
  bool all_params_valid = _v_span_params.valid() && _h_span_params.valid();
  // 각도 배열이 모두 비어 있는지 확인합니다.
  bool arrays_empty = _row_angles.empty() && _col_angles.empty();
  // 사인 및 코사인 배열이 모두 비어 있는지 확인합니다.
  bool cos_sin_empty = _row_angles_sines.empty() &&
                       _row_angles_cosines.empty() &&
                       _col_angles_sines.empty() && 
                       _col_angles_cosines.empty();
  // 모든 매개변수가 유효하지 않은 경우
  if (!all_params_valid) {
    throw std::runtime_error("Sensor parameters invalid."); // 런타임 오류 발생
  }
  // 각도 배열이 비어있는 경우
  if (arrays_empty) {
    throw std::runtime_error("Sensor parameters arrays not filled."); // 런타임 오류 발생
  }
  // 사인 및 코사인 배열이 비어 있는 경우
  if (cos_sin_empty) {
    throw std::runtime_error("Projection parameters sin and cos arrays not filled."); // 런타임 오류 발생
  }
  // 모든 검사가 통과되면 유효하다고 가정합니다.
  return true;
}


// 주어진 행(row)에 대한 각도를 반환하는 함수입니다.
const Angle SensorParams::angleFromRow(int row) const 
{
  // 행(row)이 유효한 인덱스 범위 내에 있는지 확인합니다.
  if (row >= 0 && static_cast<size_t>(row) < _row_angles.size()) {
    // 유효한 경우 해당 행(row)의 각도를 반환합니다.
    return _row_angles[row];
  }
  // 유효하지 않은 경우 오류 메시지를 출력하고 0도를 반환합니다.
  fprintf(stderr, "ERROR: row %d is wrong\n", row); // 오류 메시지 출력
  return 0.0_deg; // 0도를 반환
}


// 주어진 열(col)에 대한 각도를 반환하는 함수입니다.
const Angle SensorParams::angleFromCol(int col) const 
{
  int actual_col = col; // 실제 열 인덱스를 저장할 변수를 초기화합니다.
  // 열 인덱스가 음수인 경우
  if (col < 0) {
    // 음수 값을 처리하여 유효한 인덱스로 변환합니다.
    actual_col = col + _col_angles.size();
  } 
  // 열 인덱스가 벡터의 크기를 초과하는 경우
  else if (static_cast<size_t>(col) >= _col_angles.size()) {
    // 벡터 크기를 고려하여 유효한 인덱스로 변환합니다.
    actual_col = col - _col_angles.size();
  }
  // 모든 인덱스가 범위 내에 있을 때
  // 실제 열(actual_col)의 해당하는 각도를 반환합니다.
  return _col_angles[actual_col];
}


// 주어진 각도에 해당하는 행(row) 인덱스를 반환하는 함수입니다.
size_t SensorParams::rowFromAngle(const Angle& angle) const 
{
  // 주어진 각도(angle)에 가장 가까운 행 인덱스를 찾습니다.
  size_t row = findClosest(_row_angles, angle);
  // 찾은 행 인덱스를 반환합니다.
  return row;
}


// 주어진 각도에 해당하는 열(col) 인덱스를 반환하는 함수입니다.
size_t SensorParams::colFromAngle(const Angle& angle) const 
{
  // 주어진 각도(angle)에 가장 가까운 열 인덱스를 찾습니다.
  size_t col = findClosest(_col_angles, angle);
  // 스캔 방향이 시계 방향(CLOCK_WISE)인 경우 처리합니다.
  if (_scan_direction == ScanDirection::CLOCK_WISE) {
    // 열 인덱스를 뒤집습니다.
    col = (_col_angles.size() - 1) - col;
  }
  // 찾은 열 인덱스를 반환합니다.
  return col;
}


// 주어진 벡터에서 가장 가까운 값의 인덱스를 찾는 함수입니다.
size_t SensorParams::findClosest(const std::vector<Angle>& vec, const Angle& val) 
{
  size_t found = 0;
  // 벡터가 오름차순으로 정렬되어 있는 경우
  if (vec.front() < vec.back()) {
    
    // upper_bound를 사용하여 주어진 값보다 큰 첫 번째 요소의 위치를 찾습니다.
    found = std::upper_bound(vec.begin(), vec.end(), val) - vec.begin();
  } 
  
    // 벡터가 내림차순으로 정렬되어 있는 경우
  else {
    // upper_bound를 사용하여 주어진 값보다 작은 첫 번째 요소의 위치를 찾습니다.
    found = vec.rend() - std::upper_bound(vec.rbegin(), vec.rend(), val);
  }
 
  // 찾은 위치가 첫 번째 요소인 경우
  if (found == 0) {
    return found; // 찾은 위치를 반환합니다.
  }
 
  // 찾은 위치가 마지막 요소의 인덱스인 경우
  if (found == vec.size()) {
    return found - 1; // 마지막 요소의 인덱스를 반환합니다.
  }
    // 찾은 위치와 그 이전 요소 사이의 각도 차이를 계산합니다.
    auto diff_next = Angle::abs(vec[found] - val);
    auto diff_prev = Angle::abs(val - vec[found - 1]);
    // 두 각도 차이 중 작은 것을 갖는 인덱스를 반환합니다.
    return diff_next < diff_prev ? found : found - 1;
}


std::unique_ptr<SensorParams> SensorParams::VLP_16() 
{
  // 수직 방향 보정 테이블
  std::vector<float> correction_table = {15,13,11,9,7,5,3,1,-1,-3,-5,-7,-9,-11,-13,-15};
  // 센서 매개변수 객체를 생성합니다.
  auto params = SensorParams();
  // 수평 방향 범위를 설정합니다. (-180도부터 180도까지, 870개의 빔)
  params.setSpan(AngularRange(-180_deg, 180_deg, 870),
                 AngularRange::Direction::HORIZONTAL);
  // 수직 방향 범위를 설정합니다. (15도부터 -15도까지, 16개의 빔, 보정 테이블 포함)
  params.setSpan(AngularRange(15_deg, -15_deg, 16, correction_table),
                 AngularRange::Direction::VERTICAL);
  // 스캔 방향을 시계방향으로 설정합니다.
  params.setScanDirection("CW");
  // 삼각함수 값을 미리 계산합니다.
  params.fillCosSin();
  // 매개변수가 유효한지 확인합니다.
  if (!params.valid()) {
    // 오류 메시지를 출력합니다.
    fprintf(stderr, "ERROR: params are not valid!\n");
    // nullptr을 반환합니다.
    return nullptr;
  }
  // 유효한 경우 센서 매개변수 객체를 포인터로 래핑하여 반환합니다.
  return make_unique<SensorParams>(params);
}


std::unique_ptr<SensorParams> SensorParams::VLP_32() 
{
  // 수직 방향 보정 테이블
  std::vector<float> correction_table = {15,10.333,7,4.667,3.333,2.333,1.667,1.333,1,0.667,0.333,0,-0.333,-0.667,-1,-1.333,
                                         -1.667,-2,-2.333,-2.667,-3,-3.333,-3.667,-4,-4.667,-5.333,-6.148,-7.254,-8.843,-11.31,-15.639,-25};
  // 센서 매개변수 객체를 생성합니다.
  auto params = SensorParams();
  // 수평 방향 범위를 설정합니다. (-180도부터 180도까지, 870개의 빔)
  params.setSpan(AngularRange(-180_deg, 180_deg, 870),
                 AngularRange::Direction::HORIZONTAL);
  // 수직 방향 범위를 설정합니다. (15도부터 -25도까지, 32개의 빔, 보정 테이블 포함)
  params.setSpan(AngularRange(15_deg, -25_deg, 32, correction_table),
                 AngularRange::Direction::VERTICAL);
  // 스캔 방향을 시계방향으로 설정합니다.
  params.setScanDirection("CW");
  // 삼각함수 값을 미리 계산합니다.
  params.fillCosSin();
  // 매개변수가 유효한지 확인합니다.
  if (!params.valid()) {
    // 오류 메시지를 출력합니다.
    fprintf(stderr, "ERROR: params are not valid!\n");
    // nullptr을 반환합니다.
    return nullptr;
  }
  // 유효한 경우 센서 매개변수 객체를 포인터로 래핑하여 반환합니다.
  return make_unique<SensorParams>(params);
}

std::unique_ptr<SensorParams> SensorParams::HDL_32() 
{
  // 수직 방향 보정 테이블
  std::vector<float> correction_table = {10.67,9.33,8,6.67,5.33,4,2.67,1.33,0,-1.33,-2.67,-4,-5.33,-6.67,-8,-9.33,
                                        -10.67,-12,-13.33,-14.67,-16,-17.33,-18.67,-20,-21.33,-22.67,-24,-25.33,-26.67,-28,-29.33,-30.67};
  // 센서 매개변수 객체를 생성합니다.
  auto params = SensorParams();
  // 수평 방향 범위를 설정합니다. (-180도부터 180도까지, 870개의 빔)
  params.setSpan(AngularRange(-180_deg, 180_deg, 870),
                 AngularRange::Direction::HORIZONTAL);
  // 수직 방향 범위를 설정합니다. (10.0도부터 -30.0도까지, 32개의 빔, 보정 테이블 포함)
  params.setSpan(AngularRange(10.0_deg, -30.0_deg, 32, correction_table),
                 AngularRange::Direction::VERTICAL);
  // 스캔 방향을 시계방향으로 설정합니다.
  params.setScanDirection("CW");
  // 삼각함수 값을 미리 계산합니다.
  params.fillCosSin();
  // 매개변수가 유효한지 확인합니다.
  if (!params.valid()) {
    // 오류 메시지를 출력합니다.
    fprintf(stderr, "ERROR: params are not valid!\n");
    // nullptr을 반환합니다.
    return nullptr;
  }
  // 유효한 경우 센서 매개변수 객체를 포인터로 래핑하여 반환합니다.
  return make_unique<SensorParams>(params);
}


std::unique_ptr<SensorParams> SensorParams::HDL_64_EQUAL() 
{
  // 센서 매개변수 객체를 생성합니다.
  auto params = SensorParams();
  // 수평 방향 범위를 설정합니다. (-180도부터 180도까지, 870개의 빔)
  params.setSpan(AngularRange(-180_deg, 180_deg, 870),
                 AngularRange::Direction::HORIZONTAL);
  // 수직 방향 범위를 설정합니다. (2.0도부터 -24.8도까지, 64개의 빔)
  params.setSpan(AngularRange(2.0_deg, -24.8_deg, 64),
                 AngularRange::Direction::VERTICAL);
  // 스캔 방향을 시계방향으로 설정합니다.
  params.setScanDirection("CW");
  // 삼각함수 값을 미리 계산합니다.
  params.fillCosSin();
  // 매개변수가 유효한지 확인합니다.
  if (!params.valid()) {
    // 오류 메시지를 출력합니다.
    fprintf(stderr, "ERROR: params are not valid!\n");
    // nullptr을 반환합니다.
    return nullptr;
  }
  // 유효한 경우 센서 매개변수 객체를 포인터로 래핑하여 반환합니다.
  return make_unique<SensorParams>(params);
}


std::unique_ptr<SensorParams> SensorParams::HDL_64_S2() 
{
  // 각도 보정 테이블을 정의합니다.
  std::vector<float> correction_table = {4.97009,4.49316,4.00396,3.5025,2.97714,2.48635,1.9718,1.44522,
                                         0.976948,0.508544,-0.217594,-0.568941,-1.15441,-1.58751,-2.05552,-2.59339,
                                         -3.18918,-3.71431,-4.16892,-4.70446,-5.1927,-5.6686,-6.25946,-6.86054,
                                         -7.26426,-7.78227,-8.35633,-8.76862,-9.07171,-9.33972,-9.61906,-9.81801,
                                         -9.99435,-10.3629,-10.5387,-10.8608,-10.9457,-11.5203,-12.0702,-12.417,
                                         -12.9743,-13.4073,-14.0814,-14.5981,-15.1778,-15.6893,-16.1118,-16.554,
                                         -17.112,-17.7622,-18.2178,-18.7236,-19.1845,-19.5702,-20.1194,-20.8593,
                                         -21.308,-21.8851,-22.3575,-22.7272,-23.184,-23.8536,-24.4193,-24.8451};
  // 센서 매개변수 객체를 생성합니다.
  auto params = SensorParams();
  // 수평 방향 범위를 설정합니다. (-180도부터 180도까지, 870개의 빔)
  params.setSpan(AngularRange(-180_deg, 180_deg, 870),
                 AngularRange::Direction::HORIZONTAL);
  // 수직 방향 범위를 설정합니다. (2.0도부터 -24.8도까지, 64개의 빔, 보정 테이블 적용)
  params.setSpan(AngularRange(2.0_deg, -24.8_deg, 64, correction_table),
                 AngularRange::Direction::VERTICAL);
  // 스캔 방향을 시계방향으로 설정합니다.
  params.setScanDirection("CW");
  // 삼각함수 값을 미리 계산합니다.
  params.fillCosSin();
  // 매개변수가 유효한지 확인합니다.
  if (!params.valid()) {
    // 오류 메시지를 출력합니다.
    fprintf(stderr, "ERROR: params are not valid!\n");
    // nullptr을 반환합니다.
    return nullptr;
  }
  // 유효한 경우 센서 매개변수 객체를 포인터로 래핑하여 반환합니다.
  return make_unique<SensorParams>(params);
}


std::unique_ptr<SensorParams> SensorParams::HDL_64_S3() 
{
  // 각도 보정 테이블을 정의합니다.
  std::vector<float> correction_table = {2.28525,1.9601,1.64184,1.30008,0.986315,0.630338,0.283682,-0.088762,
                                        -0.414352,-0.772704,-1.02094,-1.44004,-1.76304,-2.10698,-2.42973,-2.8014,
                                        -3.1378,-3.49733,-3.81227,-4.19222,-4.46007,-4.82077,-5.19967,-5.58044,
                                        -5.86106,-6.20854,-6.51856,-6.87439,-7.20202,-7.57522,-7.84665,-8.28753,
                                        -8.61201,-9.19273,-9.62589,-9.93754,-10.5658,-11.1893,-11.6215,-12.1553,
                                        -12.5777,-12.9098,-13.425,-14.2421,-14.7515,-15.2585,-15.7088,-16.0361,
                                        -16.7432,-17.2659,-17.8026,-18.3023,-18.6984,-19.0674,-19.7633,-20.3359,
                                        -20.8757,-21.4035,-21.8508,-22.3214,-22.8168,-23.3184,-23.9378,-24.3474};
  // 센서 매개변수 객체를 생성합니다.
  auto params = SensorParams();
  // 수평 방향 범위를 설정합니다. (-180도부터 180도까지, 870개의 빔)
  params.setSpan(AngularRange(-180_deg, 180_deg, 870),
                 AngularRange::Direction::HORIZONTAL);
  // 수직 방향 범위를 설정합니다. (2.0도부터 -24.0도까지, 64개의 빔, 보정 테이블 적용)
  params.setSpan(AngularRange(2.0_deg, -24.0_deg, 64, correction_table),
                 AngularRange::Direction::VERTICAL);
  // 스캔 방향을 시계방향으로 설정합니다.
  params.setScanDirection("CW");
  // 삼각함수 값을 미리 계산합니다.
  params.fillCosSin();
  // 매개변수가 유효한지 확인합니다.
  if (!params.valid()) {
    // 오류 메시지를 출력합니다.
    fprintf(stderr, "ERROR: params are not valid!\n");
    // nullptr을 반환합니다.
    return nullptr;
  }
  // 유효한 경우 센서 매개변수 객체를 포인터로 래핑하여 반환합니다.
  return make_unique<SensorParams>(params);
}


std::unique_ptr<SensorParams> SensorParams::HDL_64_GENERAL() 
{
  // 센서 매개변수 객체를 생성합니다.
  auto params = SensorParams();
  
  // 수평 방향 범위를 설정합니다. (-180도부터 180도까지, 870개의 빔)
  params.setSpan(AngularRange(-180_deg, 180_deg, 870),
                 AngularRange::Direction::HORIZONTAL);
  
  // 상단과 하단의 수직 방향 범위를 정의합니다.
  AngularRange span_top(2.0_deg, -8.5_deg, 32);
  AngularRange span_bottom(-8.87_deg, -24.87_deg, 32);
  
  // 벡터에 범위를 추가합니다.
  std::vector<AngularRange> spans = {{span_top, span_bottom}};
  
  // 수직 방향 범위를 설정합니다.
  params.setSpan(spans, AngularRange::Direction::VERTICAL);
  
  // 스캔 방향을 시계방향으로 설정합니다.
  params.setScanDirection("CW");
  
  // 삼각함수 값을 미리 계산합니다.
  params.fillCosSin();
  
  // 매개변수가 유효한지 확인합니다.
  if (!params.valid()) {
    // 오류 메시지를 출력합니다.
    fprintf(stderr, "ERROR: params are not valid!\n");
    // nullptr을 반환합니다.
    return nullptr;
  }
  // 유효한 경우 센서 매개변수 객체를 포인터로 래핑하여 반환합니다.
  return make_unique<SensorParams>(params);
}


std::unique_ptr<SensorParams> SensorParams::OS_1_16_0512() 
{
  // 센서 매개변수 객체를 생성합니다.
  auto params = SensorParams();
  // 수평 방향 범위를 설정합니다. (-180도부터 180도까지, 512개의 빔)
  params.setSpan(AngularRange(-180_deg, 180_deg, 512),
                 AngularRange::Direction::HORIZONTAL);
  // 수직 방향 범위를 설정합니다. (16.6도부터 -16.6도까지, 16개의 빔)
  params.setSpan(AngularRange(16.6_deg, -16.6_deg, 16),
                 AngularRange::Direction::VERTICAL);
  // 삼각함수 값을 미리 계산합니다.
  params.fillCosSin();
  // 매개변수가 유효한지 확인합니다.
  if (!params.valid()) {
    // 오류 메시지를 출력합니다.
    fprintf(stderr, "ERROR: params are not valid!\n");
    // nullptr을 반환합니다.
    return nullptr;
  }
  // 유효한 경우 센서 매개변수 객체를 포인터로 래핑하여 반환합니다.
  return make_unique<SensorParams>(params);
}


std::unique_ptr<SensorParams> SensorParams::OS_1_16_1024() 
{
  // 센서 매개변수 객체를 생성합니다.
  auto params = SensorParams();
  // 수평 방향 범위를 설정합니다. (-180도부터 180도까지, 1024개의 빔)
  params.setSpan(AngularRange(-180_deg, 180_deg, 1024),
                 AngularRange::Direction::HORIZONTAL);
  // 수직 방향 범위를 설정합니다. (16.6도부터 -16.6도까지, 16개의 빔)
  params.setSpan(AngularRange(16.6_deg, -16.6_deg, 16),
                 AngularRange::Direction::VERTICAL);
  // 삼각함수 값을 미리 계산합니다.
  params.fillCosSin();
  // 매개변수가 유효한지 확인합니다.
  if (!params.valid()) {
    // 오류 메시지를 출력합니다.
    fprintf(stderr, "ERROR: params are not valid!\n");
    // nullptr을 반환합니다.
    return nullptr;
  }
  // 유효한 경우 센서 매개변수 객체를 포인터로 래핑하여 반환합니다.
  return make_unique<SensorParams>(params);
}


std::unique_ptr<SensorParams> SensorParams::OS_1_16_2048() 
{
  // 센서 매개변수 객체를 생성합니다.
  auto params = SensorParams();
  // 수평 방향 범위를 설정합니다. (-180도부터 180도까지, 2048개의 빔)
  params.setSpan(AngularRange(-180_deg, 180_deg, 2048),
                 AngularRange::Direction::HORIZONTAL);
  // 수직 방향 범위를 설정합니다. (16.6도부터 -16.6도까지, 16개의 빔)
  params.setSpan(AngularRange(16.6_deg, -16.6_deg, 16),
                 AngularRange::Direction::VERTICAL);
  // 삼각함수 값을 미리 계산합니다.
  params.fillCosSin();
  // 매개변수가 유효한지 확인합니다.
  if (!params.valid()) {
    // 오류 메시지를 출력합니다.
    fprintf(stderr, "ERROR: params are not valid!\n");
    // nullptr을 반환합니다.
    return nullptr;
  }
  // 유효한 경우 센서 매개변수 객체를 포인터로 래핑하여 반환합니다.
  return make_unique<SensorParams>(params);
}


std::unique_ptr<SensorParams> SensorParams::OS_1_64_0512() 
{
  // 센서 매개변수 객체를 생성합니다.
  auto params = SensorParams();
  
  // 수평 방향 범위를 설정합니다. (-180도부터 180도까지, 512개의 빔)
  params.setSpan(AngularRange(-180_deg, 180_deg, 512),
                 AngularRange::Direction::HORIZONTAL);
  // 수직 방향 범위를 설정합니다. (16.6도부터 -16.6도까지, 64개의 빔)
  params.setSpan(AngularRange(16.6_deg, -16.6_deg, 64),
                 AngularRange::Direction::VERTICAL);
  // 삼각함수 값을 미리 계산합니다.
  params.fillCosSin();
  // 매개변수가 유효한지 확인합니다.
  
  if (!params.valid()) {
    // 오류 메시지를 출력합니다.
    fprintf(stderr, "ERROR: params are not valid!\n");
    // nullptr을 반환합니다.
    return nullptr;
  }
  // 유효한 경우 센서 매개변수 객체를 포인터로 래핑하여 반환합니다.
  return make_unique<SensorParams>(params);
}


std::unique_ptr<SensorParams> SensorParams::OS_1_64_1024() 
{
  // 센서 매개변수 객체를 생성합니다.
  auto params = SensorParams();
  // 수평 방향 범위를 설정합니다. (-180도부터 180도까지, 1024개의 빔)
  params.setSpan(AngularRange(-180_deg, 180_deg, 1024),
                 AngularRange::Direction::HORIZONTAL);
  // 수직 방향 범위를 설정합니다. (16.6도부터 -16.6도까지, 64개의 빔)
  params.setSpan(AngularRange(16.6_deg, -16.6_deg, 64),
                 AngularRange::Direction::VERTICAL);
  // 삼각함수 값을 미리 계산합니다.
  params.fillCosSin();
  // 매개변수가 유효한지 확인합니다.
  if (!params.valid()) {
    // 오류 메시지를 출력합니다.
    fprintf(stderr, "ERROR: params are not valid!\n");
    // nullptr을 반환합니다.
    return nullptr;
  }
  // 유효한 경우 센서 매개변수 객체를 포인터로 래핑하여 반환합니다.
  return make_unique<SensorParams>(params);
}

std::unique_ptr<SensorParams> SensorParams::OS_1_64_2048() 
{
  // 센서 매개변수 객체를 생성합니다.
  auto params = SensorParams();
  
  // 수평 방향 범위를 설정합니다. (-180도부터 180도까지, 2048개의 빔)
  params.setSpan(AngularRange(-180_deg, 180_deg, 2048),
                 AngularRange::Direction::HORIZONTAL);
  // 수직 방향 범위를 설정합니다. (16.6도부터 -16.6도까지, 64개의 빔)
  params.setSpan(AngularRange(16.6_deg, -16.6_deg, 64),
                 AngularRange::Direction::VERTICAL);
  // 삼각함수 값을 미리 계산합니다.
  params.fillCosSin();
  
  // 매개변수가 유효한지 확인합니다.
  if (!params.valid()) {
    // 오류 메시지를 출력합니다.
    fprintf(stderr, "ERROR: params are not valid!\n");
    // nullptr을 반환합니다.
    return nullptr;
  }
  
  // 유효한 경우 센서 매개변수 객체를 포인터로 래핑하여 반환합니다.
  return make_unique<SensorParams>(params);
}


std::unique_ptr<SensorParams> SensorParams::fullSphere(const Angle& discretization) 
{
  // 센서 매개변수 객체를 생성합니다.
  auto params = SensorParams();
  // 수평 방향 범위를 설정합니다. (-180도부터 180도까지, discretization 간격으로)
  params.setSpan(AngularRange(-180_deg, 180_deg, discretization),
                 AngularRange::Direction::HORIZONTAL);
  // 수직 방향 범위를 설정합니다. (-90도부터 90도까지, discretization 간격으로)
  params.setSpan(AngularRange(-90_deg, 90_deg, discretization),
                 AngularRange::Direction::VERTICAL);
  // 삼각함수 값을 미리 계산합니다.
  params.fillCosSin();
  // 매개변수가 유효한지 확인합니다.
  if (!params.valid()) {
    // 오류 메시지를 출력합니다.
    fprintf(stderr, "ERROR: params are not valid!\n");
    // nullptr을 반환합니다.
    return nullptr;
  }
  // 유효한 경우 센서 매개변수 객체를 포인터로 래핑하여 반환합니다.
  return make_unique<SensorParams>(params);
}

void SensorParams::fillCosSin() 
{
  // 삼각함수 값을 담은 벡터를 초기화합니다.
  _row_angles_sines.clear();
  _row_angles_cosines.clear();
  // 각 행의 각도에 대한 삼각함수 값을 계산하여 저장합니다.
  for (const auto& angle : _row_angles) {
    // 각도에 대한 사인값을 계산하고 벡터에 추가합니다.
    _row_angles_sines.push_back(sin(angle.val()));
    // 각도에 대한 코사인값을 계산하고 벡터에 추가합니다.
    _row_angles_cosines.push_back(cos(angle.val()));
  }
  // 열의 각도에 대한 삼각함수 값을 담은 벡터를 초기화합니다.
  _col_angles_sines.clear();
  _col_angles_cosines.clear();
  // 각 열의 각도에 대한 삼각함수 값을 계산하여 저장합니다.
  for (const auto& angle : _col_angles) {
    // 각도에 대한 사인값을 계산하고 벡터에 추가합니다.
    _col_angles_sines.push_back(sin(angle.val()));
    // 각도에 대한 코사인값을 계산하고 벡터에 추가합니다.
    _col_angles_cosines.push_back(cos(angle.val()));
  }
}

}  // namespace cloud_to_image
