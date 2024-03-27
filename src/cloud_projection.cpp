
#include "cloud_projection.h"        // cloud_projection 헤더 파일을 포함합니다.
#include "pcl_point_types.h"         // pcl_point_types 헤더 파일을 포함합니다.
#include <string>                    // 문자열을 다루기 위한 헤더 파일을 포함합니다.
#include <vector>                    // 벡터를 다루기 위한 헤더 파일을 포함합니다.

#include <opencv2/core/core.hpp>     // OpenCV의 핵심 기능을 사용하기 위한 헤더 파일을 포함합니다.
#include <opencv2/highgui/highgui.hpp>     // OpenCV의 고수준 GUI 기능을 사용하기 위한 헤더 파일을 포함합니다.
#include <opencv2/imgcodecs/imgcodecs.hpp> // OpenCV의 이미지 코덱 기능을 사용하기 위한 헤더 파일을 포함합니다.


// This work was inspired on cloud_projection from I. Bogoslavskyi, C. Stachniss, University of Bonn 
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

namespace cloud_to_image { // cloud_to_image 네임스페이스를 정의합니다.

const std::vector<float> MOOSMAN_CORRECTIONS{ // MOOSMAN_CORRECTIONS 상수 벡터를 정의합니다.
    {0.02587499999999987,   -0.0061250000000001581, 0.031874999999999876,
     0.001874999999999849,  0.029874999999999874,   -0.1961250000000001,
     0.049874999999999892,  -0.034125000000000183,  0.0038749999999998508,
     0.0058749999999998526, 0.035874999999999879,   -0.064124999999999988,
     0.035874999999999879,  0.001874999999999849,   -0.024125000000000174,
     -0.062124999999999986, 0.039874999999999883,   -0.020125000000000171,
     0.075874999999999915,  -0.024125000000000174,  -0.0041250000000001563,
     -0.058124999999999982, -0.032125000000000181,  -0.058124999999999982,
     0.021874999999999867,  -0.032125000000000181,  0.059874999999999901,
     -0.04412499999999997,  0.075874999999999915,   -0.0041250000000001563,
     0.021874999999999867,  0.0058749999999998526,  -0.036125000000000185,
     -0.022125000000000172, -0.0041250000000001563, -0.058124999999999982,
     -0.026125000000000176, -0.030125000000000179,  0.045874999999999888,
     0.035874999999999879,  -0.026125000000000176,  0.041874999999999885,
     -0.086125000000000007, -0.060124999999999984,  0.031874999999999876,
     -0.010125000000000162, -0.024125000000000174,  -0.048124999999999973,
     -0.038125000000000187, 0.039874999999999883,   -0.026125000000000176,
     0.037874999999999881,  -0.020125000000000171,  0.051874999999999893,
     -0.014125000000000165, 0.019874999999999865,   -0.0021250000000001545,
     0.027874999999999872,  0.0058749999999998526,  0.021874999999999867,
     0.023874999999999869,  0.085874999999999702,   0.085874999999999702,
     0.11587499999999995}};

CloudProjection::CloudProjection(const SensorParams& params)
    : _params(params)  // 생성자 정의. SensorParams를 매개변수로 받아서 _params 멤버 변수를 초기화합니다.
{
  if (!_params.valid()) {  // 센서 매개변수가 유효하지 않을 경우
    throw std::runtime_error("sensor parameters not valid for projection.");  // 런타임 오류를 던집니다.
  }
  clearData();  // 데이터를 초기화하는 함수를 호출합니다.
  clearCorrections();  // 보정값을 초기화하는 함수를 호출합니다.
}

void CloudProjection::clearData()  // 데이터를 초기화하는 함수 정의
{
  _data = PointMatrix(_params.cols(), PointColumn(_params.rows()));  // 데이터를 행렬로 초기화합니다.
  _depth_image = cv::Mat::zeros(_params.rows(), _params.cols(), CV_32FC1);  // 깊이 이미지를 초기화합니다.
  _intensity_image = cv::Mat::zeros(_params.rows(), _params.cols(), CV_16UC1);  // 강도 이미지를 초기화합니다.
  _reflectance_image = cv::Mat::zeros(_params.rows(), _params.cols(), CV_16UC1);  // 반사 이미지를 초기화합니다.
  _noise_image = cv::Mat::zeros(_params.rows(), _params.cols(), CV_16UC1);  // 잡음 이미지를 초기화합니다.
}

void CloudProjection::loadMossmanCorrections()  // Mossman 보정값을 로드하는 함수 정의
{
  _corrections = MOOSMAN_CORRECTIONS;  // 보정값을 MOOSMAN_CORRECTIONS로 설정합니다.
}

void CloudProjection::clearCorrections()  // 보정값을 지우는 함수 정의
{
  _corrections.clear();  // 보정값을 모두 지웁니다.
}

void CloudProjection::initFromPoints(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud) 
{
  this->checkCloudAndStorage<pcl::PointCloud<pcl::PointXYZ>::ConstPtr>(cloud);  // 클라우드와 저장소를 확인합니다.
  for (size_t index = 0; index < cloud->points.size(); ++index) {  // 클라우드의 모든 포인트에 대해 반복합니다.
    const auto& point = cloud->points[index];  // 현재 포인트를 가져옵니다.
    float dist_to_sensor = std::sqrt(point.x*point.x + point.y*point.y + point.z*point.z);  // 센서와의 거리를 계산합니다.
    if (dist_to_sensor < 0.01f) {  // 센서와의 거리가 0.01보다 작으면 건너뜁니다.
      continue;
    }
    auto angle_rows = Angle::fromRadians(asin(point.z / dist_to_sensor));  // 행 각도를 계산합니다.
    auto angle_cols = Angle::fromRadians(atan2(point.y, point.x));  // 열 각도를 계산합니다.
    size_t bin_rows = this->_params.rowFromAngle(angle_rows);  // 행 인덱스를 가져옵니다.
    size_t bin_cols = this->_params.colFromAngle(angle_cols);  // 열 인덱스를 가져옵니다.
    // 포인트 포인터 추가
    this->at(bin_rows, bin_cols).points().push_back(index);
    auto& current_written_depth = this->_depth_image.template at<float>(bin_rows, bin_cols);  // 현재 기록된 깊이를 가져옵니다.
    if (current_written_depth < dist_to_sensor) {  // 현재 기록된 깊이가 더 작으면
      // 이 포인트를 이미지에 작성합니다.
      current_written_depth = dist_to_sensor;
    }
  }
  fixDepthSystematicErrorIfNeeded();  // 필요한 경우 깊이 시스템 오류를 수정합니다.
}

void CloudProjection::initFromPoints(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud) 
{
  this->checkCloudAndStorage<pcl::PointCloud<pcl::PointXYZI>::ConstPtr>(cloud);  // 클라우드와 저장소를 확인합니다.
  for (size_t index = 0; index < cloud->points.size(); ++index) {  // 클라우드의 모든 포인트에 대해 반복합니다.
    const auto& point = cloud->points[index];  // 현재 포인트를 가져옵니다.
    float dist_to_sensor = std::sqrt(point.x*point.x + point.y*point.y + point.z*point.z);  // 센서와의 거리를 계산합니다.
    uint16_t intensity = (uint16_t)((point.intensity/255.0)*65535.0);  // 포인트의 강도를 계산합니다.
    
    if (dist_to_sensor < 0.01f) {  // 센서와의 거리가 0.01보다 작으면 건너뜁니다.
      continue;
    }
    auto angle_rows = Angle::fromRadians(asin(point.z / dist_to_sensor));  // 행 각도를 계산합니다.
    auto angle_cols = Angle::fromRadians(atan2(point.y, point.x));  // 열 각도를 계산합니다.
    size_t bin_rows = this->_params.rowFromAngle(angle_rows);  // 행 인덱스를 가져옵니다.
    size_t bin_cols = this->_params.colFromAngle(angle_cols);  // 열 인덱스를 가져옵니다.
    //std::cout << point.x << "," << point.y << "," << point.z << "," << dist_to_sensor << "," << angle_rows.toDegrees() << "," << angle_cols.toDegrees() << "," << bin_rows << "," << bin_cols << "," << std::endl;
    // 포인트 포인터 추가
    this->at(bin_rows, bin_cols).points().push_back(index);
    auto& current_written_depth = this->_depth_image.template at<float>(bin_rows, bin_cols);  // 현재 기록된 깊이를 가져옵니다.
    auto& current_written_intensity = this->_intensity_image.template at<uint16_t>(bin_rows, bin_cols);  // 현재 기록된 강도를 가져옵니다.
    if (current_written_depth < dist_to_sensor) {  // 현재 기록된 깊이가 더 작으면
      // 이 포인트를 이미지에 작성합니다.
      current_written_depth = dist_to_sensor;
      current_written_intensity = intensity;
    }
  }
  fixDepthSystematicErrorIfNeeded();  // 필요한 경우 깊이 시스템 오류를 수정합니다.
}


void CloudProjection::initFromPoints(const pcl::PointCloud<pcl::PointXYZIR>::ConstPtr& cloud) 
{
  this->checkCloudAndStorage<pcl::PointCloud<pcl::PointXYZIR>::ConstPtr>(cloud);  // 클라우드와 저장소를 확인합니다.
  for (size_t index = 0; index < cloud->points.size(); ++index) {  // 클라우드의 모든 포인트에 대해 반복합니다.
    const auto& point = cloud->points[index];  // 현재 포인트를 가져옵니다.
    float dist_to_sensor = std::sqrt(point.x*point.x + point.y*point.y + point.z*point.z);  // 센서와의 거리를 계산합니다.
    uint16_t intensity = (uint16_t)((point.intensity/255.0)*65535.0);  // 포인트의 강도를 계산합니다.
    uint16_t ring = point.ring; // but ring is ignored here (링 값은 여기에서 무시됩니다)
    (void)ring;  // 변수를 사용하지 않는 것을 명시합니다.
    if (dist_to_sensor < 0.01f) {  // 센서와의 거리가 0.01보다 작으면 건너뜁니다.
      continue;
    }
    auto angle_rows = Angle::fromRadians(asin(point.z / dist_to_sensor));  // 행 각도를 계산합니다.
    auto angle_cols = Angle::fromRadians(atan2(point.y, point.x));  // 열 각도를 계산합니다.
    size_t bin_rows = this->_params.rowFromAngle(angle_rows);  // 행 인덱스를 가져옵니다.
    size_t bin_cols = this->_params.colFromAngle(angle_cols);  // 열 인덱스를 가져옵니다.
    // 포인트 포인터 추가
    this->at(bin_rows, bin_cols).points().push_back(index);
    auto& current_written_depth = this->_depth_image.template at<float>(bin_rows, bin_cols);  // 현재 기록된 깊이를 가져옵니다.
    auto& current_written_intensity = this->_intensity_image.template at<uint16_t>(bin_rows, bin_cols);  // 현재 기록된 강도를 가져옵니다.
    if (current_written_depth < dist_to_sensor) {  // 현재 기록된 깊이가 더 작으면
      // 이 포인트를 이미지에 작성합니다.
      current_written_depth = dist_to_sensor;
      current_written_intensity = intensity;
    }
  }
  fixDepthSystematicErrorIfNeeded();  // 필요한 경우 깊이 시스템 오류를 수정합니다.
}


void CloudProjection::initFromPoints(const pcl::PointCloud<pcl::PointXYZIF>::ConstPtr& cloud) 
{
  this->checkCloudAndStorage<pcl::PointCloud<pcl::PointXYZIF>::ConstPtr>(cloud);  // 클라우드와 저장소를 확인합니다.
  for (size_t index = 0; index < cloud->points.size(); ++index) {  // 클라우드의 모든 포인트에 대해 반복합니다.
    const auto& point = cloud->points[index];  // 현재 포인트를 가져옵니다.
    float dist_to_sensor = std::sqrt(point.x*point.x + point.y*point.y + point.z*point.z);  // 센서와의 거리를 계산합니다.
    uint16_t intensity = point.intensity;  // 포인트의 강도를 가져옵니다.
    uint16_t reflectivity = point.reflectivity;  // 포인트의 반사도를 가져옵니다.
    if (dist_to_sensor < 0.01f) {  // 센서와의 거리가 0.01보다 작으면 건너뜁니다.
      continue;
    }
    auto angle_rows = Angle::fromRadians(asin(point.z / dist_to_sensor));  // 행 각도를 계산합니다.
    auto angle_cols = Angle::fromRadians(atan2(point.y, point.x));  // 열 각도를 계산합니다.
    size_t bin_rows = this->_params.rowFromAngle(angle_rows);  // 행 인덱스를 가져옵니다.
    size_t bin_cols = this->_params.colFromAngle(angle_cols);  // 열 인덱스를 가져옵니다.
    // 포인트 포인터 추가
    this->at(bin_rows, bin_cols).points().push_back(index);
    auto& current_written_depth = this->_depth_image.template at<float>(bin_rows, bin_cols);  // 현재 기록된 깊이를 가져옵니다.
    auto& current_written_intensity = this->_intensity_image.template at<uint16_t>(bin_rows, bin_cols);  // 현재 기록된 강도를 가져옵니다.
    auto& current_written_reflectivity = this->_reflectance_image.template at<uint16_t>(bin_rows, bin_cols);  // 현재 기록된 반사도를 가져옵니다.
    if (current_written_depth < dist_to_sensor) {  // 현재 기록된 깊이가 더 작으면
      // 이 포인트를 이미지에 작성합니다.
      current_written_depth = dist_to_sensor;
      current_written_intensity = intensity;
      current_written_reflectivity = reflectivity;
    }
  }
  fixDepthSystematicErrorIfNeeded();  // 필요한 경우 깊이 시스템 오류를 수정합니다.
}


void CloudProjection::initFromPoints(const pcl::PointCloud<pcl::PointXYZIFN>::ConstPtr& cloud) 
{
  this->checkCloudAndStorage<pcl::PointCloud<pcl::PointXYZIFN>::ConstPtr>(cloud);  // 클라우드와 저장소를 확인합니다.
  for (size_t index = 0; index < cloud->points.size(); ++index) {  // 클라우드의 모든 포인트에 대해 반복합니다.
    const auto& point = cloud->points[index];  // 현재 포인트를 가져옵니다.
    float dist_to_sensor = std::sqrt(point.x*point.x + point.y*point.y + point.z*point.z);  // 센서와의 거리를 계산합니다.
    uint16_t intensity = point.intensity;  // 포인트의 강도를 가져옵니다.
    uint16_t reflectivity = point.reflectivity;  // 포인트의 반사도를 가져옵니다.
    uint16_t noise = point.noise;  // 포인트의 노이즈 값을 가져옵니다.
    if (dist_to_sensor < 0.01f) {  // 센서와의 거리가 0.01보다 작으면 건너뜁니다.
      continue;
    }
    auto angle_rows = Angle::fromRadians(asin(point.z / dist_to_sensor));  // 행 각도를 계산합니다.
    auto angle_cols = Angle::fromRadians(atan2(point.y, point.x));  // 열 각도를 계산합니다.
    size_t bin_rows = this->_params.rowFromAngle(angle_rows);  // 행 인덱스를 가져옵니다.
    size_t bin_cols = this->_params.colFromAngle(angle_cols);  // 열 인덱스를 가져옵니다.
    // 포인트 포인터 추가
    this->at(bin_rows, bin_cols).points().push_back(index);
    auto& current_written_depth = this->_depth_image.template at<float>(bin_rows, bin_cols);  // 현재 기록된 깊이를 가져옵니다.
    auto& current_written_intensity = this->_intensity_image.template at<uint16_t>(bin_rows, bin_cols);  // 현재 기록된 강도를 가져옵니다.
    auto& current_written_reflectivity = this->_reflectance_image.template at<uint16_t>(bin_rows, bin_cols);  // 현재 기록된 반사도를 가져옵니다.
    auto& current_written_noise = this->_noise_image.template at<uint16_t>(bin_rows, bin_cols);  // 현재 기록된 노이즈를 가져옵니다.
    if (current_written_depth < dist_to_sensor) {  // 현재 기록된 깊이가 더 작으면
      // 이 포인트를 이미지에 작성합니다.
      current_written_depth = dist_to_sensor;
      current_written_intensity = intensity;
      current_written_reflectivity = reflectivity;
      current_written_noise = noise;
    }
  }
  fixDepthSystematicErrorIfNeeded();  // 필요한 경우 깊이 시스템 오류를 수정합니다.
}


pcl::PointCloud<pcl::PointXYZI>::Ptr CloudProjection::fromImage(const cv::Mat& depth_image) {
  checkImageAndStorage(depth_image);  // 이미지와 저장소를 확인합니다.
  cloneDepthImage(depth_image);  // 깊이 이미지를 복제합니다.
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);  // 포인트 클라우드를 생성합니다.
  for (int r = 0; r < depth_image.rows; ++r) {  // 모든 행에 대해 반복합니다.
    for (int c = 0; c < depth_image.cols; ++c) {  // 각 행의 모든 열에 대해 반복합니다.
      if (depth_image.at<float>(r, c) < 0.0001f) {  // 깊이 값이 임계값보다 작으면 건너뜁니다.
        continue;
      }
      pcl::PointXYZ point;  // 포인트를 생성합니다.
      unprojectPoint(depth_image, r, c, point);  // 이미지로부터 포인트를 역투영합니다.
      pcl::PointXYZI point2;  // 강도를 포함한 포인트를 생성합니다.
      point2.x = point.x;  // x 좌표를 설정합니다.
      point2.y = point.y;  // y 좌표를 설정합니다.
      point2.z = point.z;  // z 좌표를 설정합니다.
      point2.intensity = 0;  // 강도 값을 0으로 설정합니다.
      cloud->points.push_back(point2);  // 클라우드에 포인트를 추가합니다.
      this->at(r, c).points().push_back(cloud->points.size() - 1);  // 현재 픽셀에 포인트 인덱스를 추가합니다.
    }
  }
  return cloud;  // 클라우드를 반환합니다.
}



pcl::PointCloud<pcl::PointXYZI>::Ptr CloudProjection::fromImage(const cv::Mat& depth_image, const cv::Mat& intensity_image) 
{
  checkImageAndStorage(depth_image);  // 깊이 이미지와 저장소를 확인합니다.
  cloneDepthImage(depth_image);  // 깊이 이미지를 복제합니다.
  checkImageAndStorage(intensity_image);  // 강도 이미지와 저장소를 확인합니다.
  cloneDepthImage(intensity_image);  // 강도 이미지를 복제합니다.
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);  // 포인트 클라우드를 생성합니다.
  for (int r = 0; r < depth_image.rows; ++r) {  // 모든 행에 대해 반복합니다.
    for (int c = 0; c < depth_image.cols; ++c) {  // 각 행의 모든 열에 대해 반복합니다.
      if (depth_image.at<float>(r, c) < 0.0001f) {  // 깊이 값이 임계값보다 작으면 건너뜁니다.
        continue;
      }
      pcl::PointXYZI point;  // 강도를 포함한 포인트를 생성합니다.
      unprojectPoint(depth_image, intensity_image, r, c, point);  // 이미지로부터 포인트를 역투영합니다.
      cloud->points.push_back(point);  // 클라우드에 포인트를 추가합니다.
      this->at(r, c).points().push_back(cloud->points.size() - 1);  // 현재 픽셀에 포인트 인덱스를 추가합니다.
    }
  }
  return cloud;  // 클라우드를 반환합니다.
}

void CloudProjection::unprojectPoint(const cv::Mat& depth_image, const int row, const int col, pcl::PointXYZ& point) const 
{
  float depth = depth_image.at<float>(row, col);  // 이미지에서 해당 위치의 깊이를 가져옵니다.
  Angle angle_z = this->_params.angleFromRow(row);  // 행 인덱스로부터 z 각도를 가져옵니다.
  Angle angle_xy = this->_params.angleFromCol(col);  // 열 인덱스로부터 xy 평면 각도를 가져옵니다.
  
  point.x = depth * cosf(angle_z.val()) * cosf(angle_xy.val());  // x 좌표를 계산합니다.
  point.y = depth * cosf(angle_z.val()) * sinf(angle_xy.val());  // y 좌표를 계산합니다.
  point.z = depth * sinf(angle_z.val());  // z 좌표를 계산합니다.
}


void CloudProjection::unprojectPoint(const cv::Mat& depth_image, const cv::Mat& intensity_image, const int row, const int col, pcl::PointXYZI& point) const 
{
  float depth = depth_image.at<float>(row, col);  // 이미지에서 해당 위치의 깊이를 가져옵니다.
  Angle angle_z = this->_params.angleFromRow(row);  // 행 인덱스로부터 z 각도를 가져옵니다.
  Angle angle_xy = this->_params.angleFromCol(col);  // 열 인덱스로부터 xy 평면 각도를 가져옵니다.
  
  point.x = depth * cosf(angle_z.val()) * cosf(angle_xy.val());  // x 좌표를 계산합니다.
  point.y = depth * cosf(angle_z.val()) * sinf(angle_xy.val());  // y 좌표를 계산합니다.
  point.z = depth * sinf(angle_z.val());  // z 좌표를 계산합니다.
  point.intensity =  intensity_image.at<uint16_t>(row, col);  // 해당 위치의 강도 값을 가져와서 포인트의 강도에 할당합니다.
}

void CloudProjection::unprojectPoint(const cv::Mat& depth_image, const cv::Mat& intensity_image, const int row, const int col, pcl::PointXYZIR& point) const 
{
  float depth = depth_image.at<float>(row, col);  // 이미지에서 해당 위치의 깊이를 가져옵니다.
  Angle angle_z = this->_params.angleFromRow(row);  // 행 인덱스로부터 z 각도를 가져옵니다.
  Angle angle_xy = this->_params.angleFromCol(col);  // 열 인덱스로부터 xy 평면 각도를 가져옵니다.
  
  point.x = depth * cosf(angle_z.val()) * cosf(angle_xy.val());  // x 좌표를 계산합니다.
  point.y = depth * cosf(angle_z.val()) * sinf(angle_xy.val());  // y 좌표를 계산합니다.
  point.z = depth * sinf(angle_z.val());  // z 좌표를 계산합니다.
  point.intensity =  intensity_image.at<uint16_t>(row, col);  // 해당 위치의 강도 값을 가져와서 포인트의 강도에 할당합니다.
  point.ring = row; // 포인트의 ring 값을 행 번호로 설정합니다.
}


void CloudProjection::unprojectPoint(const cv::Mat& depth_image, const cv::Mat& intensity_image, const cv::Mat& reflectance_image, const int row, const int col, pcl::PointXYZIF& point) const
{
  float depth = depth_image.at<float>(row, col);  // 이미지에서 해당 위치의 깊이를 가져옵니다.
  Angle angle_z = this->_params.angleFromRow(row);  // 행 인덱스로부터 z 각도를 가져옵니다.
  Angle angle_xy = this->_params.angleFromCol(col);  // 열 인덱스로부터 xy 평면 각도를 가져옵니다.
  
  point.x = depth * cosf(angle_z.val()) * cosf(angle_xy.val());  // x 좌표를 계산합니다.
  point.y = depth * cosf(angle_z.val()) * sinf(angle_xy.val());  // y 좌표를 계산합니다.
  point.z = depth * sinf(angle_z.val());  // z 좌표를 계산합니다.
  point.intensity =  intensity_image.at<uint16_t>(row, col);  // 해당 위치의 강도 값을 가져와서 포인트의 강도에 할당합니다.
  point.reflectivity =  reflectance_image.at<uint16_t>(row, col);  // 해당 위치의 반사도 값을 가져와서 포인트의 반사도에 할당합니다.
}


void CloudProjection::unprojectPoint(const cv::Mat& depth_image, const cv::Mat& intensity_image, const cv::Mat& reflectance_image, const cv::Mat& noise_image, const int row, const int col, pcl::PointXYZIFN& point) const
{
  float depth = depth_image.at<float>(row, col);  // 이미지에서 해당 위치의 깊이를 가져옵니다.
  Angle angle_z = this->_params.angleFromRow(row);  // 행 인덱스로부터 z 각도를 가져옵니다.
  Angle angle_xy = this->_params.angleFromCol(col);  // 열 인덱스로부터 xy 평면 각도를 가져옵니다.
  
  point.x = depth * cosf(angle_z.val()) * cosf(angle_xy.val());  // x 좌표를 계산합니다.
  point.y = depth * cosf(angle_z.val()) * sinf(angle_xy.val());  // y 좌표를 계산합니다.
  point.z = depth * sinf(angle_z.val());  // z 좌표를 계산합니다.
  point.intensity =  intensity_image.at<uint16_t>(row, col);  // 해당 위치의 강도 값을 가져와서 포인트의 강도에 할당합니다.
  point.reflectivity =  reflectance_image.at<uint16_t>(row, col);  // 해당 위치의 반사도 값을 가져와서 포인트의 반사도에 할당합니다.
  point.noise =  noise_image.at<uint16_t>(row, col);  // 해당 위치의 잡음 값을 가져와서 포인트의 잡음에 할당합니다.
}


template <typename T>
void CloudProjection::checkCloudAndStorage(const T& cloud) {
  if (this->_data.size() < 1) {  // _data 배열 크기가 1보다 작으면
    throw std::length_error("_data size is < 1");  // 길이 오류를 발생시킵니다.
  }
  if (cloud->points.size() == 0) {  // 입력된 포인트 클라우드의 포인트 수가 0이면
    throw std::runtime_error("cannot fill from cloud: no points");  // 런타임 오류를 발생시킵니다.
  }
}


void CloudProjection::checkImageAndStorage(const cv::Mat& image) {
  if (image.type() != CV_32F && image.type() != CV_16U) {  // 이미지의 타입이 부동 소수점 형식이 아니거나 16비트 부호없는 정수 형식이 아니면
    throw std::runtime_error("wrong image format");  // 잘못된 이미지 형식으로 오류를 발생시킵니다.
  }
  if (this->_data.size() < 1) {  // _data 배열의 크기가 1보다 작으면
    throw std::length_error("_data size is < 1");  // 길이 오류를 발생시킵니다.
  }
  if (this->rows() != static_cast<size_t>(image.rows) ||  // _data의 행 수가 이미지의 행 수와 일치하지 않거나
      this->cols() != static_cast<size_t>(image.cols)) {  // _data의 열 수가 이미지의 열 수와 일치하지 않으면
    throw std::length_error("_data dimensions do not correspond to image ones");  // _data의 차원이 이미지와 일치하지 않음을 나타내는 길이 오류를 발생시킵니다.
  }
}


void CloudProjection::fixDepthSystematicErrorIfNeeded() {
  if (_depth_image.rows < 1) {  // 깊이 이미지의 행이 1보다 작으면
    //fprintf(stderr, "[INFO]: depth image of wrong size, not correcting depth\n");  // 오류 메시지를 출력하고 깊이 보정을 수행하지 않습니다.
    return;
  }
  if (_intensity_image.rows < 1) {  // 강도 이미지의 행이 1보다 작으면
    //fprintf(stderr, "[INFO]: intensity image of wrong size, not correcting depth\n");  // 오류 메시지를 출력하고 깊이 보정을 수행하지 않습니다.
    return;
  }
  if (_reflectance_image.rows < 1) {  // 반사도 이미지의 행이 1보다 작으면
    //fprintf(stderr, "[INFO]: reflectance image of wrong size, not correcting depth\n");  // 오류 메시지를 출력하고 깊이 보정을 수행하지 않습니다.
    return;
  }
  if (_noise_image.rows < 1) {  // 잡음 이미지의 행이 1보다 작으면
    //fprintf(stderr, "[INFO]: noise image of wrong size, not correcting depth\n");  // 오류 메시지를 출력하고 깊이 보정을 수행하지 않습니다.
    return;
  }  
  if (_corrections.size() != static_cast<size_t>(_depth_image.rows)) {  // 보정 배열의 크기가 깊이 이미지의 행 수와 다르면
    //fprintf(stderr, "[INFO]: Not correcting depth data.\n");  // 오류 메시지를 출력하고 깊이 보정을 수행하지 않습니다.
    return;
  }
  for (int r = 0; r < _depth_image.rows; ++r) {  // 모든 행에 대해 반복합니다.
    auto correction = _corrections[r];  // 해당 행에 대한 보정 값을 가져옵니다.
    for (int c = 0; c < _depth_image.cols; ++c) {  // 모든 열에 대해 반복합니다.
      if (_depth_image.at<float>(r, c) < 0.001f) {  // 해당 위치의 깊이가 매우 작으면(거의 0에 가까우면)
        continue;  // 반복문을 건너뜁니다.
      }
      _depth_image.at<float>(r, c) -= correction;  // 보정 값을 깊이 이미지에서 빼줍니다.
    }
  }
}



pcl::PointCloud<pcl::PointXYZI>::Ptr CloudProjection::readKittiCloud(const std::string& filename) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
  std::fstream file(filename.c_str(), std::ios::in | std::ios::binary);  // 이진 파일로부터 읽기 위해 파일을 엽니다.
  if (file.good()) {  // 파일이 정상적으로 열렸는지 확인합니다.
    file.seekg(0, std::ios::beg);  // 파일의 시작으로 이동합니다.
    for (int i = 0; file.good() && !file.eof(); ++i) {  // 파일이 정상이고 파일의 끝이 아닌 동안 반복합니다.
      pcl::PointXYZI point;  // PCL의 XYZI 포인트를 생성합니다.
      file.read(reinterpret_cast<char*>(&point.x), sizeof(float));  // 파일에서 X 좌표를 읽습니다.
      file.read(reinterpret_cast<char*>(&point.y), sizeof(float));  // 파일에서 Y 좌표를 읽습니다.
      file.read(reinterpret_cast<char*>(&point.z), sizeof(float));  // 파일에서 Z 좌표를 읽습니다.
      // 강도는 무시합니다.
      file.read(reinterpret_cast<char*>(&point.intensity), sizeof(float));  // 파일에서 강도를 읽습니다.
      cloud->push_back(point);  // 포인트 클라우드에 포인트를 추가합니다.
    }
    file.close();  // 파일을 닫습니다.
  }
  if (!cloud->points.size()) {  // 포인트 클라우드의 포인트 수가 0이면
    throw std::runtime_error("point cloud is empty, cannot load");  // 런타임 오류를 발생시킵니다.
  } 
  return cloud;  // 포인트 클라우드를 반환합니다.
}

cv::Mat CloudProjection::fixKITTIDepth(const cv::Mat& original)  
{
  cv::Mat fixed = original;  // 원본 이미지를 수정할 이미지에 복사합니다.
  for (int r = 0; r < fixed.rows; ++r) {  // 이미지의 각 행에 대해 반복합니다.
    auto correction = MOOSMAN_CORRECTIONS[r];  // 보정값을 가져옵니다.
    for (int c = 0; c < fixed.cols; ++c) {  // 각 행의 열에 대해 반복합니다.
      if (fixed.at<float>(r, c) < 0.001f) {  // 이미지의 해당 픽셀 값이 아주 작으면
        continue;  // 다음 픽셀로 넘어갑니다.
      }
      fixed.at<float>(r, c) -= correction;  // 보정값을 현재 픽셀의 깊이 값에서 뺍니다.
    }
  }
  return fixed;  // 수정된 이미지를 반환합니다.
}



cv::Mat CloudProjection::cvMatFromDepthPNG(const std::string& filename) 
{
  cv::Mat depth_image = cv::imread(filename, cv::IMREAD_ANYDEPTH);  // 깊이 PNG 파일을 읽어옵니다.
  depth_image.convertTo(depth_image, CV_32F);  // 깊이 이미지를 부동 소수점 형식으로 변환합니다.
  if (depth_image.type() != CV_32F && depth_image.type() != CV_16U) {  // 이미지 형식을 확인합니다.
    throw std::runtime_error("wrong image format, cannot load");  // 잘못된 형식이면 예외를 던집니다.
  }
  if (depth_image.rows < 1 || depth_image.cols < 1) {  // 이미지 크기를 확인합니다.
    throw std::runtime_error("wrong image format, cannot load");  // 잘못된 크기면 예외를 던집니다.
  }

  depth_image /= 500.;  // 깊이 값을 500으로 나누어 정규화합니다.
  return fixKITTIDepth(depth_image);  // 보정된 깊이 이미지를 반환합니다.
}
   //위 코드는 깊이 PNG 파일로부터 OpenCV의 cv::Mat 형식으로 깊이 이미지를 읽어오는 함수입니다. 이미지를 읽어온 후에는 부동 소수점 형식으로 변환하고,
    //형식과 크기를 확인한 후에는 깊이 값을 500으로 나누어 정규화한 뒤에 보정된 깊이 이미지를 반환합니다.

    
void CloudProjection::cloudToPCDFile(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud, const std::string& filename)
{
    // 포인트 클라우드가 비어 있는지 확인합니다.
    if (!cloud->points.size()) {
        throw std::runtime_error("point cloud is empty, cannot save");
    }  
    // PCD 파일로 저장합니다.
    pcl::io::savePCDFileBinary(filename, *cloud);
}

void CloudProjection::cvMatToDepthPNG(const cv::Mat& image, const std::string& filename)
{
    // 이미지 형식이 잘못되었는지 확인합니다.
    if (image.type() != CV_32F && image.type() != CV_16U) {
        throw std::runtime_error("wrong image format for depth information, cannot save when saving image for file \"" + filename + "\"");
    }
    // 깊이 정보를 포함한 이미지를 PNG로 저장합니다.
    cvMatToColorPNG(image, filename);
}

void CloudProjection::cvMatToColorPNG(const cv::Mat& image, const std::string& filename)
{
    // 이미지 형식이 잘못되었는지 확인합니다.
    if (image.rows < 1 || image.cols < 1) {
        throw std::runtime_error("wrong image format, cannot save when saving image for file \"" + filename + "\"");
    }  
    // 색상 정보를 포함한 이미지를 PNG로 저장합니다.
    try {
        cv::imwrite(filename, image);
    }
    catch (std::runtime_error& ex) {
        // PNG 형식으로 변환하는 중에 예외가 발생하면 처리합니다.
        fprintf(stderr, "Exception converting image to PNG format: %s\n", ex.what());
    }
}

