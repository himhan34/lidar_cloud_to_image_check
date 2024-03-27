
#ifndef CLOUD_PROJECTION_H_  // 클라우드 프로젝션 헤더파일 시작을 나타냅니다.
#define CLOUD_PROJECTION_H_  // 중복 포함을 방지하기 위한 헤더가드입니다.

#include <opencv2/core/core.hpp>  // OpenCV의 기본 기능을 사용하기 위한 헤더 파일입니다.

#include <Eigen/Core>  // Eigen 라이브러리의 기본 기능을 사용하기 위한 헤더 파일입니다.

#include <list>  // 리스트 컨테이너를 사용하기 위한 헤더 파일입니다.
#include <memory>  // 스마트 포인터를 사용하기 위한 헤더 파일입니다.
#include <stdexcept>  // 예외 처리를 위한 헤더 파일입니다.
#include <vector>  // 벡터 컨테이너를 사용하기 위한 헤더 파일입니다.

#include <pcl_ros/point_cloud.h>  // PCL(Point Cloud Library)의 ROS 지원 헤더 파일입니다.

#include "projection_params.h"  // 투영 매개변수에 관한 헤더 파일입니다.
#include "angles.h"  // 각도 변환에 관한 헤더 파일입니다.
#include "pcl_point_types.h"  // PCL 포인트 타입에 관한 헤더 파일입니다.



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

namespace cloud_to_image {  // cloud_to_image 네임스페이스를 엽니다.

/**
 * @brief      클라우드 투영을 위한 추상 클래스입니다.
 */
class CloudProjection {
  class PointContainer;
  // 몇 가지 유용한 using 선언들
  using PointColumn = std::vector<PointContainer>;  // 포인트 컨테이너의 열을 나타내는 형식입니다.
  using PointMatrix = std::vector<PointColumn>;     // 포인트 매트릭스를 나타내는 형식입니다.

  public:
    using Ptr = shared_ptr<CloudProjection>;         // CloudProjection 포인터를 나타내는 형식입니다.
    using ConstPtr = shared_ptr<const CloudProjection>;  // 상수 CloudProjection 포인터를 나타내는 형식입니다.

    /**
     * @brief     주어진 센서 매개변수를 사용하여 CloudProjection 객체를 생성합니다.
     * @param[in] params  센서 매개변수
     */
    explicit CloudProjection(const SensorParams& params);  // 명시적 생성자입니다.
    virtual ~CloudProjection() {}  // 가상 소멸자입니다.



    /**
     * @brief      데이터를 초기화합니다.
     */
    void clearData();  // 데이터를 지우는 함수입니다.

    /**
     * @brief      3D 포인트로부터 초기화합니다.
     *
     * @param[in]  cloud  포인트
     */
    void initFromPoints(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud);  // 3D 포인트로부터 초기화합니다.
    void initFromPoints(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud);  // 3D 포인트로부터 초기화합니다.
    void initFromPoints(const pcl::PointCloud<pcl::PointXYZIR>::ConstPtr& cloud);  // 3D 포인트로부터 초기화합니다.
    void initFromPoints(const pcl::PointCloud<pcl::PointXYZIF>::ConstPtr& cloud);  // 3D 포인트로부터 초기화합니다.
    void initFromPoints(const pcl::PointCloud<pcl::PointXYZIFN>::ConstPtr& cloud);  // 3D 포인트로부터 초기화합니다.

   // 깊이 이미지를 사용하여 포인트 클라우드를 생성하는 함수
  pcl::PointCloud<pcl::PointXYZI>::Ptr fromImage(const cv::Mat& depth_image);
  
  // 깊이 이미지와 강도 이미지를 사용하여 포인트 클라우드를 생성하는 함수
  pcl::PointCloud<pcl::PointXYZI>::Ptr fromImage(const cv::Mat& depth_image, const cv::Mat& intensity_image);
  
  // const로 선언된 깊이 이미지에 대한 접근자 함수
  inline const cv::Mat& depth_image() const { return this->_depth_image; }
  
  // 깊이 이미지에 대한 접근자 함수
  inline cv::Mat& depth_image() { return this->_depth_image; }
  
  // 다른 이미지를 복제하여 깊이 이미지를 설정하는 함수
  inline void cloneDepthImage(const cv::Mat& image) { _depth_image = image.clone(); }
  
  // const로 선언된 강도 이미지에 대한 접근자 함수
  inline const cv::Mat& intensity_image() const { return this->_intensity_image; }
  
  // 강도 이미지에 대한 접근자 함수
  inline cv::Mat& intensity_image() { return this->_intensity_image; }


  // 다른 이미지를 복제하여 강도 이미지를 설정하는 함수
  inline void cloneIntensityImage(const cv::Mat& image) { _intensity_image = image.clone(); }
  
  // const로 선언된 반사 이미지에 대한 접근자 함수
  inline const cv::Mat& reflectance_image() const { return this->_reflectance_image; }
  
  // 반사 이미지에 대한 접근자 함수
  inline cv::Mat& reflectance_image() { return this->_reflectance_image; }
  
  // 다른 이미지를 복제하여 반사 이미지를 설정하는 함수
  inline void cloneReflectanceImage(const cv::Mat& image) { _reflectance_image = image.clone(); }
  
  // const로 선언된 잡음 이미지에 대한 접근자 함수
  inline const cv::Mat& noise_image() const { return this->_noise_image; }
  
  // 잡음 이미지에 대한 접근자 함수
  inline cv::Mat& noise_image() { return this->_noise_image; }
  
  // 다른 이미지를 복제하여 잡음 이미지를 설정하는 함수
  inline void cloneNoiseImage(const cv::Mat& image) { _noise_image = image.clone(); }


  // 행 수를 반환하는 접근자 함수
  inline size_t rows() const { return _params.rows(); }
  
  // 열 수를 반환하는 접근자 함수
  inline size_t cols() const { return _params.cols(); }
  
  // 데이터의 크기를 반환하는 접근자 함수
  inline size_t size() const { return _params.size(); }
  
  // 센서 매개변수에 대한 접근자 함수
  inline const SensorParams& params() const { return _params; }
  
  // 지정된 위치(row, col)에 있는 포인트 컨테이너를 반환하는 접근자 함수
  inline const PointContainer& at(const size_t row, const size_t col) const { return _data[col][row]; }
  
  // 지정된 위치(row, col)에 있는 포인트 컨테이너를 반환하는 접근자 함수
  inline PointContainer& at(const size_t row, const size_t col) { return _data[col][row]; }
  
  // 데이터 매트릭스에 대한 접근자 함수
  inline const PointMatrix& matrix() const { return _data; }


    /**
     * @brief      Check if where we store data is valid.
     *
     * @param[in]  image  The image to check
     */
    // 이미지와 저장소를 확인하는 함수
    void checkImageAndStorage(const cv::Mat& image);

    /**
     * @brief      Check if where we store data is valid.
     *
     * @param[in]  points  The points to check
     */
     // 포인트 형식을 확인하고 저장소를 확인하는 함수
    template <typename T>
    void checkCloudAndStorage(const T& points);


    /**
     * @brief      Unproject a point from depth image coordinate
     *
     * @param[in]  image  A depth image
     * @param[in]  row    A row in the image
     * @param[in]  col    A col in the image
     *
     * @return     { description_of_the_return_value }
     */
    // 깊이 이미지에서 지정된 행 및 열에 있는 포인트를 다시 투영하는 함수 (포인트 타입: pcl::PointXYZ)
    void unprojectPoint(const cv::Mat& depth_image, const int row, const int col, pcl::PointXYZ& point) const;
    
    // 깊이 이미지와 강도 이미지에서 지정된 행 및 열에 있는 포인트를 다시 투영하는 함수 (포인트 타입: pcl::PointXYZI)
    void unprojectPoint(const cv::Mat& depth_image, const cv::Mat& intensity_image, const int row, const int col, pcl::PointXYZI& point) const;
    
    // 깊이 이미지와 강도 이미지에서 지정된 행 및 열에 있는 포인트를 다시 투영하는 함수 (포인트 타입: pcl::PointXYZIR)
    void unprojectPoint(const cv::Mat& depth_image, const cv::Mat& intensity_image, const int row, const int col, pcl::PointXYZIR& point) const;
    
    // 깊이 이미지, 강도 이미지 및 반사 이미지에서 지정된 행 및 열에 있는 포인트를 다시 투영하는 함수 (포인트 타입: pcl::PointXYZIF)
    void unprojectPoint(const cv::Mat& depth_image, const cv::Mat& intensity_image, const cv::Mat& reflectance_image, const int row, const int col, pcl::PointXYZIF& point) const;
    
    // 깊이 이미지, 강도 이미지, 반사 이미지 및 잡음 이미지에서 지정된 행 및 열에 있는 포인트를 다시 투영하는 함수 (포인트 타입: pcl::PointXYZIFN)
    void unprojectPoint(const cv::Mat& depth_image, const cv::Mat& intensity_image, const cv::Mat& reflectance_image, const cv::Mat& noise_image, const int row, const int col, pcl::PointXYZIFN& point) const;


  /**
   * @brief      데이터셋의 시스템 오류에 대한 보정 설정 (저장소 내 노트북 참조)
   *
   * @param[in]  corrections  각 빔에 대한 깊이 보정값의 벡터
   */
  inline void setCorrections(const std::vector<float>& corrections) 
  {
      _corrections = corrections; // 보정 값을 설정합니다.
  }
  
  /**
   * @brief      시스템 오류 수정. 자세한 내용은 리포지토리의 노트북을 참조하십시오.
   */

  void fixDepthSystematicErrorIfNeeded(); // 필요한 경우 깊이 시스템 오류를 수정합니다.
  
  static pcl::PointCloud<pcl::PointXYZI>::Ptr readKittiCloud(const std::string& filename); // 키티 클라우드를 읽습니다.
  
  static cv::Mat cvMatFromDepthPNG(const std::string& filename); // 깊이 PNG로부터 OpenCV Mat을 생성합니다.
  
  static void cloudToPCDFile(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud, const std::string& filename); // 포인트 클라우드를 PCD 파일로 저장합니다.
  
  static void cvMatToDepthPNG(const cv::Mat& image, const std::string& filename); // OpenCV Mat을 깊이 PNG로 변환합니다.
  
  static void cvMatToColorPNG(const cv::Mat& image, const std::string& filename); // OpenCV Mat을 컬러 PNG로 변환합니다.
  
  void loadMossmanCorrections(); // Mossman 보정값을 로드합니다.
  
  void clearCorrections(); // 보정 값을 지웁니다.
  
  private:
      static cv::Mat fixKITTIDepth(const cv::Mat& original); // KITTI 깊이를 수정합니다.
  
  protected:
      // just stores addresses of the points. Does not own them.
      PointMatrix _data; // 포인트의 주소를 저장합니다. 소유하지 않습니다.
  
      SensorParams _params; // 센서 매개변수를 저장합니다.
  
      cv::Mat _depth_image; // 깊이 이미지를 저장합니다.
  
      cv::Mat _intensity_image; // 강도 이미지를 저장합니다.
  
      cv::Mat _reflectance_image; // 반사 이미지를 저장합니다.
  
      cv::Mat _noise_image; // 잡음 이미지를 저장합니다.
  
      std::vector<float> _corrections; // 보정 값을 저장합니다.
  };


/**
 * @brief      포인트 컨테이너 클래스입니다.
 */
class CloudProjection::PointContainer { // CloudProjection 내의 PointContainer 클래스입니다.
  public:
    PointContainer() {} // 생성자입니다.

    inline bool isEmpty() const { return _points.empty(); } // 비어있는지 여부를 반환하는 인라인 함수입니다.

    inline std::list<size_t>& points() { return _points; } // 포인트의 리스트에 대한 참조를 반환하는 인라인 함수입니다.

    inline const std::list<size_t>& points() const { return _points; } // 포인트의 리스트에 대한 상수 참조를 반환하는 인라인 함수입니다.

  private:
    std::list<size_t> _points; // 포인트의 리스트입니다.
};

}  // namespace cloud_to_image // cloud_to_image 네임스페이스입니다.

#endif  // CLOUD_PROJECTION_H_ // CLOUD_PROJECTION_H_ 헤더 파일의 끝입니다.
