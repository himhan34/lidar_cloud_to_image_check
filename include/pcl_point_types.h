// #ifndef PCL_POINT_TYPES_H_
// #define PCL_POINT_TYPES_H_

#pragma once // 이 헤더 파일이 한 번 이상 인클루드되는 것을 방지하기 위한 pragma 지시문
#define PCL_NO_PRECOMPILE // PCL이 포인트 타입을 사전 컴파일하지 않도록 지시하는 매크로
#include <pcl/point_types.h> // PCL의 포인트 타입 헤더 파일을 인클루드


namespace pcl
{

//PointXYZ와 PointXYZI는 PCL에서 표준이므로 여기서는 정의하지 않음

// X, Y, Z, intensity, ring 값을 가지는 포인트 구조체
struct EIGEN_ALIGN16 PointXYZIR {
    PCL_ADD_POINT4D; // X, Y, Z를 추가
    float    intensity;                 ///< 레이저 강도 값
    uint16_t ring;                      ///< 레이저 링 번호
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // 올바른 정렬을 보장하기 위한 매크로
};

// X, Y, Z, 강도, 반사율 값을 가지는 포인트 구조체
struct EIGEN_ALIGN16 PointXYZIF {
    PCL_ADD_POINT4D; // X, Y, Z를 추가
    uint16_t intensity; // 강도
    uint16_t reflectivity; // 반사율
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // 올바른 정렬을 보장하기 위한 매크로
};

// X, Y, Z, 강도, 반사율, 잡음 값을 가지는 포인트 구조체
struct EIGEN_ALIGN16 PointXYZIFN {
    PCL_ADD_POINT4D; // X, Y, Z를 추가
    uint16_t intensity; // 강도
    uint16_t reflectivity; // 반사율
    uint16_t noise; // 잡음
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // 올바른 정렬을 보장하기 위한 매크로
};


POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::PointXYZIR, // PointXYZIR 구조체를 PCL 포인트 클라우드 라이브러리에 등록
    (float, x, x) // x 좌표
    (float, y, y) // y 좌표
    (float, z, z) // z 좌표
    (float, intensity, intensity) // 강도 값
    (uint16_t, ring, ring) // 링 번호
)

POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::PointXYZIF, // PointXYZIF 구조체를 PCL 포인트 클라우드 라이브러리에 등록
    (float, x, x) // x 좌표
    (float, y, y) // y 좌표
    (float, z, z) // z 좌표
    (uint16_t, intensity, intensity) // 강도 값
    (uint16_t, reflectivity, reflectivity) // 반사율 값
)


POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::PointXYZIFN, // PointXYZIFN 구조체를 PCL 포인트 클라우드 라이브러리에 등록
    (float, x, x) // x 좌표
    (float, y, y) // y 좌표
    (float, z, z) // z 좌표
    (uint16_t, intensity, intensity) // 강도 값
    (uint16_t, reflectivity, reflectivity) // 반사율 값
    (uint16_t, noise, noise) // 잡음 값
)


// #endif // PCL_POINT_TYPES_H_ // PCL_POINT_TYPES_H_ 헤더 파일의 종료
