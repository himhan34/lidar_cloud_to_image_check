// ANGLES_H가 정의되지 않았다면
#ifndef ANGLES_H  
// ANGLES_H를 정의한다
#define ANGLES_H  

// 수학 계산을 위한 표준 라이브러리 포함
#include <cmath>  
// 표준 입출력을 위한 라이브러리 포함
#include <stdio.h>  
// 데이터 타입의 한계값들을 포함하는 라이브러리 포함
#include <limits>  
// 표준 입력 및 출력 스트림 객체를 위한 라이브러리 포함
#include <iostream>  

// YAML 파싱을 위한 yaml-cpp 라이브러리 포함
#include <yaml-cpp/yaml.h>  

// This work was inspired on radians.h from I. Bogoslavskyi, C. Stachniss, University of Bonn 
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
//
// This class provides cooked literals for Angles, following on the description in:
// https://akrzemi1.wordpress.com/2012/08/12/user-defined-literals-part-i/


// 'cloud_to_image' 네임스페이스 정의
namespace cloud_to_image 
{
  // 'Angle' 클래스 선언
  class Angle;
}

// 리터럴 연산자의 전방 선언
// long double 타입의 값을 받아 'cloud_to_image::Angle' 타입으로 변환하는 리터럴 연산자 '_rad'
constexpr cloud_to_image::Angle operator"" _rad(long double Angle);

// unsigned long long int 타입의 값을 받아 'cloud_to_image::Angle' 타입으로 변환하는 리터럴 연산자 '_deg'
constexpr cloud_to_image::Angle operator"" _deg(
    unsigned long long int Angle);

// long double 타입의 값을 받아 'cloud_to_image::Angle' 타입으로 변환하는 리터럴 연산자 '_deg'
constexpr cloud_to_image::Angle operator"" _deg(long double Angle);

// 'cloud_to_image' 네임스페이스 정의
namespace cloud_to_image
{
    // 'Angle' 클래스 정의
    class Angle 
    {
        // IsAngle 클래스 정의: 원시 생성자(raw constructor) 사용을 방지하기 위한 태그 클래스
        public:
            class IsAngle {};  

        // 기본 생성자: 각도를 0으로 설정하고 유효하지 않은 상태로 초기화
        Angle() : _raw_angle{0}, _valid{false} {}

        // 명시적 생성자: IsAngle 태그와 각도 값을 받아 초기화
        explicit constexpr Angle(IsAngle, float angle)
            : _raw_angle{angle}, _valid{true} {}

        // 각도 값을 반환하는 함수
        inline float val() const { return _raw_angle; }
        // 유효성을 반환하는 함수
        inline bool valid() const { return _valid; }


	        // 라디안 단위의 각도로부터 Angle 객체를 생성하는 정적 메소드
        static Angle fromRadians(float radians) 
        {
            return Angle{IsAngle{}, radians};
        }

        // 도 단위의 각도로부터 Angle 객체를 생성하는 정적 메소드
        static Angle fromDegrees(float degrees) 
        {
            return Angle{IsAngle{}, static_cast<float>(degrees * M_PI / 180.)};
        }

        // 객체가 나타내는 각도를 라디안 단위로 반환하는 메소드
        inline float toRadians() const { return _raw_angle; }

        // 객체가 나타내는 각도를 도 단위로 반환하는 메소드
        inline float toDegrees() const { return 180. * _raw_angle / M_PI; }

        // 두 Angle 객체 간의 차를 계산하는 연산자 오버로딩
        Angle operator-(const Angle& other) const 
        {
            return fromRadians(_raw_angle - other._raw_angle);
        }

	        // 두 Angle 객체를 더하는 연산자 오버로딩
        Angle operator+(const Angle& other) const 
        {
            return fromRadians(_raw_angle + other._raw_angle);
        }

        // 현재 Angle 객체에 다른 Angle 객체를 더하는 연산자 오버로딩
        Angle operator+=(const Angle& other) 
        {
            this->_raw_angle += other._raw_angle;
            return *this;
        }

        // 현재 Angle 객체에서 다른 Angle 객체를 빼는 연산자 오버로딩
        Angle operator-=(const Angle& other) 
        {
            this->_raw_angle -= other._raw_angle;
            return *this;
        }

        // 현재 Angle 객체를 숫자로 나누는 연산자 오버로딩
        Angle operator/(const float& num) const 
        {
            return fromRadians(_raw_angle / num);
        }

        // 현재 Angle 객체를 다른 Angle 객체로 나누어 결과를 float 값으로 반환하는 연산자 오버로딩
        float operator/(const Angle& other) const 
        {
            return _raw_angle / other._raw_angle;
        }

        // 현재 Angle 객체에 숫자를 곱하는 연산자 오버로딩
        Angle operator*(const float& num) const 
        {
            return fromRadians(_raw_angle * num);
        }

        // 현재 Angle 객체의 부호를 반전시키는 연산자 오버로딩
        Angle operator-() { return fromRadians(-_raw_angle); }

        // 현재 Angle 객체가 다른 Angle 객체보다 작은지 비교하는 연산자 오버로딩
        bool operator<(const Angle& other) const 
        {
            return _raw_angle < other._raw_angle - std::numeric_limits<float>::epsilon();
        }

        // 현재 Angle 객체가 다른 Angle 객체보다 큰지 비교하는 연산자 오버로딩
        bool operator>(const Angle& other) const 
        {
            return _raw_angle > other._raw_angle + std::numeric_limits<float>::epsilon();
        }

        // 현재 Angle 객체와 다른 Angle 객체가 동등한지 비교하는 연산자 오버로딩
        bool operator==(const Angle& other) const 
        {
            return std::fabs(_raw_angle - other._raw_angle) <= std::numeric_limits<float>::epsilon();
        }

       // 'from'과 'to' 각도 사이로 현재 객체의 각도를 정규화하는 함수
        void normalize(const Angle& from = 0_deg, const Angle& to = 360_deg) 
        {
            float diff = (to - from).val();
            // 현재 각도가 'from' 미만이면, 'from'과 'to'의 차이만큼 더해 정규화
            while (_raw_angle < from.val()) {
                _raw_angle += diff;
            }
            // 현재 각도가 'to' 초과면, 'from'과 'to'의 차이만큼 빼서 정규화
            while (_raw_angle > to.val()) {
                _raw_angle -= diff;
            }
        }

        // 현재 객체의 복사본을 생성하여 'from'과 'to' 각도 사이로 정규화하는 함수
        Angle normalize(const Angle& from = 0_deg, const Angle& to = 360_deg) const 
        {
            // 현재 각도로부터 새로운 Angle 객체 생성
            Angle new_Angle = fromRadians(_raw_angle);
            // 새로운 객체의 각도를 'from'과 'to' 사이로 정규화
            new_Angle.normalize(from, to);
            // 정규화된 새로운 Angle 객체 반환
            return new_Angle;
        }

        // Angle 객체의 절대 각도를 반환하는 정적 함수
        static Angle abs(const Angle& Angle) 
        {
            return Angle::fromRadians(std::fabs(Angle._raw_angle));
        }

        // Angle 객체의 각도를 내림하여 반환하는 정적 함수
        static Angle floor(const Angle& Angle) 
        {
            return Angle::fromDegrees(std::floor(Angle.toDegrees()));
        }

        // Angle 객체를 표준 출력 스트림에 출력하는 연산자 오버로딩
        friend std::ostream& operator <<(std::ostream& out, const Angle& val)
        {
            out << val.toDegrees() << std::endl;
            return out;
        }

        // Angle 객체를 YAML 에미터에 출력하는 연산자 오버로딩
        friend YAML::Emitter& operator <<(YAML::Emitter& out, const Angle& val)
        {
            out << val.toDegrees();
            return out;
        }

	protected:
	    // 각도 값을 저장하는 실수형 변수
	    float _raw_angle;
	    // 각도의 유효성을 나타내는 불리언 변수
	    bool _valid;
};

class AngularRange 
{
	public:
		/**
		* Enum for the direction of the angular range.
		*/
		// 각도 범위의 방향을 위한 열거형(enum).
		enum class Direction { HORIZONTAL, VERTICAL };

		// 기본 생성자: 아무런 매개변수 없이 객체를 생성합니다.
        AngularRange() {}

        // 매개변수가 있는 생성자: 시작 각도, 종료 각도, 각도 단계, 그리고 선택적으로 보정 테이블을 받아 객체를 초기화합니다.
        AngularRange(const Angle& start_angle, const Angle& end_angle, const Angle& step, const std::vector<float>& correction_table = std::vector<float>()) {
            // 시작 각도를 설정합니다.
            _start_angle = start_angle;
            // 종료 각도를 설정합니다.
            _end_angle = end_angle;
            // 각도 단계를 설정합니다.
            _step = step;
            // 시작 각도와 종료 각도 사이의 빔 수(beam count)를 계산하고 설정합니다.
            _num_beams = std::round(std::fabs((_end_angle - _start_angle) / _step));
            // 시작 각도와 종료 각도 사이의 전체 각도 범위(span)를 계산합니다.
            _span = Angle::abs(end_angle - start_angle);
            // 보정 테이블에 값이 있으면,
            if (correction_table.size()) {
                // 보정 테이블을 설정하는 함수를 호출합니다.
                setAngleCorrectionTable(correction_table);
			}
		}

		// 매개변수가 있는 생성자: 시작 각도, 종료 각도, 빔 수, 그리고 선택적으로 보정 테이블을 받아 객체를 초기화합니다.

        AngularRange(const Angle& start_angle, const Angle& end_angle, int num_beams, const std::vector<float>& correction_table = std::vector<float>()) {
            // 시작 각도를 설정합니다.
            _start_angle = start_angle;
            // 종료 각도를 설정합니다.
            _end_angle = end_angle;
            // 빔 수를 설정합니다.
            _num_beams = num_beams;
            // 각도 단계를 계산하여 설정합니다. 종료 각도와 시작 각도의 차이를 빔 수로 나누어 계산합니다.
            _step = Angle::abs((_end_angle - _start_angle) / _num_beams);
            // 시작 각도와 종료 각도 사이의 전체 각도 범위(span)를 계산합니다.
            _span = Angle::abs(end_angle - start_angle);
            
            // 보정 테이블에 값이 있으면,
            if (correction_table.size()) {
                // 보정 테이블을 설정하는 함수를 호출합니다.
                setAngleCorrectionTable(correction_table);
				}
		}

	// 매개변수가 있는 생성자: 시작 각도, 종료 각도, 각도 단계(모두 float 타입) 및 선택적으로 보정 테이블을 받아 객체를 초기화합니다.
		AngularRange(const float start_angle, const float end_angle, const float step, const std::vector<float>& correction_table = std::vector<float>()) {
		    // 다른 생성자를 호출하여 Angle 타입으로 변환된 매개변수를 사용해 객체를 초기화합니다.
		    AngularRange(Angle(Angle::IsAngle{}, start_angle), Angle(Angle::IsAngle{}, end_angle), Angle(Angle::IsAngle{}, step), correction_table);
			// _start_angle = Angle(Angle::IsAngle{}, start_angle);
			// _end_angle = Angle(Angle::IsAngle{}, end_angle);
			// _step = Angle(Angle::IsAngle{}, step);
			// _num_beams = std::round(std::fabs((_end_angle - _start_angle) / _step));
			// _span = Angle::abs(_end_angle - _start_angle);
		}

	// 매개변수가 있는 생성자: float 타입의 시작 각도, 종료 각도, 빔의 개수 및 선택적으로 보정 테이블을 받아 객체를 초기화합니다.
	        AngularRange(const float start_angle, const float end_angle, int num_beams, const std::vector<float>& correction_table = std::vector<float>()) {
	            // Angle 타입의 매개변수를 받는 다른 생성자를 호출하여 객체를 초기화합니다.
	            AngularRange(Angle(Angle::IsAngle{}, start_angle), Angle(Angle::IsAngle{}, end_angle), num_beams, correction_table);
			// _start_angle = Angle(Angle::IsAngle{}, start_angle);
			// _end_angle = Angle(Angle::IsAngle{}, end_angle);
			// _num_beams = num_beams;
			// _step = Angle::abs((_end_angle - _start_angle) / _num_beams);
			// _span = Angle::abs(_end_angle - _start_angle);
		}


	        // 복사 생성자: 다른 AngularRange 객체의 복사본을 생성합니다.
	        AngularRange(const AngularRange& other)
	        {
	            // 대입 연산자를 사용하여 다른 객체의 모든 멤버를 현재 객체에 복사합니다.
	            *this = other;
	        }

		// 시작 각도를 반환하는 함수
		        const Angle& start_angle() const { return _start_angle; }
		
		        // 종료 각도를 반환하는 함수
		        const Angle& end_angle() const { return _end_angle; }
		
		        // 각도 단계(step)를 반환하는 함수
		        const Angle& step() const { return _step; }
		
		        // 각도 범위(span)를 반환하는 함수
		        const Angle& span() const { return _span; }
		
		        // 빔의 개수를 반환하는 함수
		        int num_beams() const { return _num_beams; }
		
		        // 객체가 유효한지 여부를 반환하는 함수. 빔의 개수가 0보다 크고, 각도 범위가 0도보다 커야 유효함.
		        bool valid() const { return _num_beams > 0 && _span > 0_deg; }

		// 각도 보정 테이블을 Angle 타입의 벡터로 반환하는 함수
		const std::vector<Angle>& getAngleCorrectionTable() const { return _angle_correction_table; }
	
		// 각도 보정 테이블을 float 형태로 반환하는 함수
		const std::vector<float> getAngleCorrectionTableF() const 
		{ 
		    // float 타입의 보정 테이블을 생성합니다.
		    std::vector<float> correction_table;
		    // 테이블의 내용을 초기화합니다.
		    correction_table.clear();
	
		    // _angle_correction_table의 모든 요소에 대해 반복합니다.
		    for (auto& it : _angle_correction_table) {
			// 각 요소를 도(degree) 단위로 변환하여 float 타입의 보정 테이블에 추가합니다.
			correction_table.push_back(it.toDegrees());
		    }
		    // 변환된 보정 테이블을 반환합니다.
		    return correction_table; 
		}

	// Angle 타입의 벡터를 받아 각도 보정 테이블을 설정하는 함수
	void setAngleCorrectionTable(const std::vector<Angle>& correction_table) { 
	    _angle_correction_table = correction_table; 
	}

        // float 타입의 벡터를 받아 각도 보정 테이블을 설정하는 함수
        void setAngleCorrectionTable(const std::vector<float>& correction_table) 
        { 
            // 내부의 보정 테이블을 초기화합니다.
            _angle_correction_table.clear();

            // 주어진 float 벡터의 모든 요소에 대해 반복합니다.
            for (auto& it : correction_table) {
                // 각 float 값을 도(degree) 단위의 Angle 객체로 변환하여 보정 테이블에 추가합니다.
                _angle_correction_table.push_back(Angle::fromDegrees(it));
            }
        }

	
	// 대입 연산자 오버로딩: 다른 AngularRange 객체를 현재 객체에 복사합니다.
	        AngularRange& operator = (const AngularRange& other)
	        {
	            // 자기 자신에 대한 대입을 방지합니다.
	            if (this != &other) {
	                // 다른 객체의 멤버 변수들을 현재 객체의 멤버 변수에 복사합니다.
	                _start_angle = other._start_angle;
	                _end_angle = other._end_angle;
	                _step = other._step;
	                _span = other._span;
	                _num_beams = other._num_beams;
	                _angle_correction_table = other._angle_correction_table;
	            }
	            // 현재 객체의 참조를 반환합니다.
	            return *this;
	        }

	 // YAML 직렬화를 수행하는 친구 함수
		friend YAML::Emitter& operator <<(YAML::Emitter& out, const AngularRange& val)
		{
		    // YAML 맵 시작
		    out << YAML::BeginMap;
	
		    // 시작 각도를 YAML 맵에 추가
		    out << YAML::Key << "start_angle";
		    out << YAML::Value << val.start_angle().toDegrees();
	
		    // 종료 각도를 YAML 맵에 추가
		    out << YAML::Key << "end_angle";
		    out << YAML::Value << val.end_angle().toDegrees();
	
		    // 각도 단계(step)를 YAML 맵에 추가
		    out << YAML::Key << "step";
		    // 현재 처리 중인 각도 보정 테이블의 크기 출력
		    std::cout << "Emit angle table of size: " << val.getAngleCorrectionTable().size() << std::endl;
		    out << YAML::Value << val.step().toDegrees();
	
		    // 각도 보정 테이블이 존재하면 YAML 맵에 추가
		    if (val.getAngleCorrectionTable().size()) {
			out << YAML::Key << "angle_correction_table";
			out << YAML::Value << YAML::Flow << val.getAngleCorrectionTableF();
		    }
	
		    // YAML 맵 종료
		    out << YAML::EndMap;
	
		    // YAML Emitter 참조 반환
		    return out;
		}

	// 출력 스트림 연산자 오버로딩: AngularRange 객체의 데이터를 문자열 형태로 출력합니다.
        friend std::ostream& operator <<(std::ostream& out, const AngularRange& val)
        {
            // 시작 각도를 출력합니다.
            out << "start_angle: ";
            out << val.start_angle() << std::endl;

            // 종료 각도를 출력합니다.
            out << "end_angle: ";
            out << val.end_angle() << std::endl;

            // 각도 단계(step)를 출력합니다.
            out << "step: ";
            out << val.step() << std::endl;

            // 빔의 개수를 출력합니다.
            out << "beams: ";
            out << val.num_beams() << std::endl;

            // 각도 범위(span)를 출력합니다.
            out << "span: ";
            out << val.span() << std::endl;

            // 각도 보정 테이블이 존재하는 경우, 그 내용을 출력합니다.
            if (val.getAngleCorrectionTable().size()) {
                out << "angle_correction_table: [";
                for (auto it = val.getAngleCorrectionTable().begin(); it != val.getAngleCorrectionTable().end(); ++it) {
                    // 각도 보정 테이블의 요소를 도(degree) 단위로 출력합니다.
                    out << it->toDegrees();
                    // 요소 사이에 쉼표를 추가합니다.
                    if (it < val.getAngleCorrectionTable().end() - 1) {
                        out << ", ";
                    }
                }
                out << "]" << std::endl;
            }

            // 스트림 참조를 반환합니다.
            return out;
        }

	private:
        // 시작 각도를 나타내는 멤버 변수. 기본값은 0도입니다.
        Angle _start_angle = 0_deg;

        // 종료 각도를 나타내는 멤버 변수. 기본값은 0도입니다.
        Angle _end_angle = 0_deg;

        // 각도 단계를 나타내는 멤버 변수. 기본값은 0도입니다.
        Angle _step = 0_deg;

        // 각도 범위를 나타내는 멤버 변수. 기본값은 0도입니다.
        Angle _span = 0_deg;

        // 빔의 개수를 나타내는 멤버 변수. 기본값은 0입니다.
        int _num_beams = 0;

        // 각도 보정 테이블을 나타내는 벡터 멤버 변수.
        std::vector<Angle> _angle_correction_table;

        // ...
	};

}  // namespace cloud_to_image

// 라디안 값을 받아 Angle 객체를 생성하는 리터럴 연산자
constexpr cloud_to_image::Angle operator"" _rad(long double Angle) 
{
  // 라디안 값을 float로 변환하여 Angle 객체를 생성합니다.
  return cloud_to_image::Angle{cloud_to_image::Angle::IsAngle{}, static_cast<float>(Angle)};
}

// 정수형 도(degree) 값을 받아 Angle 객체를 생성하는 리터럴 연산자
constexpr cloud_to_image::Angle operator"" _deg(unsigned long long int Angle) 
{
  // 도 값을 라디안으로 변환하고 float로 변환하여 Angle 객체를 생성합니다.
  return cloud_to_image::Angle{cloud_to_image::Angle::IsAngle{}, static_cast<float>(Angle * M_PI / 180.0)};
}

// 부동소수점형 도(degree) 값을 받아 Angle 객체를 생성하는 리터럴 연산자
constexpr cloud_to_image::Angle operator"" _deg(long double Angle) 
{
  // 도 값을 라디안으로 변환하고 float로 변환하여 Angle 객체를 생성합니다.
  return cloud_to_image::Angle{cloud_to_image::Angle::IsAngle{}, static_cast<float>(Angle * M_PI / 180.0)};
}

namespace YAML
{
	// YAML 변환을 위한 템플릿 특수화
	template<>
	struct convert<cloud_to_image::Angle> 
	{
	    // cloud_to_image::Angle 타입을 YAML 노드로 인코딩합니다.
	    static Node encode(const cloud_to_image::Angle& rhs)
	    {
	        Node node;
	        // Angle 객체를 도(degree) 단위로 변환하여 노드에 할당합니다.
	        node = rhs.toDegrees();
	        return node;
	    }
	
	    // YAML 노드를 cloud_to_image::Angle 타입으로 디코딩합니다.
	    static bool decode(const Node& node, cloud_to_image::Angle& rhs)
	    {
	        // 노드에서 float 값을 추출하여 도(degree) 단위의 Angle 객체로 변환합니다.
	        rhs = cloud_to_image::Angle::fromDegrees(node.as<float>());
	        return true;
	    }
	};

	// cloud_to_image::AngularRange 타입에 대한 YAML 변환을 위한 템플릿 특수화
	template<>
	struct convert<cloud_to_image::AngularRange> 
	{
	    // cloud_to_image::AngularRange 타입을 YAML 노드로 인코딩하는 함수
	    static Node encode(const cloud_to_image::AngularRange& rhs)
	    {
	        Node node;
	        // 시작 각도를 YAML 노드에 추가합니다. 각도는 도(degree) 단위로 변환됩니다.
	        node["start_angle"] = rhs.start_angle().toDegrees();
	
	        // 종료 각도를 YAML 노드에 추가합니다. 각도는 도(degree) 단위로 변환됩니다.
	        node["end_angle"] = rhs.end_angle().toDegrees();
	
	        // 각도 단계(step)를 YAML 노드에 추가합니다. 각도는 도(degree) 단위로 변환됩니다.
	        node["step"] = rhs.step().toDegrees();
	
	        // 각도 보정 테이블이 존재하는 경우, 이를 YAML 노드에 추가합니다.
	        if (rhs.getAngleCorrectionTable().size()) {
	            node["angle_correction_table"] = rhs.getAngleCorrectionTable();
	        }
	
	        return node;
	    }

		// YAML 노드에서 cloud_to_image::AngularRange 타입으로 디코딩하는 함수
	    static bool decode(const Node& node, cloud_to_image::AngularRange& rhs)
	    {
	        // YAML 노드가 맵이 아니거나 필요한 요소의 개수와 다른 경우, false를 반환합니다.
	        if (!node.IsMap() || (node.size() != 3 && node.size() != 4)) {
	            return false;
	        }
	
	        // YAML 노드에서 시작 각도, 종료 각도, 각도 단계를 추출하여 AngularRange 객체를 생성합니다.
	        rhs = cloud_to_image::AngularRange(
	            cloud_to_image::Angle::fromDegrees(node["start_angle"].as<float>()), 
	            cloud_to_image::Angle::fromDegrees(node["end_angle"].as<float>()), 
	            cloud_to_image::Angle::fromDegrees(node["step"].as<float>())
	        );
	
	        // 각도 보정 테이블이 존재하는 경우, 이를 추출하여 설정합니다.
	        if (node["angle_correction_table"]) {
	            rhs.setAngleCorrectionTable(node["angle_correction_table"].as<std::vector<float>>());
	        }
	
	        return true;
	    }
	};
	
	}  // namespace cloud_to_image

#endif  // ANGLES_H
