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
		enum class Direction { HORIZONTAL, VERTICAL };

		AngularRange() {}
		AngularRange(const Angle& start_angle, const Angle& end_angle, const Angle& step, const std::vector<float>& correction_table = std::vector<float>()) {
			_start_angle = start_angle;
			_end_angle = end_angle;
			_step = step;
			_num_beams = std::round(std::fabs((_end_angle - _start_angle) / _step));
			_span = Angle::abs(end_angle - start_angle);
			if (correction_table.size()) {
				setAngleCorrectionTable(correction_table);
			}
		}
		AngularRange(const Angle& start_angle, const Angle& end_angle, int num_beams, const std::vector<float>& correction_table = std::vector<float>()) {
			_start_angle = start_angle;
			_end_angle = end_angle;
			_num_beams = num_beams;
			_step = Angle::abs((_end_angle - _start_angle) / _num_beams);
			_span = Angle::abs(end_angle - start_angle);
			if (correction_table.size()) {
				setAngleCorrectionTable(correction_table);
			}
		}
		AngularRange(const float start_angle, const float end_angle, const float step, const std::vector<float>& correction_table = std::vector<float>()) {
			AngularRange(Angle(Angle::IsAngle{}, start_angle), Angle(Angle::IsAngle{}, end_angle), Angle(Angle::IsAngle{}, step), correction_table);
			// _start_angle = Angle(Angle::IsAngle{}, start_angle);
			// _end_angle = Angle(Angle::IsAngle{}, end_angle);
			// _step = Angle(Angle::IsAngle{}, step);
			// _num_beams = std::round(std::fabs((_end_angle - _start_angle) / _step));
			// _span = Angle::abs(_end_angle - _start_angle);
		}
		AngularRange(const float start_angle, const float end_angle, int num_beams, const std::vector<float>& correction_table = std::vector<float>()) {
			AngularRange(Angle(Angle::IsAngle{}, start_angle), Angle(Angle::IsAngle{}, end_angle), num_beams, correction_table);
			// _start_angle = Angle(Angle::IsAngle{}, start_angle);
			// _end_angle = Angle(Angle::IsAngle{}, end_angle);
			// _num_beams = num_beams;
			// _step = Angle::abs((_end_angle - _start_angle) / _num_beams);
			// _span = Angle::abs(_end_angle - _start_angle);
		}

		AngularRange(const AngularRange& other)
		{
			*this = other;
		}

		const Angle& start_angle() const { return _start_angle; }
		const Angle& end_angle() const { return _end_angle; }
		const Angle& step() const { return _step; }
		const Angle& span() const { return _span; }
		int num_beams() const { return _num_beams; }

		bool valid() const { return _num_beams > 0 && _span > 0_deg; }

		const std::vector<Angle>& getAngleCorrectionTable() const { return _angle_correction_table; }
		const std::vector<float> getAngleCorrectionTableF() const 
		{ 
			std::vector<float> correction_table;
			correction_table.clear();
			for (auto& it : _angle_correction_table) {
				correction_table.push_back(it.toDegrees());
			}
			return correction_table; 
		}
		void setAngleCorrectionTable(const std::vector<Angle>& correction_table) { _angle_correction_table = correction_table; }
		void setAngleCorrectionTable(const std::vector<float>& correction_table) 
		{ 
			_angle_correction_table.clear();
			for (auto& it : correction_table) {
				_angle_correction_table.push_back(Angle::fromDegrees(it));
			}
		}

		AngularRange& operator = (const AngularRange& other)
		{
			if (this != &other) {
				_start_angle = other._start_angle;
				_end_angle = other._end_angle;
				_step = other._step;
				_span = other._span;
				_num_beams = other._num_beams;
				_angle_correction_table = other._angle_correction_table;
			}
			return *this;
		}

		friend YAML::Emitter& operator <<(YAML::Emitter& out, const AngularRange& val)
		{
			out << YAML::BeginMap;
			out << YAML::Key << "start_angle";
			out << YAML::Value << val.start_angle().toDegrees();
			out << YAML::Key << "end_angle";
			out << YAML::Value << val.end_angle().toDegrees();
			out << YAML::Key << "step";
			std::cout << "Emit angle table of size: " << val.getAngleCorrectionTable().size() << std::endl;
			out << YAML::Value  << val.step().toDegrees();
			if (val.getAngleCorrectionTable().size()) {
				out << YAML::Key << "angle_correction_table";
				out << YAML::Value << YAML::Flow << val.getAngleCorrectionTableF();
			}
			out << YAML::EndMap;
			return out;
		}

		friend std::ostream& operator <<(std::ostream& out, const AngularRange& val)
		{
			out << "start_angle: ";
			out << val.start_angle() << std::endl;
			out << "end_angle: ";
			out << val.end_angle() << std::endl;
			out << "step: ";
			out << val.step() << std::endl;
			out << "beams: ";
			out << val.num_beams() << std::endl;
			out << "span: ";
			out << val.span() << std::endl;
			if (val.getAngleCorrectionTable().size()) {
				out << "angle_correction_table: [";
				for (auto it = val.getAngleCorrectionTable().begin(); it != val.getAngleCorrectionTable().end(); ++it) {
					out << it->toDegrees();
					if (it < val.getAngleCorrectionTable().end()) {
						out << ",";
					}
				}
				out << "]" << std::endl;
			}
			return out;
		}

	private:
		Angle _start_angle = 0_deg;
		Angle _end_angle = 0_deg;
		Angle _step = 0_deg;
		Angle _span = 0_deg;
		int _num_beams = 0;
		std::vector<Angle> _angle_correction_table;
};


}  // namespace cloud_to_image

constexpr cloud_to_image::Angle operator"" _rad(long double Angle) 
{
  return cloud_to_image::Angle{cloud_to_image::Angle::IsAngle{}, static_cast<float>(Angle)};
}

constexpr cloud_to_image::Angle operator"" _deg(unsigned long long int Angle) 
{
  return cloud_to_image::Angle{cloud_to_image::Angle::IsAngle{}, static_cast<float>(Angle * M_PI / 180.0)};
}

constexpr cloud_to_image::Angle operator"" _deg(long double Angle) 
{
  return cloud_to_image::Angle{cloud_to_image::Angle::IsAngle{}, static_cast<float>(Angle * M_PI / 180.0)};
}

namespace YAML
{
	template<>
	struct convert<cloud_to_image::Angle> 
	{
		static Node encode(const cloud_to_image::Angle& rhs)
		{
			Node node;
			node = rhs.toDegrees();
			return node;
		}

		static bool decode(const Node& node, cloud_to_image::Angle& rhs)
		{
			rhs = cloud_to_image::Angle::fromDegrees(node.as<float>());
			return true;
		}
	};

	template<>
	struct convert<cloud_to_image::AngularRange> 
	{
		static Node encode(const cloud_to_image::AngularRange& rhs)
		{
			Node node;
			node["start_angle"] = rhs.start_angle().toDegrees();
			node["end_angle"] = rhs.end_angle().toDegrees();
			node["step"] = rhs.step().toDegrees();
			if (rhs.getAngleCorrectionTable().size()) {
				node["angle_correction_table"] = rhs.getAngleCorrectionTable();
			}
			return node;
		}

		static bool decode(const Node& node, cloud_to_image::AngularRange& rhs)
		{
			if (!node.IsMap() || (node.size() != 3 && node.size() != 4)) {
				return false;
			}
			rhs = cloud_to_image::AngularRange(cloud_to_image::Angle::fromDegrees(node["start_angle"].as<float>()), 
				                               cloud_to_image::Angle::fromDegrees(node["end_angle"].as<float>()), 
				                               cloud_to_image::Angle::fromDegrees(node["step"].as<float>()));
			if (node["angle_correction_table"]) {
				rhs.setAngleCorrectionTable(node["angle_correction_table"].as< std::vector<float> >());
			}

			return true;
		}
	};
}

#endif  // ANGLES_H
