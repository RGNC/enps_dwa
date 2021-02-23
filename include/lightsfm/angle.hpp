/*
    This file is part of ENPS_DWA.
    
    https://github.com/RGNC/enps_dwa
    ENPS_DWA is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    RENPSM is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with RENPSM.  If not, see <http://www.gnu.org/licenses/>.

	This file is based on the LIGHTSFM library by the Service Robotics
	Lab, Pablo de Olavide University

	https://github.com/robotics-upo/lightsfm
*/


#ifndef _ANGLE_HPP_
#define _ANGLE_HPP_

#include <iostream>
#include <cmath>

namespace utils
{
class Angle
{
public:
	enum AngleRange{
		// [0, 2*pi) or [0°, 360°)
		PositiveOnlyRange,
		// (-pi, +pi] or (-180°, 180°]
		PositiveNegativeRange
	};


	Angle() : value(0) {}
	virtual ~Angle() {}

	static Angle fromRadian(double value) {
		return Angle(value);
	}

	static Angle fromDegree(double value) {
		return Angle(value / 180 * M_PI);
	}

	double toRadian(AngleRange range = PositiveNegativeRange) const {
		if(range == PositiveNegativeRange) {
			return value;
		} else {
			return (value>=0) ? value : (value+2*M_PI);
		}
	}

	double cos() const {
		return std::cos(value);
	}

	double sin() const {
		return std::sin(value);
	}

	double toDegree(AngleRange range = PositiveNegativeRange) const {
		double degreeValue = value * 180 / M_PI;
		if(range == PositiveNegativeRange) {
			return degreeValue;
		} else {
			return (degreeValue>=0) ? degreeValue : (degreeValue+360);
		}
	}

	void setRadian(double value) { 
		Angle::value = value; 
		normalize();
	}

	void setDegree(double value) {
		Angle::value = value / 180 * M_PI;
		normalize();
	}

	int sign() const {
		if(value == 0) {
			return 0;
		} else if(value > 0) {
			return 1;
		} else {
			return -1;
		}
	}
	
	Angle operator+(const Angle& other) const {
		return Angle(value + other.value);
	}
	
	Angle operator-(const Angle& other) const {
		return Angle(value - other.value);
	}
	
	Angle& operator+=(const Angle& other) {
		value += other.value;
		normalize();
		return *this;
	}
	
	Angle& operator-=(const Angle& other) {
		value -= other.value;
		normalize();
		return *this;
	}

	bool operator==(const Angle& other) const {
		return value == other.value;
	}

	bool operator!=(const Angle& other) const {
		return value != other.value;
	}

	bool operator<(const Angle& other) const {
		return value < other.value;
	}

	bool operator<=(const Angle& other) const {
		return value <= other.value;
	}

	bool operator>(const Angle& other) const {
		return value > other.value;
	}

	bool operator>=(const Angle& other) const {
		return value >= other.value;
	}


private:
	Angle(double value) {
		Angle::value = value;
		normalize();
	}

	void normalize() { 
		while(value <= -M_PI) 
			value += 2*M_PI; 
		while(value > M_PI) 
			value -= 2*M_PI; 
	}

	double value;

};

}
namespace std
{
inline
ostream& operator<<(ostream& stream, const utils::Angle& alpha)
{
	stream<<alpha.toRadian();
	return stream;
}

}

#endif
