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

#ifndef _MAP_HPP_
#define _MAP_HPP_


#include "vector2d.hpp"

namespace sfm
{

class Map
{
public:

	struct Obstacle
	{
		Obstacle() : distance(-1) {}
		double distance;
		utils::Vector2d position;
	};

	Map() {}
	virtual ~Map() {}
	virtual const Obstacle& getNearestObstacle(const utils::Vector2d& x) = 0;
	virtual bool isObstacle(const utils::Vector2d& x) const = 0;
};

}

#endif
