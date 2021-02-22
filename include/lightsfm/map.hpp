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
