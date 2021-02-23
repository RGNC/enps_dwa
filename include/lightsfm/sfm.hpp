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

#ifndef _SFM_HPP_
#define _SFM_HPP_

#include <cmath>
#include <vector>
#include <unordered_map>
#include "vector2d.hpp"
#include "map.hpp"


namespace sfm
{

struct Forces
{
	utils::Vector2d desiredForce;
	utils::Vector2d obstacleForce;
	utils::Vector2d socialForce;
	utils::Vector2d groupGazeForce;
	utils::Vector2d groupCoherenceForce;
	utils::Vector2d groupRepulsionForce;
	utils::Vector2d groupForce;
	utils::Vector2d globalForce;
	utils::Vector2d robotSocialForce;
};

struct Parameters
{
	Parameters()
	: forceFactorDesired(0.1), 
	  forceFactorObstacle(10), 
	  forceSigmaObstacle(0.2),
	  forceFactorSocial(2.1),  
	  forceFactorGroupGaze(3.0),
	  forceFactorGroupCoherence(2.0),
	  forceFactorGroupRepulsion(1.0),
	  lambda(2.0),
	  gamma(0.35),
	  n(2.0),
	  nPrime(3.0),
	  relaxationTime(0.5) {}

	double forceFactorDesired;
	double forceFactorObstacle;
	double forceSigmaObstacle;
	double forceFactorSocial;
	double forceFactorGroupGaze;
	double forceFactorGroupCoherence;
	double forceFactorGroupRepulsion;
	double lambda;
	double gamma;
	double n;
	double nPrime;
	double relaxationTime;
};

struct Goal
{
	utils::Vector2d center;
	double radius;
};


struct Agent
{
	Agent()
	: desiredVelocity(1.2),
	  radius(0.35),
	  cyclicGoals(false),
	  teleoperated(false),
	  antimove(false),
	  linearVelocity(0),
	  angularVelocity(0),
	  groupId(-1) {}

	Agent(double linearVelocity, double angularVelocity)
	: desiredVelocity(1.2),
	  radius(0.35),
	  cyclicGoals(false),
	  teleoperated(true),
          antimove(false),
	  linearVelocity(linearVelocity),
	  angularVelocity(angularVelocity),
	  groupId(-1) {}

	Agent(const utils::Vector2d& position, const utils::Angle& yaw, double linearVelocity, double angularVelocity)
	: position(position),
	  yaw(yaw),
      desiredVelocity(1.2),
	  radius(0.35),
	  cyclicGoals(false),
	  teleoperated(true),
          antimove(false),
	  linearVelocity(linearVelocity),
	  angularVelocity(angularVelocity),
	  groupId(-1) {}


	void move(double dt); // only if teleoperated

	utils::Vector2d position; 
	utils::Vector2d velocity; 
	utils::Angle yaw; 
	
	utils::Vector2d movement;	

	double desiredVelocity;
	double radius; 

	std::list<Goal> goals; 
	bool cyclicGoals; 	

	bool teleoperated; 
	bool antimove;
	double linearVelocity; 
	double angularVelocity;	

	int groupId;

	Forces forces;
	Parameters params;
	std::vector<utils::Vector2d> obstacles;
	
};

struct Group
{
	utils::Vector2d center;
	std::vector<unsigned> agents;
};


class SocialForceModel
{
public:
	SocialForceModel(SocialForceModel const&) = delete;
        void operator=(SocialForceModel const&)  = delete;
	~SocialForceModel() {}

	static SocialForceModel& getInstance()
   	{
      		static SocialForceModel singleton;
      		return singleton;
	}

	#define SFM SocialForceModel::getInstance()

	std::vector<Agent>& computeForces(std::vector<Agent>& agents, Map* map=NULL) const;
	std::vector<Agent>& updatePosition(std::vector<Agent>& agents, double dt) const;
	


private:
	#define PW(x) ((x)*(x))
	SocialForceModel() {}
	utils::Vector2d computeDesiredForce(Agent& agent) const;
	void computeObstacleForce(Agent& agent, Map* map) const;
	void computeSocialForce(unsigned index, std::vector<Agent>& agents) const;
	void computeGroupForce(unsigned index, const utils::Vector2d& desiredDirection, std::vector<Agent>& agents, const std::unordered_map<int,Group>& groups) const;
	

};

inline
utils::Vector2d SocialForceModel::computeDesiredForce(Agent& agent) const
{
	utils::Vector2d desiredDirection;
	if (!agent.goals.empty() && (agent.goals.front().center - agent.position).norm()>agent.goals.front().radius) {
		utils::Vector2d diff = agent.goals.front().center - agent.position;
		desiredDirection = diff.normalized();
		agent.forces.desiredForce = agent.params.forceFactorDesired * (desiredDirection * agent.desiredVelocity  - agent.velocity)/agent.params.relaxationTime;
		agent.antimove=false;
	} else {
		agent.forces.desiredForce = -agent.velocity / agent.params.relaxationTime;
		agent.antimove=true;
	}
	return desiredDirection;
}


inline
void SocialForceModel::computeObstacleForce(Agent& agent, Map* map) const
{
	if (agent.obstacles.size()>0) {
		agent.forces.obstacleForce.set(0,0);
		for (unsigned i = 0; i< agent.obstacles.size(); i++) {
			utils::Vector2d diff = agent.position - agent.obstacles[i];
			double distance = diff.norm() ;
			agent.forces.obstacleForce += agent.params.forceFactorObstacle * std::exp(-distance/agent.params.forceSigmaObstacle) * diff.normalized();		
		}
		agent.forces.obstacleForce /= (double)(agent.obstacles.size());
	} else if (map != NULL) {
		const Map::Obstacle& obs = map->getNearestObstacle(agent.position);	
		utils::Vector2d minDiff = agent.position - obs.position;
		double distance = minDiff.norm() - agent.radius;
		agent.forces.obstacleForce = agent.params.forceFactorObstacle * std::exp(-distance/agent.params.forceSigmaObstacle) * minDiff.normalized();
	} else {
		agent.forces.obstacleForce.set(0,0);
	}
}

inline
void SocialForceModel::computeSocialForce(unsigned index, std::vector<Agent>& agents) const
{
	Agent& agent = agents[index];
	agent.forces.socialForce.set(0,0);
	
	for (unsigned i = 0; i< agents.size(); i++) {
		if (i == index ) {
			continue;
		}

		utils::Vector2d diff = agents[i].position - agent.position;
		utils::Vector2d diffDirection = diff.normalized();
		utils::Vector2d velDiff = agent.velocity - agents[i].velocity;
		utils::Vector2d interactionVector = agent.params.lambda * velDiff + diffDirection;
		
		double interactionLength = interactionVector.norm();
		utils::Vector2d interactionDirection = interactionVector/interactionLength;
		
		utils::Angle theta = interactionDirection.angleTo(diffDirection);
		double B = agent.params.gamma * interactionLength;
		
		double thetaRad = theta.toRadian();	
		
		double forceVelocityAmount = -std::exp(-diff.norm()/B - PW(agent.params.nPrime*B*thetaRad) );
		double forceAngleAmount = -theta.sign() * 
		std::exp(-diff.norm() / B - PW(agent.params.n * B * thetaRad));
		utils::Vector2d forceVelocity = forceVelocityAmount * interactionDirection;
		utils::Vector2d forceAngle = forceAngleAmount * interactionDirection.leftNormalVector();
		agent.forces.socialForce += agent.params.forceFactorSocial * (forceVelocity + forceAngle);
		if (i==0) {
			agent.forces.robotSocialForce = agent.params.forceFactorSocial * (forceVelocity + forceAngle);
		}
		


	}
	
}


inline
void SocialForceModel::computeGroupForce(unsigned index, const utils::Vector2d& desiredDirection, std::vector<Agent>& agents, const std::unordered_map<int,Group>& groups) const
{
	Agent& agent = agents[index];
	agent.forces.groupForce.set(0,0);
	agent.forces.groupGazeForce.set(0,0);
	agent.forces.groupCoherenceForce.set(0,0);
	agent.forces.groupRepulsionForce.set(0,0);
	if (groups.count(agent.groupId)==0 || groups.at(agent.groupId).agents.size()<2) {
		return;
	}
	const Group& group = groups.at(agent.groupId);
	
	// Gaze force
	utils::Vector2d com = group.center;
	com = (1 / (double)(group.agents.size()-1)) * (group.agents.size() * com - agent.position);

	utils::Vector2d relativeCom = com - agent.position;
	utils::Angle visionAngle = utils::Angle::fromDegree(90);
	double elementProduct = desiredDirection.dot(relativeCom);
	utils::Angle comAngle = utils::Angle::fromRadian(std::acos(elementProduct / (desiredDirection.norm() * relativeCom.norm())));
	if (comAngle > visionAngle) {
		#ifdef _PAPER_VERSION_
		utils::Angle necessaryRotation = comAngle - visionAngle;
		agent.forces.groupGazeForce = -necessaryRotation.toRadian() * desiredDirection;		
		#else
		double desiredDirectionSquared = desiredDirection.squaredNorm();
		double desiredDirectionDistance = elementProduct / desiredDirectionSquared;
		agent.forces.groupGazeForce = desiredDirectionDistance * desiredDirection;
		#endif
		agent.forces.groupGazeForce *= agent.params.forceFactorGroupGaze;
	}

	// Coherence force
	com = group.center;
	relativeCom = com - agent.position;
	double distance = relativeCom.norm();
	double maxDistance = ((double)group.agents.size()-1)/2;
	#ifdef _PAPER_VERSION_
	if (distance >= maxDistance) {
		agent.forces.groupCoherenceForce = relativeCom.normalized();
		agent.forces.groupCoherenceForce *= agent.params.forceFactorGroupCoherence;
	} 
	#else
	agent.forces.groupCoherenceForce = relativeCom;
	double softenedFactor = agent.params.forceFactorGroupCoherence * (std::tanh(distance - maxDistance) + 1) / 2;
	agent.forces.groupCoherenceForce *= softenedFactor;
	#endif

	// Repulsion Force
	for (unsigned i = 0; i< group.agents.size(); i++) {
		if (index == group.agents[i]) {
			continue;
		}
		utils::Vector2d diff = agent.position - agents.at(group.agents[i]).position;
		if (diff.norm() < agent.radius + agents.at(group.agents[i]).radius) {
			agent.forces.groupRepulsionForce += diff;
		}
	}
	agent.forces.groupRepulsionForce *= agent.params.forceFactorGroupRepulsion;

	// Group Force
	agent.forces.groupForce = agent.forces.groupGazeForce + agent.forces.groupCoherenceForce + agent.forces.groupRepulsionForce;

}


inline
std::vector<Agent>& SocialForceModel::computeForces(std::vector<Agent>& agents, Map* map) const
{
	std::unordered_map<int,Group> groups;
	for (unsigned i = 0; i< agents.size(); i++) {
		if (agents[i].groupId<0) {
			continue;
		}
		groups[agents[i].groupId].agents.push_back(i);
		groups[agents[i].groupId].center += agents[i].position;
	}
	for (auto it = groups.begin(); it!= groups.end(); ++it) {
		it->second.center /= (double) (it->second.agents.size());
	}

	for (unsigned i = 0; i< agents.size(); i++) {
		utils::Vector2d desiredDirection = computeDesiredForce(agents[i]);
		computeObstacleForce(agents[i],map);
		computeSocialForce(i,agents);
		computeGroupForce(i,desiredDirection,agents,groups);
		agents[i].forces.globalForce = agents[i].forces.desiredForce + agents[i].forces.socialForce + agents[i].forces.obstacleForce + agents[i].forces.groupForce;		
	}
	return agents;
}


inline
void Agent::move(double dt)
{
	double imd = linearVelocity * dt;
	utils::Vector2d inc(imd * std::cos(yaw.toRadian() + angularVelocity*dt*0.5), imd * std::sin(yaw.toRadian() + angularVelocity*dt*0.5));
	yaw += utils::Angle::fromRadian(angularVelocity * dt);	
	position += inc;
	velocity.set(linearVelocity * yaw.cos(), linearVelocity * yaw.sin());
}


inline
std::vector<Agent>& SocialForceModel::updatePosition(std::vector<Agent>& agents, double dt) const
{
	for (unsigned i=0; i< agents.size(); i++) {
		utils::Vector2d initPos = agents[i].position;
		if (agents[i].teleoperated) {
			double imd = agents[i].linearVelocity * dt;
			utils::Vector2d inc(imd * std::cos(agents[i].yaw.toRadian() + agents[i].angularVelocity*dt*0.5), imd * std::sin(agents[i].yaw.toRadian() + agents[i].angularVelocity*dt*0.5));
			agents[i].yaw += utils::Angle::fromRadian(agents[i].angularVelocity * dt);	
			agents[i].position += inc;
			agents[i].velocity.set(agents[i].linearVelocity * agents[i].yaw.cos(), agents[i].linearVelocity * agents[i].yaw.sin());
		} else {
			agents[i].velocity += agents[i].forces.globalForce * dt;
			if (agents[i].velocity.norm() > agents[i].desiredVelocity) {
				agents[i].velocity.normalize();
				agents[i].velocity *= agents[i].desiredVelocity;
			}
			agents[i].yaw = agents[i].velocity.angle();
			agents[i].position += agents[i].velocity * dt;
		}
		agents[i].movement = agents[i].position - initPos;
		if (!agents[i].goals.empty() && (agents[i].goals.front().center - agents[i].position).norm()<=agents[i].goals.front().radius) {
			Goal g = agents[i].goals.front();
			agents[i].goals.pop_front();
			if (agents[i].cyclicGoals) {
				agents[i].goals.push_back(g);
			}
		}
		
	}
	return agents;
}


}
#endif
