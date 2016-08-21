/*
 * MengeActorPlugin.cpp
 *
 *  Created on: Jul 2, 2016
 *      Author: Michael Huang
 */

#include "MengeActorPlugin.h"
// STL
#include <iostream>
#include <algorithm>
#include <string>

#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
using namespace Menge;

using namespace gazebo::physics;

namespace gazebo {


void MengeActorPlugin::insertAgentActor(const Agents::BaseAgent* agt)
{
	std::stringstream sstm;
	sstm << "MengeAgent_clone_" <<agt->_id;
	std::string uniq_name=sstm.str();
 	std::string skinFile("walk.dae");
  	std::string animFile("walk.dae");
  	std::string animName("walking");
 	 std::ostringstream actorStr;
 	 actorStr << "<sdf version='1.6'>"
	    << "<actor name ='" << uniq_name << "'>"
	    << "  <skin>"
	    << "    <filename>" << skinFile << "</filename>"	
	    << "  </skin>"
	    << "  <animation name='" << animName << "'>"
	    << "    <filename>" << animFile << "</filename>"
	    << "<interpolate_x>true</interpolate_x>"
	    //<<"<animation_factor>1<animation_factor>"
	    << "<scale>1</scale>"
	    << "  </animation>"
	    << "</actor>"
	    << "</sdf>";	
	msgs::Factory msg;
  	msg.set_sdf(actorStr.str());
	std::cout<<"before insert"<<std::endl;
  	factoryPub->Publish(msg);
	std::cout<<"after insert"<<std::endl;
	
}

void MengeActorPlugin::insertAgentModel(const Agents::BaseAgent* agt)
{
	

}
// for animated actors we need update the velocity, let the GPU animation to move the agent
// We also need resync the position between gz and menge periodically

bool MengeActorPlugin::updateAgent(const Agents::BaseAgent* agt)
{
//current position in rendering:
//     virtual const AxisAlignedBox& Ogre::MovableObject::getWorldBoundingBox 	( 	bool  	derive = false	) 	const
// position to move to: agt->_pos
// orientation agt->_orient
// velocity: agt->_vel
// math::Pose 	GetPose () const
// 	Get the pose of the visual. More...
 
//math::Vector3 	GetPosition () const
 //	Get the position of the visual. More...

	return true;
	
}

} /* namespace gazebo */
