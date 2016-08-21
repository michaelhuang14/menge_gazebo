/*
 * MengePlugin.cpp
 *
 *  Created on: Jul 2, 2016
 *      Author: Michael Huang
 */

#include "MengeDefaultPlugin.h"
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


void MengeDefaultPlugin::insertAgentActor(const Agents::BaseAgent* agt)
{
	std::stringstream sstm;
	sstm << "MengeAgent_clone_" <<agt->_id;
	std::string uniq_name=sstm.str();
 	std::string skinFile("walk.dae");
  	std::string animFile("walk.dae");
  	std::string animName("walking");
 	 std::ostringstream actorStr;
 	 actorStr << "<sdf version='1.5'>"
	    << "<actor name ='" << uniq_name << "'>"
	
	    << "  <skin>"
	    << "    <filename>" << skinFile << "</filename>"
	
	    << "  </skin>"
	    << "  <animation name='" << animName << "'>"
	    << "    <filename>" << animFile << "</filename>"
	
	    << "  </animation>"
	    << "</actor>"
	    << "</sdf>";	
	msgs::Factory msg;
  	msg.set_sdf(actorStr.str());
	std::cout<<"before insert"<<std::endl;
  	factoryPub->Publish(msg);
	std::cout<<"after insert"<<std::endl;
	
}

void MengeDefaultPlugin::insertAgentModel(const Agents::BaseAgent* agt)
{
        std::stringstream sstm;
        sstm << "MengeAgent_clone_" <<agt->_id;
        std::string uniq_name=sstm.str();
         std::ostringstream actorStr;
         actorStr << "<sdf version='1.5'>"
            << "<model name ='" << uniq_name << "'>"
            << "<include>\
                     <uri>model://cylinder</uri>\
               </include>\n"<<"</model>"
            << "</sdf>";  
        msgs::Factory msg;
        msg.set_sdf(actorStr.str());
        msgs::Set(msg.mutable_pose(),
          ignition::math::Pose3d(
            ignition::math::Vector3d(agt->_pos.x(), agt->_pos.y(),0.0),
            ignition::math::Quaterniond(0, 0, 0)));
        std::cout<<"before insert"<<std::endl;
        factoryPub->Publish(msg);
        std::cout<<"after insert"<<std::endl;

}



} /* namespace gazebo */
