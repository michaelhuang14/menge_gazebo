/*
 * MengePlugin.cpp
 *
 *  Created on: Jul 2, 2016
 *      Author: Michael Huang
 */

#include "MengeRand.h"
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


void MengeRand::insertAgentActor(const Agents::BaseAgent* agt)
{
	std::stringstream sstm;
	sstm << "MengeAgent_clone_" <<agt->_id;
	std::string uniq_name=sstm.str();
 	std::string skinFile("Gamechar-male.mesh");
  	std::string animFile("Gamechar-male.skeleton");
  	std::string animName("Walk");
 	 std::ostringstream actorStr;
 	 actorStr << "<sdf version='1.6'>"
	    << "<instanced_actor name ='" << uniq_name << "'>"
	<<"<pose>0 0 0 1.5707 0 3.1416</pose>"
	    << "  <skin>"
	    << "    <filename>" << skinFile << "</filename>"
	      <<"  <scale>0.1</scale>"
	    << "  </skin>"
	    << "  <animation name='" << animName << "'>"
	    << "    <filename>" << animFile << "</filename>"
	
	    << "  </animation>"
	    << "</instanced_actor>"
	    << "</sdf>";	
	msgs::Factory msg;
  	msg.set_sdf(actorStr.str());
	std::cout<<"before insert"<<std::endl;
  	factoryPub->Publish(msg);
	std::cout<<"after insert"<<std::endl;
	
}

void MengeRand::insertAgentModel(const Agents::BaseAgent* agt)
{
	if(agt->_id ==260)
	{
		sdf::SDF cylinderSDF;
        std::stringstream ssdf;
	ssdf <<"<sdf version ='1.4'>\
  <model name ='cylinder'>\
    <pose>20 0 0 0 0 0</pose>\
    <static>true</static>\
    <link name ='link'>\
      <pose>0 0 .5 0 0 0</pose>\
      <collision name ='collision'>\
        <geometry>\
          <cylinder><radius>"<<agt->_radius<<"</radius><length>1</length></cylinder>\
        </geometry>\
      </collision>\
      <visual name='visual'>\
        <geometry>\
          <cylinder><radius>"<<agt->_radius<<"</radius><length>1</length></cylinder>\
        </geometry>\
      </visual>\
    </link>\
  </model>\
</sdf>";
	cylinderSDF.SetFromString(ssdf.str());
	
	// Demonstrate using a custom model name.
	sdf::ElementPtr model = cylinderSDF.Root()->GetElement("model");
	std::stringstream sstm;
	sstm << "MengeAgent_clone_" <<agt->_id;
	std::string uniq_name=sstm.str();
	model->GetAttribute("name")->SetFromString(uniq_name);
	std::cout<<"before insert"<<std::endl;
	_world->InsertModelSDF(cylinderSDF);
	std::cout<<"after insert"<<std::endl;
	}
	else{
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

}


} /* namespace gazebo */
