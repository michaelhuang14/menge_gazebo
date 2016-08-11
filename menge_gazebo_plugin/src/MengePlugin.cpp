/*
 * MengePlugin.cpp
 *
 *  Created on: Jul 2, 2016
 *      Author: Michael Huang
 */

#include "MengePlugin.h"
// STL
#include <iostream>
#include <algorithm>
#include <string>
#include <exception>

// UTILS
#include "ProjectSpec.h"
// Command-line parser
#include "tclap/CmdLine.h"
#include "MyViewer.h"
// Menge Runtime
#include "SimSystem.h"
#include "PluginEngine.h"
#include "SimulatorDB.h"
#include "os.h"
#include "Logger.h"
// SceneGraph
#include "GLScene.h"
#include "TextWriter.h"
// Menge Math
#include "RandGenerator.h"
//actor
#include "gazebo/math/Helpers.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/transport/Node.hh"

#include <ignition/math.hh>
#include <ignition/math/Pose3.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
using namespace Menge;

// Time step (gets set by the scene.xml file
float TIME_STEP = 0.01f;
// The number of uniform simulation steps to take between logical time steps
size_t SUB_STEPS = 0;
#define WALKING_ANIMATION "walking"


// Maximum duration of simulation (in seconds)
//		Can be set on command line.
float SIM_DURATION = 800.f;
float simStartTime = 0.0f;
// Controls whether the simulation is verbose or not
bool VERBOSE = false;
bool agentsSynced = false;
float simDelayPerAgent=0.5f;


// The location of the executable - for basic executable resources
std::string ROOT;

SimulatorDB simDB;
using namespace gazebo::physics;

namespace gazebo {

/*!
 *	@brief		Initialize and start the simulation.
 *
 *	@param		dbEntry			The SimulatorDBEntry that describes the simulator
 *								to be instantiated.
 *	@param		behaveFile		The path to a valid behavior specification file.
 *	@param		sceneFile		The path to a valid scene specification file.
 *	@param		outFile			The path to the output file to write.  If it is the
 *								empty string, no output file will be written.
 *	@param		scbVersion		The string indicating the version of scb file to write.
  *	@param		dumpPath		The path to write screen grabs.  Only used in windows.
 *	@returns	0 for a successful run, non-zero otherwise.
 */
int MengePlugin::simMain( SimulatorDBEntry * dbEntry, const std::string & behaveFile, const std::string & sceneFile, const std::string & outFile, const std::string & scbVersion, const std::string & dumpPath ) {
	size_t agentCount;
	if ( outFile != "" ) logger << Logger::INFO_MSG << "Attempting to write scb file: " << outFile << "\n";
	SimSystem* _simSystem = dbEntry->getSimulatorSystem( agentCount, TIME_STEP, SUB_STEPS, SIM_DURATION, behaveFile, sceneFile, outFile, scbVersion, false, VERBOSE );

	if ( _simSystem == 0x0 ) {
		return 1;
	}

	SceneGraph::GLScene * scene = new SceneGraph::GLScene();
	scene->addSystem( _simSystem );
    _simulator=_simSystem->getSimulator();

	logger << Logger::INFO_MSG << "NO VISUALIZATION!\n";
	_mengeView.setScene( scene );
    _mengeView.setFixedStep( TIME_STEP );
	logger.line();
	std::cout << "Simulation time: " << dbEntry->simDuration() << "\n";
	logger << Logger::INFO_MSG << "Simulation time: " << dbEntry->simDuration() << "\n";
	return 0;
}
void MengePlugin::simLoad() {
	logger.setFile("log.html");
	logger << Logger::INFO_MSG << "initialized logger";
// !TODO: quick and dirty hacks here. need better design
	ROOT = getenv("MENGE_EXE"); //User sets MENGE_HOME env var to Menge Exe dir
	static const int argc = 3;
	static char* argv[3];
	argv[0] = "libmengeSim.so";
	argv[1] = "-p";
	argv[2] = getenv("MENGE_PROJECT_PATH"); //User sets MENGE_PROJECT_PATH env var to menge project file to be loaded

	PluginEngine plugins(&simDB);
	std::string pluginPath = os::path::join(2, ROOT.c_str(), "plugins");
	logger.line();
	logger << Logger::INFO_MSG << "Plugin path: " << pluginPath;
	plugins.loadPlugins(pluginPath);
	if (simDB.modelCount() == 0) {
		logger << Logger::INFO_MSG
				<< "There were no pedestrian models in the plugins folder\n";
		return;
	}

	SceneGraph::TextWriter::setDefaultFont(
			os::path::join(2, ROOT.c_str(), "arial.ttf"));
	ProjectSpec projSpec;

	if (!projSpec.parseCommandParameters(argc, argv, &simDB)) {
		return;
	}

	if (!projSpec.fullySpecified()) {
		return;
	}

	VERBOSE = projSpec.getVerbosity();
	TIME_STEP = projSpec.getTimeStep();
	SUB_STEPS = projSpec.getSubSteps();
	SIM_DURATION = projSpec.getDuration();
	std::string dumpPath = projSpec.getDumpPath();
	setDefaultGeneratorSeed(projSpec.getRandomSeed());
	std::string outFile = projSpec.getOutputName();

	std::string model(projSpec.getModel());

	SimulatorDBEntry * simDBEntry = simDB.getDBEntry(model);
	if (simDBEntry == 0x0) {
		std::cerr << "!!!  The specified model is not recognized: " << model
				<< "\n";
		logger.close();
		return ;
	}

	int result = simMain(simDBEntry, projSpec.getBehavior(),
			projSpec.getScene(), projSpec.getOutputName(),
			projSpec.getSCBVersion(), dumpPath);

	if (result) {
		std::cerr
				<< "Simulation terminated through error.  See error log for details.\n";
	}
	logger.close();
}
void MengePlugin::insertAgentActor(const Agents::BaseAgent* agt)
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

void MengePlugin::insertAgentModel(const Agents::BaseAgent* agt)
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

void MengePlugin::insertAgents()
{
	size_t numAgt=_simulator->getNumAgents();
	for(size_t i=0;i<numAgt;i++)
	{
#ifdef USE_ACTOR
             insertAgentActor(_simulator->getAgent(i));
#else
             insertAgentModel(_simulator->getAgent(i));
#endif
	}
}
void MengePlugin::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf) {
	 this->_world = _parent;
	simLoad();
 

  this->connections.push_back(event::Events::ConnectWorldUpdateBegin(
          std::bind(&MengePlugin::OnUpdate, this, std::placeholders::_1)));

	transport::NodePtr _node(new transport::Node());
	this -> node = _node;
	node->Init(this->_world->GetName());
	this ->factoryPub = node->Advertise<msgs::Factory>("~/factory");
	
	insertAgents();
        _mengeView.start();	
}
void MengePlugin::controlActor(physics::ActorPtr _actor )
{
      std::cout<<"*** we got you first time!\n";
  auto skelAnims = _actor->SkeletonAnimations();
  if (skelAnims.find(WALKING_ANIMATION) == skelAnims.end())
  {
    gzerr << "Skeleton animation " << WALKING_ANIMATION << " not found.\n";
  }
  {
      std::cout<<"*** Insert trajectory for"<<_actor->GetName()<<"!\n";
    // Create custom trajectory

    this->trajectoryInfo.reset(new physics::TrajectoryInfo());
    this->trajectoryInfo->type = WALKING_ANIMATION;
    this->trajectoryInfo->duration = 1.0;

    _actor->SetCustomTrajectory(this->trajectoryInfo);
  }
}
bool MengePlugin::updateAgent(const Agents::BaseAgent* agt)
{
  std::stringstream sstm;
  sstm << "MengeAgent_clone_" <<agt->_id;
  std::string uniq_name=sstm.str();
  ModelPtr mdl=_world->GetModel(uniq_name);	
#ifdef USE_ACTOR
  auto _actor = boost::dynamic_pointer_cast<physics::Actor>(mdl);
  
  if(_actor==nullptr) {
    std::cout<<"did not get actor\n";  
    return false;
  }
#else

  if(mdl==nullptr) {
    std::cout<<"did not get actor\n";
    return false;
  }
#endif 
  math::Pose pose;

  // Compute the yaw orientation
  ignition::math::Angle yaw = atan2(agt->_orient.y(),agt->_orient.x()); 
  yaw.Normalize();
  
  
  // Distance traveled is used to coordinate motion with the walking
  // animation
 // double distanceTraveled = (pose.Pos() -
 //     actor->GetWorldPose().Ign().Pos()).Length();
  pose.Set(agt->_pos.x(), agt->_pos.y(),0.00,0,0,yaw.Radian());
 
  //std::cout<<"setting position for "<< uniq_name<<std::endl;
#ifdef USE_ACTOR
  mdl->SetWorldPose(pose, false, false);
#else
  mdl->SetWorldPose(pose, true, true);
#endif
  //_actor->SetScriptTime(_actor->ScriptTime()+this->dt);
	return true;
	
}
void MengePlugin::updateAgents()
{
	size_t numAgt=_simulator->getNumAgents();
	for(size_t i=0;i<numAgt;i++)
	{
            updateAgent(_simulator->getAgent(i));
	}
}


void MengePlugin::controlActorAgents()
{
        size_t numAgt=_simulator->getNumAgents();
        for(size_t i=0;i<numAgt;i++)
        {
            const Agents::BaseAgent* agt=_simulator->getAgent(i);
            std::stringstream sstm;
            sstm << "MengeAgent_clone_" <<agt->_id;
            std::string uniq_name=sstm.str();
            ModelPtr mdl=_world->GetModel(uniq_name);       
            ActorPtr _actor = boost::dynamic_pointer_cast<physics::Actor>(mdl);
            if(_actor==nullptr){
                 std::cerr<<"something weird happened, actor for "<< uniq_name<<" is null\n";
                 continue;
            }
            std::cout<<"hi,"<<uniq_name<<std::endl;
            controlActor(_actor);
        }
}
void MengePlugin::simUpdate(float simTime)
{
	_mengeView.pause();
	updateAgents();
	_mengeView.resume();
	_mengeView.update(simTime);

}
bool MengePlugin::modelsInserted()
{
	size_t numAgt=_simulator->getNumAgents();
	for(size_t i=0;i<numAgt;i++) // go through every agent and check to see if pointer is null
	{
        const Agents::BaseAgent* agt=_simulator->getAgent(i);
        std::stringstream sstm;
        sstm << "MengeAgent_clone_" <<agt->_id;
        std::string uniq_name=sstm.str();
	    ModelPtr mdl=_world->GetModel(uniq_name);
	    if(mdl==nullptr) {
            std::cout<<"NULL POINTER"<<std::endl;
            return false;
	    }
	}

	return true;
}

// Called by the world update start event
void MengePlugin::OnUpdate(const common::UpdateInfo & _info) {
	if(!agentsSynced)
	{	
            std::cout<<"syncing at sim time:"<<_info.simTime.Double()<<std::endl;
            agentsSynced = modelsInserted();
            if(agentsSynced){
                  std::cout<<"agents synced and using physics engine:"<<_world->GetPhysicsEngine()->GetType()<<std::endl;
                  simStartTime = _info.simTime.Float()+simDelayPerAgent*_simulator->getNumAgents();
#ifdef USE_ACTOR
                  controlActorAgents();
#endif
                  updateAgents();
	     }
		
	}else if(_info.simTime.Double()>=simStartTime){
            this->dt = (_info.simTime - this->lastUpdate).Double();
            if(this->dt<_simulator->getTimeStep()) return;
            std::cout<<"updating at sim time:"<<_info.simTime.Double()<<" Menge time step:"<<_simulator->getTimeStep()<<"physics engine:"<<_world->GetPhysicsEngine()->GetType()<<std::endl;
            simUpdate(_info.simTime.Float()-simStartTime);
            this->lastUpdate = _info.simTime;
    }
}


} /* namespace gazebo */
