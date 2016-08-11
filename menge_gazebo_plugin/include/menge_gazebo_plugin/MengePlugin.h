/*
 * MengePlugin.h
 *
 *  Created on: Jul 2, 2016
 *      Author: Michael Huang
 */

#ifndef MENGEPLUGIN_H_
#define MENGEPLUGIN_H_
#include <ignition/math/Pose3.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include "Agents/BaseAgent.h"
#include "MyViewer.h"
#include <SimulatorDB.h>
#include <SimulatorDBEntry.h>
#include <SimulatorInterface.h>

namespace gazebo {

class MengePlugin: public WorldPlugin {
public:
	void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);
	void OnUpdate(const common::UpdateInfo & _info);

private:
	void simLoad();
	void insertAgentModel(const Menge::Agents::BaseAgent* agt);
	void insertAgentActor(const Menge::Agents::BaseAgent* agt);
	void insertAgents();
	bool updateAgent(const Menge::Agents::BaseAgent* agt);
	void updateAgents();
	void controlActor(physics::ActorPtr _actor);
	void controlActorAgents();
	bool modelsInserted();
	void simStart();
	void simUpdate(float simTime);
	int simMain( Menge::SimulatorDBEntry * dbEntry, const std::string & behaveFile,
			const std::string & sceneFile, const std::string & outFile, const std::string & scbVersion,
			const std::string & dumpPath ) ;

	Menge::Vis::MyViewer _mengeView;
	const Menge::Agents::SimulatorInterface * _simulator;
	void TakeControlOnActor();
	private: std::vector<event::ConnectionPtr> connections;
        transport::NodePtr node;
	private: physics::WorldPtr _world;
	private: double dt;
	transport::PublisherPtr factoryPub;
	
	private: ignition::math::Vector3d velocity;
	private: double targetWeight = 1.0;
	private: common::Time lastUpdate;
	private: physics::TrajectoryInfoPtr trajectoryInfo;
};
GZ_REGISTER_WORLD_PLUGIN(MengePlugin)
} /* namespace gazebo */

#endif /* MENGEPLUGIN_H_ */
