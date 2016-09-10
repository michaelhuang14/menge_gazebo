
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
#include <gazebo/physics/InstancedActor.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include "Agents/BaseAgent.h"
#include "MyViewer.h"
#include <SimulatorDB.h>
#include <SimulatorDBEntry.h>
#include <SimulatorInterface.h>
#include <string>

namespace gazebo {

class MengePlugin: public WorldPlugin {
public:
	void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);
	void OnUpdate(const common::UpdateInfo & _info);

protected:
	void simLoad();
	virtual void insertAgentModel(const Menge::Agents::BaseAgent* agt) =0;
	virtual void insertAgentActor(const Menge::Agents::BaseAgent* agt) =0;
	void insertAgents();
	virtual bool updateAgent(const Menge::Agents::BaseAgent* agt);
	void updateAgents();
	void updateGoals();
	void controlActor(physics::InstancedActorPtr _actor);
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
	protected: std::vector<event::ConnectionPtr> connections;
        transport::NodePtr node;
	protected: physics::WorldPtr _world;
	protected: double dt;
	transport::PublisherPtr factoryPub;
	
	protected: ignition::math::Vector3d velocity;
	protected: double targetWeight = 1.0;
	protected: common::Time lastUpdate;
	protected: physics::TrajectoryInfoPtr trajectoryInfo;
	protected: std::string menge_project_file;
};
} /* namespace gazebo */

#endif /* MENGEPLUGIN_H_ */
