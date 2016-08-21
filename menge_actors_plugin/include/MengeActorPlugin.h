/*
 * MengeRand.h
 *
 *  Created on: Jul 2, 2016
 *      Author: Michael Huang
 */

#ifndef MENGEACTORPLUGIN_H_
#define MENGEACTORPLUGIN_H_
#include "MengePlugin.h"
namespace gazebo {

class MengeActorPlugin: public MengePlugin {

protected:

	virtual void insertAgentModel(const Menge::Agents::BaseAgent* agt);
	virtual void insertAgentActor(const Menge::Agents::BaseAgent* agt);
	virtual bool updateAgent(const Menge::Agents::BaseAgent* agt);

};
GZ_REGISTER_WORLD_PLUGIN(MengeActorPlugin)
} /* namespace gazebo */

#endif /* MENGEPLUGIN_H_ */
