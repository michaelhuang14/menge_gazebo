/*
 * MengePlugin.h
 *
 *  Created on: Jul 2, 2016
 *      Author: Michael Huang
 */

#ifndef MENGEDEFAULTPLUGIN_H_
#define MENGEDEFAULTPLUGIN_H_
#include "MengePlugin.h"

namespace gazebo {

class MengeDefaultPlugin: public MengePlugin {

protected:

	virtual void insertAgentModel(const Menge::Agents::BaseAgent* agt);
	virtual void insertAgentActor(const Menge::Agents::BaseAgent* agt);

};

GZ_REGISTER_WORLD_PLUGIN(MengeDefaultPlugin)
} /* namespace gazebo */

#endif /* MENGEDEFAULTPLUGIN_H_ */
