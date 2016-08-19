/*
 * MengeRand.h
 *
 *  Created on: Jul 2, 2016
 *      Author: Michael Huang
 */

#ifndef MENGERAND_H_
#define MENGERAND_H_
#include "MengePlugin.h"
namespace gazebo {

class MengeRand: public MengePlugin {

protected:

	virtual void insertAgentModel(const Menge::Agents::BaseAgent* agt);
	virtual void insertAgentActor(const Menge::Agents::BaseAgent* agt);

};
GZ_REGISTER_WORLD_PLUGIN(MengeRand)
} /* namespace gazebo */

#endif /* MENGEPLUGIN_H_ */
