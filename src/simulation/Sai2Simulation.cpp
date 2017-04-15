/*
 * Sai2Simulation.cpp
 *
 *  Created on: Dec 27, 2016
 *      Author: Shameek Ganguly
 */

#include "Sai2Simulation.h"
#include <dynamics3d.h>

#include "parser/URDFToDynamics3d.h"
#include <math/CMatrix3d.h> // chai math library

using namespace std;
using namespace chai3d;

namespace Simulation {

// ctor
Sai2Simulation::Sai2Simulation(const std::string& path_to_world_file,
	            		Simulation::ParserType parser,
	            		bool verbose)
: _world(NULL)
{
	// create a dynamics world
	_world = new cDynamicWorld(NULL);
	assert(_world);

	// parse world
	switch(parser) {
		case urdf:
			Parser::URDFToDynamics3dWorld(path_to_world_file, _world, verbose);
			break;
		case yml:
			cerr << "YAML parser for Sai2Simulation still not supported!" << endl;
			abort();
			break;
	}

	// enable dynamics for all robots in this world
	// TODO: consider pushing up to the API?
	for (auto robot: _world->m_dynamicObjects) {
		robot->enableDynamics(true);
	}
}

// dtor
Sai2Simulation::~Sai2Simulation() {
	//TODO: ensure deallocation within cDynamicWorld class
	delete _world;
	_world = NULL;
}

// read degrees of freedom
unsigned int Sai2Simulation::dof(const std::string& robot_name) const {
	auto robot = _world->getBaseNode(robot_name);
	assert(robot);
	return robot->m_dynamicJoints.size();
}

// set joint positions
void Sai2Simulation::setJointPositions(const std::string& robot_name,
											const Eigen::VectorXd& q) {
	auto robot = _world->getBaseNode(robot_name);
	assert(robot);
	assert(q.size() == robot->m_dynamicJoints.size());
	for (unsigned int i = 0; i < robot->m_dynamicJoints.size(); ++i) {
		robot->m_dynamicJoints[i]->setPos(q[i]);
	}
}

// read joint positions
void Sai2Simulation::getJointPositions(const std::string& robot_name,
												Eigen::VectorXd& q_ret) const {
	auto robot = _world->getBaseNode(robot_name);
	assert(robot);
	q_ret.setZero(robot->m_dynamicJoints.size());
	for (unsigned int i = 0; i < robot->m_dynamicJoints.size(); ++i) {
		q_ret[i] = robot->m_dynamicJoints[i]->getPos();
	}
}

// set joint position for a single joint
void Sai2Simulation::setJointPosition(const std::string& robot_name,
											unsigned int joint_id,
											double position) {
	auto robot = _world->getBaseNode(robot_name);
	assert(robot);
	assert(joint_id < robot->m_dynamicJoints.size());
	robot->m_dynamicJoints[joint_id]->setPos(position);
}

// set joint velocities
void Sai2Simulation::setJointVelocities(const std::string& robot_name,
												const Eigen::VectorXd& dq) {
	auto robot = _world->getBaseNode(robot_name);
	assert(robot);
	assert(dq.size() == robot->m_dynamicJoints.size());
	for (unsigned int i = 0; i < robot->m_dynamicJoints.size(); ++i) {
		robot->m_dynamicJoints[i]->setVel(dq[i]);
	}
}

// set joint velocity for a single joint
void Sai2Simulation::setJointVelocity(const std::string& robot_name,
											unsigned int joint_id,
											double velocity) {
	auto robot = _world->getBaseNode(robot_name);
	assert(robot);
	assert(joint_id < robot->m_dynamicJoints.size());
	robot->m_dynamicJoints[joint_id]->setVel(velocity);
}

// read joint velocities
void Sai2Simulation::getJointVelocities(const std::string& robot_name,
												Eigen::VectorXd& dq_ret) const {
	auto robot = _world->getBaseNode(robot_name);
	assert(robot);
	dq_ret.setZero(robot->m_dynamicJoints.size());
	for (unsigned int i = 0; i < robot->m_dynamicJoints.size(); ++i) {
		dq_ret[i] = robot->m_dynamicJoints[i]->getVel();
	}
}

// set joint torques
void Sai2Simulation::setJointTorques(const std::string& robot_name,
											const Eigen::VectorXd& tau) {
	auto robot = _world->getBaseNode(robot_name);
	assert(robot);
	assert(tau.size() == robot->m_dynamicJoints.size());
	for (unsigned int i = 0; i < robot->m_dynamicJoints.size(); ++i) {
		robot->m_dynamicJoints[i]->setForce(tau[i]);
		// NOTE: we don't support spherical joints currently
		// but in cDynamicJoint, spherical joints have a different function
		// for setting torque: void setTorque(chai3d::cVector3d&)
	}
}

// set joint torque for a single joint
void Sai2Simulation::setJointTorque(const std::string& robot_name,
											unsigned int joint_id,
											double tau) {
	auto robot = _world->getBaseNode(robot_name);
	assert(robot);
	assert(joint_id < robot->m_dynamicJoints.size());
	robot->m_dynamicJoints[joint_id]->setForce(tau);
	// NOTE: we don't support spherical joints currently
	// but in cDynamicJoint, spherical joints have a different function
	// for setting torque: void setTorque(chai3d::cVector3d&)
}

// read joint torques
// NOTE: currently unsupported due to lack of support in dynamics3d
void Sai2Simulation::getJointTorques(const std::string& robot_name,
											Eigen::VectorXd& tau_ret) const {
	cerr << "Unsupported function Sai2Simulation::getJointTorques" << endl;
	abort();
}

// read joint accelerations
// NOTE: currently unsupported due to lack of support in dynamics3d
void Sai2Simulation::getJointAcclerations(const std::string& robot_name,
												Eigen::VectorXd& ddq_ret) const {
	cerr << "Unsupported function Sai2Simulation::getJointTorques" << endl;
	abort();
}

// integrate ahead
void Sai2Simulation::integrate(double timestep) {
	_world->updateDynamics(timestep);
}

void Sai2Simulation::showContactInfo()
{

    // clear contact points for non contacting objects
    list<cDynamicBase*>::iterator i;
    for(i = _world->m_dynamicObjects.begin(); i != _world->m_dynamicObjects.end(); ++i)
    {
        cDynamicBase* object = *i;
    	int num_contacts = object->m_dynamicContacts->getNumContacts();
        if(num_contacts > 0)
        {
	    	std::cout << "object name : " << object->m_name << std::endl;
	    	std::cout << "num contacts : " << num_contacts << std::endl;
	    	for(int k=0; k < num_contacts; k++)
	    	{
	    		cDynamicContact* contact = object->m_dynamicContacts->getContact(k);
		    	std::cout << "contact " << k << " at link : " << contact->m_dynamicLink->m_name << std::endl;
		    	std::cout << "contact position : " << contact->m_globalPos << std::endl;
		    	std::cout << "contact normal : " << contact->m_globalNormal << std::endl;
		    	std::cout << "contact normal force : " << contact->m_globalNormalForce << std::endl;
		    	std::cout << "contact friction force : " << contact->m_globalFrictionForce << std::endl;
		    	std::cout << "contact force magnitude : " << contact->m_normalForceMagnitude << std::endl;
	    	}
	    	std::cout << std::endl;
        }

        // cDynContactList* contactList = nextItem->m_dynBaseNode->contact();
        // if (contactList->n() == 0)
        // {
        //     nextItem->m_dynamicContacts->clear();
        // }
    }
}

}