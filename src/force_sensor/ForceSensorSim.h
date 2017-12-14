// ForceSensorSim.h
// Force sensor for SAI2-Simulation
#ifndef FORCE_SENSOR_SIM_H
#define FORCE_SENSOR_SIM_H

#include <Sai2Model.h>
#include <Sai2Simulation.h>
#include <Eigen/Dense>
#include <string>
#include <vector>

// Basic data structure for force sensor data
struct ForceSensorData {
public:
	// TODO: should probably add some sensor identity as well
	std::string _robot_name; // name of robot to which sensor is attached
	std::string _link_name; // name of link to which sensor is attached
	// transform from link to sensor frame. Measured moments are with respect to
	// the sensor frame origin
	Eigen::Affine3d _transform_in_link;
	Eigen::Vector3d _force; // in sensor frame
	Eigen::Vector3d _moment; // in sensor frame

public:
	// ctor: assign defaults
	ForceSensorData()
	: _robot_name(""),
	_link_name(""),
	_transform_in_link(Eigen::Affine3d::Identity()),
	_force(Eigen::Vector3d::Zero()),
	_moment(Eigen::Vector3d::Zero())
	{
		// nothing to do
	}
};

// Simulated force sensor type.
// Note that this implementation ignores the mass and inertia of the object
// attached beyond the force sensor.
// Note also that this assumes the force sensor to be located just between the
// link and the end-effector. That is, it cannot account for any joint reaction
// forces in case the sensor is to be attached on the link between two joints.
// Finally note that the sensor returns values in the global frame
class ForceSensorSim {
public:
	// ctor
	ForceSensorSim(
		const std::string& robot_name,
		const std::string& link_name,
		const Eigen::Affine3d& transform_in_link,
		Sai2Model::Sai2Model* model);

	//dtor
	~ForceSensorSim();

	// update force information
	void update(Simulation::Sai2Simulation* sim);

	// get force
	void getForce(Eigen::Vector3d& ret_force);

	// get moment
	void getMoment(Eigen::Vector3d& ret_moment);

public:
	// handle to model interface
	Sai2Model::Sai2Model* _model;

	// last updated data
	ForceSensorData* _data;
};

#endif //FORCE_SENSOR_SIM_H
