#include "ForceSensorSim.h"


ForceSensorSim::ForceSensorSim(
	const std::string& robot_name,
	const std::string& link_name,
	const Eigen::Affine3d& transform_in_link,
	Sai2Model::Sai2Model* model)
:	_model(model)
{
	_data = new ForceSensorData();
	_data->_robot_name = robot_name;
	_data->_link_name = link_name;
	_data->_transform_in_link = transform_in_link;

	_force_filter = new ButterworthFilter(3);
	_moment_filter = new ButterworthFilter(3);
	_filter_on = false;
}

//dtor
ForceSensorSim::~ForceSensorSim() {
}

// update force information
void ForceSensorSim::update(Simulation::Sai2Simulation* sim) {
	// NOTE that this assumes that the robot model is updated
	// get the list of contact forces acting on the link
	std::vector<Eigen::Vector3d> force_list;
	std::vector<Eigen::Vector3d> point_list;
	sim->getContactList(
		point_list,
		force_list,
		_data->_robot_name,
		_data->_link_name);
	// zero out current forces and moments
	_data->_force.setZero();
	_data->_moment.setZero();
	// if list is empty, simply set forces and moments to 0
	if(point_list.empty()) {
		return;
	}
	// transform to sensor frame
	Eigen::Vector3d rel_pos;
	Eigen::Vector3d link_pos;
	_model->position(link_pos, _data->_link_name, _data->_transform_in_link.translation());
	for (uint pt_ind=0; pt_ind < point_list.size(); ++pt_ind) {
		_data->_force += force_list[pt_ind];
		rel_pos = point_list[pt_ind] - link_pos;
		//unfortunately, it is defined in global frame
		_data->_moment += rel_pos.cross(force_list[pt_ind]);
	}

	Eigen::VectorXd force_raw = _data->_transform_in_link.inverse().rotation()*_data->_force;
	Eigen::VectorXd moment_raw = _data->_transform_in_link.inverse().rotation()*_data->_moment;

	if(_filter_on)
	{
		_data->_force = _force_filter->update(force_raw);
		_data->_moment = _moment_filter->update(moment_raw);
	}
	else
	{
		_data->_force = force_raw;
		_data->_moment = moment_raw;
	}
}

// get force
void ForceSensorSim::getForce(Eigen::Vector3d& ret_force) {
	ret_force = _data->_force;
}

// get moment
void ForceSensorSim::getMoment(Eigen::Vector3d& ret_moment) {
	ret_moment = _data->_moment;
}

void ForceSensorSim::enableFilter(const double fc)
{
	_filter_on = true;
	_force_filter->setCutoffFrequency(fc);
	_moment_filter->setCutoffFrequency(fc);
}

