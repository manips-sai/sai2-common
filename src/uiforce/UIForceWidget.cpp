//UIForceWidget.cpp

#include "UIForceWidget.h"
#include <iostream>

using namespace chai3d;
using namespace std;

// enable or disable
void UIForceWidget::setEnable(bool enable) {
	if (_state == Disabled && enable) {
		_state = Inactive; // we only switch to active when the interaction paramters are set
	} else if (!enable) {
		_state = Disabled;
		// hide display line
		_display_line->setShowEnabled(false);
	}
}

// set current window and cursor properties
// this updates the internal parameters for calculating the ui interaction force
bool UIForceWidget::setInteractionParams(const string& camera_name, int viewx, int viewy, int window_width, int window_height) {
	if (_state == Disabled) {return false;}

	// if state is inactive, check if link selection is in progress
	if (_state == Inactive) {
		bool fLinkSelected = _graphics->getRobotLinkInCamera(camera_name, 
												_robot_name,
												viewx,
												viewy,
                                   				window_width,
                                   				window_height,
                                   				_link_name,
                                   				_link_local_pos);
		if (fLinkSelected) {
			_state = Active;
			_robot->positionInWorld(_initial_click_point, _link_name, _link_local_pos);
			// std::cout << "Active: link " << _link_name << std::endl;
		}
		else
		{
			return false;
		}
		//TODO: as an optimization we could perform this check only once per click
	}
	// if state is active,
	if (_state == Active) {
		// update line point A in global graphics frame
		Eigen::Vector3d pointA_pos_base;
		_robot->positionInWorld(pointA_pos_base, _link_name, _link_local_pos);
		_display_line->m_pointA.set(pointA_pos_base[0], pointA_pos_base[1], pointA_pos_base[2]);

		// update line point B. Assumes perspective view!
		// m_fieldViewAngleDeg / 2.0 would correspond to the _top_ of the window
		cCamera* camera = _graphics->getCamera(camera_name);
		double distCam = (window_height/2.0) / cTanDeg(camera->getFieldViewAngleDeg()/2.0);

        Eigen::Vector3d selectRay;
        selectRay << -distCam,
                      (viewx - (window_width/2.0)),
                      (viewy - (window_height/2.0));

        // create a point that's at the same axial distance from the camera as the initial click point
        Eigen::Vector3d camera_pos, camera_lookat, camera_vertical;
		_graphics->getCameraPose(camera_name, camera_pos, camera_vertical, camera_lookat);
        double lookat_dist = (_initial_click_point - camera_pos).norm();
        selectRay = selectRay * lookat_dist/selectRay.x();
        selectRay = camera->getGlobalRot().eigen() * selectRay;
		_display_line->m_pointB = camera->getGlobalPos() - cVector3d(selectRay);

		// display line
		_display_line->setShowEnabled(true);
	}

	return true;
}

// get interaction force
void UIForceWidget::getUIForce(Eigen::Vector3d& ret_force) const {
	ret_force = Eigen::Vector3d::Zero();

	// nothing to do if state is not active
	if (_state == Disabled || _state == Inactive) {
		return;
	}

	// calculate spring force in global frame
	cVector3d temp = _display_line->m_pointB - _display_line->m_pointA;
	ret_force << temp.x(), temp.y(), temp.z();
	ret_force = ret_force*_spring_k;

	// adjust to keep below max force
	if (ret_force.norm() > _max_force) {
		ret_force.normalize();
		ret_force = ret_force*_max_force;
	}
	return;
}

// get interaction joint torques
void UIForceWidget::getUIJointTorques(Eigen::VectorXd& ret_torques) const {
	ret_torques = Eigen::VectorXd::Zero(_robot->dof());
	// nothing to do if state is not active
	if (_state == Disabled || _state == Inactive) {
		return;
	}

	Eigen::Vector3d force;
	getUIForce(force);
	// compute Jv'F for the force
	Eigen::MatrixXd Jv;
	_robot->Jv(Jv, _link_name, _link_local_pos);
	ret_torques = Jv.transpose() * force;
}

