/**
 * OTG.cpp
 *
 *	A wrapper to use the Reflexxes lybrary type II with Eigen library
 *
 * Author: Mikael Jorda
 * Created: October 2018
 */

#include "OTG.h"
#include <stdexcept>

OTG::OTG(const int task_dof, const double loop_time)
{
	_task_dof = task_dof;
	_loop_time = loop_time;

	_RML = new ReflexxesAPI(_task_dof, _loop_time);
    _IP  = new RMLPositionInputParameters(_task_dof);
    _OP  = new RMLPositionOutputParameters(_task_dof);

    for(int i=0 ; i<_task_dof ; i++)
    {
    	_IP->SelectionVector->VecData[i] = true;
    }

}

OTG::~OTG()
{
	delete _RML;
	delete _IP;
	delete _OP;
	_RML = NULL;
	_IP = NULL;
	_OP = NULL;
}

void OTG::setMaxVelocity(const Eigen::VectorXd max_velocity)
{
	if(max_velocity.size() != _task_dof)
	{
		throw std::invalid_argument("size of input max velocity vector does not match task size in OTG::setMaxVelocity\n");
	}
	if(max_velocity.minCoeff() <= 0)
	{
		throw std::invalid_argument("max velocity set to 0 or negative value in some directions in OTG::setMaxVelocity\n");
	}

	for(int i=0 ; i<_task_dof ; i++)
	{
	    _IP->MaxVelocityVector->VecData[i] = max_velocity(i);
	}
}

void OTG::setMaxVelocity(const double max_velocity)
{
	setMaxVelocity(max_velocity * Eigen::VectorXd::Ones(_task_dof));
}

void OTG::setMaxAcceleration(const Eigen::VectorXd max_acceleration)
{
	if(max_acceleration.size() != _task_dof)
	{
		throw std::invalid_argument("size of input max acceleration vector does not match task size in OTG::setMaxAcceleration\n");
	}
	if(max_acceleration.minCoeff() <= 0)
	{
		throw std::invalid_argument("max acceleration set to 0 or negative value in some directions in OTG::setMaxAcceleration\n");
	}

	for(int i=0 ; i<_task_dof ; i++)
	{
	    _IP->MaxAccelerationVector->VecData[i] = max_acceleration(i);
	}
}

void OTG::setMaxAcceleration(const double max_acceleration)
{
	setMaxAcceleration(max_acceleration * Eigen::VectorXd::Ones(_task_dof));
}


void OTG::setGoalPosition(const Eigen::VectorXd goal_position)
{
	if(goal_position.size() != _task_dof)
	{
		throw std::invalid_argument("size of input goal position does not match task size in OTG::setGoalPosition\n");
	}
	setGoalPositionAndVelocity(goal_position, Eigen::VectorXd::Zero(_task_dof));
}


void OTG::setGoalPositionAndVelocity(const Eigen::VectorXd goal_position,
									const Eigen::VectorXd goal_velocity)
{
	if(goal_position.size() != _task_dof)
	{
		throw std::invalid_argument("size of input goal position does not match task size in OTG::setGoalPositionAndVelocity\n");
	}
	if(goal_velocity.size() != _task_dof)
	{
		throw std::invalid_argument("size of input goal velocity does not match task size in OTG::setGoalPositionAndVelocity\n");
	}
	for(int i=0 ; i<_task_dof ; i++)
	{
		_IP->TargetPositionVector->VecData[i] = goal_position(i);
	    _IP->TargetVelocityVector->VecData[i] = goal_velocity(i);
	}
	_goal_reached = false;
}

void OTG::updateState(const Eigen::VectorXd current_position, 
			const Eigen::VectorXd current_velocity)
{
	if(current_position.size() != _task_dof)
	{
		throw std::invalid_argument("size of input current position does not match task size in OTG::updateState\n");
	}
	if(current_velocity.size() != _task_dof)
	{
		throw std::invalid_argument("size of input current velocity does not match task size in OTG::updateState\n");
	}

	for(int i=0 ; i<_task_dof ; i++)
	{
		_IP->CurrentPositionVector->VecData[i] = current_position(i);
	    _IP->CurrentVelocityVector->VecData[i] = current_velocity(i);
	}
}

void OTG::computeNextState(Eigen::VectorXd& next_position,
				Eigen::VectorXd& next_velocity)
{
	next_position.setZero(_task_dof);
	next_velocity.setZero(_task_dof);
	for(int i=0 ; i<_task_dof ; i++)
	{
		next_position(i) = _IP->CurrentPositionVector->VecData[i];
		next_velocity(i) = _IP->CurrentVelocityVector->VecData[i];
	}

	if(!_goal_reached)
	{
		_ResultValue = _RML->RMLPosition(*_IP, _OP, _Flags);
        if (_ResultValue < 0)
        {
        	printf("An error occurred (%d) in OTG::computeNextState.\n", _ResultValue );
            throw std::runtime_error("error in computing next state in OTG::computeNextState.\n");
        }

		for(int i=0 ; i<_task_dof ; i++)
		{
			next_position(i) = _OP->NewPositionVector->VecData[i];
	        next_velocity(i) = _OP->NewVelocityVector->VecData[i];
		}
	}

	_goal_reached = (_ResultValue == ReflexxesAPI::RML_FINAL_STATE_REACHED);
	
}

bool OTG::goalReached()
{
	return _goal_reached;
}