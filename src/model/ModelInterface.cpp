/*
 * ModelInterface.cpp
 *
 *  Created on: Dec 19, 2016
 *      Author: Mikael Jorda
 */

#include "ModelInterface.h"
#include "ModelInternal.h"
#include "RBDLModel.h"

#include <stdexcept>

namespace Model{

ModelInterface::ModelInterface (const std::string path_to_model_file, Model::ModelType model_type, Model::ParserType parser, bool verbose)
{

    // Create the desired model
    switch(model_type)
    {
	case rbdl :
	    _model_internal = new RBDLModel(path_to_model_file, parser, verbose);
	    break;

	default:
	    throw (std::runtime_error ("ModelInterface : Unsupported model type"));

    }

    // resize state vectors
    _q.setZero(dof());
    _dq.setZero(dof());
    _ddq.setZero(dof());
    _M.setIdentity(dof(),dof());
    _M_inv.setIdentity(dof(),dof());

}

ModelInterface::~ModelInterface (){delete _model_internal;}

void ModelInterface::updateModel()
{
	_model_internal->updateModel(_q, _dq, _ddq);
	_model_internal->massMatrix(_M,_q);
	_M_inv = _M.inverse();
}

void ModelInterface::massMatrix(Eigen::MatrixXd& A)
{_model_internal->massMatrix(A,_q);}

void ModelInterface::gravityVector(Eigen::VectorXd& g,
                                   const Eigen::Vector3d& gravity)
{_model_internal->gravityVector(g,_q,gravity);}

void ModelInterface::coriolisForce(Eigen::VectorXd& b)
{_model_internal->coriolisForce(b,_q,_dq);}

void ModelInterface::J(Eigen::MatrixXd& J,
                       const std::string& link_name,
                       const Eigen::Vector3d& pos_in_link)
{_model_internal->J(J,link_name,pos_in_link,_q);}

void ModelInterface::Jv(Eigen::MatrixXd& J,
                        const std::string& link_name,
                        const Eigen::Vector3d& pos_in_link)
{_model_internal->Jv(J,link_name,pos_in_link,_q);}

void ModelInterface::Jw(Eigen::MatrixXd& J,
                        const std::string& link_name)
{_model_internal->Jw(J,link_name,_q);}

void ModelInterface::transform(Eigen::Affine3d& T,
                               const std::string& link_name)
{_model_internal->transform(T,link_name,_q);}

void ModelInterface::position(Eigen::Vector3d& pos,
                              const std::string& link_name,
                              const Eigen::Vector3d& pos_in_link)
{_model_internal->position(pos,link_name,pos_in_link,_q);}

void ModelInterface::linearVelocity(Eigen::Vector3d& vel,
                              const std::string& link_name,
                              const Eigen::Vector3d& pos_in_link)
{_model_internal->linearVelocity(vel,link_name,pos_in_link,_q,_dq);}

void ModelInterface::linearAcceleration(Eigen::Vector3d& accel,
                                  const std::string& link_name,
                                  const Eigen::Vector3d& pos_in_link)
{_model_internal->linearAcceleration(accel, link_name,pos_in_link,_q,_dq,_ddq);}

void ModelInterface::rotation(Eigen::Matrix3d& rot,
                              const std::string& link_name)
{_model_internal->rotation(rot, link_name,_q);}

void ModelInterface::angularVelocity(Eigen::Vector3d& avel,
                                     const std::string& link_name)
{_model_internal->angularVelocity(avel,link_name,_q,_dq);}

void ModelInterface::angularAcceleration(Eigen::Vector3d& aaccel,
                                         const std::string& link_name)
{_model_internal->angularAcceleration(aaccel,link_name,_q,_dq,_ddq);}

int ModelInterface::dof()
{return _model_internal->_dof;}

// TODO : Untested
void ModelInterface::orientationError(Eigen::Vector3d& delta_phi,
		              const Eigen::Matrix3d& desired_orientation,
		              const Eigen::Matrix3d& current_orientation)
{
	// check that the matrices are valid rotations
	Eigen::Matrix3d Q1 = desired_orientation*desired_orientation.transpose() - Eigen::Matrix3d::Identity();
	Eigen::Matrix3d Q2 = current_orientation*current_orientation.transpose() - Eigen::Matrix3d::Identity();
	if(Q1.norm() > 0.0001 || Q2.norm() > 0.0001)
	{
		throw std::invalid_argument("Invalid rotation matrices in ModelInterface::orientationError");
		return;
	}
	else
	{
		Eigen::Vector3d rc1 = current_orientation.block<3,1>(0,0);
		Eigen::Vector3d rc2 = current_orientation.block<3,1>(0,1);
		Eigen::Vector3d rc3 = current_orientation.block<3,1>(0,2);
		Eigen::Vector3d rd1 = desired_orientation.block<3,1>(0,0);
		Eigen::Vector3d rd2 = desired_orientation.block<3,1>(0,1);
		Eigen::Vector3d rd3 = desired_orientation.block<3,1>(0,2);
		delta_phi = -1.0/2.0*(rc1.cross(rd1) + rc2.cross(rd2) + rc3.cross(rd3));
	}
}

// TODO : Untested
void ModelInterface::orientationError(Eigen::Vector3d& delta_phi,
		              const Eigen::Quaterniond& desired_orientation,
		              const Eigen::Quaterniond& current_orientation)
{
	Eigen::MatrixXd lambda_hat_t(3,4);
	double lambda_0 = current_orientation.w();
	double lambda_1 = current_orientation.x();
	double lambda_2 = current_orientation.y();
	double lambda_3 = current_orientation.z();

	lambda_hat_t << -lambda_1, lambda_0, -lambda_3, lambda_2,
			        -lambda_2, lambda_3, lambda_0, -lambda_1,
			        -lambda_3, -lambda_2, lambda_1, lambda_0;

	Eigen::VectorXd lambda_d(4);
	lambda_d << desired_orientation.w(), desired_orientation.x(), desired_orientation.y(), desired_orientation.z();

	delta_phi = -2*lambda_hat_t*lambda_d;
}

// TODO : Untested
void ModelInterface::taskInertiaMatrix(Eigen::MatrixXd& Lambda,
    					   const Eigen::MatrixXd& task_jacobian)
{
	// check matrices have the right size
	if(Lambda.rows() != Lambda.cols())
	{
		throw std::invalid_argument("Lambda matrix not square in ModelInterface::taksInertiaMatrix");
		return;
	}
	else if (Lambda.rows() != task_jacobian.rows())
	{
		throw std::invalid_argument("Rows of Jacobian inconsistent with size of Lambda matrix in ModelInterface::taksInertiaMatrix");
		return;
	}
	else if(task_jacobian.cols() != _model_internal->_dof)
	{
		throw std::invalid_argument("Jacobian size inconsistent with DOF of robot model in ModelInterface::taksInertiaMatrix");
		return;
	}
	// compute task inertia
	else
	{
		Eigen::MatrixXd inv_inertia = task_jacobian*_M_inv*task_jacobian.transpose();
		Lambda = inv_inertia.inverse();
	}
}

//TODO : Untested
void ModelInterface::dynConsistantInverseJacobian(Eigen::MatrixXd& Jbar,
									const Eigen::MatrixXd& task_jacobian)
{
	// check matrices have the right size
	if(Jbar.rows() != task_jacobian.cols() || Jbar.cols() != task_jacobian.rows())
	{
		throw std::invalid_argument("Matrix dimmensions inconsistent in ModelInterface::dynConsistantInverseJacobian");
		return;
	}
	// compute Jbar
	else
	{
		Eigen::MatrixXd task_inertia(task_jacobian.rows(),task_jacobian.rows());
		taskInertiaMatrix(task_inertia,task_jacobian);
		Jbar = _M_inv*task_jacobian.transpose()*task_inertia;
	}
}

void ModelInterface::nullspaceMatrix(Eigen::MatrixXd& N,
        					 const Eigen::MatrixXd& jacobian)
{
	Eigen::MatrixXd N_prec = Eigen::MatrixXd::Identity(dof(),dof());
	nullspaceMatrix(N,jacobian,N_prec);
}

//TODO :: Untested
void ModelInterface::nullspaceMatrix(Eigen::MatrixXd& N,
    					 const Eigen::MatrixXd& jacobian,
    					 const Eigen::MatrixXd& N_prec)
{
	// check matrices dimmnsions
	if(N.rows() != N.cols() || N.rows() != _model_internal->_dof)
	{
		throw std::invalid_argument("N matrix dimensions inconsistent in ModelInterface::nullspaceMatrix");
		return;
	}
	else if(N_prec.rows() != N_prec.cols() || N_prec.rows() != _model_internal->_dof)
	{
		throw std::invalid_argument("N_prec matrix dimensions inconsistent in ModelInterface::nullspaceMatrix");
		return;
	}
	else if(jacobian.cols() != N.rows())
	{
		throw std::invalid_argument("jacobian matrix dimensions inconsistent with model dof in ModelInterface::nullspaceMatrix");
		return;
	}
	else
	{
		Eigen::MatrixXd Jbar = Eigen::MatrixXd::Zero(jacobian.cols(),jacobian.rows());
		dynConsistantInverseJacobian(Jbar,jacobian);
		Eigen::MatrixXd Ni = Eigen::MatrixXd::Identity(N.rows(),N.cols());
		Ni = Ni - Jbar*jacobian;
		N = Ni*N_prec;
	}
}

}
