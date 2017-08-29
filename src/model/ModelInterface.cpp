/*
 * ModelInterface.cpp
 *
 *  Created on: Dec 19, 2016
 *      Author: Mikael Jorda
 */

#include "ModelInterface.h"
#include "ModelInternal.h"
#include "RBDLModel.h"

#ifdef USE_KUKA_LBR_DYNAMICS
	#include "KukaRBDLModel.h"
#endif

#include <stdexcept>

namespace Model{

ModelInterface::ModelInterface(const std::string& path_to_model_file, Model::ModelType model_type,
                               Model::ParserType parser, bool verbose) {

	// Create the desired model
	switch (model_type) {
		case rbdl:
			_model_internal = new RBDLModel(path_to_model_file, parser, verbose);
			break;

		case rbdl_kuka :
#ifdef USE_KUKA_LBR_DYNAMICS
			_model_internal = new KukaRBDLModel(path_to_model_file, parser, verbose);
			break;
#endif
			throw std::runtime_error("Compile sai2-common using KUKA_LBR_DYNAMICS option in order to use the kuka_rbdl model");

		default:
			throw std::runtime_error("ModelInterface : Unsupported model type");
	}

	// resize state vectors
	_q.setZero(dof());
	_dq.setZero(dof());
	_ddq.setZero(dof());
	_M.setIdentity(dof(),dof());
	_M_inv.setIdentity(dof(),dof());

}

ModelInterface::~ModelInterface() {
	delete _model_internal;
}

void ModelInterface::updateModel() {
	_model_internal->updateModel(_q, _dq, _ddq);
	_model_internal->massMatrix(_M,_q);
	_M_inv = _M.inverse();
}

void ModelInterface::massMatrix(Eigen::MatrixXd& A) const {
	_model_internal->massMatrix(A,_q);
}

Eigen::MatrixXd ModelInterface::gravityVector(const Eigen::Vector3d& gravity) const {
	Eigen::VectorXd g;
	_model_internal->gravityVector(g,_q,gravity);
	return g;
}

void ModelInterface::gravityVector(Eigen::VectorXd& g,
                                   const Eigen::Vector3d& gravity) const {
	_model_internal->gravityVector(g,_q,gravity);
}

void ModelInterface::coriolisForce(Eigen::VectorXd& b) const {
	_model_internal->coriolisForce(b,_q,_dq);
}

Eigen::MatrixXd ModelInterface::coriolisForce() const {
	Eigen::VectorXd b;
	_model_internal->coriolisForce(b,_q,_dq);
	return b;
}

void ModelInterface::J_0(Eigen::MatrixXd& J,
                         const std::string& link_name,
                         const Eigen::Vector3d& pos_in_link) const {
	_model_internal->J_0(J,link_name,pos_in_link,_q);
}

void ModelInterface::J(Eigen::MatrixXd& J,
                       const std::string& link_name,
                       const Eigen::Vector3d& pos_in_link) const {
	_model_internal->J(J,link_name,pos_in_link,_q);
}

Eigen::MatrixXd ModelInterface::J(const std::string& link_name, const Eigen::Vector3d& pos_in_link) const {
	Eigen::MatrixXd J;
	_model_internal->J_0(J,link_name,pos_in_link,_q);
	return J;
}

void ModelInterface::Jv(Eigen::MatrixXd& J,
                        const std::string& link_name,
                        const Eigen::Vector3d& pos_in_link) const {
	_model_internal->Jv(J,link_name,pos_in_link,_q);
}

Eigen::MatrixXd ModelInterface::Jv(const std::string& link_name, const Eigen::Vector3d& pos_in_link) const {
	Eigen::MatrixXd Jv;
	_model_internal->Jv(Jv,link_name,pos_in_link,_q);
	return Jv;
}

void ModelInterface::Jw(Eigen::MatrixXd& J,
                        const std::string& link_name) const {
	_model_internal->Jw(J,link_name,_q);
}

Eigen::MatrixXd ModelInterface::Jw(const std::string& link_name) const {
	Eigen::MatrixXd Jw;
	_model_internal->Jw(Jw,link_name,_q);
	return Jw;
}

void ModelInterface::transform(Eigen::Affine3d& T, const std::string& link_name) const {
	_model_internal->transform(T,link_name,_q);
}

Eigen::Affine3d ModelInterface::transform(const std::string& link_name) const {
	Eigen::Affine3d T;
	_model_internal->transform(T,link_name,_q);
	return T;
}

void ModelInterface::position(Eigen::Vector3d& pos,
                              const std::string& link_name,
                              const Eigen::Vector3d& pos_in_link) const {
	_model_internal->position(pos,link_name,pos_in_link,_q);
}

Eigen::Vector3d ModelInterface::position(const std::string& link_name, const Eigen::Vector3d& pos_in_link) const {
	Eigen::Vector3d pos;
	_model_internal->position(pos,link_name,pos_in_link,_q);
	return pos;
}

void ModelInterface::linearVelocity(Eigen::Vector3d& vel,
                                    const std::string& link_name,
                                    const Eigen::Vector3d& pos_in_link) const {
	_model_internal->linearVelocity(vel,link_name,pos_in_link,_q,_dq);
}

Eigen::Vector3d ModelInterface::linearVelocity(const std::string& link_name,
                                               const Eigen::Vector3d& pos_in_link) const {
	Eigen::Vector3d vel;
	_model_internal->linearVelocity(vel,link_name,pos_in_link,_q,_dq);
	return vel;
}

void ModelInterface::linearAcceleration(Eigen::Vector3d& accel,
                                        const std::string& link_name,
                                        const Eigen::Vector3d& pos_in_link) const {
	_model_internal->linearAcceleration(accel, link_name,pos_in_link,_q,_dq,_ddq);
}

Eigen::Vector3d ModelInterface::linearAcceleration(const std::string& link_name, const Eigen::Vector3d& pos_in_link) const {
	Eigen::Vector3d accel;
	_model_internal->linearAcceleration(accel, link_name,pos_in_link,_q,_dq,_ddq);
	return accel;
}

void ModelInterface::rotation(Eigen::Matrix3d& rot, const std::string& link_name) const {
	_model_internal->rotation(rot, link_name,_q);
}

Eigen::Matrix3d ModelInterface::rotation(const std::string& link_name) const {
	Eigen::Matrix3d rot;
	_model_internal->rotation(rot, link_name,_q);
	return rot;
}

void ModelInterface::angularVelocity(Eigen::Vector3d& avel, const std::string& link_name) const {
	_model_internal->angularVelocity(avel,link_name,_q,_dq);
}

Eigen::Vector3d ModelInterface::angularVelocity(const std::string& link_name) const {
	Eigen::Vector3d avel;
	_model_internal->angularVelocity(avel,link_name,_q,_dq);
	return avel;
}

void ModelInterface::angularAcceleration(Eigen::Vector3d& aaccel, const std::string& link_name) const {
	_model_internal->angularAcceleration(aaccel,link_name,_q,_dq,_ddq);
}

Eigen::Vector3d ModelInterface::angularAcceleration(const std::string& link_name) const {
	Eigen::Vector3d aaccel;
	_model_internal->angularAcceleration(aaccel,link_name,_q,_dq,_ddq);
	return aaccel;
}

int ModelInterface::dof() const {
	return _model_internal->_dof;
}

// TODO : Untested
Eigen::Vector3d ModelInterface::orientationError(const Eigen::Matrix3d& desired_orientation,
                                                 const Eigen::Matrix3d& current_orientation) const {
	// check that the matrices are valid rotations
	Eigen::Matrix3d Q1 = desired_orientation*desired_orientation.transpose() - Eigen::Matrix3d::Identity();
	Eigen::Matrix3d Q2 = current_orientation*current_orientation.transpose() - Eigen::Matrix3d::Identity();
	if (Q1.norm() > 0.0001 || Q2.norm() > 0.0001) {
		throw std::invalid_argument("Invalid rotation matrices in ModelInterface::orientationError");
	}

	Eigen::Vector3d rc1 = current_orientation.block<3,1>(0,0);
	Eigen::Vector3d rc2 = current_orientation.block<3,1>(0,1);
	Eigen::Vector3d rc3 = current_orientation.block<3,1>(0,2);
	Eigen::Vector3d rd1 = desired_orientation.block<3,1>(0,0);
	Eigen::Vector3d rd2 = desired_orientation.block<3,1>(0,1);
	Eigen::Vector3d rd3 = desired_orientation.block<3,1>(0,2);
	Eigen::Vector3d delta_phi = -0.5 * (rc1.cross(rd1) + rc2.cross(rd2) + rc3.cross(rd3));
	return delta_phi;
}

void ModelInterface::orientationError(Eigen::Vector3d& delta_phi,
                                      const Eigen::Matrix3d& desired_orientation,
                                      const Eigen::Matrix3d& current_orientation) const {
	delta_phi = orientationError(desired_orientation, current_orientation);
}

Eigen::Vector3d ModelInterface::orientationError(const Eigen::Quaterniond& desired_orientation,
                                                 const Eigen::Quaterniond& current_orientation) const {
	Eigen::Quaterniond inv_dlambda = desired_orientation * current_orientation.conjugate();
	Eigen::Vector3d delta_phi = 2.0 * inv_dlambda.vec();
	return delta_phi;
}

void ModelInterface::orientationError(Eigen::Vector3d& delta_phi,
                                      const Eigen::Quaterniond& desired_orientation,
                                      const Eigen::Quaterniond& current_orientation) const {
	delta_phi = orientationError(desired_orientation, current_orientation);
}

// TODO : Untested
Eigen::MatrixXd ModelInterface::taskInertiaMatrix(const Eigen::MatrixXd& task_jacobian) const {
	// check matrices have the right size
	if (task_jacobian.cols() != _model_internal->_dof) {
		throw std::invalid_argument("Jacobian size inconsistent with DOF of robot model in ModelInterface::taskInertiaMatrix");
	}

	// compute task inertia
	Eigen::MatrixXd Lambda = (task_jacobian * _M_inv * task_jacobian.transpose()).inverse();
	return Lambda;
}

void ModelInterface::taskInertiaMatrix(Eigen::MatrixXd& Lambda, const Eigen::MatrixXd& task_jacobian) const {
	if(Lambda.rows() != Lambda.cols()) {
		throw std::invalid_argument("Lambda matrix not square in ModelInterface::taksInertiaMatrix");
	} else if (Lambda.rows() != task_jacobian.rows()) {
		throw std::invalid_argument("Rows of Jacobian inconsistent with size of Lambda matrix in ModelInterface::taksInertiaMatrix");
	}
	Lambda = taskInertiaMatrix(task_jacobian);
}

// TODO : Untested
Eigen::MatrixXd ModelInterface::taskInertiaMatrixWithPseudoInv(const Eigen::MatrixXd& task_jacobian) const {
	// check matrices have the right size
	if (task_jacobian.cols() != _model_internal->_dof) {
		throw std::invalid_argument("Jacobian size inconsistent with DOF of robot model in ModelInterface::taskInertiaMatrixWithPseudoInv");
	}

	// compute task inertia
	Eigen::MatrixXd inv_inertia = task_jacobian * _M_inv * task_jacobian.transpose();

	// compute SVD pseudoinverse
	// TODO: make class function?
	Eigen::JacobiSVD<Eigen::MatrixXd> svd(inv_inertia, Eigen::ComputeThinU | Eigen::ComputeThinV);
	const double epsilon = std::numeric_limits<double>::epsilon();
	double tolerance = epsilon * std::max(inv_inertia.cols(), inv_inertia.rows()) * svd.singularValues().array().abs()(0);
	Eigen::MatrixXd Lambda = svd.matrixV() * (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
	return Lambda;
}

void ModelInterface::taskInertiaMatrixWithPseudoInv(Eigen::MatrixXd& Lambda,
                                                    const Eigen::MatrixXd& task_jacobian) const {
	// check matrices have the right size
	if (Lambda.rows() != Lambda.cols()) {
		throw std::invalid_argument("Lambda matrix not square in ModelInterface::taskInertiaMatrixWithPseudoInv");
	} else if (Lambda.rows() != task_jacobian.rows()) {
		throw std::invalid_argument("Rows of Jacobian inconsistent with size of Lambda matrix in ModelInterface::taskInertiaMatrixWithPseudoInv");
	}
	Lambda = taskInertiaMatrixWithPseudoInv(task_jacobian);
}

//TODO : Untested
Eigen::MatrixXd ModelInterface::dynConsistentInverseJacobian(const Eigen::MatrixXd& task_jacobian) const {
	// compute Jbar
	Eigen::MatrixXd task_inertia = taskInertiaMatrix(task_jacobian);
	Eigen::MatrixXd Jbar = _M_inv * task_jacobian.transpose() * task_inertia;
	return Jbar;
}

void ModelInterface::dynConsistentInverseJacobian(Eigen::MatrixXd& Jbar,
                                                  const Eigen::MatrixXd& task_jacobian) const {
	// check matrices have the right size
	if (Jbar.rows() != task_jacobian.cols() || Jbar.cols() != task_jacobian.rows()) {
		throw std::invalid_argument("Matrix dimmensions inconsistent in ModelInterface::dynConsistentInverseJacobian");
	}
	Jbar = dynConsistentInverseJacobian(task_jacobian);
}

Eigen::MatrixXd ModelInterface::nullspaceMatrix(const Eigen::MatrixXd& task_jacobian) const {
	Eigen::MatrixXd N_prec = Eigen::MatrixXd::Identity(dof(),dof());
	return nullspaceMatrix(task_jacobian, N_prec);
}

// void ModelInterface::nullspaceMatrix(Eigen::MatrixXd& N, const Eigen::MatrixXd& task_jacobian) const {
//     N = nullspaceMatrix(task_jacobian);
// }

//TODO :: Untested
Eigen::MatrixXd ModelInterface::nullspaceMatrix(const Eigen::MatrixXd& task_jacobian,
                                                const Eigen::MatrixXd& N_prec) const {
	// check matrices dimmnsions
	if (N_prec.rows() != N_prec.cols() || N_prec.rows() != _model_internal->_dof) {
		throw std::invalid_argument("N_prec matrix dimensions inconsistent in ModelInterface::nullspaceMatrix");
	}

	// Compute N
	Eigen::MatrixXd Jbar = dynConsistentInverseJacobian(task_jacobian);
	Eigen::MatrixXd Ni = Eigen::MatrixXd::Identity(dof(),dof()) - Jbar * task_jacobian;
	Eigen::MatrixXd N = Ni * N_prec;
	return N;
}

void ModelInterface::nullspaceMatrix(Eigen::MatrixXd& N,
                                     const Eigen::MatrixXd& task_jacobian,
                                     const Eigen::MatrixXd& N_prec) const {
	if (N.rows() != N.cols() || N.rows() != _model_internal->_dof) {
		throw std::invalid_argument("N matrix dimensions inconsistent in ModelInterface::nullspaceMatrix");
	} else if(task_jacobian.cols() != N.rows()) {
		throw std::invalid_argument("jacobian matrix dimensions inconsistent with model dof in ModelInterface::nullspaceMatrix");
	}
	N = nullspaceMatrix(task_jacobian, N_prec);
}

// TODO : Untested
void ModelInterface::operationalSpaceMatrices(Eigen::MatrixXd& Lambda,
                                              Eigen::MatrixXd& Jbar,
                                              Eigen::MatrixXd& N,
                                              const Eigen::MatrixXd& task_jacobian) const {
	Eigen::MatrixXd N_prec = Eigen::MatrixXd::Identity(dof(),dof());
	operationalSpaceMatrices(Lambda,Jbar,N,task_jacobian,N_prec);
}

// TODO : Untested
void ModelInterface::operationalSpaceMatrices(Eigen::MatrixXd& Lambda,
                                              Eigen::MatrixXd& Jbar,
                                              Eigen::MatrixXd& N,
                                              const Eigen::MatrixXd& task_jacobian,
                                              const Eigen::MatrixXd& N_prec) const {
	// check matrices have the right size
	// if (Lambda.rows() != Lambda.cols()) {
	//     throw std::invalid_argument("Lambda matrix not square in ModelInterface::operationalSpaceMatrices");
	// } else if (Lambda.rows() != task_jacobian.rows()) {
	//     throw std::invalid_argument("Rows of Jacobian inconsistent with size of Lambda matrix in ModelInterface::operationalSpaceMatrices");
	// } else if (Jbar.rows() != task_jacobian.cols() || Jbar.cols() != task_jacobian.rows()) {
	//     throw std::invalid_argument("Matrix dimmensions inconsistent in ModelInterface::operationalSpaceMatrices");
	// } else if (N.rows() != N.cols() || N.rows() != _model_internal->_dof) {
	//     throw std::invalid_argument("N matrix dimensions inconsistent in ModelInterface::operationalSpaceMatrices");
	// }
	if (N_prec.rows() != N_prec.cols() || N_prec.rows() != _model_internal->_dof) {
		throw std::invalid_argument("N_prec matrix dimensions inconsistent in ModelInterface::operationalSpaceMatrices");
	} else if (task_jacobian.cols() != _model_internal->_dof) {
		throw std::invalid_argument("Jacobian size inconsistent with DOF of robot model in ModelInterface::operationalSpaceMatrices");
	}

	// Compute the matrices
	Lambda = (task_jacobian * _M_inv * task_jacobian.transpose()).inverse();
	Jbar = _M_inv * task_jacobian.transpose() * Lambda;
	Eigen::MatrixXd Ni = Eigen::MatrixXd::Identity(dof(),dof()) - Jbar * task_jacobian;
	N = Ni * N_prec;
}

void ModelInterface::GraspMatrix(Eigen::MatrixXd& G,
	Eigen::Matrix3d& R,
	const std::vector<std::string> link_names,
	const std::vector<Eigen::Vector3d> pos_in_links,
	const std::vector<ContactNature> contact_natures,
	const Eigen::Vector3d center_point) const
{
	G = Eigen::MatrixXd::Zero(1,1);
	R = Eigen::Matrix3d::Identity();

	// number of contact points
	int n = link_names.size();
	if(n < 2)
	{
		throw std::invalid_argument("invalid number of contact points (2 points min)\n");
	}
	if(n > 4)
	{
		throw std::invalid_argument("invalid number of contact points (4 points max)\n");
	}
	if((pos_in_links.size() != n) || (contact_natures.size() != n))
	{
		throw std::invalid_argument("input vectors for the link names, pos in links and contact natures don't have the same size\n");
	}
	// number of surface contacts (that can apply a moment)
	int k = std::count(contact_natures.begin(), contact_natures.end(), SurfaceContact);

	Eigen::MatrixXd Wf = Eigen::MatrixXd::Zero(6, 3*n);
	Eigen::MatrixXd Wm = Eigen::MatrixXd::Zero(6, 3*k);

	std::vector<Eigen::Vector3d> positions_in_world;

	for(int i=0; i<n; i++)
	{
		Eigen::Vector3d pi;
		position(pi, link_names[i], pos_in_links[i]);
		positions_in_world.push_back(pi);
		Eigen::Vector3d ri = pi-center_point;
		Wf.block<3,3>(0,3*i) = Eigen::Matrix3d::Identity();
		Wf.block<3,3>(3,3*i) = CrossProductOperator(ri);
	}
	for(int i=0; i<k; i++)
	{
		Wm.block<3,3>(3,3*i) = Eigen::Matrix3d::Identity();
	}

	Eigen::MatrixXd E, I;

	switch (n)
	{
		case 2: 
		{
			// resize E
			E = Eigen::MatrixXd::Zero(6,1);
			
			// compute the point to point vectors
			Eigen::Vector3d e12 = positions_in_world[1] - positions_in_world[0];
			e12.normalize();

			// fill in E matrix
			E.block<3,1>(0,0) = -e12;
			E.block<3,1>(3,0) = e12;

			// create Ebar
			Eigen::MatrixXd Ebar = (E.transpose()*E).inverse() * E.transpose();

			// find R
			Eigen::Vector3d x = e12;
			// std::cout << "new x : " << x.transpose() << std::endl;
			// std::cout << "new x cross world x : " << (x.cross(Eigen::Vector3d(1,0,0))).transpose() << std::endl;
			// std::cout << "new x cross world x norm : " << (x.cross(Eigen::Vector3d(1,0,0))).norm() << std::endl;
			// std::cout << "abs : " << std::abs((x.cross(Eigen::Vector3d(1,0,0))).norm()) << std::endl;
			// std::cout << std::endl;
			if(std::abs((x.cross(Eigen::Vector3d(1,0,0))).norm()) < 1e-3) // new x is aligned with world x
			{
				if(x.dot(Eigen::Vector3d(1,0,0)) > 0) // same direction
				{
					R = Eigen::Matrix3d::Identity();
					// std::cout << "R is identity" << std::endl;
				}
				else // rotation around Z axis by 180 degrees
				{
					R << -1, 0, 0,
						 0, -1, 0, 
						 0, 0, 1;
				}
			}
			else
			{
				Eigen::Vector3d y = x.cross(Eigen::Vector3d(1,0,0));
				y.normalize();
				Eigen::Vector3d z = x.cross(y);
				z.normalize();
				R.block<3,1>(0,0) = x;
				R.block<3,1>(0,1) = y;
				R.block<3,1>(0,2) = z;
			}

			Eigen::MatrixXd Rr = Eigen::MatrixXd::Zero(6,6);
			Rr.block<3,3>(0,0) = R;
			Rr.block<3,3>(3,3) = R;

			Wf = Rr.transpose() * Wf;

			switch(k)
			{
				case 0:
				{
					throw std::runtime_error("Case 2-0 not implemented yet\n");
					break;
				}
				case 1: 
				{
					// only 2 internal moments
					I = Eigen::MatrixXd::Zero(2,3);

					I << 0, 1, 0,
					     0, 0, 1;
					I = I*R.transpose();

					Wm = Rr.transpose()*Wm;

					// populate G
					G = Eigen::MatrixXd::Zero(9,9);
					G.block<6,6>(0,0) = Wf;
					G.block<6,3>(0,6) = Wm;
					G.block<1,6>(6,0) = Ebar;
					G.block<2,3>(7,6) = I;
					break;
				}
				case 2: 
				{
					I = Eigen::MatrixXd::Zero(5,6);

					// find I
					I << -0.5, 0, 0, 0.5, 0, 0,
						  0, 1, 0, 0, 0, 0,
						  0, 0, 1, 0, 0, 0,
						  0, 0, 0, 0, 1, 0,
						  0, 0, 0, 0, 0, 1;
					I = I*Rr.transpose();

					Wm = Rr.transpose()*Wm;

					// populate G
					G = Eigen::MatrixXd::Zero(12,12);
					G.block<6,6>(0,0) = Wf;
					G.block<6,6>(0,6) = Wm;
					G.block<1,6>(6,0) = Ebar;
					G.block<5,6>(7,6) = I;
					break;
				}
				default: 
				throw std::runtime_error("Should not arrive here (number of contact points is 2, number of surface contacts incoherent)\n");

			}
			break;

		}

		case 3: 
		{
			// resize E
			E = Eigen::MatrixXd::Zero(9,3);
			
			// compute the point to point vectors
			Eigen::Vector3d e12 = positions_in_world[1] - positions_in_world[0];
			Eigen::Vector3d e13 = positions_in_world[2] - positions_in_world[0];
			Eigen::Vector3d e23 = positions_in_world[2] - positions_in_world[1];

			e12.normalize();
			e13.normalize();
			e23.normalize();

			// fill in E matrix
			E.block<3,1>(0,0) = -e12;
			E.block<3,1>(3,0) = e12;
			E.block<3,1>(0,1) = -e13;
			E.block<3,1>(6,1) = e13;
			E.block<3,1>(3,2) = -e23;
			E.block<3,1>(6,2) = e23;

			// std::cout << "E : \n" << E << std::endl << std::endl;

			// create Ebar
			Eigen::MatrixXd Ebar = (E.transpose()*E).inverse() * E.transpose();

			switch(k)
			{
				case 0:
				{
					// populate G
					G = Eigen::MatrixXd::Zero(9,9);
					G.block<6,9>(0,0) = Wf;
					G.block<3,9>(6,0) = Ebar;
					break;
				}
				case 1: 
				{
					// compute I
					Eigen::MatrixXd I = Eigen::MatrixXd::Identity(3,3);

					// populate G
					G = Eigen::MatrixXd::Zero(12,12);
					G.block<6,9>(0,0) = Wf;
					G.block<6,3>(0,9) = Wm;
					G.block<3,9>(6,0) = Ebar;
					G.block<3,3>(9,9) = I;
					break;
				}
				case 2: 
				{
					// compute I
					Eigen::MatrixXd I = Eigen::MatrixXd::Identity(6,6);

					// populate G
					G = Eigen::MatrixXd::Zero(15,15);
					G.block<6,9>(0,0) = Wf;
					G.block<6,6>(0,9) = Wm;
					G.block<3,9>(6,0) = Ebar;
					G.block<6,6>(9,9) = I;
					break;
				}
				case 3: 
				{
					// compute I
					Eigen::MatrixXd I = Eigen::MatrixXd::Identity(9,9);

					// populate G
					G = Eigen::MatrixXd::Zero(18,18);
					G.block<6,9>(0,0) = Wf;
					G.block<6,9>(0,9) = Wm;
					G.block<3,9>(6,0) = Ebar;
					G.block<9,9>(9,9) = I;
					break;
				}

				default: 
				throw std::runtime_error("Should not arrive here (number of contact points is 3, number of surface contacts incoherent)\n");

			}
			break;

		}		

		case 4: 
		{
			// resize E
			E = Eigen::MatrixXd::Zero(12,6);
			
			// compute the point to point vectors
			Eigen::Vector3d e12 = positions_in_world[1] - positions_in_world[0];
			Eigen::Vector3d e13 = positions_in_world[2] - positions_in_world[0];
			Eigen::Vector3d e14 = positions_in_world[3] - positions_in_world[0];
			Eigen::Vector3d e23 = positions_in_world[2] - positions_in_world[1];
			Eigen::Vector3d e24 = positions_in_world[3] - positions_in_world[1];
			Eigen::Vector3d e34 = positions_in_world[3] - positions_in_world[2];

			e12.normalize();
			e13.normalize();
			e14.normalize();
			e23.normalize();
			e24.normalize();
			e34.normalize();

			// fill in E matrix
			E.block<3,1>(0,0) = -e12;
			E.block<3,1>(3,0) = e12;
			E.block<3,1>(0,1) = -e13;
			E.block<3,1>(6,1) = e13;
			E.block<3,1>(0,2) = -e14;
			E.block<3,1>(9,2) = e14;
			E.block<3,1>(3,3) = -e23;
			E.block<3,1>(6,3) = e23;
			E.block<3,1>(3,4) = -e24;
			E.block<3,1>(9,4) = e24;
			E.block<3,1>(6,5) = -e34;
			E.block<3,1>(9,5) = e34;


			// create Ebar
			Eigen::MatrixXd Ebar = (E.transpose()*E).inverse() * E.transpose();

			switch(k)
			{
				case 0:
				{
					// populate G
					G = Eigen::MatrixXd::Zero(12,12);
					G.block<6,12>(0,0) = Wf;
					G.block<6,12>(6,0) = Ebar;
					break;
				}
				case 1: 
				{
					// compute I
					Eigen::MatrixXd I = Eigen::MatrixXd::Identity(3,3);

					// populate G
					G = Eigen::MatrixXd::Zero(15,15);
					G.block<6,12>(0,0) = Wf;
					G.block<6,3>(0,12) = Wm;
					G.block<6,12>(6,0) = Ebar;
					G.block<3,3>(12,12) = I;
					break;
				}
				case 2: 
				{
					// compute I
					Eigen::MatrixXd I = Eigen::MatrixXd::Identity(6,6);

					// populate G
					G = Eigen::MatrixXd::Zero(18,18);
					G.block<6,12>(0,0) = Wf;
					G.block<6,6>(0,12) = Wm;
					G.block<6,12>(6,0) = Ebar;
					G.block<6,6>(12,12) = I;
					break;
				}
				case 3: 
				{
					// compute I
					Eigen::MatrixXd I = Eigen::MatrixXd::Identity(9,9);

					// populate G
					G = Eigen::MatrixXd::Zero(21,21);
					G.block<6,12>(0,0) = Wf;
					G.block<6,9>(0,12) = Wm;
					G.block<6,12>(6,0) = Ebar;
					G.block<9,9>(12,12) = I;
					break;
				}
				case 4: 
				{
					// compute I
					Eigen::MatrixXd I = Eigen::MatrixXd::Identity(12,12);

					// populate G
					G = Eigen::MatrixXd::Zero(24,24);
					G.block<6,12>(0,0) = Wf;
					G.block<6,12>(0,12) = Wm;
					G.block<6,12>(6,0) = Ebar;
					G.block<12,12>(12,12) = I;
					break;
				}

				default: 
				throw std::runtime_error("Should not arrive here (number of contact points is 4, number of surface contacts incoherent)\n");

			}
			break;

		}

		default:
		throw std::runtime_error("Should not arrive here (number of contact points is not 2, 3 or 4) \n");

	}

}

void ModelInterface::GraspMatrixAtGeometricCenter(Eigen::MatrixXd& G,
                     Eigen::Matrix3d& R,
                     Eigen::Vector3d& geometric_center,
                     const std::vector<std::string> link_names,
                     const std::vector<Eigen::Vector3d> pos_in_links,
                     const std::vector<ContactNature> contact_natures) const
{
	// number of contact points
	int n = link_names.size();
	if(n < 2)
	{
		throw std::invalid_argument("invalid number of contact points (2 points min)\n");
	}
	if(n > 4)
	{
		throw std::invalid_argument("invalid number of contact points (4 points max)\n");
	}
	if((pos_in_links.size() != n) || (contact_natures.size() != n))
	{
		throw std::invalid_argument("input vectors for the link names, pos in links and contact natures don't have the same size\n");
	}

	geometric_center.setZero();

	for(int i=0; i<n; i++)
	{
		Eigen::Vector3d pi;
		position(pi, link_names[i], pos_in_links[i]);
		geometric_center += pi;
	}
	geometric_center = geometric_center/(double)n;

	GraspMatrix(G, R, link_names, pos_in_links, contact_natures, geometric_center);
}

}

