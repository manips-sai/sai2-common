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

ModelInterface::ModelInterface (const std::string path_to_model_file, Model::ModelType model_type, Model::ParserType parser, bool verbose)
{

    // Create the desired model
    switch(model_type)
    {
	case rbdl :
	    _model_internal = new RBDLModel(path_to_model_file, parser, verbose);
	    break;

    case rbdl_kuka : 
    #ifdef USE_KUKA_LBR_DYNAMICS
    	_model_internal = new KukaRBDLModel(path_to_model_file, parser, verbose);
    	break;
	#endif
    	throw (std::runtime_error ("Compile sai2-common using KUKA_LBR_DYNAMICS option in order to use the kuka_rbdl model"));

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

void ModelInterface::J_0(Eigen::MatrixXd& J,
                       const std::string& link_name,
                       const Eigen::Vector3d& pos_in_link)
{_model_internal->J_0(J,link_name,pos_in_link,_q);}

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

void ModelInterface::orientationError(Eigen::Vector3d& delta_phi,
		              const Eigen::Quaterniond& desired_orientation,
		              const Eigen::Quaterniond& current_orientation)
{
	Eigen::Quaterniond inv_dlambda = desired_orientation*current_orientation.conjugate();
	delta_phi = 2.0*inv_dlambda.vec();
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

// TODO : Untested
void ModelInterface::taskInertiaMatrixWithPseudoInv(Eigen::MatrixXd& Lambda,
    					   const Eigen::MatrixXd& task_jacobian)
{
	// check matrices have the right size
	if (Lambda.rows() != Lambda.cols())
	{
		throw std::invalid_argument("Lambda matrix not square in ModelInterface::taskInertiaMatrixWithPseudoInv");
		return;
	}
	else if (Lambda.rows() != task_jacobian.rows())
	{
		throw std::invalid_argument("Rows of Jacobian inconsistent with size of Lambda matrix in ModelInterface::taskInertiaMatrixWithPseudoInv");
		return;
	}
	else if (task_jacobian.cols() != _model_internal->_dof)
	{
		throw std::invalid_argument("Jacobian size inconsistent with DOF of robot model in ModelInterface::taskInertiaMatrixWithPseudoInv");
		return;
	}

	// compute task inertia
	Eigen::MatrixXd inv_inertia = task_jacobian*_M_inv*task_jacobian.transpose();

	// compute SVD pseudoinverse
	// TODO: make class function?
	Eigen::JacobiSVD<Eigen::MatrixXd> svd(inv_inertia, Eigen::ComputeThinU | Eigen::ComputeThinV);
	const double epsilon = std::numeric_limits<double>::epsilon();
	double tolerance = epsilon * std::max(inv_inertia.cols(), inv_inertia.rows()) * svd.singularValues().array().abs()(0);
	Lambda = svd.matrixV() * (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
}

//TODO : Untested
void ModelInterface::dynConsistentInverseJacobian(Eigen::MatrixXd& Jbar,
									const Eigen::MatrixXd& task_jacobian)
{
	// check matrices have the right size
	if(Jbar.rows() != task_jacobian.cols() || Jbar.cols() != task_jacobian.rows())
	{
		throw std::invalid_argument("Matrix dimmensions inconsistent in ModelInterface::dynConsistentInverseJacobian");
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
        					 const Eigen::MatrixXd& task_jacobian)
{
	Eigen::MatrixXd N_prec = Eigen::MatrixXd::Identity(dof(),dof());
	nullspaceMatrix(N,task_jacobian,N_prec);
}

//TODO :: Untested
void ModelInterface::nullspaceMatrix(Eigen::MatrixXd& N,
    					 const Eigen::MatrixXd& task_jacobian,
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
	else if(task_jacobian.cols() != N.rows())
	{
		throw std::invalid_argument("jacobian matrix dimensions inconsistent with model dof in ModelInterface::nullspaceMatrix");
		return;
	}
	// Compute N
	else
	{
		Eigen::MatrixXd Jbar = Eigen::MatrixXd::Zero(task_jacobian.cols(),task_jacobian.rows());
		dynConsistentInverseJacobian(Jbar,task_jacobian);
		Eigen::MatrixXd Ni = Eigen::MatrixXd::Identity(N.rows(),N.cols());
		Ni = Ni - Jbar*task_jacobian;
		N = Ni*N_prec;
	}
}

// TODO : Untested
void ModelInterface::operationalSpaceMatrices(Eigen::MatrixXd& Lambda, Eigen::MatrixXd& Jbar, Eigen::MatrixXd& N,
                                    const Eigen::MatrixXd& task_jacobian)
{
	Eigen::MatrixXd N_prec = Eigen::MatrixXd::Identity(dof(),dof());
	operationalSpaceMatrices(Lambda,Jbar,N,task_jacobian,N_prec);
}

// TODO : Untested
void ModelInterface::operationalSpaceMatrices(Eigen::MatrixXd& Lambda, Eigen::MatrixXd& Jbar, Eigen::MatrixXd& N,
                                    const Eigen::MatrixXd& task_jacobian,
                                    const Eigen::MatrixXd& N_prec)
{
	// check matrices have the right size
	if(Lambda.rows() != Lambda.cols())
	{
		throw std::invalid_argument("Lambda matrix not square in ModelInterface::operationalSpaceMatrices");
		return;
	}
	else if (Lambda.rows() != task_jacobian.rows())
	{
		throw std::invalid_argument("Rows of Jacobian inconsistent with size of Lambda matrix in ModelInterface::operationalSpaceMatrices");
		return;
	}
	else if(task_jacobian.cols() != _model_internal->_dof)
	{
		throw std::invalid_argument("Jacobian size inconsistent with DOF of robot model in ModelInterface::operationalSpaceMatrices");
		return;
	}
	else if(Jbar.rows() != task_jacobian.cols() || Jbar.cols() != task_jacobian.rows())
	{
		throw std::invalid_argument("Matrix dimmensions inconsistent in ModelInterface::operationalSpaceMatrices");
		return;
	}
	else if(N.rows() != N.cols() || N.rows() != _model_internal->_dof)
	{
		throw std::invalid_argument("N matrix dimensions inconsistent in ModelInterface::operationalSpaceMatrices");
		return;
	}
	else if(N_prec.rows() != N_prec.cols() || N_prec.rows() != _model_internal->_dof)
	{
		throw std::invalid_argument("N_prec matrix dimensions inconsistent in ModelInterface::operationalSpaceMatrices");
		return;
	}
	else if(task_jacobian.cols() != N.rows())
	{
		throw std::invalid_argument("jacobian matrix dimensions inconsistent with model dof in ModelInterface::operationalSpaceMatrices");
		return;
	}
	// Compute the matrices
	else
	{
		Eigen::MatrixXd inv_inertia = task_jacobian*_M_inv*task_jacobian.transpose();
		Lambda = inv_inertia.inverse();
		Jbar = _M_inv*task_jacobian.transpose()*Lambda;
		Eigen::MatrixXd Ni = Eigen::MatrixXd::Identity(N.rows(),N.cols());
		Ni = Ni - Jbar*task_jacobian;
		N = Ni*N_prec;
	}
}

void ModelInterface::GraspMatrix(Eigen::MatrixXd& G,
	Eigen::Matrix3d& R,
	const std::vector<std::string> link_names,
	const std::vector<Eigen::Vector3d> pos_in_links,
	const std::vector<ContactNature> contact_natures,
	const Eigen::Vector3d center_point)
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
			if(abs(x(0)-1) < 1e-3) // new x is aligned with world x
			{
				R = Eigen::Matrix3d::Identity();
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
                     const std::vector<ContactNature> contact_natures)
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

