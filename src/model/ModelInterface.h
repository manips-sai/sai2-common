/*
 * ModelInterface.h
 *
 *  Created on: Dec 13, 2016
 *      Author: Mikael Jorda
 */

#ifndef MODELINTERFACE_H_
#define MODELINTERFACE_H_

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <string>
#include <vector>
#include <cmath>

namespace Model
{

enum ParserType {yml, urdf};

enum ModelType {rbdl, rbdl_kuka};

enum ContactNature {PointContact, SurfaceContact};

//forward define
class ModelInternal;

class ModelInterface
{
public:


    /**
     * @brief creates a model interface object that contains a model of the robot, and functions to compute the jacobians, transformation matrices ...
     * @param path_to_model_file a path to the file containing the model of the robot (urdf and yml files supported)
     * @param model_type The type of the model we want to create (rbdl model supported)
     * @param parser the type of parser to use in function of the file type (urdf or yml)
     * @param verbose to display information about the model vreation in the terminal or not
     */
    ModelInterface (const std::string path_to_model_file,
                    Model::ModelType model_type,
                    Model::ParserType parser,
                    bool verbose = false);

    virtual ~ModelInterface ();

    /**
     * @brief update the model. call after updating joint positions, velocities and acceleration.
     * This will compute all the transformation matrices and the mass matrix, gravity vector, and coriolis vector.
     * The other functions are accessors only.
     * The jacobian functions will compute the jacobians based on the robot configuration when updatedModel was last called
     */
    virtual void updateModel();

protected :
    /**
     * @brief Gives the mass matrix for the last updated configuration
     * @param A matrix to which the mass matrix will be written. Size must be _dof x _dof
     */
    void massMatrix(Eigen::MatrixXd& A);

public :
    /**
     * @brief Gives the joint gravity torques vector for the last updated configuration
     * @param g Vector to which the joint gravity torques will be written
     * @param gravity the 3d gravity vector of the world in base frame
     */
    void gravityVector(Eigen::VectorXd& g,
                       const Eigen::Vector3d& gravity = Eigen::Vector3d(0,0,-9.8));

    /**
     * @brief Gives the joint coriolis and centrifugal forces for the last updated configuration
     * @param b Vector to which the joint coriolis and centrifugal forces will be written
     */
    void coriolisForce(Eigen::VectorXd& b);

    /**
     * @brief Full jacobian for point on link, relative to base (id=0) for the last updated configuration in the form [Jv; Jw]
     * @param J Matrix to which the jacobian will be written
     * @param link_name the name of the link where to compute the jacobian
     * @param pos_in_link the position of the point in the link where the jacobian is computed (in local link frame)
     */
    void J_0(Eigen::MatrixXd& J,
            const std::string& link_name,
            const Eigen::Vector3d& pos_in_link);

    /**
     * @brief Full jacobian for point on link, relative to base (id=0) for the last updated configuration in the form [Jw; Jv]
     * @param J Matrix to which the jacobian will be written
     * @param link_name the name of the link where to compute the jacobian
     * @param pos_in_link the position of the point in the link where the jacobian is computed (in local link frame)
     */
    void J(Eigen::MatrixXd& J,
            const std::string& link_name,
            const Eigen::Vector3d& pos_in_link);

    /**
     * @brief Velocity jacobian for point on link, relative to base (id=0) for the last updated configuration
     * @param J Matrix to which the jacobian will be written
     * @param link_name the name of the link where to compute the jacobian
     * @param pos_in_link the position of the point in the link where the jacobian is computed (in local link frame)
     */
    void Jv(Eigen::MatrixXd& J,
            const std::string& link_name,
            const Eigen::Vector3d& pos_in_link);

    /**
     * @brief Angular velocity jacobian for link, relative to base (id=0) for the last updated configuration
     * @param J Matrix to which the jacobian will be written
     * @param link_name the name of the link where to compute the jacobian
     */
    void Jw(Eigen::MatrixXd& J,
            const std::string& link_name);

    /**
     * @brief transformation from base to link, in base coordinates, for the last updated configuration
     * @param T Transformation matrix to which the result is computed
     * @param link_name name of the link where to compute the transformation matrix
     */
    void transform(Eigen::Affine3d& T,
                   const std::string& link_name);

    /**
     * @brief Position from base to point in link, in base coordinates for the last updated configuration
     * @param pos Vector of position to which the result is written
     * @param link_name name of the link in which is the point where to compute the position
     * @param pos_in_link the position of the point in the link, in local link frame
     */
    void position(Eigen::Vector3d& pos,
                  const std::string& link_name,
                  const Eigen::Vector3d& pos_in_link);

    /**
     * @brief Velocity of point in link, in base coordinates for the last updated configuration
     * @param vel Vector of velocities to which the result is written
     * @param link_name name of the link in which is the point where to compute the velocity
     * @param pos_in_link the position of the point in the link, in local link frame
     */
    void linearVelocity(Eigen::Vector3d& vel,
                  const std::string& link_name,
                  const Eigen::Vector3d& pos_in_link);

    /**
     * @brief Acceleration of point in link, in base coordinates for the last updated configuration
     * @param accel Vector of accelerations to which the result is written
     * @param link_name name of the link in which is the point where to compute the acceleration
     * @param pos_in_link the position of the point in the link, in local link frame
     */
    void linearAcceleration(Eigen::Vector3d& accel,
                      const std::string& link_name,
                      const Eigen::Vector3d& pos_in_link);

    /**
     * @brief Rotation of a link with respect to base frame for the last updated configuration
     * @param rot Rotation matrix to which the result is written
     * @param link_name name of the link for which to compute the rotation
     */
    void rotation(Eigen::Matrix3d& rot,
                  const std::string& link_name);

    /**
     * @brief Angular velocity of a link with respect to base frame for the last updated configuration
     * @param avel Vector to which the result is written
     * @param link_name name of the link for which to compute the angular velocity
     */
    void angularVelocity(Eigen::Vector3d& avel,
                         const std::string& link_name);

    /**
     * @brief Angular acceleration of a link with respect to base frame for the last updated configuration
     * @param aaccel Vector to which the result is written
     * @param link_name name of the link for which to compute the angular acceleration
     */
    void angularAcceleration(Eigen::Vector3d& aaccel,
                             const std::string& link_name);

    /**
     * @brief returns the number of degrees of freedom of the robot
     */
    int dof();

    /**
     * @brief Gives orientation error from rotation matrices
     * @param oerr Vector on which the orientation error will be written
     * @param desired_orientation desired orientation rotation matrix
     * @param current_orientation current orientation matrix
     */
    void orientationError(Eigen::Vector3d& delta_phi,
    		              const Eigen::Matrix3d& desired_orientation,
    		              const Eigen::Matrix3d& current_orientation);


    /**
     * @brief Gives orientation error from quaternions
     * @param oerr Vector on which the orientation error will be written
     * @param desired_orientation desired orientation quaternion
     * @param current_orientation current orientation quaternion
     */
    void orientationError(Eigen::Vector3d& delta_phi,
    		              const Eigen::Quaterniond& desired_orientation,
    		              const Eigen::Quaterniond& current_orientation);

    /**
     * @brief Computes the operational space matrix corresponding to a given Jacobian
     * @param Lambda Matrix on which the operational space mass matrix will be written
     * @param task_jacobian The jacobian of the task for which we want the op space mass matrix
     */
    void taskInertiaMatrix(Eigen::MatrixXd& Lambda,
    					   const Eigen::MatrixXd& task_jacobian);

    /**
     * @brief Computes the operational space matrix robust to singularities
     * @param Lambda Matrix on which the operational space mass matrix will be written
     * @param task_jacobian The jacobian of the task for which we want the op space mass matrix
     */
    void taskInertiaMatrixWithPseudoInv(Eigen::MatrixXd& Lambda,
    					   const Eigen::MatrixXd& task_jacobian);

    /**
     * @brief      Computes the dynamically consistent inverse of the jacobian for a given task. Recomputes the task inertia at each call
     *
     * @param      Jbar           Matrix to which the dynamically consistent inverse will be written
     * @param[in]  task_jacobian  The task jacobian
     */
    void dynConsistentInverseJacobian(Eigen::MatrixXd& Jbar,
    								  const Eigen::MatrixXd& task_jacobian);


    /**
     * @brief      Computes the nullspace matrix for the highest priority task. Recomputes the dynamically consistent inverse and the task mass matrix at each call
     *
     * @param      N              Matrix to which the nullspace matrix will be written
     * @param[in]  task_jacobian  The task jacobian
     */
    void nullspaceMatrix(Eigen::MatrixXd& N,
        					 const Eigen::MatrixXd& task_jacobian);

    /**
     * @brief      Computes the nullspace matrix of the task, consistent with the previous nullspace
     *             Recomputes the dynamically consistent inverse and the task mass matrix at each call   
     *  
     * @param      N              Matrix to which the nullspace matrix will be written
     * @param[in]  task_jacobian  The task jacobian
     * @param[in]  N_prec         The previous nullspace matrix
     */
    void nullspaceMatrix(Eigen::MatrixXd& N,
        					 const Eigen::MatrixXd& task_jacobian,
        					 const Eigen::MatrixXd& N_prec);

    /**
     * @brief      Computes the operational spce matrices (task inertia, dynamically consistent inverse of the jacobian and nullspace) for a given task,
     *             for the first task. More efficient than calling the three individual functions.
     *
     * @param      Lambda         Matrix to which the operational space mass matrix will be written
     * @param      Jbar           Matrix to which the dynamically consistent inverse of the jacobian will be written
     * @param      N              Matrix to which the nullspace matrix will be written
     * @param[in]  task_jacobian  Task jacobian
     */
    void operationalSpaceMatrices(Eigen::MatrixXd& Lambda, Eigen::MatrixXd& Jbar, Eigen::MatrixXd& N,
                                    const Eigen::MatrixXd& task_jacobian);

    /**
     * @brief      Computes the operational spce matrices (task inertia, dynamically consistent inverse of the jacobian and nullspace) for a given task,
     *             In the nullspace of the previous task. More efficient than calling the three individual functions.
     *
     * @param      Lambda         Matrix to which the operational space mass matrix will be written
     * @param      Jbar           Matrix to which the dynamically consistent inverse of the jacobian will be written
     * @param      N              Matrix to which the nullspace matrix will be written
     * @param[in]  task_jacobian  Task jacobian
     * @param[in]  N_prec         Previous nullspace matrix
     */
    void operationalSpaceMatrices(Eigen::MatrixXd& Lambda, Eigen::MatrixXd& Jbar, Eigen::MatrixXd& N,
                                    const Eigen::MatrixXd& task_jacobian,
                                    const Eigen::MatrixXd& N_prec);

   
    /**
     * @brief Computes the grasp matrix in the cases where there are 
     * 2, 3 or 4 contact points.
     * the external forces and moments are assumed to be in world frame
     * for 2 contact points, the output quantities are given in local frame, and the description of the local frame is given by R
     * for 3 and 4 contacts, the output quantities are given in world frame
     * the convention for the output is the following order : support forces, support moments, internal tensions, internal moments
     * the internal tensions are given in the order 1-2, 1-3, 2-3 in the 3 contact case
     * and 1-2, 1-3, 1-4, 2-3, 2-4, 3-4 in the 4 contact case.
     * @param G  :  The grasp matrix that is going to be populated
     * @param R : the rotation matrix between the world frame and the frame attached to the object (useful when 2 contacts only)
     * @param link_names  :  a vector of the names of the links where the contact occur
     * @param pos_in_links  :  a vector of the position of the contact in each link
     * @param contact_natures  :  a vector containing the nature of each contact (we only consider point contact and surface contact)
     * @param center_point  :  The position (in world frame) of the point on which we resolve the resultant forces and moments
     */
    void GraspMatrix(Eigen::MatrixXd& G,
                     Eigen::Matrix3d& R,
                     const std::vector<std::string> link_names,
                     const std::vector<Eigen::Vector3d> pos_in_links,
                     const std::vector<ContactNature> contact_natures,
                     const Eigen::Vector3d center_point);

    /**
     * @brief Computes the grasp matrix in the cases where there are 
     * 2, 3 or 4 contact points.
     * @param G  :  The grasp matrix that is going to be populated
     * @param R : the rotation matrix between the world frame and the frame attached to the object (useful when 2 contacts only)
     * @param geopetric_center  :  The position (in world frame) of the geometric center (found and returned by the function) on which we resolve the resultant forces and moments
     * @param link_names  :  a vector of the names of the links where the contact occur
     * @param pos_in_links  :  a vector of the position of the contact in each link
     * @param contact_natures  :  a vector containing the nature of each contact (we only consider point contact and surface contact)
     */
    void GraspMatrixAtGeometricCenter(Eigen::MatrixXd& G,
                     Eigen::Matrix3d& R,
                     Eigen::Vector3d& geometric_center,
                     const std::vector<std::string> link_names,
                     const std::vector<Eigen::Vector3d> pos_in_links,
                     const std::vector<ContactNature> contact_natures);


    /// \brief pointer to the internal specific model
    ModelInternal* _model_internal;

    ////// Robot State ///////

    /// \brief Joint positions
    Eigen::VectorXd _q;

    /// \brief Joint velocities
    Eigen::VectorXd _dq;

    /// \brief Joint accelerations
    Eigen::VectorXd _ddq;

    /// \brief Mass Matrix
    Eigen::MatrixXd _M;

    /// \brief Inverse of the mass matrix
    Eigen::MatrixXd _M_inv;
>>>>>>> 1d96559c75ae176923839f4b7994902ee27cd218

protected:
	/**
	 * @brief Gives the mass matrix for the last updated configuration
	 * @param A matrix to which the mass matrix will be written. Size must be _dof x _dof
	 */
	void massMatrix(Eigen::MatrixXd& A) const;

public:
	/**
	 * @brief Gives the joint gravity torques vector for the last updated configuration
	 * @param g Vector to which the joint gravity torques will be written
	 * @param gravity the 3d gravity vector of the world in base frame
	 */
	Eigen::MatrixXd gravityVector(const Eigen::Vector3d& gravity = Eigen::Vector3d(0,0,-9.8)) const;
	void gravityVector(Eigen::VectorXd& g,
	                   const Eigen::Vector3d& gravity = Eigen::Vector3d(0,0,-9.8)) const;

	/**
	 * @brief Gives the joint coriolis and centrifugal forces for the last updated configuration
	 * @param b Vector to which the joint coriolis and centrifugal forces will be written
	 */
	Eigen::MatrixXd coriolisForce() const;
	void coriolisForce(Eigen::VectorXd& b) const;

	/**
	 * @brief Full jacobian for point on link, relative to base (id=0) for the last updated configuration in the form [Jv; Jw]
	 * @param J Matrix to which the jacobian will be written
	 * @param link_name the name of the link where to compute the jacobian
	 * @param pos_in_link the position of the point in the link where the jacobian is computed (in local link frame)
	 */
	void J_0(Eigen::MatrixXd& J,
	        const std::string& link_name,
	        const Eigen::Vector3d& pos_in_link) const;

	/**
	 * @brief Full jacobian for point on link, relative to base (id=0) for the last updated configuration in the form [Jw; Jv]
	 * @param J Matrix to which the jacobian will be written
	 * @param link_name the name of the link where to compute the jacobian
	 * @param pos_in_link the position of the point in the link where the jacobian is computed (in local link frame)
	 */
	Eigen::MatrixXd J(const std::string& link_name,
	                  const Eigen::Vector3d& pos_in_link) const;
	void J(Eigen::MatrixXd& J,
	       const std::string& link_name,
	       const Eigen::Vector3d& pos_in_link) const;

	/**
	 * @brief Velocity jacobian for point on link, relative to base (id=0) for the last updated configuration
	 * @param J Matrix to which the jacobian will be written
	 * @param link_name the name of the link where to compute the jacobian
	 * @param pos_in_link the position of the point in the link where the jacobian is computed (in local link frame)
	 */
	Eigen::MatrixXd Jv(const std::string& link_name,
	                   const Eigen::Vector3d& pos_in_link = Eigen::Vector3d::Zero()) const;
	void Jv(Eigen::MatrixXd& J,
	        const std::string& link_name,
	        const Eigen::Vector3d& pos_in_link = Eigen::Vector3d::Zero()) const;

	/**
	 * @brief Angular velocity jacobian for link, relative to base (id=0) for the last updated configuration
	 * @param J Matrix to which the jacobian will be written
	 * @param link_name the name of the link where to compute the jacobian
	 */
	Eigen::MatrixXd Jw(const std::string& link_name) const;
	void Jw(Eigen::MatrixXd& J,
	        const std::string& link_name) const;

	/**
	 * @brief transformation from base to link, in base coordinates, for the last updated configuration
	 * @param T Transformation matrix to which the result is computed
	 * @param link_name name of the link where to compute the transformation matrix
	 */
	Eigen::Affine3d transform(const std::string& link_name) const;
	void transform(Eigen::Affine3d& T,
	               const std::string& link_name) const;

	/**
	 * @brief Position from base to point in link, in base coordinates for the last updated configuration
	 * @param pos Vector of position to which the result is written
	 * @param link_name name of the link in which is the point where to compute the position
	 * @param pos_in_link the position of the point in the link, in local link frame
	 */
	Eigen::Vector3d position(const std::string& link_name,
	                         const Eigen::Vector3d& pos_in_link) const;
	void position(Eigen::Vector3d& pos,
	              const std::string& link_name,
	              const Eigen::Vector3d& pos_in_link) const;

	/**
	 * @brief Velocity of point in link, in base coordinates for the last updated configuration
	 * @param vel Vector of velocities to which the result is written
	 * @param link_name name of the link in which is the point where to compute the velocity
	 * @param pos_in_link the position of the point in the link, in local link frame
	 */
	Eigen::Vector3d linearVelocity(const std::string& link_name,
	                               const Eigen::Vector3d& pos_in_link) const;
	void linearVelocity(Eigen::Vector3d& vel,
	                    const std::string& link_name,
	                    const Eigen::Vector3d& pos_in_link) const;

	/**
	 * @brief Acceleration of point in link, in base coordinates for the last updated configuration
	 * @param accel Vector of accelerations to which the result is written
	 * @param link_name name of the link in which is the point where to compute the acceleration
	 * @param pos_in_link the position of the point in the link, in local link frame
	 */
	Eigen::Vector3d linearAcceleration(const std::string& link_name,
	                                   const Eigen::Vector3d& pos_in_link) const;
	void linearAcceleration(Eigen::Vector3d& accel,
	                  const std::string& link_name,
	                  const Eigen::Vector3d& pos_in_link) const;

	/**
	 * @brief Rotation of a link with respect to base frame for the last updated configuration
	 * @param rot Rotation matrix to which the result is written
	 * @param link_name name of the link for which to compute the rotation
	 */
	Eigen::Matrix3d rotation(const std::string& link_name) const;
	void rotation(Eigen::Matrix3d& rot,
	              const std::string& link_name) const;

	/**
	 * @brief Angular velocity of a link with respect to base frame for the last updated configuration
	 * @param avel Vector to which the result is written
	 * @param link_name name of the link for which to compute the angular velocity
	 */
	Eigen::Vector3d angularVelocity(const std::string& link_name) const;
	void angularVelocity(Eigen::Vector3d& avel,
	                     const std::string& link_name) const;

	/**
	 * @brief Angular acceleration of a link with respect to base frame for the last updated configuration
	 * @param aaccel Vector to which the result is written
	 * @param link_name name of the link for which to compute the angular acceleration
	 */
	Eigen::Vector3d angularAcceleration(const std::string& link_name) const;
	void angularAcceleration(Eigen::Vector3d& aaccel,
                             const std::string& link_name) const;

	/**
	 * @brief returns the number of degrees of freedom of the robot
	 */
	int dof() const;

	/**
	 * @brief Gives orientation error from rotation matrices
	 * @param oerr Vector on which the orientation error will be written
	 * @param desired_orientation desired orientation rotation matrix
	 * @param current_orientation current orientation matrix
	 */
	Eigen::Vector3d orientationError(const Eigen::Matrix3d& desired_orientation,
	                                 const Eigen::Matrix3d& current_orientation) const;
	void orientationError(Eigen::Vector3d& delta_phi,
	                      const Eigen::Matrix3d& desired_orientation,
	                      const Eigen::Matrix3d& current_orientation) const;

	/**
	 * @brief Gives orientation error from quaternions
	 * @param oerr Vector on which the orientation error will be written
	 * @param desired_orientation desired orientation quaternion
	 * @param current_orientation current orientation quaternion
	 */
	Eigen::Vector3d orientationError(const Eigen::Quaterniond& desired_orientation,
	                                 const Eigen::Quaterniond& current_orientation) const;
	void orientationError(Eigen::Vector3d& delta_phi,
	                      const Eigen::Quaterniond& desired_orientation,
	                      const Eigen::Quaterniond& current_orientation) const;

	/**
	 * @brief Computes the operational space matrix corresponding to a given Jacobian
	 * @param Lambda Matrix on which the operational space mass matrix will be written
	 * @param task_jacobian The jacobian of the task for which we want the op space mass matrix
	 */
	Eigen::MatrixXd taskInertiaMatrix(const Eigen::MatrixXd& task_jacobian) const;
	void taskInertiaMatrix(Eigen::MatrixXd& Lambda,
	                       const Eigen::MatrixXd& task_jacobian) const;

	/**
	 * @brief Computes the operational space matrix robust to singularities
	 * @param Lambda Matrix on which the operational space mass matrix will be written
	 * @param task_jacobian The jacobian of the task for which we want the op space mass matrix
	 */
	Eigen::MatrixXd taskInertiaMatrixWithPseudoInv(const Eigen::MatrixXd& task_jacobian) const;
	void taskInertiaMatrixWithPseudoInv(Eigen::MatrixXd& Lambda,
	                                    const Eigen::MatrixXd& task_jacobian) const;

	/**
	 * @brief      Computes the dynamically consistent inverse of the jacobian for a given task. Recomputes the task inertia at each call
	 *
	 * @param      Jbar           Matrix to which the dynamically consistent inverse will be written
	 * @param[in]  task_jacobian  The task jacobian
	 */
	Eigen::MatrixXd dynConsistentInverseJacobian(const Eigen::MatrixXd& task_jacobian) const;
	void dynConsistentInverseJacobian(Eigen::MatrixXd& Jbar,
	                                  const Eigen::MatrixXd& task_jacobian) const;


	/**
	 * @brief      Computes the nullspace matrix for the highest priority task. Recomputes the dynamically consistent inverse and the task mass matrix at each call
	 *
	 * @param      N              Matrix to which the nullspace matrix will be written
	 * @param[in]  task_jacobian  The task jacobian
	 */
	Eigen::MatrixXd nullspaceMatrix(const Eigen::MatrixXd& task_jacobian) const;
	// void nullspaceMatrix(Eigen::MatrixXd& N,
	//                      const Eigen::MatrixXd& task_jacobian) const;

	/**
	 * @brief      Computes the nullspace matrix of the task, consistent with the previous nullspace
	 *             Recomputes the dynamically consistent inverse and the task mass matrix at each call   
	 *  
	 * @param      N              Matrix to which the nullspace matrix will be written
	 * @param[in]  task_jacobian  The task jacobian
	 * @param[in]  N_prec         The previous nullspace matrix
	 */
	Eigen::MatrixXd nullspaceMatrix(const Eigen::MatrixXd& task_jacobian,
	                                const Eigen::MatrixXd& N_prec) const;
	void nullspaceMatrix(Eigen::MatrixXd& N,
	                     const Eigen::MatrixXd& task_jacobian,
	                     const Eigen::MatrixXd& N_prec) const;

	/**
	 * @brief      Computes the operational spce matrices (task inertia, dynamically consistent inverse of the jacobian and nullspace) for a given task,
	 *             for the first task. More efficient than calling the three individual functions.
	 *
	 * @param      Lambda         Matrix to which the operational space mass matrix will be written
	 * @param      Jbar           Matrix to which the dynamically consistent inverse of the jacobian will be written
	 * @param      N              Matrix to which the nullspace matrix will be written
	 * @param[in]  task_jacobian  Task jacobian
	 */
	void operationalSpaceMatrices(Eigen::MatrixXd& Lambda,
	                              Eigen::MatrixXd& Jbar,
	                              Eigen::MatrixXd& N,
	                              const Eigen::MatrixXd& task_jacobian) const;

	/**
	 * @brief      Computes the operational spce matrices (task inertia, dynamically consistent inverse of the jacobian and nullspace) for a given task,
	 *             In the nullspace of the previous task. More efficient than calling the three individual functions.
	 *
	 * @param      Lambda         Matrix to which the operational space mass matrix will be written
	 * @param      Jbar           Matrix to which the dynamically consistent inverse of the jacobian will be written
	 * @param      N              Matrix to which the nullspace matrix will be written
	 * @param[in]  task_jacobian  Task jacobian
	 * @param[in]  N_prec         Previous nullspace matrix
	 */
	void operationalSpaceMatrices(Eigen::MatrixXd& Lambda,
	                              Eigen::MatrixXd& Jbar,
	                              Eigen::MatrixXd& N,
	                              const Eigen::MatrixXd& task_jacobian,
	                              const Eigen::MatrixXd& N_prec) const;


	/**
	 * @brief Computes the grasp matrix in the cases where there are 
	 * 2, 3 or 4 contact points.
	 * the external forces and moments are assumed to be in world frame
	 * for 2 contact points, the output quantities are given in local frame, and the description of the local frame is given by R
	 * for 3 and 4 contacts, the output quantities are given in world frame
	 * the convention for the output is the following order : support forces, support moments, internal tensions, internal moments
	 * the internal tensions are given in the order 1-2, 1-3, 2-3 in the 3 contact case
	 * and 1-2, 1-3, 1-4, 2-3, 2-4, 3-4 in the 4 contact case.
	 * @param G  :  The grasp matrix that is going to be populated
	 * @param R : the rotation matrix between the world frame and the frame attached to the object (useful when 2 contacts only)
	 * @param link_names  :  a vector of the names of the links where the contact occur
	 * @param pos_in_links  :  a vector of the position of the contact in each link
	 * @param contact_natures  :  a vector containing the nature of each contact (we only consider point contact and surface contact)
	 * @param center_point  :  The position (in world frame) of the point on which we resolve the resultant forces and moments
	 */
	void GraspMatrix(Eigen::MatrixXd& G,
	                 Eigen::Matrix3d& R,
	                 const std::vector<std::string> link_names,
	                 const std::vector<Eigen::Vector3d> pos_in_links,
	                 const std::vector<ContactNature> contact_natures,
	                 const Eigen::Vector3d center_point) const;

	/**
	 * @brief Computes the grasp matrix in the cases where there are 
	 * 2, 3 or 4 contact points.
	 * @param G  :  The grasp matrix that is going to be populated
	 * @param R : the rotation matrix between the world frame and the frame attached to the object (useful when 2 contacts only)
	 * @param geopetric_center  :  The position (in world frame) of the geometric center (found and returned by the function) on which we resolve the resultant forces and moments
	 * @param link_names  :  a vector of the names of the links where the contact occur
	 * @param pos_in_links  :  a vector of the position of the contact in each link
	 * @param contact_natures  :  a vector containing the nature of each contact (we only consider point contact and surface contact)
	 */
	void GraspMatrixAtGeometricCenter(Eigen::MatrixXd& G,
	                 Eigen::Matrix3d& R,
	                 Eigen::Vector3d& geometric_center,
	                 const std::vector<std::string> link_names,
	                 const std::vector<Eigen::Vector3d> pos_in_links,
	                 const std::vector<ContactNature> contact_natures) const;


	/// \brief pointer to the internal specific model
	ModelInternal* _model_internal;

	////// Robot State ///////

	/// \brief Joint positions
	Eigen::VectorXd _q;

	/// \brief Joint velocities
	Eigen::VectorXd _dq;

	/// \brief Joint accelerations
	Eigen::VectorXd _ddq;

	/// \brief Mass Matrix
	Eigen::MatrixXd _M;

	/// \brief Inverse of the mass matrix
	Eigen::MatrixXd _M_inv;

protected:
	/// \brief compute the cross product operator of a 3d vector
	static Eigen::Matrix3d CrossProductOperator(const Eigen::Vector3d& v) {
		Eigen::Matrix3d v_hat;
		v_hat <<  0,    -v(2),  v(1),
			      v(2),  0,    -v(0),
			     -v(1),  v(0),  0   ;
		return v_hat;
	}


};



}
#endif /* MODELINTERFACE_H_ */
