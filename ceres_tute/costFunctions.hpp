#include <ceres/ceres.h>
#include <ceres/rotation.h>


// Define a struct to hold the distance error. This error specifies that the solution vector
// must be close to the control (initial) vector.
struct DistanceError{

	// Constructor
	DistanceError(double *P) : P_(P) {}

	// The operator method. Evaluates the cost function and computes the jacobians.
	template <typename T>
	bool operator() (const T* const Q, T* residuals) const {
		residuals[0] = T(2)*(Q[0] - T(P_[0]));
		residuals[1] = T(2)*(Q[1] - T(P_[1]));
		residuals[2] = T(2)*(Q[2] - T(P_[2]));
		return true;
	}

	// Control Mesh
	double *P_;


struct ScaleError{

	// Constructor
	ScaleError(double *cube_ref) : cube_(cube) , cube_ref_(cube_ref) {}

	// The operator method. Evaluates the cost function and computes the jacobians.
	template <typename T>
	bool operator() (const T* const scale_factor, T* residuals) const {
		residuals[0] = T(cube_[0])*scale_factor - T(cube_ref_[0]);
		residuals[1] = T(cube_[1])*scale_factor - T(cube_ref_[1]);
		residuals[2] = T(cube_[2])*scale_factor - T(cube_ref_[2]);
		return true;
	}

	// Control Mesh
	double *cube_;
	double *cube_ref_;





struct RotTransError{

	// Constructor
	RotTransError(double *cube_ref) : cube_(cube) , cube_ref_(cube_ref) {}

	// The operator method. Evaluates the cost function and computes the jacobians.
	template <typename T>
	bool operator() (const T* const rot,T* const trans, T* residuals) const {
		
		T temp point[3];
		temp_point[0]=T(cube_[0]);
		temp_point[1]=T(cube_[1]);
		temp_point[2]=T(cube_[2]);


		ceres::AngleAxisRotatePoint()



		residuals[0] = T(cube_[0])*scale_factor - T(cube_ref_[0]);
		residuals[1] = T(cube_[1])*scale_factor - T(cube_ref_[1]);
		residuals[2] = T(cube_[2])*scale_factor - T(cube_ref_[2]);
		return true;
	}

	// Control Mesh
	double *cube_;
	double *cube_ref_;



};

