/*
 * Pose adjustmer using keypoint likelihoods from a single image
 * Author: Krishna Murthy
*/

#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>

#include <ceres/loss_function.h>
#include <ceres/iteration_callback.h>
#include <ceres/rotation.h>

// Contains definitions of various problem structs
#include "problemStructs.hpp"
// Contains various cost function struct specifications
#include "costFunctions.hpp"


int main(int argc, char** argv){

	google::InitGoogleLogging(argv[0]);

	const char *cubeFileName;
	const char *referenceCubeFileName;
	const char *outputFileName;

	cubeFileName = "../data/cube.txt";
	referenceCubeFileName = "../data/cube_shifted.txt";
	outputFileName = "ceres_output.txt";

	// Create a 'SingleViewPoseAdjustmentProblem' instance to hold the BA problem
	TranslationProblem myProblem;
	// Read the problem file and store relevant information
	if(!myProblem.loadFile(cubeFileName, referenceCubeFileName)){
		std::cerr << "ERROR: Unable to open file " << cubeFileName << " or " << referenceCubeFileName << std::endl;
		return 1;
	}


	ceres::Problem problem;

	for (int i ; i<56 ; i++)
	{
		ceres::CostFunction *scaleE = new ceres::AutoDiffCostFunction<ScaleError,3 , 1>(
			new ScaleError(myProblem.getCubePoints() + 3*i, myProblem.getReferenceCubePoints()+ 3*i));

		problem.AndResidualBlock(ScaleE,NULL.&scale_p);
	}	

	ceres::Solver::Options options;
	options.linear_solver_type = ceres::DENSE_SCHUR;
	options.preconditioner_type = ceres::JACOBI;
	options.use_linear_iterations = true ;
	options.minimizer_progress_to_stdout = false ;


	ceres::Solver::Summary summary;
	ceres::Solver(options.&problem,&summary);


	std::cout << scale_p << std::endl;

	// double *array_ref, *array;
	// array = myProblem.getCubePoints();
	// array_ref = myProblem.getReferenceCubePoints();

	// for(int i = 0;i <  56*3; i++ )

	// {
	// 	std::cout<<array[i]<<" "<<array_ref[i]<<std::endl;
	// }

 	return 0;

}
