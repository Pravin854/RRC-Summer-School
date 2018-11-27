#include <ceres/ceres.h>
#include <ceres/rotation.h>


class TranslationProblem{

public:	

	// Get a pointer to the array containing points on the cube
	double *getCubePoints() { return cubePoints_; }
	// Get a pointer to the array containing points on the reference cube
	double *getReferenceCubePoints() { return referenceCubePoints_; }

	// Read data from input files
	bool loadFile(const char *fileName, const char *fileNameRef){

		// For this example, there are 56 points on the cube
		int numCubePts_ = 56;

		FILE *fptr = fopen(fileName, "r");
		if(fptr == NULL){
			return false;
		}


		// Initialize the cubePoints_ array
		cubePoints_ = new double[3*numCubePts_];

		// Read in cube points from text file
		for(int i = 0; i < 3*numCubePts_; ++i){
			fscanfOrDie(fptr, "%lf", cubePoints_ + i);
		}

		FILE *fptrRef = fopen(fileNameRef, "r");
		if(fptrRef == NULL){
			return false;
		}

		// Initialize the cubePoints_ array
		referenceCubePoints_ = new double[3*numCubePts_];

		// Read in cube points from text file
		for(int i = 0; i < 3*numCubePts_; ++i){
			fscanfOrDie(fptrRef, "%lf", referenceCubePoints_ + i);
		}

		return true;
	}

private:

	// Helper function to read in one value to a text file
	template <typename T>
	void fscanfOrDie(FILE *fptr, const char *format, T *value){
		int numScanned = fscanf(fptr, format, value);
		if(numScanned != 1){
			LOG(FATAL) << "Invalid data file";
		}
	}

	// Variable that stores the transformed cube
	double *cubePoints_;
	// Variable that stores the reference cube
	double *referenceCubePoints_;

};
