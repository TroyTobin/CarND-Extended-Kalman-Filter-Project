#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools()
{
	;
}

Tools::~Tools()
{
	;
}

/**
 * @brief Calculates the Root-Mean-Square Error
 *        Adapted from the Udacity lecture "Evaluating KF Performance 2"
 *
 * @param estimations x,y position and velocity in the x/y axis estimates
 * @param ground_truth the actual position and velocities
 * @return root-mean-square errors (4 elements) of the position and velocity elements
 */
VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
							  const vector<VectorXd> &ground_truth) 
{
	// Create the RMSE output vector and initialize
	uint32_t i;
	VectorXd rmse(4);

	rmse << 0,0,0,0;

	// Sanity check the inputs that we have the same number of 
	// estimations as we do reference data
	if (estimations.size() != ground_truth.size())
	{
		cerr << "CalculateRMSE: estimate and ground_truth size mismatch ";
		cerr << "(" << estimations.size() << ", " << ground_truth.size() << ")" << endl;
		goto Error;
	}

	// if there are no estimations nothing to do
	if (estimations.size() == 0)
	{
		cerr << "CalculateRMSE: No estimations ";
		cerr << "(" << estimations.size() << ")" << endl;
		goto Error;
	}

	// Sum the squared residuals for each sample
	for(i = 0; i < estimations.size(); i++)
	{
		// Sanity check the the samples are the correct sizes
		VectorXd est_sample = estimations[i];
		VectorXd gt_sample  = ground_truth[i];

		if ((est_sample.size() != 4) ||
			(gt_sample.size()  != 4))
		{
			cerr << "CalculateRMSE: Invalid estimate and ground_truth sample size ";
			cerr << "(" << est_sample.size() << ", " << gt_sample.size() << ") != " << 4 << endl;
			goto Error;			
		}

		VectorXd residual = est_sample - gt_sample;

		//coefficient-wise multiplication
		residual = residual.array()*residual.array();
		rmse += residual;
	}

	//calculate the mean
	rmse = rmse/estimations.size();

	//calculate the squared root
	rmse = rmse.array().sqrt();

Error:
	//return the result
	return rmse;
}


/**
 * @brief Calculate the Jacobian Matrix
 *        Adapted from the Udacity lecture "Jacobian Matrix Part 2"
 * 
 * @param state the current position and velocity state
 * @return The Jacobian Matrix
 */
MatrixXd Tools::CalculateJacobian(const VectorXd& state)
{

	float position_x, position_y, velocity_x, velocity_y;
	float c1, c2, c3, c4, c5;

	// Jacobian Matrix
	MatrixXd Hj(3, 4);

	// Sanity check the x_state input
	if (state.size() != 4)
	{
		cerr << "CalculateRMSE: Invalid state size ";
		cerr << state.size() << " != " << 4 << endl;
		goto Error;
	}

	// Extract the Position and Velocity state
	position_x = state(0);
	position_y = state(1);
	velocity_x = state(2);
	velocity_y = state(3);

	// Compute some values useful in the Jacobian matrix
	c1 = (position_x*position_x) + (position_y*position_y);
	c2 = sqrt(c1);
	c3 = (c1*c2);
	c4 = (velocity_x*position_y) - (velocity_y*position_x);
	c5 = (position_x*velocity_y) - (position_y*velocity_x);

	// Check division by zero
	if(fabs(c1) < numeric_limits<float>::epsilon())
	{
		cerr << "CalculateJacobian () - Error - Division by Zero" << endl;
		goto Error;
	}

	// Compute the Jacobian matrix
	Hj << (position_x/c2),    (position_y/c2),    0,     		 0,
	      -(position_y/c1),   (position_x/c1),    0,     		 0,
	      (position_y*c4)/c3, (position_x*c5)/c3, position_x/c2, position_y/c2;

Error:
	return Hj;
}
