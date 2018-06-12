// ButterworthFilter: Implements a digital second order Butterworth filter for an Eigen::VectorXd type object 

#ifndef SAI2_COMMON_BUTTERWORTH_FILTER_H_
#define SAI2_COMMON_BUTTERWORTH_FILTER_H_

#include <math.h>
#include <stdexcept>
#include <iostream>
#include <Eigen/Dense>

class ButterworthFilter
{
public:
	/** \brief Constructor. Call setDimension before using */
	// ButterworthFilter(){}

	ButterworthFilter(const int input_size);

	ButterworthFilter(const int input_size, const double normalized_cutoff);

	ButterworthFilter(const int input_size, const double sampling_rate, const double cutoff_freq);

	// default initialize to zero values
	void initializeFilter(const Eigen::VectorXd& x);

	void setCutoffFrequency(const double fc);

	void setCutoffFrequency(const double sampling_rate, const double cutoff_freq);

	/** \brief Update the filter
	 * \param x State vector. Each element is filtered individually. */
	Eigen::VectorXd update(const Eigen::VectorXd& x);

// private:
	unsigned int _state_size = 0;

	// each column is a sample. Most recent sample at the left
	Eigen::MatrixXd _raw_buffer;
	Eigen::MatrixXd _filtered_buffer;

	// coeff associated with the most recent sample at the top
	Eigen::VectorXd _raw_coeff;
	Eigen::VectorXd _filtered_coeff;

	double _normalized_cutoff;
	double _pre_warp_cutoff;

	double _filter_order;
};


#endif //SAI2_COMMON_BUTTERWORTH_FILTER_H_