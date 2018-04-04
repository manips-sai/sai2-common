// ButterworthFilter: Implements a digital second order Butterworth filter for an Eigen::VectorXd type object 

#include "ButterworthFilter.h"

ButterworthFilter::ButterworthFilter(const int input_size)
{
	_filter_order = 2;
	_state_size = input_size;

	_raw_buffer.setZero(_state_size, _filter_order + 1);
	_filtered_buffer.setZero(_state_size, _filter_order);

	_raw_coeff.setZero(_filter_order + 1);
	_filtered_coeff.setZero(_filter_order);
}

ButterworthFilter::ButterworthFilter(const int input_size, const double normalized_cutoff)
{
	_filter_order = 2;
	_state_size = input_size;

	_raw_buffer.setZero(_state_size, _filter_order + 1);
	_filtered_buffer.setZero(_state_size, _filter_order);

	_raw_coeff.setZero(_filter_order + 1);
	_filtered_coeff.setZero(_filter_order);

	setCutoffFrequency(normalized_cutoff);
}

ButterworthFilter::ButterworthFilter(const int input_size, const double sampling_rate, const double cutoff_freq)
{
	_filter_order = 2;
	_state_size = input_size;

	_raw_buffer.setZero(_state_size, _filter_order + 1);
	_filtered_buffer.setZero(_state_size, _filter_order);

	_raw_coeff.setZero(_filter_order + 1);
	_filtered_coeff.setZero(_filter_order);

	setCutoffFrequency(sampling_rate, cutoff_freq);
}

// default initialize to zero values
void ButterworthFilter::initializeFilter(const Eigen::VectorXd& x)
{
	for(int i=0; i<_filter_order; i++)
	{
		_raw_buffer.col(i) = x;
		_filtered_buffer.col(i) = x;
	}
}

void ButterworthFilter::setCutoffFrequency(const double fc)
{
	if (fc >= 0.5){ throw std::runtime_error("ButterworthFilter. normalized cutoff frequency should be between 0 and 0.5\n"); }
	if (fc < 0.0){ throw std::runtime_error("ButterworthFilter. normalized cutoff frequency should be between 0 and 0.5\n"); }

	_normalized_cutoff = fc;
	_pre_warp_cutoff = tan(M_PI*fc);

	double gain = (1/(_pre_warp_cutoff*_pre_warp_cutoff) + sqrt(2)/_pre_warp_cutoff + 1);

	const double ita =1.0/ tan(M_PI*fc);
	const double q=sqrt(2.0);

	_raw_coeff << 1, 2, 1;
	_raw_coeff /= gain;

	_filtered_coeff << 2-2/(_pre_warp_cutoff*_pre_warp_cutoff), 1/(_pre_warp_cutoff*_pre_warp_cutoff) - sqrt(2)/_pre_warp_cutoff + 1;
	_filtered_coeff /= gain;
}

void ButterworthFilter::setCutoffFrequency(const double sampling_rate, const double cutoff_freq)
{
	setCutoffFrequency(cutoff_freq/sampling_rate);
}

/** \brief Update the filter
 * \param x State vector. Each element is filtered individually. */
Eigen::VectorXd ButterworthFilter::update(const Eigen::VectorXd& x)
{
	for(int i = _filter_order; i>0; i--)
	{
		_raw_buffer.col(i) = _raw_buffer.col(i-1);
	}
	_raw_buffer.col(0) = x;

	Eigen::VectorXd y = _raw_buffer*_raw_coeff - _filtered_buffer*_filtered_coeff;

	for(int i = _filter_order-1; i>0; i--)
	{
		_filtered_buffer.col(i) = _filtered_buffer.col(i-1);
	}
	_filtered_buffer.col(0) = y;

	return y;
}

