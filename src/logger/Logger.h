#ifndef WLOGGER_H
#define WLOGGER_H

#include <fstream>
#include <unistd.h>
#include <chrono>
using std::chrono::system_clock;
using std::chrono::milliseconds;
using std::chrono::microseconds;
#include <Eigen/Dense>
#include <thread>
#include <vector>

namespace Logging {

// Log formatter
// TODO: allow user defined log formatter
Eigen::IOFormat logVecFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ");

// interface class
class IEigenVector {
public:
	virtual void print(std::ostream& os) = 0;
};

// template class to encapsulate matrix pointer
template <typename Derived>
class EigenVector: public IEigenVector {
public:
	// ctor
	EigenVector(Eigen::MatrixBase <Derived>* data) { _data = data;}
	// data to encapsulate
	Eigen::MatrixBase <Derived> * _data;
	// implementation of pure virtual function in interface class
	void print (std::ostream& os) {
		 os << _data->transpose().format(logVecFmt);
	}
};

// Logger class
/*
------------
Basic usage:
------------
```
Eigen::Vector3d my_signal;
Logger log(100000, "log.csv");
log.addVectorToLog(my_signal, "Signal");
// ^Signal_0, Signal_1 and Signal_2 are added to the header row of the log as channel names
logger.start(); // this starts the logger in a different thread

bool experiment_is_running = true; // set to false by Ctrl-C handler or UI or redis or timer
while(experiment_is_running) {
	// update my_signal
}
if(logger._f_is_logging)
	logger.stop();
```

----------------------
File rotation example:
----------------------
```
Logger log(100000); //global within application

void redisFileSetCallback(string log_file_name) {
	log.newFileStart(log_file_name); // saves the current file and starts a new one
}

int main() {
	Eigen::Vector3d my_signal1;
	Eigen::Vector3d my_signal2;
	log.addVectorToLog(my_signal1, "Signal1");
	log.addVectorToLog(my_signal2, "Signal2");
	while(experiment_is_running) {
		// update my_signals
	}
	if(logger._f_is_logging)
		logger.stop();
    ...
}
```

------
Notes:
------
- This class is meant to be a high speed signal logger with low computational impact
	on the application thread. The logger writes to file on a separate thread.
- Variables to be logged are registered before the logger starts. The variables have to
	persist across the lifetime of the logger, otherwise a segfault will occur.
- Currently, only Eigen::Vector variables are supported.
- Time since start is automatically logged as the first column of the output file.
- Logging interval is specified in microseconds.
- Output is a csv file with following format:

timestamp,	 Signal1_0,	 Signal1_1,  Signal2_0,  Signal2_1
0,				5.0,		5.0,		5.0,		5.0
100000,			5.0,		5.0,		5.0,		5.0
200000,			5.0,		5.0,		5.0,		5.0
...
*/
class Logger {
public:
	// ctor
	Logger(long interval, std::string fname)
	: _log_interval_(interval), _logname(fname), _f_is_logging(false), _max_log_time_us(0), _num_vars_to_log(0)
	{
		// create log file
		_logfile.open(fname, std::ios::out);
		_header = "timestamp, ";
	}

	// ctor without creating a log file
	Logger(long interval)
	: _log_interval_(interval), _logname(""), _f_is_logging(false), _max_log_time_us(0), _num_vars_to_log(0)
	{
		_header = "timestamp, ";
	}

	// add Eigen vector type variable to watch
	template <typename Derived>
	bool addVectorToLog (Eigen::MatrixBase <Derived>* var, const std::string& var_name = "") {
		if (_f_is_logging) {
			return false;
		}
		auto e = new EigenVector<Derived>(var);
		_vars_to_log.push_back(dynamic_cast<IEigenVector* >(e));
		_num_vars_to_log += var->size();
		for (uint i = 0; i < var->size(); i++) {
			if (!var_name.empty()) {
				_header += var_name + "_" + std::to_string(i) + ", ";
			} else {
				_header += "var" + std::to_string(_vars_to_log.size()) + "_" + std::to_string(i) + ", ";
			}
		}
		return true;
	}

	bool newFileStart(std::string fname) {
		// do not overwrite old file
		if(fname.compare(_logname) == 0) {
            std::cerr << "Log file name requested matches existing file. Disregarding request." << std::endl;
			return false;
		}
		if(_f_is_logging) {
			stop();
		}
		_logfile.open(fname, std::ios::out);
		fname = _logname;
		return start();
	}

	// start logging
	bool start() {
		// save start time
		_t_start = system_clock::now();

		// set logging to true
		_f_is_logging = true;

		// complete header line
		_logfile << _header << "\n";

		// calculate max log time to keep log under 2GB
		if(_num_vars_to_log > 0) {
			_max_log_time_us = _log_interval_*2e9/(_num_vars_to_log*7+10);
		} else {
			_max_log_time_us = 3600*1e6; // 1 hour
		}

		// start logging thread by move assignment
		_log_thread = std::thread{&Logger::logWorker, this};

		return true;
	}

	void stop() {
		// set logging false
		_f_is_logging = false;

		// join thread
		_log_thread.join();

		// close file
		_logfile.close();
	}

	// vector of pointers to encapsulated Eigen vector objects that are registered with
	// the logger
	std::vector<IEigenVector *> _vars_to_log;
	uint _num_vars_to_log;

	// header string
	std::string _header;

	// state
	bool _f_is_logging;

	// start time
	system_clock::time_point _t_start;

	// log interval in microseconds
	long _log_interval_;

	// maximum allowed log time in microseconds
	long _max_log_time_us;

	// log file
	std::fstream _logfile;

	// log file name
	std::string _logname;

	// thread
	std::thread _log_thread;

private:
	// thread function for logging. Note that we are not using mutexes here, no there might be weirdness
	void logWorker () {
		system_clock::time_point curr_time;
		system_clock::time_point last_time = system_clock::now();
		while (_f_is_logging) {
			usleep(_log_interval_/2);
			curr_time = system_clock::now();
			auto time_diff = std::chrono::duration_cast<microseconds>(curr_time - last_time);
			if (_log_interval_ > 0 && time_diff >= microseconds(static_cast<uint>(_log_interval_))) {
				microseconds t_elapsed = std::chrono::duration_cast<microseconds>(curr_time - _t_start);
				_logfile << t_elapsed.count();
				for (auto iter: _vars_to_log) {
					_logfile << ", ";
					iter->print(_logfile);
				}
				_logfile << "\n";

				// log stop on max time limit
				if(t_elapsed.count() > _max_log_time_us) {
					std::cerr << "Logging stopped due to time limit" << std::endl;
					break;
				}
				last_time = curr_time;
			}
		}
		_logfile.flush();
	}

	// hide default constructor
	Logger () {

	}
};

}

#endif //WLOGGER_H