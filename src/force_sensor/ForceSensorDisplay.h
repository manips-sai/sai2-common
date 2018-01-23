#ifndef FORCE_SENSOR_DISPLAY_H
#define FORCE_SENSOR_DISPLAY_H

#include "ForceSensorSim.h"
#include <Sai2Graphics.h>
#include <chai3d.h>

// Class to display force and moments sensed by a force/moment sensor
class ForceSensorDisplay {
public:
	ForceSensorDisplay(ForceSensorSim* sensor_sim, Sai2Graphics::Sai2Graphics* graphics);
	
	void update();

public:
	// a line to be displayed when a contact force is active
	chai3d::cShapeLine* _display_line_force;

	// a line to be displayed when a contact moment is active
	chai3d::cShapeLine* _display_line_moment;

	// simulated force sensor. TODO: should be a const pointer
	ForceSensorSim* _sensor_sim;

	// handle to graphics interface to query interaction state change
	Sai2Graphics::Sai2Graphics* _graphics;

	// scale of the force line displayed from 0 to 1
	double _force_line_scale;

	// scale of the moment line displayed from 0 to 1
	double _moment_line_scale;

};

#endif //FORCE_SENSOR_DISPLAY_H
