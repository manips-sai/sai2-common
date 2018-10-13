## Project name: sai2-common (version: 0.1.0)
Note: This project uses Semantic Versioning (http://semver.org/).

### Project description:
The Sai2-Common library implements a set of helper modules to create robot control applications. This includes the following modules:

* __filters__ module: to implement Butterworth filters on Eigen objects
* __force_sensor__ module: implement force sensor and display forces as lines when using sai2-simulation and sai2-graphics
* __redis__ module: wrapper around hiredis for Eigen objects
* __timer__ module: to easily implement timer for control loops
* __trajectory_generation__ module : a wrapper around the Reflexxes library typeII for eigen objects

### 3rdParty dependencies (* = installation required):

* Redis*: Redis server [brew, apt-get]
* Hiredis*: Redis minimalist client [brew, apt-get]
* ReflexxesTypeII* : Online trajectory generation. [ http://www.reflexxes.ws/index.html ]
* Chai3d: Graphics and haptics library [custom forked repository at https://github.com/manips-sai-org/chai3d ]

### SAI library dependencies:
* Sai2-UrdfReader: xml and urdf parser [ https://github.com/manips-sai-org/sai2-urdfreader ]
* Sai2-Model: Robot kinematics and Dynamics library based on RBDL [ https://github.com/manips-sai-org/sai2-model ]
* Sai2-Graphics: Graphics interface built around chai3d [ https://github.com/manips-sai-org/sai2-graphics ]
* Sai2-Simulation: Articulated rigid-body physics simulation [private at https://github.com/manips-sai/sai2-simulation, contact shameekg@stanford.edu]

### Installation instructions:

## First compile ReflexxesTypeII for your system
-- Linux 64 bits
```
cd external/ReflexxesTypeII/Linux
make all64
cd ../../..
```

-- Mac 64 bits
```
cd external/ReflexxesTypeII/MacOS
make all64
cd ../../..
```

## Then compile the library
-- OSX/ Linux:

```
mkdir build
cd build
cmake ..
make -j4
```

### Uninstallation instructions: 
-- OSX/ Linux:
```
rm -r build
rm -r ~/.cmake/packages/SAI2-COMMON
```

### Getting started:
Take a look at sample applications under examples/.
You can run them from build/examples/<x-example>/.

### License:
Currently pending licensing. PLEASE DO NOT DISTRIBUTE.

### Project contributors:
* Gerald Brantner
* Brian Soe
* Mikael Jorda
* Shameek Ganguly
* Toki Migimatsu

### For questions, contact:
manips.sai@gmail.com  
mjorda@stanford.edu  
shameekg@stanford.edu
