# Intro

This project is designed to learn good navigation skills for simulated characters


# Setup

This section covers some of the steps to setup and compile the code. The software depends on many libraries that need to be carefully prepared and placed for the building and linking to work properly. 
```
git clone https://github.com/UBCMOCCA/TerrainRLSim.git
```

## Linux (Ubuntu 16.04)

### Install system dependencies

Run the `deb_deps.sh` script to install the system dependencies required to build the project.


### OpenGL >= 3.3

Ensure that your machine is capable of running OpenGL version 3.3 or greater.

You can verify the version of OpenGL your machine currently has installed via the following command:
```
glxinfo | grep "OpenGL version"
```

If `glxinfo` is not installed on your system, install the `mesa-utils` package.

OpenGL should come as part of the drivers for your graphics hardware (whether part of the motherboard or dedicated graphics card). If you are missing a compatible version of OpenGL, consider updating your graphics drivers; if you have a GPU, ensure that the system is actually using it.


### Build Instructions

1. Download the most recent compressed external file from the [newest release](https://github.com/xbpeng/DeepLoco/releases/tag/0.81). 
1. Extract it and move into the TerrainRL directory. The top directory of the TerrainRL repository should now contain a directory called `external`, in addition to all the other directories that were there before.
1. *If you are using an OS other and Ubuntu 16.04 64-bit* build the source code for caffe that came in the `external` directory.
	```
	cd external/caffe
	make clean
	make
	cd ../../
	```
1. Copy the compiled caffe lib directory from external/caffe/build/lib to the top directory of TerrainRL.
	```
	cp -r external/caffe/build/lib .
	```
1. Copy other prebuilt libraries from the external folder

	```
	cp external/caffe/build/lib/libcaffe.* lib/
	cp external/Bullet/bin/*.so lib/
	cp external/jsoncpp/build/debug/src/lib_json/*.so* lib/
	```
1. Generate the python code wrappers (optional, if you are not planning on simulating things from python)
	```
	cd simAdapter/
	sudo apt-get install swig3.0 python3-dev python3-pip -y
	./gen_swig.sh
	cd ../
	```
1. Generate makefiles using premake4.
	```
	./premake4_linux clean
	./premake4_linux gmake
	```
1. Build all the targets!
	```
	cd gmake
	make config=release64
	cd ../
	```
	Note: you can speed up the build by appending the `-j8` flag to this last `make` command, where `8` here is the number of concurrent build threads that `make` will launch. Choose a number that makes sense based on the hardware resources you have available to you.

 
**Note:** There are some issues with the installation on Ubuntu 14.04. Some of the libraries have changed their location and name (see https://github.com/BVLC/caffe/issues/2347 for a solution).

## Windows

This setup has been tested on Windows 7 and 10 with visual studio 2013.

  1. Download the library.zip file that contains almost all of the relevant pre-compiled external libraries and source code.
  2. Unpack this library in the same directory the project is located in. For example, TerrainRL/../.
  3. You might need to install opengl/glu/GL headers. We have been using freeglut for this project. glew might already be included in library.zip.
  4. You will need to copy some dll files from dynamic_lib.zip to the directory the project is compiled to. For example, optimizer/x64/Debug/. These files are needed by the framework during runtime.
  5. Might need to create a folder in TerrainRL called "output", This is where temporary and current policies will be dumped.

## Running The System

After the system has been build there are two executable files that server different purposes. The **TerrainRL** program is for visually simulating the a controller and **TerrainRL_Optimize** is for optimizing the parameters of some controller.

Examples:

	# To simulate a controller/character  
	./TerrainRL -arg_file= args/biped3D/test_biped_3d_args.txt
	# To Train a controller
	./TerrainRL_Optimizer -arg_file= args/biped3D/opt_train_biped_3d_cacla.txt
	./TerrainRL_Optimizer -arg_file= args/genBiped2D/opt_args_imitate_biped_full_phase.txt

## Key Bindings

Most of these are toggles

 - c fixed camera mode
 - y draw COM path and contact locations
 - q draw "filmstrip" like rendering
 - f draw torques, forces
 - h draw Actor value functions?
 - shift + '>' step one frame
 - p toggle draw value function
 - ',' and '.' change render speed, decrease and increase.
 - "spacebar" to pause simulation
 - r restart the scenario
 - l reload the simulation (reparses the arg file)
 - g draw state features
 - x, throw small box
 - z, throw Large box
 - v, disable/enable character shape drawing
 - t, disable/enable recording torques
 - w, rotate sun
 - e, rotate sun (in other diection)
 - W, change sun elevation
 - W, change sun elevation (in other direction)
 - o, CRASH....
 - a, enable/disable record actions
 - j, enable/disable draw info (pos, vel, ect..)
 - k, enable/disable draw kinematic character (mocap poses)
 - m, record motion
 - "enter", switch to 3d rendering
 - t, disable/enable training

## Contents of external dependencies folders
These lists are provided for reference only. Normally, if you follow the instructions above, you shouldn't need to know about any of this.

### Linux
 - caffe source code (must still be built)
 	- Specific version (https://github.com/niuzhiheng/caffe.git @ 7b3e6f2341fe7374243ee0126f5cad1fa1e44e14)
	 - 	In the instruction to make and build Caffe we uncomment the CPU only line  
		```
		# CPU-only switch (uncomment to build without GPU support).
		CPU_ONLY := 1
		```
 - BulletPhysics ([This specific threadsafe version](https://github.com/lunkhound/bullet3))
 - [Json_cpp](https://github.com/open-source-parsers/jsoncpp)
 - [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page)
 - [CMA-ES](https://github.com/AlexanderFabisch/CMA-ESpp)  
 - [LodePNG](https://github.com/lvandeve/lodepng)

### Windows
 - Caffe: https://github.com/initialneil/caffe-vs2013
 - TODO: Finish documenting this

# Installing Python Library Version

Follow instructions for building on Linux (Ubuntu)
 1.  Create a variable for the path to the TerrainRL folder  
  ```export TERRAINRL_PATH=/path/to/terrainRL``` 
  or
   ```setenv TERRAINRL_PATH /path/to/terrainRL``` 
   Depending on your shell
 1.  Run ```pip3 install --user -v -e $TERRAINRL_PATH```
 
