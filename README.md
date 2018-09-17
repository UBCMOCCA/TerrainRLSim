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

1. Download the most recent compressed external file from the [newest release](https://github.com/xbpeng/DeepTerrainRL/releases).
1. Extract it and move into the TerrainRL directory. The top directory of the TerrainRL repository should now contain a directory called `external`, in addition to all the other directories that were there before.
1. When build Bullet, manually add `-fPIC` to all the generated make files in `build3/gmake/`
    ```
    CPPFLAGS  += -MMD -MP -fPIC $(DEFINES) $(INCLUDES)
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


## Running The System

After the system has been build there are two executable files that server different purposes. The **TerrainRL** program is for visually simulating the a controller and **TerrainRL_Optimize** is for optimizing the parameters of some controller.

Examples:

    # To simulate a controller/character
    ./TerrainRL -arg_file= args/biped3D/test_biped_3d_args.txt

## Key Bindings

Most of these are toggles

-   c fixed camera mode
-   y draw COM path and contact locations
-   q draw "filmstrip" like rendering
-   f draw torques, forces
-   h draw Actor value functions?
-   shift + '>' step one frame
-   p toggle draw value function
-   ',' and '.' change render speed, decrease and increase.
-   "spacebar" to pause simulation
-   r restart the scenario
-   l reload the simulation (reparses the arg file)
-   g draw state features
-   x, throw small box
-   z, throw Large box
-   v, disable/enable character shape drawing
-   t, disable/enable recording torques
-   w, rotate sun
-   e, rotate sun (in other diection)
-   W, change sun elevation
-   W, change sun elevation (in other direction)
-   o, CRASH....
-   a, enable/disable record actions
-   j, enable/disable draw info (pos, vel, ect..)
-   k, enable/disable draw kinematic character (mocap poses)
-   m, record motion
-   "enter", switch to 3d rendering
-   t, disable/enable training

## Contents of external dependencies folders

These lists are provided for reference only. Normally, if you follow the instructions above, you shouldn't need to know about any of this.

### Linux

-   BulletPhysics ([This specific threadsafe version](https://github.com/lunkhound/bullet3))
-   [Json_cpp](https://github.com/open-source-parsers/jsoncpp)
-   [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page)
-   [CMA-ES](https://github.com/AlexanderFabisch/CMA-ESpp)
-   [LodePNG](https://github.com/lvandeve/lodepng)

# Installing Python Library Version

Follow instructions for building on Linux (Ubuntu)

1.  Create a variable for the path to the TerrainRL folder
    `export TERRAINRL_PATH=/path/to/terrainRL`
    or
    `setenv TERRAINRL_PATH /path/to/terrainRL`
    Depending on your shell
1.  Run `pip3 install --user -v -e $TERRAINRL_PATH`
