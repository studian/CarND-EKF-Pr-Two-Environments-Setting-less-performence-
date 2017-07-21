[image1]: ./results_image/EKF_simul.png
[image2]: ./results_image/using_cmake_EKF.png
[image3]: ./results_image/using_eclipse_EKF.png
[image4]: ./using_eclips_oxygen/ide_profiles/Eclipse/images/eclipse_oxygen.png

# Extended Kalman Filter Project Starter Code
Self-Driving Car Engineer Nanodegree Program

In this project you will utilize a kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. Passing the project requires obtaining RMSE values that are lower that the tolerance outlined in the project rubric. 

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

![alt text][image1]

**My build environments are two types in the Ubuntu 16.04.**
**1. using cmake**
**2. using eclipse(oxygen)**

Both of the two environments, it must be installed [uWebSocketIO](https://github.com/uWebSockets/uWebSockets).
Please see [this concept in the classroom](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77) for the required version and installation scripts.

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

**Other Important Dependencies**
* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros

## 1. Build & Make in the cmake environment : Refer './using_cmake' folder
* Clone this repo.
* mkdir build
* cd build
* cmake ..
* make
* ./ExtendedKF

### Editor Settings
We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:
* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

### Code Style
Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## 2. Build & Make in the Eclipse environment : Refer './using_eclips_oxygen folder

### 1) Create a build directory
Create a build directory in your project folder as a sibling to the source directory (`/src`):

```
mkdir build
```

### 2) Change `CMakeLists.txt`
1. Be sure to move `CMakeLists.txt` file to the `/src` directory:
```
mv CMakeLists.txt src
```
2. Change the project name in the  `CMakeLists.txt` 
**(IMPORTANT) Your project name should be different from your executable name and different from your build folder name.**

3. Remove  "_src_" from the path to `cpp` "_sources_". Your final `CMakeLists.txt`  should be similar to this:
```
project(Extended_Kalman_Filter)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(sources 
	./main.cpp 
	./tools.cpp 
	./FusionEKF.cpp 
	./kalman_filter.cpp)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

add_executable(ExtendedKF ${sources})

target_link_libraries(ExtendedKF z ssl uv uWS)
```
### 3) CMake 
Go to `build` directory
```
cd build
```

and run CMake with the following parameters (see below for commandline). Make sure you set your CMAKE_BUILD_TYPE to Debug if you want to debug your project with gdb inside of Eclipse CDT. This is not done automatically
```
cmake -G"Eclipse CDT4 - Unix Makefiles" -D CMAKE_BUILD_TYPE=Debug ../src/
```
### 4) Make
Now, make the project:
```
make
```
You will now find two Eclipse project files in your build tree:  `.project` and `.cproject`

### 5) Import the project into Eclipse
![alt text][image4]

Please refer to [README.md of ide_profiles(eclipse)](./using_eclips_oxygen/ide_profiles/Eclipse/README.md).

### 6) Code Style
Import the google coding style settings to Eclipse:
1. Download the style sheet to some location:
[Google's C++ style guide for Eclipse](https://github.com/google/styleguide/blob/gh-pages/eclipse-cpp-google-style.xml)
2. From Eclipse go to `Window > Preferences > C/C++ > Code Style > Formatter`
3. Click `Import`
4. Select the downloaded sheet.
5. Click `Ok`

## 3. Generating Additional Data

This is optional!

If you'd like to generate your own radar and lidar data, see the
[utilities repo](https://github.com/udacity/CarND-Mercedes-SF-Utilities) for
Matlab scripts that can generate additional data.

If you want to do optional process, you must download and use to version of `project-assistant-ready` of `CarND-Extended-Kalman-Filter-Project`

I did not yet.

## 4. Project Results

Here are Cmake and Eclipse based results.

* Cmake result is follows:
![alt text][image2]

* Eclipse result is follows:
![alt text][image3]

