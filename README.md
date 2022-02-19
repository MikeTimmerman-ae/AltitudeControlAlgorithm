# Altitude Control Algorithm
Model predictive control algorithm which controls altitude

## Installation
The project uses cmake to compile and link the project. This means that the user should have cmake installed to run the code. \
Following steps will enable you to set-up and build the project:
1. Set-up directory and initialize with git:
```console
foo@bar:~$ git init
```
2. Clone this project into the directory:
```console
foo@bar:~$ git clone https://github.com/MikeTimmerman-ae/AltitudeControlAlgorithm.git
```
3. (Skip step in case you have cmake) Download the CMake zip file from: https://cmake.org/download/ and unpack
4. Open the workspace file "Control Algorithm" in vs code
5. Install the Cmake Tools and CMake extensions
6. Go to File > Preferences > Settings > Extensions > CMake Tools > Cmake: Cmake Path\
Input the directory to the cmake.exe executable which is in  ${install_directory}\bin and restart vs code
7. Delete both 'eigen' and 'qpOASES' directories in the directory libraries and from git:
```console
foo@bar:~$ git rm -r --cached libraries/qpOASES
```
```console
foo@bar:~$ git rm -r --cached libraries/eigen
```
8. Install the submodules:
```console
foo@bar:~$ git submodule add https://github.com/coin-or/qpOASES.git libraries/qpOASES
```
```console
foo@bar:~$ git submodule add https://gitlab.com/libeigen/eigen.git libraries/eigen
```
9. Select a kit for compilation in the blue status bar at the bottom of vs code
10. Configure the build folder by pressing the play button in the blue status bar at the bottom of vs code
![image info](./data/vscodestatusbar.png)
11. Select the executable to execute: ${working_directory}\build\ControlSoftware.exe
12. (Optional) Replace minimum required version of cmake in libraries\qpOASES\CMakeLists.txt to 'VERSION 3.0'
13. Run code by again clicking the play button at the bottom of vs code