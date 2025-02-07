# Robot 2025-A
An implementation of the Clemson robot for the IEEE 2025 SoutheastCon
# Development
Visual Studio Code with Python and Pi Pico plugin is recommended

This repository has submodules to other repositories.
When cloning, do `git clone --recurse-submodules https://github.com/IEEE-Clemson/Robot2025-A.git` to clone the submodules.
## Structure
* `docs`: Documentation on higher level topics such as methodology and general robot design
* `common`: Commonly used code that is guaranteed to work on both full Raspberry Pis and Raspberry Pi Picos
* `micro`: Projects designed to run on Pi Picos with micropython
* `hal`: Hardware abstraction layer that enables code to be ran with and without physical hardware
* `sim`: Simulation layer for testing general strategy
* `subsystems`: Code to control core components from Raspberry Pi
* `control`: Primary code to control the higher level strategy of the robot
* `comms`: Git submodule
## Dependencies
* Python 3.11 or higher
### Packages
* robotpy\[commands2\] (if on Pi, run `python3 -m pip install --extra-index-url=https://wpilib.jfrog.io/artifactory/api/pypi/wpilib-python-release-2024/simple robotpy`, then install robotpy\[commands2\] as normal)
* pyside6
* pyserial
* pupil_apriltag
* numpy
* opencv
