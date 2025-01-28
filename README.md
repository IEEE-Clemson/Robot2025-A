# Robot 2025-A
An implementation of the Clemson robot for the IEEE 2025 SoutheastCon
# Development
Visual Studio Code with Python plugin is recommended
## Structure
* `docs`: Documentation on higher level topics such as methodology and general robot design
* `common`: Commonly used code that is guaranteed to work on both full Raspberry Pis and Raspberry Pi Picos
* `micro`: Projects designed to run on Pi Picos with micropython
* `hal`: Hardware abstraction layer that enables code to be ran with and without physical hardware
* `sim`: Simulation layer for testing general strategy
* `subsystems`: Code to control core components from Raspberry Pi
* `control`: Primary code to control the higher level strategy of the robot
