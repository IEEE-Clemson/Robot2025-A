# Robot 2025-A
Primary implementation of the Clemson robot for the IEEE 2025 SoutheastCon

The main code for the path routine is found in `tools/auto_demo.py`

## Robot architecture
The robot control consists of a main Raspberry Pi 5 and two Raspberry Pi Pico microcontrollers. The Picos are configured to be I2C
slaves in order to accept commands from the Pi 5.


### Pi 5
The Pi 5 is the main controller which controls the high level strategy, controls algorithms, and Vision tasks. 

For the controls algorithms aspect, we used [WPILib](https://docs.wpilib.org/en/stable/index.html) and [PathPlanner](https://pathplanner.dev/home.html). 
WPILib is a library designed for the FIRST Robotics Competition; however, it is able to be used independently of the hardware. It provides a good architecture
for making asynchronous systems as well as algorithms such as odometry, sensor fusion, geometry math wrappers, etc. PathPlanner provides a method for generating 
complex trajectories. Although we only use trapezoidal trajectories for our current robot, it has the capability of making complex spline based trajectories with
a visual editor. 

One of the architectures that WPILib provides is the [command based architecture](https://docs.wpilib.org/en/stable/docs/software/commandbased/what-is-command-based.html).
In this architecture, robot code is split into "commands" and "subsystems." Subsystems represent components on a robot that can be controlled independently
from one another. For example, a typical robot might have a drivetrain, intake, and arm subsystem. Commands represent actions that can be performed on 
these subsystems. Some commands you may have on your robot are `DriveToPosition`, `CloseGripper`, `StartIntake`, etc. Note that a given subsystem can only be
used by one command at any given time e.g. you can't use `CloseGripper` and `OpenGripper` at the same time. Additionally, these commands can be composed into
larger commands using decorator functions such as `andThen`, `onlyIf`, `withTimeout`, `alongWith`, etc. allowing you to create entire autonomous robot procedures.
More explanation can be found in the [official WPILib docs](https://docs.wpilib.org/en/stable/docs/software/commandbased/what-is-command-based.html)

In our implementation, commands are found in [control]() and subsystems are found in [subystems](). The main robot procedure is found in [tools/auto_demo]()

### Picos
The two Picos handle real time tasks that the Pi 5 would not normally be able to handle by itself.

For the code, we use the [Pico-SDK](https://github.com/raspberrypi/pico-sdk) Although we intially attempted to use Micropython for the Pico,
we found that there were too many bugs (random hangs) and limitations (no debugging, no i2c slave, less documentation) for our robot.
We mostly attempt to use basic C for our implementation since it is taught to everyone; however, there are times where we 
need to use C++ for external libraries. More details for the development process can be found in [micro/aux_native]() and [micro/drivetrain_native]()

### Drivetrain Pico [micro/drivetrain_native]()
The first Pico is dedicated to controlling the drivetrain. It tracks the encoders and motor output of each of 
the four wheels of the robot with PID control. Additionally, it handles communication with an BMI055 gyro. The module takes commands
to set the target velocity of the drivetrain and gives feedback on actual velocity and orientation.

### Aux. Pico [micro/aux_native]()
The second Pico is dedicated to all other microcontroller tasks. This includes the intake motor PID, start LED detector, 
beacon and box mover servos, and astral material dropper.

# Development
Visual Studio Code with Python and Pi Pico plugin is recommended

This repository has submodules to other repositories.
When cloning, do `git clone --recurse-submodules https://github.com/IEEE-Clemson/Robot2025-A.git` to clone the submodules.
## Structure
* `docs`: Documentation on higher level topics such as methodology and general robot design
* `micro`: Projects designed to run on Pi Picos
* `hal`: Hardware abstraction layer that enables code to be ran with and without physical hardware
* `sim`: Simulation layer for testing general strategy
* `subsystems`: Code to control core components from Raspberry Pi
* `control`: Primary code to control the higher level strategy of the robot
* `tests`: Unit tests mostly used for testing controls math
* `tools`: Various tools to test functionality of the robot
## Dependencies
* Python 3.11 or higher
### Packages
* robotpy\[commands2\] (if on Pi, run `python3 -m pip install --extra-index-url=https://wpilib.jfrog.io/artifactory/api/pypi/wpilib-python-release-2024/simple robotpy`, then install robotpy\[commands2\] as normal)
* pyside6
* pyserial
* pupil_apriltag
* numpy
* opencv
* smbus2
