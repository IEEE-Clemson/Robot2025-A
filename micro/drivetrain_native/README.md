# drivetrain_native
Native implementation of the drivetrain using the Pi Pico SDK

## Building
### Requirements
* arm-none-eabi-gcc or gcc-multiarch
    * Windows: Install AArch32 bare-metal target (arm-none-eabi)
               from https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads
    * Debian / Ubuntu / Raspberry Pi: run `sudo apt install gcc-arm-none-eabi`
* Visual Studio Code with the following extensions:
    * Raspberry Pi Pico
    * C/C++
    * CMake
    * CMake Tools
    * Cortex-Debug
### Build instructions
1. Verify pico-sdk submodule and its submodules have been initialized
    * If not, do `git submodule update --init --recursive` in top directory
2. Open `micro/drivetrain_native` in vscode
3. `ctrl+shift+p` -> `CMake: Configure`
4. `ctrl+shift+p` -> `CMake: Build`
5. To deploy without picoprobe, hold down bootsel button on pico, then plug usb into pico
6. Copy build/drivetrain_native.uf2 to the pico's virtual flashdrive in file explorer
## Debugging
Currently, the project is configured to deploy and debug through the SWD port
**TODO**: Tutorial on how to use picoprobe

## Used codebases
* I2C Slave for Pico by Valentin Milea (MIT): https://github.com/vmilea/pico_i2c_slave
* Madgwick Filter by Blake Johnson (MIT): https://github.com/bjohnsonfl/Madgwick_Filter
* BMI270_SensorAPI by Bosch Sensor Tec (BSD 3): https://github.com/boschsensortec/BMI270_SensorAPI