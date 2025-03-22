# aux_native
Native implementation of the auxilliary subsystems (intake, box mover, etc.) using the Pi Pico SDK

## Building
### Requirements
* cmake

* arm-none-eabi-gcc
    * Windows: Install AArch32 bare-metal target (arm-none-eabi)
               from https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads
    * Debian / Ubuntu / Raspberry Pi: run `sudo apt install gcc-arm-none-eabi`
* openocd 
    * Windows: https://github.com/openocd-org/openocd/releases/tag/v0.12.0
    * Debian / Ubuntu / Raspberry Pi: run `sudo apt install openocd`
        * Add `https://github.com/arduino/OpenOCD/blob/master/contrib/60-openocd.rules` to `/dev/udev/rules.d`
        * Reload udev rules with `sudo udevadm control --reload-rules && sudo udevadm trigger`
* gdb-multiarch
    * Windows: 
* Visual Studio Code with the following extensions:
    * Raspberry Pi Pico
    * C/C++
    * CMake
    * CMake Tools
    * Cortex-Debug
### Build instructions
1. In `CMakeLists.txt`, change the value of `PICO_BOARD` to the board type:
    * `pico` for normal Pico
    * `pico_w` for Pico W
2. Verify pico-sdk submodule and its submodules have been initialized
    * If not, do `git submodule update --init --recursive` in top directory
3. Open `micro/drivetrain_native` in vscode
4. `ctrl+shift+p` -> `CMake: Configure`
5. `ctrl+shift+p` -> `CMake: Build`
6. To deploy without picoprobe:
    1. Hold down bootsel button on pico, then plug usb into pico
    2. Copy build/drivetrain_native.uf2 to the pico's virtual flashdrive in file explorer
7. To deploy with picoprobe:
    1. Follow the guide found in [debugging](#debugging) if the picoprobe has not been set up
    2. Press f5 in vscode to start debugging

## Debugging
Currently, the project is configured to deploy and debug through the SWD port

### Setting up picoprobe
1. Obtain picoprobe uf2 from https://github.com/raspberrypi/debugprobe/releases
2. Flash a pico to use as a picoprobe (this is separate from the pico that the code will run on)
3. Wire according to the diagram where the left pico is the probe

![pico_probe_wiring](https://mcuoneclipse.com/wp-content/uploads/2022/09/picoprobewiring.jpg)

## Used codebases
* I2C Slave for Pico by Valentin Milea (MIT): https://github.com/vmilea/pico_i2c_slave