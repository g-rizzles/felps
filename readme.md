Using scripts in tools (assuming a WSL or other Linux environment):
- Set FELPS_ROOT env variable with:
  - `export FELPS_ROOT=$(wslpath -a .)/..` in `./tools` in a WSL session.
- Build the latest version of the docker container with:
  - `./build-docker.sh`
- Run the latest version of the container with:
  - `./run-docker`
- In the container, build the latest .elf with:
  - `/opt/cmd.sh build`
- In the container, clean the build directory with:
  - `/opt/cmd.sh clean`


Research during development:
- [SPI as write only](https://community.platformio.org/t/stm32f103-using-spi-in-transmit-only-mode/17997)

Resources:
- [arduino-cli](https://github.com/arduino/arduino-cli/releases)
- [arduino-docker](https://github.com/suculent/arduino-docker-build/tree/master)
- [arduino-stm32](https://github.com/stm32duino/Arduino_Core_STM32)
- [ti-bq25790-driver](https://git.ti.com/gitweb?p=ti-analog-linux-kernel/dmurphy-analog.git;a=commit;h=75997c21e9dfc0d54b7f774bfb37e6af796ff293)