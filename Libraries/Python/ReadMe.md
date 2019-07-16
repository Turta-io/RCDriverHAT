# Python Libraries
This directory includes Python libraries for Turta RCDriver HAT.

## Index
* __Turta_IMU.py:__ Python Library for ST LSM6DS33 IMU.
* __Turta_RCDriver.py:__ Python Library for PWM Driver, Power Monitor, ADC, Tachometer and PWM Decoder.

## Installation of Python Libraries
* Use 'pip3 install turta-rcdriverhat' to download and install libraries automatically.
* Use 'pip3 install --upgrade --user turta-rcdriverhat' to update your libraries.
* Use 'pip3 uninstall turta-rcdriverhat' to uninstall the libraries.
* If you wish to install libraries manually, copy the ingredients of Python folder to the project folder.

## Dependencies for Python Libraries
The package installer installs other libraries required for RC Driver HAT's operation.
* We're using 'SMBus' for I2C communication. To install it manually, type 'sudo pip3 install smbus' to the terminal.
* We're using 'spidev' for SPI communication. To install it manually, type 'sudo pip3 install spidev' to the terminal.
* We're using Python 3 for the libraries and samples.

## Documentation
Visit [docs.turta.io](https://docs.turta.io) for documentation.
