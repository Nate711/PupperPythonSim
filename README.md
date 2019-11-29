# Pupper Robot: Python Simulation

## Overview
This repository contains Python code to run Pupper, a Raspberry Pi-based quadruped robot. In addition to the robot code, this repository also contains a wrapper to simulate the robot in MuJoCo or PyBullet using the same code that runs on the robot.

## Installation for PyBullet Simulation
The PyBullet simulator is free for academic use and requires no license whatsoever, but in my experience PyBullet is much slower than MuJoCo and is less clear about how to tune the contact parameters.

0. Clone this repository
```shell
git clone https://github.com/Nate711/PupperSimulation.git
```
1. Install pybullet (in a python 3.7 environment)
```shell
pip install pybullet
```
## Run PyBullet Simulation
1. Run simulation:
```shell
python3 simulate_pybullet.py
```

## Installation for MuJoCo Simulation
MuJoCo has been faster than PyBullet in my experience, but requires that you request and activate a license. The process to acquire a license can take several days.

0. Clone this repository
```shell
git clone https://github.com/Nate711/PupperSimulation.git
```
1. Acquire a license for MuJoCo at http://mujoco.org/. You can get a free trial of the professional license for a month, or with a student account, a free year.

2. Follow the instructions at https://github.com/openai/mujoco-py to correctly install MuJoCo. 

If you have trouble installing MuJoCo on macOS because gcc can't find certain header files, like "limits.h" or "stdio.h", then try completing the installation wizard that pops up when you run:
```shell
sudo open /Library/Developer/CommandLineTools/Packages/macOS_SDK_headers_for_macOS_10.14.pkg 
```
And also do:
```shell
brew update
brew install gcc@8
brew link --overwrite gcc
```


3. Install the python requirements:
```bash
bash install_packages_sim.sh
```

## Run MuJoCo Simulation
1. Run
```shell
python3 simulate.py
``` 
2. The MuJoCo simulator should then pop up in a new window with various interactive options. Press space to stop or start the simulation.

## Installation for Raspberry Pi Robot
### Materials
- Raspberry Pi 4
- SD Card (32GB recommended)
- Raspberry Pi 4 power supply (USB-C, 5V, >=3A)
- Ethernet cable

### Steps
- Install Raspbian Buster Lite onto the Pi
    - Download https://www.raspberrypi.org/downloads/raspbian/
    - Use BalenaEtcher to flash the OS to the SD card
- Set up the Raspberry Pi
    - Before even ejecting the SD Card, follow the instructions on this repo to put the self-installing setup script on the Pi: https://github.com/stanfordroboticsclub/RPI-Setup 
    - Complete the “Actually Doing It”, “Getting Internet Access”, and “Getting Started With the Pi” sections
- Test that the Pi works and connects to the internet
- Install the PREEMPT-RT kernel onto the Pi
    - Download the kernel patch https://github.com/lemariva/RT-Tools-RPi/tree/master/preempt-rt/kernel_4_19_59-rt23-v7l%2B
    - Follow these instructions starting from “Transfer the Kernel” https://lemariva.com/blog/2019/09/raspberry-pi-4b-preempt-rt-kernel-419y-performance-test
    - Test by running in the shell:
        ```shell
        uname -r
        ```
- Install pigpio
    - Run
    ```shell
    sudo apt-get install python3-distutils
    ```
    - Follow the instructions available at http://abyz.me.uk/rpi/pigpio/download.html to install a GPIO library for the Pi that adds software PWM capability.
- Get the Pupper Code
    - Clone the Pupper repository https://github.com/Nate711/PupperPythonSim/
    - Install requirements:
    ```shell
    bash install_packages_robot.sh
    ```
## Running the Robot
- Start the PiGPIO daemon by executing in shell:
    ```shell
    sudo pigpiod
    ```
- Load the robot code in the Julia REPL: 
    ```shell
    python3 run_robot.py
    ``` 
