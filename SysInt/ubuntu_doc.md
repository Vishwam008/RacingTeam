## Adding a new user:
`# adduser USRNM`

Output: `Full Name[]: ` Add data if needed or press Enter to give default values

### To give root priveleges
`visudo` then go to user privelege specification and write `NEWUSR ALL=(ALL:ALL) ALL` [like root]

`su NEWUSR` to change user

`sudo -i` to change to root




## Installing software
`apt-get update && apt-get install -y nano && rm -rf /var/lib/apt/lists/` to install nano

`sudo apt install curl` to install curl


## Adding to path
**Temporary:** `export PATH="/Directory1:$PATH"` where directory1 is the dir you want to add to path

**Permanent:** Add the command to .bashrc


## Conda
`conda config --set auto_activate_base false` to stop conda from running automatically on startup.


## IITBdv

sudo apt-get install ros-noetic-fkie-multimaster-msgs

remove mrpt_icp_slam_2d , generic sensor and sensorlib

While building IITBdv delete mrpt_sensors from IITBdv and install all the dependencies from <a href="https://docs.mrpt.org/reference/latest/compiling.html"> here </a>(except ros2).

## my laptop specific
* I encountered several issues while installing nvidia drivers and have found the issue and correct way to install them

* My laptop is a new one so kernel 5.x was too old for my gpu. I had to install a custom liquorix kernel 6.x and then manually install a firmware specific to my laptop.

* then install the nvidia drivers only using `sudo ubuntu-drivers autoinstall`

* To install CUDA, use:
	```
	wget https://developer.download.nvidia.com/compute/cuda/12.3.1/local_installers/cuda_12.3.1_545.23.08_linux.run
	sudo sh cuda_12.3.1_545.23.08_linux.run
	```

  change PATH and LD_LIBRARY_PATH as mentioned by the installer.

  and then use <a href="https://docs.nvidia.com/deeplearning/cudnn/install-guide/index.html">this</a> to install CUDNN. I used the one with ubuntu 22 due to my updated kernel. My CUDA version was 12.3. 

  You may check the CUDA version needed by `nvidia-smi` and current version by `nvcc --version`

