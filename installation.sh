#! /bin/bash

# if [[ $UID != 0 ]]; then
#     echo "Please run this script with sudo:"
#     echo "sudo $0 $*"
#     exit 1
# fi

distribution="noetic"
username="ubuntu"
if [[ $(cat /proc/device-tree/model | grep "Raspberry Pi 2"*) != "" ]] ; then
	# distribution="kinetic"
	piv="2"
	wireless_ad="wlx74da381aa53a" # name of usb adapter
else
	piv="3"
	wireless_ad="wlan0" # name of usb adapter
	# distribution="noetic"
fi
sudo apt update && sudo apt -y upgrade
sudo apt install -y curl
sudo apt install -y build-essential
sudo apt install -y net-tools

# sudo apt install wireless-tools firmware-b43-installer
# echo "Wifi interface "$(ls /sys/class/net | grep -i wl*)
# # sudo cp network-config /etc/netplan/50-cloud-init.yaml

sudo apt install -y python3-dev
sudo apt install -y python3-pip
sudo apt install -y python3-opencv
sudo python3 -m pip install numpy
sudo python3 -m pip install imutils
# sudo python3 -m pip install sklearn
sudo apt install -y python3-sklearn
sudo python3 -m pip install cython
# sudo python3 -m pip install opencv-contrib-python==4.1.0.25
sudo python3 -m pip install opencv-contrib-python==4.4.0.46
sudo apt install -y python3-smbus
sudo apt install -y python3-rpi.gpio
sudo apt install -y python3-pigpio
sudo python3 -m pip install cython
sudo python3 -m pip install pyserial
sudo apt install -y i2c-tools
sudo groupadd i2c
sudo chown :i2c /dev/i2c-1
sudo chmod g+rw /dev/i2c-1
sudo usermod -aG i2c "$username"
sudo echo 'KERNEL=="i2c-[0-9]*", GROUP="i2c"' >> /etc/udev/rules.d/10-local_i2c_group.rules

 
echo "Setting up sources list for ROS"
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/sudo apt/sources.list.d/ros-latest.list' # sources list
echo "Setting up keys for ROS"
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 # keys
sudo apt update
echo "ROS $distribution install"
sudo apt install -y python3-rosdep2
sudo apt install -y python3-roslaunch
sudo apt install -y rospack-tools
sudo apt install -y ros-"$distribution"-ros-base
sudo apt install -y ros-"$distribution"-usb-cam
sudo apt install -y ros-"$distribution"-cv-bridge
sudo apt install -y ros-"$distribution"-tf
sudo apt install -y ros-"$distribution"-image-view
sudo apt install -y ros-"$distribution"-gmapping
sudo apt install -y ros-"$distribution"-camera-calibration
sudo apt install -y ros-"$distribution"-robot-localization
sudo apt install -y ros-"$distribution"-rospy
# sudo apt install -y ros-"$distribution"-joy # uncomment to plug joystick to our robot

echo "source /opt/ros/$distribution/setup.bash" >> .bashrc
echo "export ROS_IP="$(ifconfig $wireless_ad | grep "inet " | awk '{print $2}') >> .bashrc
echo "export ROS_MASTER_URI=http://"$ROS_IP":11311" >> .bashrc
source .bashrc
# rosdep init
# rosdep update

catkin_workspace="robot_ws/"
# read -p "Enter destination to create catkin workspace:"  dir
if [[ ! -e $catkin_workspace ]]; then
	if [[ $catkin_workspace != *\/ ]] #if it doesnt end with "/", add it
	then
		catkin_workspace="${catkin_workspace}/"
	fi
	mkdir -p "${catkin_workspace}"
	cd $catkin_workspace
	rsync -r --chown=$username:$username ../src . # copy source files to catkin workspace and take ownership
	# cp -r src/ $catkin_workspace/
	catkin_make
	if [[ $catkin_workspace == \/* ]]
	then
		echo "source ${catkin_workspace}devel/setup.bash" >> ~/.bashrc
	else
		SCRIPTPATH="$( cd "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"
		echo "source ${SCRIPTPATH}/devel/setup.bash" >> ~/.bashrc
	fi
	. ~/.bashrc
	# sudo chmod +x ./src/my_navigate/src/Brain.py # make Brain ex
	cd ./src/my_navigate/src/ # make all scripts executable
	./setup_cython.sh
	cd "$(realpath "$catkin_workspace")"
elif [[ ! -d $catkin_workspace ]]; then
	echo "$catkin_workspace already exists but is not a directory" 1>&2
	exit 1
else
	echo "Directory already exists."
	echo    # (optional) move to a new line
	[[ "$0" = "$BASH_SOURCE" ]] && exit 1 || return 1 # handle exits from shell or function but don't exit interactive shell
fi

read -p "Need to update-alternatives. Are you sure? " -r REPLY
echo    # (optional) move to a new line

if [[ $REPLY =~ ^[Yy]$ ]]
then
	echo "Confirmed"
	sudo update-alternatives --install /usr/local/bin/python python /usr/bin/python2 2
	sudo update-alternatives --install /usr/local/bin/python python /usr/bin/python3 3
    [[ "$0" = "$BASH_SOURCE" ]] && exit 1 || return 1 # handle exits from shell or function but don't exit interactive shell
fi



read -p "Do you want to install samba for sharing files? Default is same as catkin_ws" -r REPLY
echo    # (optional) move to a new line

if [[ $REPLY =~ ^[Yy]$ ]]
then
	sudo apt install -y samba samba-common-bin
	echo "[$catkin_workspace]
	path = /home/$username/$catkin_workspace
	writable = yes
	create mask = 0777
	directory mask = 0777
	public = yes
	browsable = yes
	" >> /etc/samba/smb.conf
	sudo smbpasswd -a "$username"
	systemctl restart smbd.service
	ufw allow samba
fi

sudo apt -y autoremove