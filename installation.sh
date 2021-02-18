#!/bin/sh
# RUN USING . ./name_script.sh
sudo apt update
sudo apt install -y curl
sudo apt update && sudo apt install -y build-essential
echo "Installing Gazebo simulator"
curl -sSL http://get.gazebosim.org | sh
echo "export SVGA_VGPU10=0" >> ~/.profile # for vm machines
. ~/.profile
echo "Copy custom_ground_plane @ Gazebo model database"
mkdir -p ~/.gazebo/models
cp -r custom_ground_plane ~/.gazebo/models/
echo "Setting up sources list for ROS"
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' # sources list
echo "Setting up keys for ROS"
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 # keys
sudo apt update
echo "ROS noetic install"
sudo apt install -y ros-noetic-desktop
sudo apt install -y ros-noetic-gazebo-ros
sudo apt install -y ros-noetic-gazebo-ros-pkgs
sudo apt install -y ros-noetic-usb-cam

echo "ROS successfully installed"
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
sudo apt install -y python3-pip
sudo apt install -y python3-pygame
sudo python3 -m pip install numpy
sudo python3 -m pip install scipy
sudo python3 -m pip install sklearn
sudo python3 -m pip install imutils
sudo apt install -y python3-opencv

read -p "Enter destination to create catkin workspace:"  dir
if [[ ! -e $dir ]]; then
	if [[ $dir != *\/ ]] #if it doesnt end with "/", add it
	then
		$dir="${dir}/"
	mkdir -p "${dir}src"
	cp catkin_ws/*sh $dir # copy all self-made scripts 
	cp -r catkin_ws/src $dir
	cd $dir
	catkin_make
	if [[ $dir == \/* ]]
	then
		echo "source ${dir}devel/setup.bash" >> ~/.bashrc
	else
		SCRIPTPATH="$( cd "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"
		echo "source ${SCRIPTPATH}/devel/setup.bash" >> ~/.bashrc
	fi
	. ~/.bashrc
	sudo chmod +x src/my_navigate/src/Brain.py # make Brain ex
	sudo chmod +x *.sh # make all scripts executable
elif [[ ! -d $dir ]]; then
	echo "$dir already exists but is not a directory" 1>&2
	exit 1
else
	echo "Directory already exists."
	echo    # (optional) move to a new line
	[[ "$0" = "$BASH_SOURCE" ]] && exit 1 || return 1 # handle exits from shell or 	function but don't exit interactive shell
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

