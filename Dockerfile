# Ubuntu 18.04
FROM dorowu/ubuntu-desktop-lxde-vnc:bionic

ENV DEBIAN_FRONTEND noninteractive
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES compute,utility,display,graphics,video

#RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 4EB27DB2A3B88B8B

# Install base utilities
RUN apt-get update && apt-get install -y \
 git \
 gcc \
 vim \
 mc \
 xvfb \
 libx11-6 \
 dbus-x11 \
 libxcb1 \
 libxau6 \
 x11-utils \
 libgl1-mesa-dev \
 libavformat-dev  \
 libxkbcommon-x11-0 \
 libavcodec-dev \
 libavformat-dev \
 libswscale-dev \
 xserver-xorg-core \
 apt-utils \
 libxcb-randr0-dev \ 
 libxrender-dev \
 libxkbcommon-dev \
 wget \ 
 gnupg \
 software-properties-common \ 
 python3-tk \
 curl

RUN mkdir -p /home/ubuntu/good-robot
WORKDIR /home/ubuntu/good-robot

RUN ln -s /usr/bin/python3.9 /usr/local/bin/python3
 
# CopelliaSim
RUN wget https://www.coppeliarobotics.com/files/CoppeliaSim_Edu_V4_5_1_rev4_Ubuntu18_04.tar.xz
RUN tar -xf CoppeliaSim_Edu_V4_5_1_rev4_Ubuntu18_04.tar.xz
RUN rm CoppeliaSim_Edu_V4_5_1_rev4_Ubuntu18_04.tar.xz

# Add correct paths to the .bashrc
ENV COPPELIASIM_ROOT=/home/ubuntu/good-robot/CoppeliaSim_Edu_V4_5_1_rev4_Ubuntu18_04
ENV LD_LIBRARY_PATH=/home/ubuntu/good-robot/CoppeliaSim_Edu_V4_5_1_rev4_Ubuntu18_04
ENV QT_QPA_PLATFORM_PLUGIN_PATH=/home/ubuntu/good-robot/CoppeliaSim_Edu_V4_5_1_rev4_Ubuntu18_04

RUN git clone https://github.com/jhu-lcsr/good_robot.git

RUN apt-get install -y python3-pip virtualenv gedit
RUN virtualenv venv --python=python3

RUN ./venv/bin/pip3 install torch==1.9.1
RUN ./venv/bin/pip3 install numpy==1.19.5
RUN ./venv/bin/pip3 install scipy==1.5.4
RUN ./venv/bin/pip3 install --upgrade pip
RUN ./venv/bin/pip3 install opencv-python==4.6.0.66
RUN ./venv/bin/pip3 install pyyaml
RUN ./venv/bin/pip3 install matplotlib==3.3.4
RUN ./venv/bin/pip3 install spacy==3.0.9
RUN ./venv/bin/pip3 install transformers==4.8.2
RUN ./venv/bin/pip3 install einops==0.4.1
RUN ./venv/bin/pip3 install allennlp==2.6.0
RUN ./venv/bin/pip3 install pandas==1.1.5
RUN ./venv/bin/pip3 install scikit-image==0.17.2
RUN ./venv/bin/pip3 install IPython
RUN ./venv/bin/pip3 install rospkg
RUN ./venv/bin/pip3 install pyqt5

RUN mkdir /root/.cache/torch
RUN mkdir /root/.cache/torch/hub
RUN mkdir /root/.cache/torch/hub/checkpoints

RUN wget -O /root/.cache/torch/hub/checkpoints/densenet121-a639ec97.pth -c --no-check-certificate https://download.pytorch.org/models/densenet121-a639ec97.pth
RUN ln -s /home/ubuntu/good-robot/ /root/good-robot

RUN wget --no-check-certificate https://github.com/jhu-lcsr/good_robot/archive/refs/tags/v0.17.1.tar.gz
RUN tar -xf v0.17.1.tar.gz
RUN rm v0.17.1.tar.gz

# Download the trained RL model for stacking cubes from Good robot github
RUN mkdir ./models
RUN wget --no-check-certificate  https://github.com/jhu-lcsr/good_robot/releases/download/v0.17.1/2020-05-13-12-51-39_Sim-Stack-SPOT-Trial-Reward-Masked-Training-simtoreal-model.zip
RUN unzip -a 2020-05-13-12-51-39_Sim-Stack-SPOT-Trial-Reward-Masked-Training-simtoreal-model.zip -d ./models
RUN rm 2020-05-13-12-51-39_Sim-Stack-SPOT-Trial-Reward-Masked-Training-simtoreal-model.zip

RUN ./venv/bin/pip3 install efficientnet-pytorch
RUN ./venv/bin/pip3 install gpustat
RUN ./venv/bin/pip3 install --upgrade pymodbus
RUN ./venv/bin/pip3 install --upgrade git+https://github.com/sovrasov/flops-counter.pytorch.git
RUN sudo apt install psmisc at-spi2-core

# Install Yandex browser
RUN wget --no-check-certificate https://cachev2-mskmar05.cdn.yandex.net/download.cdn.yandex.net/browser/beta-public/23_5_1_799_45584/Yandex.deb
RUN sudo apt install fonts-liberation jq libjq1 libonig4 libvulkan1
RUN sudo dpkg -i ./Yandex.deb

#--------------------------ROS part----------------------------
RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections

RUN sudo apt install gpg-agent lsof ca-certificates python3-empy python3-dev python3-distutils libpcl-dev -y

# ROS dependencies installation
RUN sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
RUN sudo apt update -y
RUN sudo apt install -y python-rosdep \
 python-rosinstall \
 python-rosinstall-generator \
 python-wstool build-essential 

# Create a catkin Workspace
RUN mkdir -p /home/ros/ros_catkin_ws
RUN mkdir -p /home/ros/ros_catkin_ws/src
WORKDIR /home/ros/ros_catkin_ws

RUN git clone https://github.com/ros-drivers/openni2_launch.git /home/ros/ros_catkin_ws/src/openni2_launch
RUN git clone https://github.com/ros-drivers/rgbd_launch.git    /home/ros/ros_catkin_ws/src/rgbd_launch
RUN git clone https://github.com/UbiquityRobotics/fiducials.git /home/ros/ros_catkin_ws/src/fiducials

RUN sudo apt install ros-melodic-desktop-full ros-melodic-openni2-launch ros-melodic-rosbash ros-melodic-rviz htop -y

# Create Python env with Python3
RUN virtualenv ros-venv --python=python3
RUN ./ros-venv/bin/pip3 install rosdep rospkg rosinstall_generator rosinstall wstool vcstools catkin_tools catkin_pkg vcstool pyyaml==5.4.1 numpy
RUN pip3 install numpy
ENV PATH="./ros-venv/bin:$PATH"
RUN echo $PATH

# Installing SIP 
RUN wget https://distfiles.macports.org/py-sip/sip-4.19.22.tar.gz --no-check-certificate
#RUN wget https://www.riverbankcomputing.com/static/Downloads/sip/4.19.25/sip-4.19.25.tar.gz --no-check-certificate
RUN tar -xf sip-4.19.22.tar.gz
RUN cd sip-4.19.22 && python3 configure.py 
RUN cd sip-4.19.22 && sudo make
RUN cd sip-4.19.22 && sudo make install

# Building the catkin Workspace
RUN python3 --version
RUN apt install python3-catkin-pkg python3-dev python3-pyqt5 python3-yaml python3-rospkg usbutils python3-defusedxml python3-netifaces -y  

RUN sudo apt update -qq
RUN sudo rosdep init 
RUN sudo rosdep update --include-eol-distros

# Fetch the core packages so we can build them
RUN export ROS_PYTHON_VERSION=3 && export SSL_CERT_FILE=/usr/lib/ssl/certs/ca-certificates.crt && \
rosinstall_generator desktop_full --rosdistro melodic --deps --tar > melodic-desktop-full.rosinstall && wstool init -j8 src melodic-desktop-full.rosinstall

# Vcs clones all repositories which are passed in via stdin in YAML format.
RUN export SSL_CERT_FILE=/usr/lib/ssl/certs/ca-certificates.crt && \ 
vcs import src < melodic-desktop-full.rosinstall

# Resolving Dependencies (install system dependencies)
RUN export ROS_PYTHON_VERSION=3 && \
sudo rosdep install --from-paths src --ignore-src --rosdistro melodic -y 

# Add some files needed to compile fiducials package
RUN mkdir git && git clone https://github.com/wpirm/catkin_wpirm.git ./git
RUN cp -R git/devel/include/vision_msgs/ src/fiducials/stag_detect/include/vision_msgs

RUN mkdir -p devel_isolated/share && mkdir devel_isolated/include 
RUN ln -s /opt/ros/melodic/share/vision_msgs/ devel_isolated/share/vision_msgs

RUN mkdir -p install_isolated/share && mkdir install_isolated/include
RUN ln -s /opt/ros/melodic/share/vision_msgs/ install_isolated/share/vision_msgs

RUN mkdir -p devel_isolated/aruco_detect/include && mkdir -p src/fiducials/aruco_detect/include/
RUN cp -R git/devel/include/vision_msgs/ devel_isolated/aruco_detect/include/vision_msgs
RUN cp -R git/devel/include/vision_msgs/ src/fiducials/aruco_detect/include/vision_msgs

# Installation
RUN sudo python3 ./src/catkin/bin/catkin_make_isolated --install  -DCMAKE_BUILD_TYPE=Release -DPYTHON_VERSION=3.6

# Addidtional packages
RUN sudo apt install ros-melodic-desktop-full ros-melodic-image-pipeline ros-melodic-openni2-launch ros-melodic-image-pipeline -y

# Uncomment UsbInterface for NiViewer
RUN sed -i 's/;UsbInterface=2/UsbInterface=2/g' /etc/openni/GlobalDefaults.ini

# ----------------RealSense camera--------------
# RealSense camera driver installation
# https://dev.intelrealsense.com/docs/ros-wrapper
RUN sudo apt-get install ros-melodic-realsense2-camera ros-melodic-realsense2-description -y

# Install Realsense viewer
RUN sudo mkdir -p /etc/apt/keyrings
RUN curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null
RUN echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
	sudo tee /etc/apt/sources.list.d/librealsense.list
RUN sudo apt-get update
RUN sudo apt-get install librealsense2-dkms -y
RUN sudo apt-get install librealsense2-utils -y

RUN sudo add-apt-repository ppa:graphics-drivers
RUN sudo apt update -y
RUN sudo apt-get install nvidia-driver-510 -y

# Copying the files
RUN ln -s /home/ros/ros_catkin_ws/ /root/ros_catkin_ws
COPY ./files/ ./files

RUN mv ~/good-robot/good_robot-0.17.1 ~/good-robot/good_robot-0.17.1_default
RUN tar -xf /root/ros_catkin_ws/files/good_robot-0.17.1-rqc.tar -C ~/good-robot/


