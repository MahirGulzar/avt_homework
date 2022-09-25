FROM nvidia/cuda:11.2.0-devel-ubuntu20.04

ARG DEBIAN_FRONTEND=noninteractive
ARG ROS_DISTRO_ARG="noetic"
ENV ROS_DISTRO ${ROS_DISTRO_ARG}
ENV TZ=Europe/Tallinn

# Install python and other required packages
RUN apt update && apt-get install -y tzdata &&
    apt-get install -y git wget &&
    apt-get install -y python &&
    apt-get install -y python3-pip

# Setup ROS
RUN apt-get update -y || true &&
    apt-get install -y lsb-release && apt clean all &&
    sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' &&
    apt install -y curl &&
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - &&
    apt-get update || true &&
    apt-get install -y ros-$ROS_DISTRO-ros-base build-essential cmake usbutils libusb-1.0-0-dev git -y --allow-unauthenticated

# Install Packages depending on ROS distro
RUN apt-get install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool

RUN rm -rf /var/lib/apt/lists/*

# Miniconda
ENV PATH="/root/miniconda3/bin:${PATH}"
ARG PATH="/root/miniconda3/bin:${PATH}"

RUN apt-get update && wget \
    https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh &&
    mkdir /root/.conda &&
    bash Miniconda3-latest-Linux-x86_64.sh -b &&
    rm -f Miniconda3-latest-Linux-x86_64.sh

RUN conda init bash

SHELL ["conda", "run", "-n", "base", "/bin/bash", "-c"]
RUN conda create -n avt_hw_env python=3.9 &&
    conda config --set channel_priority true --set ssl_verify false &&
    conda update --all --yes &&
    conda update conda &&
    conda install -c conda-forge catkin_tools

# Install cudatoolkit & cudnn inside avt_hw_env environment
SHELL ["conda", "run", "-n", "avt_hw_env", "/bin/bash", "-c"]
RUN conda install -c conda-forge cudatoolkit=11.2 cudnn=8.1.0 -y

# Python vision libraries
COPY requirements.txt ./
RUN pip install -r requirements.txt

# Export variables to .bashrc
RUN echo "export PATH=$PATH:/root/miniconda3/bin:/usr/bin:/usr/local/cuda-11.2/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin" >>/root/.bashrc &&
    echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$CONDA_PREFIX/lib/" >>/root/.bashrc &&
    echo "source activate avt_hw_env" >>~/.bashrc &&
    echo "source /opt/ros/$ROS_DISTRO/setup.bash" >>~/.bashrc

# Clone task repo and build it
ENV AVT_HOME="$HOME/avt_homework/"
RUN git clone https://github.com/MahirGulzar/avt_homework.git $AVT_HOME &&
    source /opt/ros/$ROS_DISTRO/setup.bash &&
    cd $AVT_HOME &&
    catkin_make

# Use ros entrypoint bash
COPY ros_entrypoint.sh ./
RUN chmod +x ros_entrypoint.sh

ENTRYPOINT [ "/bin/bash", "ros_entrypoint.sh"]
