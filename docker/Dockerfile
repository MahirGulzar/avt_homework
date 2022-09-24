FROM nvidia/cuda:11.2.0-devel-ubuntu20.04

ENV PATH="/root/miniconda3/bin:${PATH}:"
ARG PATH="/root/miniconda3/bin:${PATH}"
ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=Europe/Tallinn
RUN apt update && apt-get install -y tzdata

ARG ROS_DISTRO_ARG="noetic"
ENV LOGNAME root
ENV ROS_DISTRO ${ROS_DISTRO_ARG}
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

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

RUN apt-get update &&
    apt-get install -y git wget &&
    apt-get install nano &&
    apt-get install -y python

RUN apt-get update && apt-get install -y python3-pip

# Miniconda
RUN wget \
    https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh &&
    mkdir /root/.conda &&
    bash Miniconda3-latest-Linux-x86_64.sh -b &&
    rm -f Miniconda3-latest-Linux-x86_64.sh

RUN conda --version

SHELL ["conda", "run", "-n", "base", "/bin/bash", "-c"]
# RUN conda install mamba -c conda-forge -y

RUN conda create -n avt_hw_env python=3.9

RUN conda install -c conda-forge catkin_tools

# Make RUN commands use the new environment:
SHELL ["conda", "run", "-n", "avt_hw_env", "/bin/bash", "-c"]
RUN conda install -c conda-forge cudatoolkit=11.2 cudnn=8.1.0 -y

COPY requirements.txt ./

RUN pip install -r requirements.txt

RUN git clone https://github.com/MahirGulzar/avt_homework.git ~/avt_homework

# RUN echo "export PATH=$PATH:/usr/bin:/usr/local/cuda-11.2/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin" >> /root/.bashrc \
#     &&  echo "export LD_LIBRARY_PATH=/usr/local/cuda-11.2/lib64:$LD_LIBRARY_PATH" >> /root/.bashrc
#     &&  echo "source activate avt_hw_env" >> ~/.bashrc

RUN echo "export PATH=$PATH:/root/miniconda3/bin:/usr/bin:/usr/local/cuda-11.2/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin" >>/root/.bashrc &&
    echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$CONDA_PREFIX/lib/" >>/root/.bashrc &&
    echo "source activate avt_hw_env" >>~/.bashrc

RUN echo "source /opt/ros/noetic/setup.bash" >>~/.bashrc

CMD ["bash"]
