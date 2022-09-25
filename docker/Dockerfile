FROM nvidia/cuda:11.2.0-devel-ubuntu20.04

ARG ROS_DISTRO_ARG="noetic"
ENV ROS_DISTRO ${ROS_DISTRO_ARG}
ENV TZ=Europe/Tallinn
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get -y install tzdata sudo

# Add avt user
ENV USERNAME avt_user
ARG USER_ID=1000
ARG GROUP_ID=15214

# Creating avt user
RUN groupadd --gid $GROUP_ID $USERNAME && \
    useradd --gid $GROUP_ID -m $USERNAME && \
    echo "$USERNAME:$USERNAME" | chpasswd && \
    usermod --shell /bin/bash $USERNAME && \
    usermod -aG sudo $USERNAME && \
    usermod  --uid $USER_ID $USERNAME && \
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

USER ${USERNAME}

# Install python and other required packages
RUN sudo apt update && sudo apt install -y -qq --no-install-recommends \
    git \
    wget \
    python \
    python3-pip \
    ca-certificates && \
    sudo update-ca-certificates

# Setup ROS
RUN sudo apt-get update -y || true && \
    sudo apt-get install -y lsb-release && sudo apt clean all && \
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    sudo apt install -y curl && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add - && \
    sudo apt-get update || true && \
    sudo apt-get install -y ros-$ROS_DISTRO-ros-base build-essential cmake usbutils libusb-1.0-0-dev ros-$ROS_DISTRO-cv-bridge git -y --allow-unauthenticated

# Install Packages depending on ROS distro
RUN sudo apt-get install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool

RUN sudo rm -rf /var/lib/apt/lists/*

# Setup working directory
USER ${USERNAME}
ENV HOME="/home/${USERNAME}"
WORKDIR ${HOME}

# Miniconda
ENV PATH="${HOME}/miniconda3/bin:${PATH}"
ARG PATH="${HOME}/miniconda3/bin:${PATH}"

RUN sudo apt-get update && sudo wget \
    https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh && \
    sudo mkdir .conda && \
    bash Miniconda3-latest-Linux-x86_64.sh -b && \
    sudo rm -f Miniconda3-latest-Linux-x86_64.sh

RUN conda init bash

SHELL ["conda", "run", "-n", "base", "/bin/bash", "-c"]
RUN conda create -n avt_hw_env python=3.9 && \
    conda config --set channel_priority true --set ssl_verify false && \
    conda update --all --yes && \
    conda update conda && \
    conda install -c conda-forge catkin_tools 

# Install cudatoolkit & cudnn inside avt_hw_env environment
SHELL ["conda", "run", "-n", "avt_hw_env", "/bin/bash", "-c"]
RUN conda install -c conda-forge cudatoolkit=11.2 cudnn=8.1.0 -y

# Python vision libraries
COPY requirements.txt ./
RUN pip install -r requirements.txt

# Export variables to .bashrc
RUN echo "export PATH=$PATH:${HOME}/miniconda3/bin:/usr/bin:/usr/local/cuda-11.2/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin" >>${HOME}/.bashrc && \
    echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$CONDA_PREFIX/lib/" >>${HOME}/.bashrc && \
    echo "source activate avt_hw_env" >>~/.bashrc && \
    echo "source /opt/ros/$ROS_DISTRO/setup.bash" >>~/.bashrc

# Clone task repo and build it
ENV AVT_HOME="$HOME/avt_homework/"
RUN git clone https://github.com/MahirGulzar/avt_homework.git $AVT_HOME && \
    source /opt/ros/$ROS_DISTRO/setup.bash && \
    cd $AVT_HOME && \
    catkin_make

# Use ros entrypoint bash
COPY ros_entrypoint.sh ./
RUN sudo chmod +x ros_entrypoint.sh

ENTRYPOINT [ "/bin/bash", "ros_entrypoint.sh"]
