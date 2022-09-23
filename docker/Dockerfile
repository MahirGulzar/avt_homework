FROM nvidia/cuda:11.2.0-runtime-ubuntu18.04

ENV PATH="/root/miniconda3/bin:${PATH}"
ARG PATH="/root/miniconda3/bin:${PATH}"
RUN apt-get update

RUN apt-get install -y wget && rm -rf /var/lib/apt/lists/*
RUN apt-get update && apt-get install -y \
    python-pip

RUN wget \
    https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh \
    && mkdir /root/.conda \
    && bash Miniconda3-latest-Linux-x86_64.sh -b \
    && rm -f Miniconda3-latest-Linux-x86_64.sh 

RUN conda --version
# Make RUN commands use the new environment:
SHELL ["conda", "run", "-n", "base", "/bin/bash", "-c"]
RUN conda install mamba -c conda-forge -y \
    && mamba create -n robostackenv ros-noetic-desktop python=3.9 -c robostack -c robostack-experimental -c conda-forge --no-channel-priority --override-channels -y \
    && conda install -c conda-forge catkin_tools

COPY requirements.txt ./

# Make RUN commands use the new environment:
SHELL ["conda", "run", "-n", "robostackenv", "/bin/bash", "-c"]
RUN conda install -c conda-forge cudatoolkit=11.2 cudnn=8.1.0 -y
RUN pip install -r requirements.txt

RUN echo "export PATH=/root/miniconda3/bin:/usr/local/cuda-11.2/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin" >> /root/.bashrc \
    &&  echo "export LD_LIBRARY_PATH=/usr/local/cuda-11.2/lib64:$CONDA_PREFIX/lib/" >> /root/.bashrc

RUN apt-get update && \
    apt-get install -y git && \
    apt-get install nano

RUN echo "source activate robostackenv" >> ~/.bashrc

USER root
ENTRYPOINT ["/bin/bash"]
