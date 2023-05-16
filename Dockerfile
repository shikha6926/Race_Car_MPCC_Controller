# syntax=docker/dockerfile:1
FROM ubuntu:focal

# -------- Basic ROS setup --------
# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends tzdata && \
    rm -rf /var/lib/apt/lists/*

# install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    && rm -rf /var/lib/apt/lists/*

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros1-latest.list

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

ENV ROS_DISTRO noetic

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop=1.5.0-1* \
    && rm -rf /var/lib/apt/lists/*

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    python3-rosdep \
    python3-rosinstall \
    python3-vcstools \
    && rm -rf /var/lib/apt/lists/*

# bootstrap rosdep
RUN rosdep init && \
  rosdep update --rosdistro $ROS_DISTRO

# -------- CRS setup --------

WORKDIR /code

# PYTHON DEPENDENCIES
RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    libsasl2-dev \
    python3-pip \
    libgmp3-dev \
    libsnmp-dev \
    libeigen3-dev \
    ros-noetic-plotjuggler \
    python3-tk \
    # required by Matplotlib
    libfreetype6-dev \
    # required by wifi_com
    protobuf-compiler \
    libprotobuf-dev \
    && rm -rf /var/lib/apt/lists/*


# INSTALL REQUIREMENTS
COPY requirements.txt requirements.txt
RUN python3 -m pip install --no-cache-dir -r requirements.txt

# Install catkin
RUN pip3 install --no-cache-dir catkin_tools

COPY .crs/ .crs/
COPY .setup/ .setup/
RUN sudo ./.crs/setup-crs.sh

# IPOPT
RUN apt-get update && apt-get install -y --no-install-recommends \
    gfortran \
    coinor-libipopt-dev \
    && rm -rf /var/lib/apt/lists/*

# CASADI
RUN sudo ./.setup/ubuntu/install_casadi.sh

# ACADOS
RUN sudo ./.setup/ubuntu/acados_setup.sh
ENV ACADOS_SOURCE_DIR="/acados"
ENV LD_LIBRARY_PATH="/acados/lib:$LD_LIBRARY_PATH"

# OPENMP SETTINGS
ENV OMP_NUM_THREADS=2
ENV OMP_PLACES=cores
ENV OMP_PROC_BIND=TRUE

# GTEST
RUN sudo ./.setup/ubuntu/install_gtest.sh

# MANIPULATE BASHRC
RUN echo "[[ -f /code/devel/setup.bash ]] && source /code/devel/setup.bash" >> ~/.bashrc
