# **********************************************************
#      2024 : Arthur JEULIN
#      Copyright (c) Arthur JEULIN 
#      Created by Arthur JEULIN on 12/30/2024.
# **********************************************************
#
# Module : ROS 2 environment on Jetson
#

# FROM ros:humble
FROM docker.io/dustynv/ros:humble-desktop-l4t-r35.4.1

RUN apt-get update && apt-get install -y \
    autoconf \
    automake \
    apt-utils \
    bison \
    checkinstall \
    curl \
    gdb \
    gdbserver \
    git \
    libcanberra-gtk-module \
    libcurl4-openssl-dev \
    libfile-copy-recursive-perl \
    libxcb-cursor0 \
    libgles2-mesa-dev \
    libssl-dev \
    libtool \
    libxcursor-dev \
    libxdamage-dev \
    libdrm-dev \
    libxext-dev \
    libxft-dev \
    libxinerama-dev \
    libxi-dev \
    libxrandr-dev \
    libxrender-dev \
    libxtst-dev \
    libx11-dev \
    m4 \
    ninja-build \
    pkg-config build-essential \
    python3 \
    python3-pip \ 
    python3-setuptools \
    python3-wheel \
    software-properties-common \
    strace \
    tar \
    unzip \ 
    vim \
    wget \
    xdg-utils \
    zip \
    zlib1g-dev \
    && apt-get clean  \
    && rm -rf /var/lib/apt/lists/*

# Ajouter la commande pour sourcer le setup.bash dans le .bashrc
# RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /opt/ros/humble/install/setup.bash" >> ~/.bashrc

# RUN wget https://github.com/Kitware/CMake/releases/download/v3.27.5/cmake-3.27.5.tar.gz && \
# tar -zxvf cmake-3.27.5.tar.gz && \
# cd cmake-3.27.5 && \
# ./bootstrap && \
# make && \
# make install && \
# cd ..  && \
# rm cmake-3.27.5.tar.gz

ENTRYPOINT [ "bash" ]