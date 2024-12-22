# **********************************************************
#      2024 : Arthur JEULIN
#      Copyright (c) Arthur JEULIN 
#      Created by Arthur JEULIN on 12/30/2024.
# **********************************************************
#
# Module : ROS 2 environment on Jetson
#

# Image de base adaptée à votre environnement
FROM dustynv/ros:humble-desktop-l4t-r35.4.1

# Mise à jour et installation des dépendances nécessaires
RUN apt-get update && apt-get install -y \
    autoconf \
    automake \
    apt-utils \
    bison \
    build-essential \
    checkinstall \
    curl \
    gdb \
    gdbserver \
    git \
    git-lfs \
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
    pkg-config \
    python3 \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
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
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Ajouter des commandes au .bashrc
RUN echo "source /opt/ros/humble/install/setup.bash" >> ~/.bashrc && \
    echo "export ISAAC_ROS_WS=/isaac_ros-dev" >> ~/.bashrc

# Créer un espace de travail pour Isaac ROS
RUN mkdir -p /isaac_ros-dev/src

# Définir un point d'entrée par défaut
ENTRYPOINT ["bash"]
