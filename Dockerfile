FROM osrf/ros:humble-desktop

ARG DEBIAN_FRONTEND=noninteractive

# System dependencies for building ROS 2 Python packages and running tools.
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-colcon-common-extensions \
    ros-dev-tools \
    git \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Python deps used by the nodes (pyserial, numpy, yaml).
RUN pip3 install --no-cache-dir \
    pyserial \
    numpy \
    pyyaml

# Useful defaults for interactive devcontainers.
ENV PYTHONUNBUFFERED=1
SHELL ["/bin/bash", "-lc"]

