FROM osrf/ros:humble-desktop AS release

ARG DEBIAN_FRONTEND=noninteractive
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=1000

# System dependencies for building ROS 2 Python packages and running tools.
RUN apt-get update && apt-get install -y --no-install-recommends \
    bash-completion \
    curl \
    git \
    less \
    python3-colcon-common-extensions \
    python3-numpy \
    python3-pytest \
    python3-serial \
    python3-yaml \
    ros-dev-tools \
    sudo \
    usbutils \
    && rm -rf /var/lib/apt/lists/*

RUN groupadd --gid "${USER_GID}" "${USERNAME}" \
    && useradd --uid "${USER_UID}" --gid "${USER_GID}" -m "${USERNAME}" \
    && usermod -aG dialout,plugdev "${USERNAME}" \
    && echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > "/etc/sudoers.d/${USERNAME}" \
    && chmod 0440 "/etc/sudoers.d/${USERNAME}"

# Useful defaults for interactive devcontainers.
ENV PYTHONUNBUFFERED=1
SHELL ["/bin/bash", "-lc"]

