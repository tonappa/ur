# recall arguments from docker-compose
ARG ROS_DISTRO=jazzy
ARG ROS_VERSION=2

# base image
FROM ros:${ROS_DISTRO}

# ================================== ARGS ================================== #
# re-declare ARGs after FROM (they reset after each FROM)
ARG ROS_DISTRO
ARG ROS_VERSION
ARG MYUID=1000
ARG MYGID=1000
ARG USER=ros
ARG WORKSPACE_NAME=ros_ws

# ================================ ENV ================================ #
ENV DEBIAN_FRONTEND=noninteractive

# ============================ common packages ============================ #
RUN apt-get update && apt-get install -y --no-install-recommends \
    sudo \
    git \
    bash-completion \
    build-essential \
    python3-pip \
    python3-dev \
    python3-rosdep \
    curl \
    wget \
    unzip \
    gnupg \
    lsb-release \
    apt-utils \
    software-properties-common \
    iputils-ping \
    vim \
    figlet \
    toilet \
    libgl1 \
    libsm6 \
    libxext6 && \
    rm -rf /var/lib/apt/lists/*

# ============================ ROS2 build tools ============================ #
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-core \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-vcstool && \
    rm -rf /var/lib/apt/lists/*

# ============================ ROS packages ============================ #
RUN apt-get update && \
    rosdep update --rosdistro=${ROS_DISTRO} && \
    apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-ur \
    ros-jazzy-ur-simulation-gz \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-rqt-runtime-monitor \
    ros-${ROS_DISTRO}-plotjuggler-ros \
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-trac-ik-kinematics-plugin && \
    rm -rf /var/lib/apt/lists/*

# ========== Python packages ========== #
COPY docker/requirements.txt /tmp/requirements.txt
RUN pip install --break-system-packages --no-cache-dir -r /tmp/requirements.txt

# ============================= user creation (as ROOT) ============================ #
RUN OLD_USER=$(getent passwd ${MYUID} | cut -d: -f1) \
    && OLD_GROUP=$(getent group ${MYGID} | cut -d: -f1) \
    && if [ -n "${OLD_GROUP}" ] && [ "${OLD_GROUP}" != "${USER}" ]; then \
    groupmod -n ${USER} ${OLD_GROUP}; \
    fi \
    && if [ -n "${OLD_USER}" ] && [ "${OLD_USER}" != "${USER}" ]; then \
    usermod -l ${USER} ${OLD_USER} \
    && usermod -d /home/${USER} -m ${USER}; \
    fi \
    && if ! id -u ${USER} >/dev/null 2>&1; then \
    addgroup --gid ${MYGID} ${USER} \
    && adduser --gecos "ROS User" --disabled-password --uid ${MYUID} --gid ${MYGID} ${USER}; \
    fi \
    && usermod -a -G dialout,video,sudo ${USER} \
    && echo "${USER} ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/${USER} \
    && chmod 0440 /etc/sudoers.d/${USER}

# =============================== switch to ros user ============================== #
USER ${USER}
ENV HOME=/home/${USER}

# ================================ workspace config (as USER) ================================ #
# Note: workspace source is mounted as a volume at runtime — build with:
#   colcon build --cmake-args -DCMAKE_CXX_FLAGS="-w" --executor sequential
WORKDIR /home/${USER}/${WORKSPACE_NAME}
RUN mkdir -p ./src

# ==================================== entrypoint (as USER) ==================================== #
COPY docker/entrypoint.sh /usr/local/bin/entrypoint.sh
RUN sudo chmod +x /usr/local/bin/entrypoint.sh

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]
