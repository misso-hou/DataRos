FROM ubuntu:focal

ENV DEBIAN_FRONTEND=nontinteractive

#软件源会影响库的正常下载,这里需要注意
RUN sed -i "s@http://.*archive.ubuntu.com@http://mirrors.huaweicloud.com@g" /etc/apt/sources.list && \
    sed -i "s@http://.*security.ubuntu.com@http://mirrors.huaweicloud.com@g" /etc/apt/sources.list

# ros-core
# Ref: https://github.com/osrf/docker_images/blob/master/ros/noetic/ubuntu/focal/ros-core/Dockerfile
# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    # ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
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
    ros-noetic-ros-core=1.5.0-1* \
    && rm -rf /var/lib/apt/lists/*

# ros-base
# Ref: https://github.com/osrf/docker_images/blob/master/ros/noetic/ubuntu/focal/ros-base/Dockerfile
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
# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-ros-base=1.5.0-1* \
    && rm -rf /var/lib/apt/lists/*

# ros-robot
# Ref: https://github.com/osrf/docker_images/blob/master/ros/noetic/ubuntu/focal/robot/Dockerfile
# install ros packages
# RUN apt-get update && apt-get install -y --no-install-recommends \
#     ros-noetic-robot=1.5.0-1* \
#     && rm -rf /var/lib/apt/lists/*

# ros-desktop
# Ref: https://github.com/osrf/docker_images/blob/master/ros/noetic/ubuntu/focal/desktop/Dockerfile
# install ros packages
# RUN apt-get update && apt-get install -y --no-install-recommends \
#     ros-noetic-desktop=1.5.0-1* \
#     && rm -rf /var/lib/apt/lists/*

# ros-desktop-full
# Ref: https://github.com/osrf/docker_images/blob/master/ros/noetic/ubuntu/focal/desktop-full/Dockerfile
# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop-full=1.5.0-1* \
    ros-noetic-plotjuggler \   
    ros-noetic-plotjuggler-ros \
    && rm -rf /var/lib/apt/lists/*

# ros-core
# Ref: https://github.com/osrf/docker_images/blob/master/ros/noetic/ubuntu/focal/ros-core/Dockerfile
# setup entrypoint
COPY ./thirdparty/ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]


#更新软件安装包列表
RUN apt-get update && apt-get upgrade -y

#安装基础库
RUN apt-get install -y build-essential libssl-dev \
    git wget unzip libboost-all-dev libeigen3-dev \
    clang-format

# 下载并完整安装 CMake 3.26
RUN wget https://github.com/Kitware/CMake/releases/download/v3.26.0/cmake-3.26.0-linux-x86_64.tar.gz && \
    tar -zxvf cmake-3.26.0-linux-x86_64.tar.gz && \
    # 移动到 /opt 目录
    mv cmake-3.26.0-linux-x86_64 /opt/cmake-3.26.0 && \
    # 创建符号链接
    ln -sf /opt/cmake-3.26.0/bin/cmake /usr/local/bin/cmake && \
    ln -sf /opt/cmake-3.26.0/bin/ccmake /usr/local/bin/ccmake && \
    ln -sf /opt/cmake-3.26.0/bin/ctest /usr/local/bin/ctest && \
    ln -sf /opt/cmake-3.26.0/bin/cpack /usr/local/bin/cpack && \
    # 设置环境变量
    echo 'export PATH=/opt/cmake-3.26.0/bin:$PATH' >> /etc/profile.d/cmake.sh && \
    # 清理
    rm -f cmake-3.26.0-linux-x86_64.tar.gz && \
    # 验证
    cmake --version

#安装python3及其以来
RUN apt-get update && apt-get install -y python3 python3-dev python3-pip python3-pyqt5
#使用python3-pip下載安装numpy，matplotlib, pyqt5
RUN pip3 install -i https://mirrors.aliyun.com/pypi/simple numpy==1.22.0 matplotlib==3.7.5
#安装pybind11混合编译器
RUN apt-get update && apt install pybind11-dev

#install opencv
RUN apt-get update && apt-get install libopencv-dev -y
#opencv4软连接
RUN ln -s /usr/include/opencv4/opencv2 /usr/include/opencv2

# #osqp库安装
RUN wget -c https://github.com/osqp/osqp/archive/refs/heads/master.zip -P /tmp/library/osqp \
    && unzip /tmp/library/osqp/master.zip -d /tmp/library/osqp  \
    && mkdir /tmp/library/osqp/osqp-master/build  \
    && cd /tmp/library/osqp/osqp-master/build  \
    && cmake -G "Unix Makefiles" .. \
    && cmake --build . --target install
    # && rm /tmp/library/osqp -r

#osqp-eigen库安装
RUN wget -c https://github.com/robotology/osqp-eigen/archive/refs/heads/master.zip -P /tmp/library/osqp-eigen  \
    && unzip /tmp/library/osqp-eigen/master.zip -d /tmp/library/osqp-eigen  \
    && mkdir /tmp/library/osqp-eigen/osqp-eigen-master/build  \
    && cd /tmp/library/osqp-eigen/osqp-eigen-master/build  \
    && cmake -DCMAKE_INSTALL_PREFIX:PATH=/usr/local .. \
    && make \
    && make install
    # && rm /home/osqp-eigen -r
ENV OsqpEigen_DIR=/usr/local/include/OsqpEigen

RUN wget -c https://github.com/xtensor-stack/xtl/archive/refs/heads/master.zip -P /tmp/library/xtl \
    && unzip /tmp/library/xtl/master.zip -d /tmp/library/xtl  \
    && mkdir /tmp/library/xtl/xtl-master/build  \
    && cd /tmp/library/xtl/xtl-master/build  \
    && cmake .. \
    && make install

RUN wget -c https://github.com/xtensor-stack/xtensor/archive/refs/heads/master.zip -P /tmp/library/xtensor \
    && unzip /tmp/library/xtensor/master.zip -d /tmp/library/xtensor  \
    && mkdir /tmp/library/xtensor/xtensor-master/build  \
    && cd /tmp/library/xtensor/xtensor-master/build  \
    && cmake ..  \
    && make install

# matplotlibcpp17库安装
RUN wget -c https://github.com/soblin/matplotlibcpp17/archive/refs/heads/master.zip -P /tmp/library/matplotlibcpp17 \
    && unzip /tmp/library/matplotlibcpp17/master.zip -d /tmp/library/matplotlibcpp17  \
    && mkdir /tmp/library/matplotlibcpp17/matplotlibcpp17-master/build  \
    && cd /tmp/library/matplotlibcpp17/matplotlibcpp17-master/build  \
    && cmake -DADD_DEMO=0 .. \
    && make -j20  \
    && make install 
    # && rm /home/matplotlibcpp17 -r

CMD ["/bin/bash"]