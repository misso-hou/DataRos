#!/bin/bash
NAME="ros-noetic-desktop-full"
CONTAINER_NAME="ros_noetic_gui"
TAG="test"
HOST_PATH=$(dirname $(dirname $(dirname $PWD)))
echo $HOST_PATH

#创建docker镜像
build_docker() {
    docker build -t ${NAME}:${TAG} .
}

#启动镜像
run_docker() {
    xhost +local:root > /dev/null 2>&1
    # 创建临时.bashrc文件
    cat > /tmp/docker_bashrc << 'EOF'
# 自定义PS1
# export PS1='root@ROS> '
# 或者带颜色的简短版本
export PS1='\[\033[01;38;5;214m\]root@ROS:\W>\[\033[00m\] '
# 设置其他需要的环境变量
# source /opt/ros/noetic/setup.bash  # 如果需要
EOF

    sudo docker run -v /tmp/.X11-unix:/tmp/.X11-unix \
    -e DISPLAY=unix$DISPLAY                      \
    -e QT_X11_NO_MITSHM=1                        \
    -e LIBGL_ALWAYS_SOFTWARE=1                   \
    -e GLX_FORCE_INDIRECT=1                      \
    -e LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libGL.so.1 \
    --rm -it --privileged                        \
    --env ROS_MASTER_URI=http://172.17.0.1:11311 \
    --name ${CONTAINER_NAME}                     \
    --net=host                                   \
    -v $HOST_PATH/:/home/workspace               \
    -v /tmp/docker_bashrc:/root/.bashrc          \
    ${NAME}:${TAG} /bin/bash -c "
        # echo 'source /home/workspace/HMS/DataRos/devel/setup.bash' >> ~/.bashrc &&
        # cd /home/workspace/HMS/catkin_ws && 
        exec /bin/bash
    "
}

exec_docker() {
  if [ $# -eq 0 ];then
    sudo docker exec -it ${CONTAINER_NAME} /bin/bash -c "cd /home/workspace/HMS/catkin_ws && source /opt/ros/noetic/setup.bash && exec /bin/bash"
  elif [ $1 = "build" ];then
    shift
    sudo docker exec ${NAME}_${TAG} /bin/bash -c "cd /home/workspace/ && ./build.sh $@"
  fi
}

case $1 in
build)
    build_docker
    ;;
run)
    run_docker
    ;;
exec)
    shift
    exec_docker $@
    ;;
clean)
    docker image prune -f
    ;;
*) ;;
esac
