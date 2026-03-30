#!/bin/bash
NAME="ros-noetic-desktop-full"
CONTAINER_NAME="ros_noetic_gui"
TAG="test"
HOST_PATH=$(dirname $(dirname $(dirname $PWD)))
echo $HOST_PATH

# 接收传入的参数
ROS_MASTER_IP="$2"

#创建docker镜像
build_docker() {
    docker build -t ${NAME}:${TAG} .
}

#启动镜像
run_docker() {
    xhost +local:root > /dev/null 2>&1
    # 创建临时.bashrc文件
    cat > /tmp/docker_bashrc << 'EOF'
# 终端标识:
export PS1='\[\033[01;38;5;214m\]root@ROS:\W>\[\033[00m\] '
EOF

    # 设置链接的ros master
    ROS_ENV=""
    if [ -n "$ROS_MASTER_IP" ]; then
        if [ "$ROS_MASTER_IP" = "host" ]; then
            ROS_ENV=(--env ROS_MASTER_URI=http://172.17.0.1:11311)
            echo "✅ 模式：连接宿主机 ROS Master"
        else
            # 连接 外部ADU 的 ROS Master
            ROS_ENV=(--env ROS_MASTER_URI=http://$ROS_MASTER_IP:11311)
            echo "✅ 模式：连接外部 ADU ROS Master -> $ROS_MASTER_IP"
        fi
    else
        # 不设置，使用 docker 内部自己的 master
        ROS_ENV=()
        echo "✅ 模式：Docker 内部独立 ROS Master"
    fi

    #启动docker
    sudo docker run -v /tmp/.X11-unix:/tmp/.X11-unix   \
    -e DISPLAY=unix$DISPLAY                            \
    -e QT_X11_NO_MITSHM=1                              \
    -e LIBGL_ALWAYS_SOFTWARE=1                         \
    -e GLX_FORCE_INDIRECT=1                            \
    -e LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libGL.so.1 \
    --rm -it --privileged                              \
    --name ${CONTAINER_NAME}                           \
    --net=host                                         \
    -v $HOST_PATH/:/home/workspace                     \
    -v /tmp/docker_bashrc:/root/.bashrc                \
    "${ROS_ENV[@]}"                                    \
    ${NAME}:${TAG} /bin/bash -c "
        echo 'source /home/workspace/HMS/DataRos/devel/setup.bash' >> ~/.bashrc &&
        cd /home/workspace/HMS/DataRos && 
        exec /bin/bash
    "
}

exec_docker() {
  if [ $# -eq 0 ];then
    sudo docker exec -it ${CONTAINER_NAME} /bin/bash -c "cd /home/workspace/HMS/DataRos && source /opt/ros/noetic/setup.bash && exec /bin/bash"
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
*) 
    ;;
esac
