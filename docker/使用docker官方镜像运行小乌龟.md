# 使用官方镜像运行小乌龟

## 1. 下载镜像

```bash
docker pull osrf/ros:humble-desktop-full
```

## 2. 运行容器

```bash
docker run -itd --privileged=true  -v /tmp/.X11-unix/:/tmp/.X11-unix/ -e DISPLAY=:1  osrf/ros:humble-desktop-full /bin/bash
```

若希望docker一直运行，退出之后容器不会消失，则需要添加

> --restart always 

## 3. 另起一个终端连接容器

```bash
# 假设已经知道容器的id
docker exec -it ee37906142 /bin/bash
```

## 4. 运行小乌龟demo

```bash
ros2 run turtlesim turtlesim_node

# 在另一个终端运行
ros2 run turtlesim turtle_teleop_key
```

