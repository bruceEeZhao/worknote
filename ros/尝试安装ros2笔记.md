[TOC]



# 安装记录

## 1. set locale

确认系统是UTF-8编码

```bash
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```



## 2. 安装基础软件

> ```bash
> sudo apt install software-properties-common
> sudo add-apt-repository universe
> sudo apt install -y \
> build-essential \
> cmake \
> git \
> wget python3-dev python3
> 
> pip install flake8 colcon-common-extensions pytest rosdep setuptools vcstool rosdepc
> ```





# question





# docker

docker run -idt -v /home/zcl/workspace/docker/docker-mount/ros/:/home --restart always  --privileged=true --name ros2 ubuntu:20.04