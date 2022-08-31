[TOC]

# 1 节点

ros2中的节点与ros1中的节点概念相同，都是一个进程。

## 1.1 相关命令

相比于ros1,ros2对命令进行了集成，均使用ros2命令来操作。

### 1.1.1 启动节点

使用指令：

```
ros2 run <package_name> <executable_name>
```

指令意义：启动 包下的 中的节点。

使用样例：

```
ros2 run turtlesim turtlesim_node
```

大家可以尝试一下上面的指令，就是我们在第二章中启动小乌龟模拟器的那条指令。

运行之后可以看到一只小乌龟，接下来就可以试试下一节中提到的几个指令来查看节点信息和列表。

### 1.1.2 查看节点信息

运行节点(常用)

```
ros2 run <package_name> <executable_name>
```

查看节点列表(常用)：

```
ros2 node list
```

查看节点信息(常用)：

```
ros2 node info <node_name>
```

重映射节点名称

```
ros2 run turtlesim turtlesim_node --ros-args --remap __node:=my_turtle
```

# 2. 工作空间

## 2.1 创建工作区

与ros1相同，ros2的工作空间也需要一个src目录，用来保存其他软件包

```bash
mkdir -p ~/dev_ws/src
cd ~/dev_ws/src
```

使用colcon构建工作区

```bash
colcon build
```

构建工作区之后，工作区会包含4个目录

```bash
# ls
build  install  log  src
```

## 2.2 目录结构

工作区的目录结构如下所示

```bash
# tree
.
├── build
│   └── COLCON_IGNORE
├── install
│   ├── COLCON_IGNORE
│   ├── _local_setup_util_ps1.py
│   ├── _local_setup_util_sh.py
│   ├── local_setup.bash
│   ├── local_setup.ps1
│   ├── local_setup.sh
│   ├── local_setup.zsh
│   ├── setup.bash
│   ├── setup.ps1
│   ├── setup.sh
│   └── setup.zsh
├── log
│   ├── COLCON_IGNORE
│   ├── build_2022-08-30_00-39-45
│   │   ├── events.log
│   │   └── logger_all.log
│   ├── latest -> latest_build
│   └── latest_build -> build_2022-08-30_00-39-45
└── src
```

# 3 package 软件包（ament 包）

ROS 2中的包创建使用ament作为其构建系统，colcon作为其构建工具。

一个package中必须要包含的文件有两个，分别是`package.xml`和`CMakeList.txt`，这一点与ros1相同。

一个最简单的包，可能具有如下所示的文件结构：

```bash
my_package/
     CMakeLists.txt
     package.xml
```

## 3.1 创建包

首先要进入工作区的src目录

```bash
cd ~/dev_ws/src
```

创建包的命令语法为：

```bash
ros2 pkg create --build-type ament_cmake <package_name>
```

也可以使用可选参数`--node-name`指定节点的名字

```bash
ros2 pkg create --build-type ament_cmake --node-name my_node my_package
```

## 3.2 构建包

在完成包的编写之后，需要对包进行构建才能使用。在工作空间直接执行下面的命令即可构建工作空间内所有的包

```bash
colcon build
```

也可以指定对某个包进行构建

```bash
colcon build --packages-select my_package
```

## 3.3 使用包

首先需要更新环境变量

```bash
. install/setup.bash
```

或

```bash
source install/setup.bash
```

之后可以通过ros2调用新的包，命令如下

```bash
ros2 run my_package my_node
```



# 4 colcon-构建工具

ros1使用`catkin`作为构建系统，ros2中使用`ament`作为构建系统。更改`catkin`的名字为`ament`的原因是希望不和 `catkin` 冲突 

`ament`由几个重要的库组成：

- [The `ament_package` Package](http://docs.ros.org/en/humble/Concepts/About-Build-System.html?highlight=colcon#the-ament-package-package)
- [The `ament_cmake` Repository](http://docs.ros.org/en/humble/Concepts/About-Build-System.html?highlight=colcon#the-ament-cmake-repository)
- [The `ament_lint` Repository](http://docs.ros.org/en/humble/Concepts/About-Build-System.html?highlight=colcon#the-ament-lint-repository)
- [Build tools](http://docs.ros.org/en/humble/Concepts/About-Build-System.html?highlight=colcon#build-tools)

##　4.1 ament_package

由ament Python package组成，为ament提供各种实用工具。

ament包必须包含一个`package.xml`文件， `package.xml`"清单" 文件包含处理和操作包所需的信息。此包信息包括 包的名称 (全局唯一) 和包的依赖项。`package.xml`文件也用作标记文件，指示包在文件系统上的位置。

`package.xml`文件的解析由`catkin_pkg`完成（这一点与ros1相同），而定位包的位置则是通过`colcon`实现的。



## 4.2 构建工具

对于ROS 2到Ardent的发行版本，提供此功能的构建工具被称作 `ament_tools` 。从ROS 2 Bouncy开始，如 [通用构建工具文章](http://design.ros2.org/articles/build_tool.html) 中所描述的， `ament_tools` 已经被 `colcon` 取代。