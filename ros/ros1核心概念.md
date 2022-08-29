[TOC]

# 1 ROS简介

。。。略

# 2 文件系统

## 2.1 Catkin编译系统

**==Catkin是基于Cmake的编译构建系统==**

<img src="https://sychaichangkun.gitbooks.io/ros-tutorial-icourse163/content/pics/catkin.jpg" alt="img" style="zoom:80%;" />

**==产生背景==**

- 由于gcc/g++命令方式的效率低下，因此产生了Makefile。
- 随着工程体量的增大，产生了Cmake工具。Cmake是对make工具的生成器，简化了编译构建过程，可以管理大型项目。
- ROS对Cmake进行可扩展，形成了Catkin编译系统。



**==必要文件==**

一个Catkin的软件包（package）必须要包括两个文件：package.xml和CMakeList.txt

1. package.xml: 包括了package的描述信息
   - name, description, version, maintainer(s), license
   - opt. authors, url's, dependencies, plugins, etc...
2. CMakeLists.txt: 构建package所需的CMake文件
   - 调用Catkin的函数/宏
   - 解析`package.xml`
   - 找到其他依赖的catkin软件包
   - 将本软件包添加到环境变量

### 2.1.1 Catkin特点

1. 沿用了包管理的传统，`find_package()`基础结构，`pkg-config`
2. 扩展了Cmake
   1. 软件包编译后无需安装即可使用
   2. 自动生成`find_package()`代码，`pkg-config`文件
   3. 解决多个软件包构建顺序问题

### 2.1.2 编译命令

```bash
$ cd ~/catkin_ws #回到工作空间,catkin_make必须在工作空间下执行
$ catkin_make    #开始编译
$ source ~/catkin_ws/devel/setup.bash #刷新坏境
```



命令可选参数

```
catkin_make [args]
  -h, --help            帮助信息
  -C DIRECTORY, --directory DIRECTORY
                        工作空间的路径 (默认为 '.')
  --source SOURCE       src的路径 (默认为'workspace_base/src')
  --build BUILD         build的路径 (默认为'workspace_base/build')
  --use-ninja           用ninja取代make
  --use-nmake           用nmake取'make
  --force-cmake         强制cmake，即使已经cmake过
  --no-color            禁止彩色输出(只对catkin_make和CMake生效)
  --pkg PKG [PKG ...]   只对某个PKG进行make
  --only-pkg-with-deps  ONLY_PKG_WITH_DEPS [ONLY_PKG_WITH_DEPS ...]
                        将指定的package列入白名单CATKIN_WHITELIST_PACKAGES，
                        之编译白名单里的package。该环境变量存在于CMakeCache.txt。
  --cmake-args [CMAKE_ARGS [CMAKE_ARGS ...]]
                        传给CMake的参数
  --make-args [MAKE_ARGS [MAKE_ARGS ...]]
                        传给Make的参数
  --override-build-tool-check
                        用来覆盖由于不同编译工具产生的错误
```



## 2.2 Catkin工作空间

### 2.2.1 初始化

```bash
$ mkdir -p ~/catkin_ws/src　　
$ cd ~/catkin_ws/
$ catkin_make #初始化工作空间
```

### 2.2.2 目录结构

工作空间在初始化后会包含3个文件夹，`build`,`devel`和`src`.

- src/: ROS的catkin软件包（源代码包）
- build/: catkin（CMake）的缓存信息和中间文件
- devel/: 生成的目标文件（包括头文件，动态链接库，静态链接库，可执行文件等）、环境变量

```
─ build
│   ├── catkin
│   │   └── catkin_generated
│   │       └── version
│   │           └── package.cmake
│   ├──
......

│   ├── catkin_make.cache
│   ├── CMakeCache.txt
│   ├── CMakeFiles
│   │   ├──
......

├── devel
│   ├── env.sh
│   ├── lib
│   ├── setup.bash
│   ├── setup.sh
│   ├── _setup_util.py
│   └── setup.zsh
└── src
└── CMakeLists.txt -> /opt/ros/kinetic/share/catkin/cmake/toplevel.cmake
```

在编译过程中，它们的工作流程如图：

<img src="https://sychaichangkun.gitbooks.io/ros-tutorial-icourse163/content/pics/catkin_flow.jpg" alt="img" style="zoom:80%;" />

后两个路径由catkin系统自动生成、管理，我们日常的开发一般不会去涉及，而主要用到的是src文件夹，我们写的ROS程序、网上下载的ROS源代码包都存放在这里。

在编译时，catkin编译系统会**递归**的查找和编译`src/`下的每一个源代码包。因此你也可以把几个源代码包放到同一个文件夹下，如下图所示：

<img src="https://sychaichangkun.gitbooks.io/ros-tutorial-icourse163/content/pics/catkin_ws.jpg" alt="img" style="zoom:80%;" />

## 2.3 Package软件包

ROS中的package的定义更加具体，它不仅是Linux上的软件包，更是catkin编译的基本单元，我们调用`catkin_make`编译的对象就是一个个ROS的package，也就是说任何ROS程序只有组织成package才能编译。所以package也是ROS源代码存放的地方，任何ROS的代码无论是C++还是Python都要放到package中，这样才能正常的编译和运行。
一个package可以编译出来多个目标文件（ROS可执行程序、动态静态库、头文件等等）。

### 2.3.1 package结构

一个package下常见的文件、路径有：

```
  ├── CMakeLists.txt    #package的编译规则(必须)
  ├── package.xml       #package的描述信息(必须)
  ├── src/              #源代码文件
  ├── include/          #C++头文件
  ├── scripts/          #可执行脚本
  ├── msg/              #自定义消息
  ├── srv/              #自定义服务
  ├── models/           #3D模型文件
  ├── urdf/             #urdf文件
  ├── launch/           #launch文件
```

其中定义package的是`CMakeLists.txt`和`package.xml`，这两个文件是package中必不可少的。catkin编译系统在编译前，首先就要解析这两个文件。这两个文件就定义了一个package。

- CMakeLists.txt: 定义package的包名、依赖、源文件、目标文件等编译规则，是package不可少的成分
- package.xml: 描述package的包名、版本号、作者、依赖等信息，是package不可少的成分
- src/: 存放ROS的源代码，包括C++的源码和(.cpp)以及Python的module(.py)
- include/: 存放C++源码对应的头文件
- scripts/: 存放可执行脚本，例如shell脚本(.sh)、Python脚本(.py)
- msg/: 存放自定义格式的消息(.msg)
- srv/: 存放自定义格式的服务(.srv)
- models/: 存放机器人或仿真场景的3D模型(.sda, .stl, .dae等)
- urdf/: 存放机器人的模型描述(.urdf或.xacro)
- launch/: 存放launch文件(.launch或.xml)

通常ROS文件组织都是按照以上的形式，这是约定俗成的命名习惯，建议遵守。以上路径中，只有`CMakeLists.txt`和`package.xml`是必须的，其余路径根据软件包是否需要来决定。

### 2.3.2 package的创建

创建一个package需要在`catkin_ws/src`下,用到`catkin_create_pkg`命令，用法是：

```bash
catkin_create_pkg package depends
```


其中package是包名，depends是依赖的包名，可以依赖多个软件包。

例如，新建一个package叫做`test_pkg`,依赖roscpp、rospy、std_msgs(常用依赖)。

```bash
$ catkin_create_pkg test_pkg roscpp rospy std_msgs
```

这样就会在当前路径下新建`test_pkg`软件包，包括：

```
  ├── CMakeLists.txt
  ├── include
  │   └── test_pkg
  ├── package.xml
  └── src
```

`catkin_create_pkg`帮你完成了软件包的初始化，填充好了`CMakeLists.txt`和`package.xml`，并且将依赖项填进了这两个文件中。

### 2.3.3 package相关命令

**rospack**

rospack是对package管理的工具，命令的用法如下：

|         rospack命令         |           作用            |
| :-------------------------: | :-----------------------: |
|       `rospack help`        |     显示rospack的用法     |
|       `rospack list`        |    列出本机所有package    |
| `rospack depends [package]` |    显示package的依赖包    |
|  `rospack find [package]`   |      定位某个package      |
|      `rospack profile`      | 刷新所有package的位置记录 |

以上命令如果package缺省，则默认为当前目录(如果当前目录包含package.xml)

**roscd**

`roscd`命令类似与Linux系统的`cd`，改进之处在于`roscd`可以直接`cd`到ROS的软件包。

|     roscd命令     |          作用           |
| :---------------: | :---------------------: |
| `roscd [pacakge]` | cd到ROS package所在路径 |

**rosls**

`rosls`也可以视为Linux指令`ls`的改进版，可以直接`ls`ROS软件包的内容。

|     rosls命令     |        作用         |
| :---------------: | :-----------------: |
| `rosls [pacakge]` | 列出pacakge下的文件 |

**rosdep**

`rosdep`是用于管理ROS package依赖项的命令行工具，用法如下：

|         rosdep命令         |            作用             |
| :------------------------: | :-------------------------: |
|  `rosdep check [pacakge]`  |  检查package的依赖是否满足  |
| `rosdep install [pacakge]` |      安装pacakge的依赖      |
|        `rosdep db`         |    生成和显示依赖数据库     |
|       `rosdep init`        | 初始化/etc/ros/rosdep中的源 |
|       `rosdep keys`        |  检查package的依赖是否满足  |
|      `rosdep update`       |   更新本地的rosdep数据库    |

一个较常使用的命令是`rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y`,用于安装工作空间中`src`路径下所有package的依赖项（由pacakge.xml文件指定）。



## 2.4 CMakeLists.txt

### 2.4.1 CMakeLists.txt作用

`CMakeLists.txt`原本是Cmake编译系统的规则文件，而Catkin编译系统基本沿用了CMake的编译风格，只是针对ROS工程添加了一些宏定义。所以在写法上，catkin的`CMakeLists.txt`与CMake的基本一致。

这个文件直接规定了这个package要依赖哪些package，要编译生成哪些目标，如何编译等等流程。所以`CMakeLists.txt`非常重要，它指定了由源码到目标文件的规则，catkin编译系统在工作时首先会找到每个package下的`CMakeLists.txt`，然后按照规则来编译构建。

### 2.4.1 CMakeLists.txt写法

`CMakeLists.txt`的基本语法都还是按照CMake，而Catkin在其中加入了少量的宏，总体的结构如下：

```cmake
cmake_minimum_required() #CMake的版本号 
project()                #项目名称 
find_package()           #找到编译需要的其他CMake/Catkin package
catkin_python_setup()    #catkin新加宏，打开catkin的Python Module的支持
add_message_files()      #catkin新加宏，添加自定义Message/Service/Action文件
add_service_files()
add_action_files()
generate_message()       #catkin新加宏，生成不同语言版本的msg/srv/action接口
catkin_package()         #catkin新加宏，生成当前package的cmake配置，供依赖本包的其他软件包调用
add_library()            #生成库
add_executable()         #生成可执行二进制文件
add_dependencies()       #定义目标文件依赖于其他目标文件，确保其他目标已被构建
target_link_libraries()  #链接
catkin_add_gtest()       #catkin新加宏，生成测试
install()                #安装至本机
```

如果你从未接触过CMake的语法，请阅读《CMake实践》：https://github.com/Akagi201/learning-cmake/blob/master/docs/cmake-practice.pdf 。掌握CMake语法对于理解ROS工程很有帮助。

### 2.4.2 CMakeLists例子

为了详细的解释`CMakeLists.txt`的写法，我们以turtlesim小海龟这个pacakge为例，读者可`roscd`到`tuetlesim`包下查看，在`turtlesim/CMakeLists.txt`的写法如下,:

```
cmake_minimum_required(VERSION 2.8.3)
#CMake至少为2.8.3版

project(turtlesim)
#项目(package)名称为turtlesim，在后续文件中可使用变量${PROJECT_NAME}来引用项目名称turltesim

find_package(catkin REQUIRED COMPONENTS geometry_msgs message_generation rosconsole roscpp roscpp_serialization roslib rostime std_msgs std_srvs)
#cmake宏，指定依赖的其他pacakge，实际是生成了一些环境变量，如<NAME>_FOUND, <NAME>_INCLUDE_DIRS, <NAME>_LIBRARYIS
#此处catkin是必备依赖 其余的geometry_msgs...为组件

find_package(Qt5Widgets REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
#指定C++的头文件路径
link_directories(${catkin_LIBRARY_DIRS})
#指定链接库的路径

add_message_files(DIRECTORY msg FILES
Color.msg Pose.msg)
#自定义msg文件

add_service_files(DIRECTORY srv FILES
Kill.srv
SetPen.srv
Spawn.srv
TeleportAbsolute.srv
TeleportRelative.srv)
#自定义srv文件

generate_messages(DEPENDENCIES geometry_msgs std_msgs std_srvs)
#在add_message_files、add_service_files宏之后必须加上这句话，用于生成srv msg头文件/module，生成的文件位于devel/include中

catkin_package(CATKIN_DEPENDS geometry_msgs message_runtime std_msgs std_srvs)
# catkin宏命令，用于配置ROS的package配置文件和CMake文件
# 这个命令必须在add_library()或者add_executable()之前调用，该函数有5个可选参数：
# (1) INCLUDE_DIRS - 导出包的include路径
# (2) LIBRARIES - 导出项目中的库
# (3) CATKIN_DEPENDS - 该项目依赖的其他catkin项目
# (4) DEPENDS - 该项目所依赖的非catkin CMake项目。
# (5) CFG_EXTRAS - 其他配置选项

set(turtlesim_node_SRCS
src/turtlesim.cpp
src/turtle.cpp
src/turtle_frame.cpp
)
set(turtlesim_node_HDRS
include/turtlesim/turtle_frame.h
)
#指定turtlesim_node_SRCS、turtlesim_node_HDRS变量

qt5_wrap_cpp(turtlesim_node_MOCS ${turtlesim_node_HDRS})

add_executable(turtlesim_node ${turtlesim_node_SRCS} ${turtlesim_node_MOCS})
# 指定可执行文件目标turtlesim_node
target_link_libraries(turtlesim_node Qt5::Widgets ${catkin_LIBRARIES} ${Boost_LIBRARIES})
# 指定链接可执行文件
add_dependencies(turtlesim_node turtlesim_gencpp)

add_executable(turtle_teleop_key tutorials/teleop_turtle_key.cpp)
target_link_libraries(turtle_teleop_key ${catkin_LIBRARIES})
add_dependencies(turtle_teleop_key turtlesim_gencpp)

add_executable(draw_square tutorials/draw_square.cpp)
target_link_libraries(draw_square ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(draw_square turtlesim_gencpp)

add_executable(mimic tutorials/mimic.cpp)
target_link_libraries(mimic ${catkin_LIBRARIES})
add_dependencies(mimic turtlesim_gencpp)
# 同样指定可执行目标、链接、依赖

install(TARGETS turtlesim_node turtle_teleop_key draw_square mimic
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
# 安装目标文件到本地系统

install(DIRECTORY images
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
FILES_MATCHING PATTERN "*.png" PATTERN "*.svg")
```



## 2.5 package.xml

`package.xml`也是一个catkin的package必备文件，它是这个软件包的描述文件，在较早的ROS版本(rosbuild编译系统)中，这个文件叫做`manifest.xml`，用于描述pacakge的基本信息。如果你在网上看到一些ROS项目里包含着`manifest.xml`，那么它多半是hydro版本之前的项目了。

### 2.5.1 package.xml作用

`pacakge.xml`包含了package的名称、版本号、内容描述、维护人员、软件许可、编译构建工具、编译依赖、运行依赖等信息。
实际上`rospack find`、`rosdep`等命令之所以能快速定位和分析出package的依赖项信息，就是直接读取了每一个pacakge中的`package.xml`文件。它为用户提供了快速了解一个pacakge的渠道。

### 2.5.2 package.xml写法

`pacakge.xml`遵循xml标签文本的写法，由于版本更迭原因，现在有两种格式并存（format1与format2），不过区别不大。老版本（format1）的`pacakge.xml`通常包含以下标签:

```xml
<pacakge>           根标记文件  
<name>              包名  
<version>           版本号  
<description>       内容描述  
<maintainer>        维护者 
<license>           软件许可证  
<buildtool_depend>  编译构建工具，通常为catkin  
<build_depend>      编译依赖项，与Catkin中的  
<run_depend>        运行依赖项
```

说明：其中1-6为必备标签，1是根标签，嵌套了其余的所有标签，2-6为包的各种属性，7-9为编译相关信息。

在新版本（format2）中，包含的标签为：

```xml
<pacakge>               根标记文件  
<name>                  包名  
<version>               版本号  
<description>           内容描述  
<maintainer>            维护者 
<license>               软件许可证  
<buildtool_depend>      编译构建工具，通常为catkin    
<depend>                指定依赖项为编译、导出、运行需要的依赖，最常用
<build_depend>          编译依赖项  
<build_export_depend>   导出依赖项
<exec_depend>           运行依赖项
<test_depend>           测试用例依赖项  
<doc_depend>            文档依赖项
```

由此看见新版本的`pacakge.xml`格式上增加了 、、、 ,相当于将之前的build和run依赖项描述进行了细分。

目前Indigo、Kinetic、Lunar等版本的ROS都同时支持两种版本的`package.xml`，所以无论选哪种格式都可以。

### 2.5.3 pacakge.xml例子

为了说明pacakge.xml写法，还是以turtlesim软件包为例，其`pacakge.xml`文件内容如下，我们添加了相关的注释：

```xml
<?xml version="1.0"?>       <!--本示例为老版本的pacakge.xml-->
<package>                   <!--pacakge为根标签，写在最外面-->
  <name>turtlesim</name>
  <version>0.8.1</version>
  <description>
    turtlesim is a tool made for teaching ROS and ROS packages.
  </description>
  <maintainer email="dthomas@osrfoundation.org">Dirk Thomas</maintainer>
  <license>BSD</license>

  <url type="website">http://www.ros.org/wiki/turtlesim</url>
  <url type="bugtracker">https://github.com/ros/ros_tutorials/issues</url>
  <url type="repository">https://github.com/ros/ros_tutorials</url>
  <author>Josh Faust</author>

  <!--编译工具为catkin-->
  <buildtool_depend>catkin</buildtool_depend>

  <!--编译时需要依赖以下包-->  
  <build_depend>geometry_msgs</build_depend>    
  <build_depend>qtbase5-dev</build_depend>
  <build_depend>message_generation</build_depend>
  <build_depend>qt5-qmake</build_depend>
  <build_depend>rosconsole</build_depend>
  <build_depend>roscpp</build_depend>
  <build_depend>roscpp_serialization</build_depend>
  <build_depend>roslib</build_depend>
  <build_depend>rostime</build_depend>
  <build_depend>std_msgs</build_depend>
  <build_depend>std_srvs</build_depend>

  <!--运行时需要依赖以下包-->
  <run_depend>geometry_msgs</run_depend>
  <run_depend>libqt5-core</run_depend>
  <run_depend>libqt5-gui</run_depend>
  <run_depend>message_runtime</run_depend>
  <run_depend>rosconsole</run_depend>
  <run_depend>roscpp</run_depend>
  <run_depend>roscpp_serialization</run_depend>
  <run_depend>roslib</run_depend>
  <run_depend>rostime</run_depend>
  <run_depend>std_msgs</run_depend>
  <run_depend>std_srvs</run_depend>
</package>
```

以上内容是老版本（format1）的写法，如果要写成新版本（format2）则可以改为：

```xml
<?xml version="1.0"?>
<package format="2">      <!--在声明pacakge时指定format2，为新版格式-->
  <name>turtlesim</name>
  <version>0.8.1</version>
  <description>
    turtlesim is a tool made for teaching ROS and ROS packages.
  </description>
  <maintainer email="dthomas@osrfoundation.org">Dirk Thomas</maintainer>
  <license>BSD</license>

  <url type="website">http://www.ros.org/wiki/turtlesim</url>
  <url type="bugtracker">https://github.com/ros/ros_tutorials/issues</url>
  <url type="repository">https://github.com/ros/ros_tutorials</url>
  <author>Josh Faust</author>

  <!--编译工具为catkin-->
  <buildtool_depend>catkin</buildtool_depend>

  <!--用depend来整合build_depend和run_depend-->  
  <depend>geometry_msgs</depend>
  <depend>rosconsole</depend>
  <depend>roscpp</depend>
  <depend>roscpp_serialization</depend>
  <depend>roslib</depend>
  <depend>rostime</depend>
  <depend>std_msgs</depend>
  <depend>std_srvs</depend>

  <!--build_depend标签未变-->
  <build_depend>qtbase5-dev</build_depend>
  <build_depend>message_generation</build_depend>
  <build_depend>qt5-qmake</build_depend>

  <!--run_depend要改为exec_depend-->
  <exec_depend>libqt5-core</exec_depend>
  <exec_depend>libqt5-gui</exec_depend>
  <exec_depend>message_runtime</exec_depend>
</package>
```

## 2.6 Metapackage

### 2.6.1 Metapackage介绍

在一些ROS的教学资料和博客里，你可能还会看到一个Stack（功能包集）的概念，它指的是将多个功能接近、甚至相互依赖的软件包的放到一个集合中去。但Stack这个概念在Hydro之后就取消了，取而代之的就是Metapackage。尽管换了个马甲，但它的作用没变，都是把一些相近的功能模块、软件包放到一起。

ROS里常见的Metapacakge有：

| Metapacakge名称 |                  描述                  |                      链接                       |
| :-------------: | :------------------------------------: | :---------------------------------------------: |
|   navigation    |           导航相关的功能包集           |   https://github.com/ros-planning/navigation    |
|     moveit      | 运动规划相关的（主要是机械臂）功能包集 |     https://github.com/ros-planning/moveit      |
| image_pipeline  |      图像获取、处理相关的功能包集      | https://github.com/ros-perception/image_common  |
|  vision_opencv  |       ROS与OpenCV交互的功能包集        | https://github.com/ros-perception/vision_opencv |
|    turtlebot    |     Turtlebot机器人相关的功能包集      |     https://github.com/turtlebot/turtlebot      |
|    pr2_robot    |         pr2机器人驱动功能包集          |        https://github.com/PR2/pr2_robot         |
|       ...       |                  ...                   |                       ...                       |

以上列举了一些常见的功能包集，例如navigation、turtlebot，他们都是用于某一方面的功能，以navigation metapackage（官方介绍里仍然沿用stack的叫法）为例，它包括了以下软件包：

|        包名        |               功能               |      |
| :----------------: | :------------------------------: | ---- |
|     navigation     | Metapacakge，依赖以下所有pacakge |      |
|        amcl        |               定位               |      |
| fake_localization  |               定位               |      |
|     map_server     |             提供地图             |      |
|     move_base      |           路径规划节点           |      |
|      nav_core      |         路径规划的接口类         |      |
| base_local_planner |             局部规划             |      |
| dwa_local_planner  |             局部规划             |      |
|        ...         |               ...                | ...  |

具体功能介绍，我们留到第九章，这里只看一个软件包navigation。这个navigation就是一个简单的pacakge，里面只有几个文件，但由于它依赖了其他所有的软件包。Catkin编译系统会明白，这些软件包都属于navigation metapacakge。

这个道理并不难理解，比如我们在安装ROS时，用到了`sudo apt-get install ros-kinetic-desktop-full`命令，由于它依赖了ROS所有的核心组件，我们在安装时也就能够安装整个ROS。

### 2.6.2 Metapackage写法

我们以ROS-Academy-for-beginners为例介绍meteapckage的写法，在教学包内，有一个`ros-academy-for-beginners`软件包，该包即为一个metapacakge，其中有且仅有两个文件：`CMakeLists.txt`和`pacakge.xml`。

`CMakeLists.txt`写法如下：

```cmake
cmake_minimum_required(VERSION 2.8.3)
project(ros_academy_for_beginners)
find_package(catkin REQUIRED)
catkin_metapackage()   #声明本软件包是一个metapacakge
```

`pacakge.xml`写法如下：

```xml
<package>
    <name>ros_academy_for_beginners</name>
    <version>17.12.4</version>
    <description>
        --------------------------------------------------------------------------
        A ROS tutorial for beginner level learners. This metapacakge includes some
        demos of topic, service, parameter server, tf, urdf, navigation, SLAM...
        It tries to explain the basic concepts and usages of ROS.
        --------------------------------------------------------------------------
    </description>
    <maintainer email="chaichangkun@163.com">Chai Changkun</maintainer>
    <author>Chai Changkun</author>
    <license>BSD</license>  
    <url>http://http://www.droid.ac.cn</url>

    <buildtool_depend>catkin</buildtool_depend>

    <run_depend>navigation_sim_demo</run_depend>  <!--注意这里的run_depend标签，将其他软件包都设为依赖项-->
    <run_depend>param_demo</run_depend>
    <run_depend>robot_sim_demo</run_depend>
    <run_depend>service_demo</run_depend>
    <run_depend>slam_sim_demo</run_depend>
    <run_depend>tf_demo</run_depend>
    <run_depend>topic_demo</run_depend>

    <export>    <!--这里需要有export和metapacakge标签，注意这种固定写法-->
        <metapackage/>
    </export>
</package>
```

metapacakge中的以上两个文件和普通pacakge不同点是：

- `CMakeLists.txt`:加入了catkin_metapackage()宏，指定本软件包为一个metapacakge。
- `package.xml`:标签将所有软件包列为依赖项，标签中添加标签声明。

metapacakge在我们实际开发一个大工程时可能有用

## 2.7 其他常见文件类型

在ROS的pacakge中，还有其他许多常见的文件类型，这里做个总结。

### 2.7.1 launch文件

launch文件一般以.launch或.xml结尾，它对ROS需要运行程序进行了打包，通过一句命令来启动。一般launch文件中会指定要启动哪些package下的哪些可执行程序，指定以什么参数启动，以及一些管理控制的命令。 launch文件通常放在软件包的`launch/`路径中中。 launch文件的具体写法见3.2节。

### 2.7.2 msg/srv/action文件

ROS程序中有可能有一些自定义的消息/服务/动作文件，为程序的发者所设计的数据结构，这类的文件以`.msg`,`.srv`,`.action`结尾，通常放在package的`msg/`,`srv/`,`action/`路径下。

msg文件写法见3.4节，srv文件写法见3.6节。

### 2.7.3 urdf/xacro文件

urdf/xacro文件是机器人模型的描述文件，以.urdf或.xacro结尾。它定义了机器人的连杆和关节的信息，以及它们之间的位置、角度等信息，通过urdf文件可以将机器人的物理连接信息表示出来。并在可视化调试和仿真中显示。

urdf文件的写法见第七章。

### 2.7.4 yaml文件

yaml文件一般存储了ROS需要加载的参数信息，一些属性的配置。通常在launch文件或程序中读取.yaml文件，把参数加载到参数服务器上。通常我们会把yaml文件存放在`param/`路径下

### 2.7.5 dae/stl文件

dae或stl文件是3D模型文件，机器人的urdf或仿真环境通常会引用这类文件，它们描述了机器人的三维模型。相比urdf文件简单定义的性状，dae/stl文件可以定义复杂的模型，可以直接从solidworks或其他建模软件导出机器人装配模型，从而显示出更加精确的外形。

### 2.7.6 rviz文件

rviz文件本质上是固定格式的文本文件，其中存储了RViz窗口的配置（显示哪些控件、视角、参数）。通常rviz文件不需要我们去手动修改，而是直接在RViz工具里保存，下次运行时直接读取。