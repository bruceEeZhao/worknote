[TOC]

# 1. 概述

## 1.1 框架图

![img](https://pic3.zhimg.com/80/v2-76e0c0dd2e7ff77d5cd8157ee0096a02_720w.jpg)

上图给出了ros1与ros2的架构，图左侧是ROS1的架构图。

ROS和ROS2中间件不同之处在于，ROS2不要了master节点。去中心化后，各个节点直接可以通过DDS的进行节点之间的相互发现，各个节点都是平等的，且可以1对1、1对n、n对n进行互相通信。

## 1.2 通信方式

ROS的通信方式有以下四种：

- Topic 主题（PUB/SUB）
- Service 服务
- Parameter Service 参数服务器
- Actionlib 动作库

# 2. Node&Master

## 2.1 Node

最小的进程单元就是节点（node）。一个软件包里可以有多个可执行文件，可执行文件在运行之后就成了一个进程(process)，这个进程在ROS中就叫做**节点**。 从程序角度来说，node就是一个可执行文件（通常为C++编译生成的可执行文件、Python脚本）被执行，加载到了内存之中；从功能角度来说，通常一个node负责者机器人的某一个单独的功能。



通常一个模块只负责一个功能，模块间通过通信机制进行通信。

## 2.2 Master

  master在整个网络通信架构里相当于管理中心，管理着各个node。node首先在master处进行注册，之后master会将该node纳入整个ROS程序中。node之间的通信也是先由master进行“牵线”，才能两两的进行点对点通信。当ROS程序启动时，第一步首先启动master，由节点管理器处理依次启动node。

## 2.3 启动master和node

当我们要启动ROS时，首先输入命令:

```
$ roscore
```

此时ROS master启动，同时启动的还有`rosout`和`parameter server`,其中`rosout`是负责日志输出的一个节点，其作用是告知用户当前系统的状态，包括输出系统的error、warning等等，并且将log记录于日志文件中，`parameter server`即是参数服务器，它并不是一个node，而是存储参数配置的一个服务器，后文我们会单独介绍。每一次我们运行ROS的节点前，都需要把master启动起来，这样才能够让节点启动和注册。

master之后，节点管理器就开始按照系统的安排协调进行启动具体的节点。节点就是一个进程，只不过在ROS中它被赋予了专用的名字里——node。在第二章我们介绍了ROS的文件系统，我们知道一个package中存放着可执行文件，可执行文件是静态的，当系统执行这些可执行文件，将这些文件加载到内存中，它就成为了动态的node。具体启动node的语句是：

```
$ rosrun pkg_name node_name
```

通常我们运行ROS，就是按照这样的顺序启动，有时候节点太多，我们会选择用launch文件来启动，下一小节会有介绍。 Master、Node之间以及Node之间的关系如下图所示：

![img](https://sychaichangkun.gitbooks.io/ros-tutorial-icourse163/content/pics/masterandnode.png)

## 2.4 rosrun和rosnode命令

**rosrun命令的详细用法如下**：

```
$ rosrun [--prefix cmd] [--debug] pkg_name node_name [ARGS]
```

rosrun将会寻找PACKAGE下的名为EXECUTABLE的可执行程序，将可选参数ARGS传入。 例如在GDB下运行ros程序：

```
$ rosrun --prefix 'gdb -ex run --args' pkg_name node_name
```

**rosnode命令的详细作用列表如下**：

|       rosnode命令        |                 作用                 |
| :----------------------: | :----------------------------------: |
|      `rosnode list`      |        列出当前运行的node信息        |
| `rosnode info node_name` |         显示出node的详细信息         |
| `rosnode kill node_name` |             结束某个node             |
|      `rosnode ping`      |             测试连接节点             |
|    `rosnode machine`     | 列出在特定机器或列表机器上运行的节点 |
|    `rosnode cleanup`     |      清除不可到达节点的注册信息      |

以上命令中常用的为前三个，在开发调试时经常会需要查看当前node以及node信息，所以请记住这些常用命令。如果你想不起来，也可以通过`rosnode help`来查看`rosnode`命令的用法。



# 3. 通信方式

## 3.1 Topic(PUB/SUB)

topic的通信方式是ROS中比较常见的==单向异步==通信方式，它在很多时候的通信是比较易用且高效的。但是有些需要交互的通信时该方式就显露出自己的不足之处了，后续我们会介绍==双向同步==的通信方式service。

### 3.1.1 通信流程

topic是一种点对点的单向通信方式，这里的“点”指的是node，也就是说node之间可以通过topic方式来传递信息。topic要经历下面几步的初始化过程：首先，publisher节点和subscriber节点都要到节点管理器进行注册，然后publisher会发布topic，subscriber在master的指挥下会订阅该topic，从而建立起sub-pub之间的通信。注意整个过程是单向的。其结构示意图如下：

<img src="https://sychaichangkun.gitbooks.io/ros-tutorial-icourse163/content/pics/topic-stru.jpg" alt="img" style="zoom:67%;" />

Subscriber接收消息会进行处理，一般这个过程叫做**回调(Callback)**。所谓回调就是提前定义好了一个处理函数（写在代码中），当有消息来就会触发这个处理函数，函数会对消息进行处理。

上图就是ROS的topic通信方式的流程示意图。topic通信属于一种异步的通信方式。下面我们通过一个示例来了解下如何使用topic通信。

#### 3.1.1.1 总结

1. topic通信方式是异步的，发送时调用publish()方法，发送完成立即返回，不用等待反馈。
2. subscriber通过回调函数的方式来处理消息。
3. topic可以同时有多个subscribers，也可以同时有多个publishers。ROS中这样的例子有：/rosout、/tf等等。

### 3.1.2 操作命令

在实际应用中，我们应该熟悉topic的几种使用命令，下表详细的列出了各自的命令及其作用。

|             命令              |           作用           |
| :---------------------------: | :----------------------: |
|        `rostopic list`        |   列出当前所有的topic    |
|  `rostopic info topic_name`   | 显示某个topic的属性信息  |
|  `rostopic echo topic_name`   |   显示某个topic的内容    |
| `rostopic pub topic_name ...` |   向某个topic发布内容    |
|   `rostopic bw topic_name`    |   查看某个topic的带宽    |
|   `rostopic hz topic_name`    |   查看某个topic的频率    |
|  `rostopic find topic_type`   |   查找某个类型的topic    |
|  `rostopic type topic_name`   | 查看某个topic的类型(msg) |

如果你一时忘记了命令的写法，可以通过`rostopic help`或`rostopic command -h`查看具体用法。

## 3.2 Service

### 3.2.1 通信流程

Service通信是双向的，它不仅可以发送消息，同时还会有反馈。所以service包括两部分，一部分是请求方（Clinet），另一部分是应答方/服务提供方（Server）。这时请求方（Client）就会发送一个request，要等待server处理，反馈回一个reply，这样通过类似“请求-应答”的机制完成整个服务通信。

这种通信方式的示意图如下：
Node B是server（应答方），提供了一个服务的接口，叫做`/Service`，我们一般都会用string类型来指定service的名称，类似于topic。Node A向Node B发起了请求，经过处理后得到了反馈。
<img src="https://sychaichangkun.gitbooks.io/ros-tutorial-icourse163/content/pics/service_structure.png" alt="img" style="zoom:80%;" />

Service是同步通信方式，所谓同步就是说，此时Node A发布请求后会在原地等待reply，直到Node  B处理完了请求并且完成了reply，Node A才会继续执行。Node  A等待过程中，是处于阻塞状态的成通信。这样的通信模型没有频繁的消息传递，没有冲突与高系统资源的占用，只有接受请求才执行服务，简单而且高效。

### 3.2.2 操作命令

在实际应用中，service通信方式的命令时`rosservice`，具体的命令参数如下表：

|  rosservice 命令  |           作用           |
| :---------------: | :----------------------: |
| `rosservice list` |       显示服务列表       |
| `rosservice info` |       打印服务信息       |
| `rosservice type` |       打印服务类型       |
| `rosservice uri`  |    打印服务ROSRPC uri    |
| `rosservice find` |    按服务类型查找服务    |
| `rosservice call` | 使用所提供的args调用服务 |
| `rosservice args` |       打印服务参数       |

## 3.3 Parameter Service 

数服务器是节点存储参数的地方、用于配置参数，全局共享参数。参数服务器使用互联网传输，在节点管理器中运行，实现整个通信过程。

参数服务器，作为ROS中另外一种数据传输方式，有别于topic和service，它更加的静态。参数服务器维护着一个数据字典，字典里存储着各种参数和配置。

### 3.3.1 维护方式

参数服务器的维护方式非常的简单灵活，总的来讲有三种方式：

- 命令行维护
- launch文件内读写
- node源码

#### 3.3.1.1 命令行维护

使用命令行来维护参数服务器，主要使用`rosparam`语句来进行操作的各种命令，如下表：

|            rosparam 命令             |      作用      |
| :----------------------------------: | :------------: |
| `rosparam set param_key param_value` |    设置参数    |
|       `rosparam get param_key`       |    显示参数    |
|      `rosparam load file_name`       | 从文件加载参数 |
|      `rosparam dump file_name`       | 保存参数到文件 |
|          `rosparam delete`           |    删除参数    |
|           `rosparam list`            |  列出参数名称  |

**load&&dump文件**

load和dump文件需要遵守YAML格式，YAML格式具体示例如下：

```
name:'Zhangsan'
age:20
gender:'M'
score{Chinese:80,Math:90}
score_history:[85,82,88,90]
```

简明解释。就是“名称+：+值”这样一种常用的解释方式。一般格式如下：

```
key : value
```

遵循格式进行定义参数。其实就可以把YAML文件的内容理解为字典，因为它也是键值对的形式。

#### 3.3.1.2 launch文件内读写

launch文件中有很多标签，而与参数服务器相关的标签只有两个，一个是`<param>`，另一个是`<rosparam>`。这两个标签功能比较相近，但`<param>`一般只设置一个参数，请看下例：

​         （1）                 （2）                                （3）          

观察上例比如序号3的param就定义了一个key和一个value，交给了参数服务器维护。而序号1的param只给出了key，没有直接给出value，这里的value是由后没的脚本运行结果作为value进行定义的。序号(2)就是rosparam的典型用法，先指定一个YAML文件，然后施加command,其效果等于`rosparam load file_name` 。

#### 3.3.1.3 node源码

除了上述最常用的两种读写参数服务器的方法，还有一种就是修改ROS的源码，也就是利用API来对参数服务器进行操作。具体内容我们学习完后面章节再进行介绍。



## 3.4 Actionlib

Actionlib是ROS中一个很重要的库，类似service通信机制，actionlib也是一种请求响应机制的通信方式，actionlib主要弥补了service通信的一个不足，就是当机器人执行一个长时间的任务时，假如利用service通信方式，那么publisher会很长时间接受不到反馈的reply，致使通信受阻。当service通信不能很好的完成任务时候，actionlib则可以比较适合实现长时间的通信过程，actionlib通信过程可以随时被查看过程进度，也可以终止请求，这样的一个特性，使得它在一些特别的机制中拥有很高的效率。

### 3.4.1 通信流程

> Action的工作原理是client-server模式，也是一个双向的通信模式。通信双方在ROS Action  Protocol下通过消息进行数据的交流通信。client和server为用户提供一个简单的API来请求目标（在客户端）或通过函数调用和回调来执行目标（在服务器端）。 

工作模式的结构示意图如下：

<img src="https://sychaichangkun.gitbooks.io/ros-tutorial-icourse163/content/pics/actionlib.png" alt="img" style="zoom:80%;" />

通信双方在ROS Action Protocal下进行交流通信是通过接口来实现,如下图:

![img](https://sychaichangkun.gitbooks.io/ros-tutorial-icourse163/content/pics/action_interface.png)

我们可以看到,客户端会向服务器发送目标指令和取消动作指令,而服务器则可以给客户端发送实时的状态信息,结果信息,反馈信息等等,从而完成了service没法做到的部分.

### 3.4.2 Action规范

利用动作库进行请求响应，动作的内容格式应包含三个部分，目标、反馈、结果。

- 目标

机器人执行一个动作，应该有明确的移动目标信息，包括一些参数的设定，方向、角度、速度等等。从而使机器人完成动作任务。

- 反馈

在动作进行的过程中，应该有实时的状态信息反馈给服务器的实施者，告诉实施者动作完成的状态，可以使实施者作出准确的判断去修正命令。

- 结果

当运动完成时，动作服务器把本次运动的结果数据发送给客户端，使客户端得到本次动作的全部信息，例如可能包含机器人的运动时长，最终姿势等等。

### 3.4.3 Action规范文件格式

Action规范文件的后缀名是.action，它的内容格式如下：

```
# Define the goal
uint32 dishwasher_id  # Specify which dishwasher we want to use
---
# Define the result
uint32 total_dishes_cleaned
---
# Define a feedback message
float32 percent_complete
```



## 3.5 对比

### 3.5.1 Topic V.S. service

|   名称   |              Topic               |              Service              |
| :------: | :------------------------------: | :-------------------------------: |
| 通信方式 |             异步通信             |             同步通信              |
| 实现原理 |              TCP/IP              |              TCP/IP               |
| 通信模型 |        Publish-Subscribe         |           Request-Reply           |
| 映射关系 |    Publish-Subscribe(多对多)     |      Request-Reply（多对一）      |
|   特点   | 接受者收到数据会回调（Callback） | 远程过程调用（RPC）服务器端的服务 |
| 应用场景 |       连续、高频的数据发布       |     偶尔使用的功能/具体的任务     |
|   举例   |     激光雷达、里程计发布数据     |    开关传感器、拍照、逆解计算     |

**注意：**远程过程调用(Remote Procedure Call，RPC),可以简单通俗的理解为在一个进程里调用另一个进程的函数。

