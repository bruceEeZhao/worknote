[TOC]

# 1 topic in roscpp

## 1.1 Topic通信

Topic是ROS里一种异步通信的模型，一般是节点间分工明确，有的只负责发送，有的只负责接收处理。对于绝大多数的机器人应用场景，比如传感器数据收发，速度控制指令的收发，Topic模型是最适合的通信方式。

为了讲明白topic通信的编程思路，我们首先来看`topic_demo`中的代码,这个程序是一个消息收发的例子：**自定义一个类型为gps的消息（包括位置x，y和工作状态state信息），一个node以一定频率发布模拟的gps消息，另一个node接收并处理，算出到原点的距离。** 源代码见`ROS-Academy-for-Beginners/topic_demo`

## 1.2 创建gps消息

在代码中，我们会用到自定义类型的gps消息，因此就需要来自定义gps消息，在msg路径下创建`gps.msg`： 见`topic_demo/msg/gps.msg`

```
string state   #工作状态
float32 x      #x坐标
float32 y      #y坐标
```

以上就定义了一个gps类型的消息，你可以把它理解成一个C语言中的结构体，类似于

```cpp
struct gps
{
    string state;
    float32 x;
    float32 y;
}
```

在程序中对一个gps消息进行创建修改的方法和对结构体的操作一样。

当你创建完了msg文件，记得修改`CMakeLists.txt`和`package.xml`，从而让系统能够编译自定义消息。 在`CMakeLists.txt`中需要改动

```cmake
find_package(catkin REQUIRED COMPONENTS
roscpp
std_msgs
message_generation   #需要添加的地方
)

add_message_files(FILES gps.msg)  
#catkin在cmake之上新增的命令，指定从哪个消息文件生成

generate_messages(DEPENDENCIES std_msgs) 
#catkin新增的命令，用于生成消息
#DEPENDENCIES后面指定生成msg需要依赖其他什么消息，由于gps.msg用到了flaot32这种ROS标准消息，因此需要再把std_msgs作为依赖
```

`package.xml`中需要的改动

```xml
<build_depend>message_generation</build_depend>
<run_depend>message_runtime</run_depend>
```

当你完成了以上所有工作，就可以回到工作空间，然后编译了。编译完成之后会在`devel`路径下生成`gps.msg`对应的头文件，头文件按照C++的语法规则定义了`topic_demo::gps`类型的数据。

要在代码中使用自定义消息类型，只要`#include <topic_demo/gps.h>`，然后声明，按照对结构体操作的方式修改内容即可。

```cpp
topic_demo::gps mygpsmsg;
mygpsmsg.x = 1.6;
mygpsmsg.y = 5.5;
mygpsmsg.state = "working";
```

## 1.3 消息发布节点

定义完了消息，就可以开始写ROS代码了。通常我们会把消息收发的两端分成两个节点来写，一个节点就是一个完整的C++程序。

见`topic_demo/src/talker.cpp`

```cpp
#include <ros/ros.h>   
#include <topic_demo/gps.h>  //自定义msg产生的头文件

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");  //用于解析ROS参数，第三个参数为本节点名
  ros::NodeHandle nh;    //实例化句柄，初始化node

  topic_demo::gps msg;  //自定义gps消息并初始化 
   ...

  ros::Publisher pub = nh.advertise<topic_demo::gps>("gps_info", 1); //创建publisher，往"gps_info"话题上发布消息
  ros::Rate loop_rate(1.0);   //定义发布的频率，1HZ 
  while (ros::ok())   //循环发布msg
  {
    ...   //处理msg
    pub.publish(msg);//以1Hz的频率发布msg
    loop_rate.sleep();//根据前面的定义的loop_rate,设置1s的暂停
  }
  return 0;
}
```

机器人上几乎所有的传感器，几乎都是按照固定频率发布消息这种通信方式来传输数据，只是发布频率和数据类型的区别。

## 1.4 消息接收节点

见`topic_demo/src/listener.cpp`

```cpp
#include <ros/ros.h>
#include <topic_demo/gps.h>
#include <std_msgs/Float32.h>

void gpsCallback(const topic_demo::gps::ConstPtr &msg)
{  
    std_msgs::Float32 distance;  //计算离原点(0,0)的距离
    distance.data = sqrt(pow(msg->x,2)+pow(msg->y,2));
    ROS_INFO("Listener: Distance to origin = %f, state: %s",distance.data,msg->state.c_str()); //输出
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("gps_info", 1, gpsCallback);  //设置回调函数gpsCallback
  ros::spin(); //ros::spin()用于调用所有可触发的回调函数，将进入循环，不会返回，类似于在循环里反复调用spinOnce() 
  //而ros::spinOnce()只会去触发一次
  return 0;
}
```

在topic接收方，有一个比较重要的概念，就是**回调(CallBack)**，在本例中，回调就是预先给`gps_info`话题传来的消息准备一个回调函数，你事先定义好回调函数的操作，本例中是计算到原点的距离。只有当有消息来时，回调函数才会被触发执行。具体去触发的命令就是`ros::spin()`，它会反复的查看有没有消息来，如果有就会让回调函数去处理。

因此千万不要认为，只要指定了回调函数，系统就回去自动触发，你必须`ros::spin()`或者`ros::spinOnce()`才能真正使回调函数生效。

## 1.5 CMakeLists.txt文件修改

在`CMakeLists.txt`添加以下内容，生成可执行文件

```cmake
add_executable(talker src/talker.cpp) #生成可执行文件talker
add_dependencies(talker topic_demo_generate_messages_cpp)
#表明在编译talker前，必须先生编译完成自定义消息
#必须添加add_dependencies，否则找不到自定义的msg产生的头文件
#表明在编译talker前，必须先生编译完成自定义消息
target_link_libraries(talker ${catkin_LIBRARIES}) #链接

add_executable(listener src/listener.cpp ) #声称可执行文件listener
add_dependencies(listener topic_demo_generate_messages_cpp)
target_link_libraries(listener ${catkin_LIBRARIES})#链接
```

以上cmake语句告诉catkin编译系统如何去编译生成我们的程序。这些命令都是标准的cmake命令，如果不理解，请查阅cmake教程。

之后经过`catkin_make`，一个自定义消息+发布接收的基本模型就完成了。

## 扩展：回调函数与spin()方法

回调函数在编程中是一种重要的方法，在维基百科上的解释是：

```
In computer programming, a callback is any executable code that is passed as an argument to other code, which is expected to call back (execute) the argument at a given time.
```

回调函数作为参数被传入到了另一个函数中（在本例中传递的是函数指针），在未来某个时刻（当有新的message到达），就会立即执行。Subscriber接收到消息，实际上是先把消息放到一个**队列**中去，如图所示。队列的长度在Subscriber构建的时候设置好了。当有spin函数执行，就会去处理消息队列中队首的消息。

<img src="https://sychaichangkun.gitbooks.io/ros-tutorial-icourse163/content/pics/cb_queue.png" alt="img" style="zoom:67%;" />

spin具体处理的方法又可分为阻塞/非阻塞,单线程/多线程，在ROS函数接口层面我们有4种spin的方式：

|            spin方法             |  阻塞  |  线程  |
| :-----------------------------: | :----: | :----: |
|          `ros::spin()`          |  阻塞  | 单线程 |
|        `ros::spinOnce()`        | 非阻塞 | 单线程 |
|   `ros::MultiThreadedSpin()`    |  阻塞  | 多线程 |
| `ros::AsyncMultiThreadedSpin()` | 非阻塞 | 多线程 |

阻塞与非阻塞的区别我们已经讲了，下面来看看单线程与多线程的区别：

<img src="https://sychaichangkun.gitbooks.io/ros-tutorial-icourse163/content/pics/single-multi-spin.png" alt="img" style="zoom:67%;" />



我们常用的`spin()`、`spinOnce()`是单个线程逐个处理回调队列里的数据。有些场合需要用到多线程分别处理，则可以用到`MultiThreadedSpin()`、`AsyncMultiThreadedSpin()`。



# 2. source code

## 2.1 talker.cpp

```bash
# cat src/test_pkg/src/talker.cpp 
```



```cpp
#include <ros/ros.h>   
#include <test_pkg/gps.h>  //自定义msg产生的头文件

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");  //用于解析ROS参数，第三个参数为本节点名
  ros::NodeHandle nh;    //实例化句柄，初始化node

  test_pkg::gps msg;  //自定义gps消息并初始化 
   //...

  ros::Publisher pub = nh.advertise<test_pkg::gps>("gps_info", 1); //创建publisher，往"gps_info"话题上发布消息
  ros::Rate loop_rate(1.0);   //定义发布的频率，1HZ 
  while (ros::ok())   //循环发布msg
  {
   // ...   //处理msg
    msg.state = "hhhh";
    msg.x = 1;
    msg.y = 2;
    pub.publish(msg);//以1Hz的频率发布msg
    loop_rate.sleep();//根据前面的定义的loop_rate,设置1s的暂停
  }
  return 0;
}

```

## 2.2 listener.cpp

```bash
# cat src/test_pkg/src/listener.cpp 
```



```cpp
#include <ros/ros.h>
#include <test_pkg/gps.h>
#include <std_msgs/Float32.h>

void gpsCallback(const test_pkg::gps::ConstPtr &msg)
{  
    std_msgs::Float32 distance;  //计算离原点(0,0)的距离
    distance.data = sqrt(pow(msg->x,2)+pow(msg->y,2));
    ROS_INFO("Listener: Distance to origin = %f, state: %s",distance.data,msg->state.c_str()); //输出
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("gps_info", 1, gpsCallback);  //设置回调函数gpsCallback
  ros::spin(); //ros::spin()用于调用所有可触发的回调函数，将进入循环，不会返回，类似于在循环里反复调用spinOnce() 
  //而ros::spinOnce()只会去触发一次
  return 0;
}

```

## 2.3 执行

需要运行3个终端

### 2.3.1 master终端

```bash
# roscore 
```



### 2.3.2 talker终端

```bash
# source /opt/ros/lunar/setup.bash 
# source /home/test/devel/setup.bash 
# rosrun test_pkg talker
```



### 2.3.3 listener终端

```bash
# source /opt/ros/lunar/setup.bash 
# source /home/test/devel/setup.bash 
# rosrun test_pkg listener 
```



### 2.3.4 查看节点情况

```bash
# rosnode list
/listener
/rosout
/talker
```

