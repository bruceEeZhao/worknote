[TOC]

## linux内核网站

https://www.kernel.org/



## 下载linux内核

```bash
wget https://cdn.kernel.org/pub/linux/kernel/v5.x/linux-5.15.64.tar.xz
```

ubuntu 20.04 和22.04均使用5.15的内核因此为了尽量减小内核的差异，本次编译也使用5.15的内核，此版本也是长期支持的版本。本机的系统为ubuntu 20.04 ，内核版本为：

```bash
$ uname -a
Linux zcl-VirtualBox 5.15.0-46-generic #49~20.04.1-Ubuntu SMP Thu Aug 4 19:15:44 UTC 2022 x86_64 x86_64 x86_64 GNU/Linux
```

## 安装依赖

```bash
sudo apt-get install git fakeroot build-essential ncurses-dev xz-utils libssl-dev bc flex libelf-dev bison
```

## 配置内核

内核下载完之后，需要解压压缩包，并进入该目录。

```bash
tar -xvf linux-5.15.64.tar.xz
cd linux-5.15.64
```



在正式编译内核之前，必须首先配置内核中需要包含哪些模块。linux中提供了一个极其简单的方式来实现上述的需求，仅需一个命令，可以拷贝当前使用的内核配置文件，之后可以使用`menuconfig`命令对配置文件做出可能的修改。使用如下命令：

```bash
cp /boot/config-$(uname -r) .config
```

现在我们拥有了内核配置文件，之后可以使用`make menuconfig`命令，这个命令会打开一个配置工具（如下图），该工具允许你查看每一个模块，并可以打开或关闭那些你需要或不需要的模块。

```bash
make menuconfig
```

![img](https://lcom.static.linuxfound.org/sites/lcom/files/kernel_compile_1.jpg)

很可能你需要禁用一个模块，因此请仔细的查看每一个模块选项。如果对一个选项不确定，那就保持原样。或者最好对刚拷贝的内核配置文件不做更改，因为它是可以正常工作的。一旦你过完了配置文件中的每一个配置（它相当长），你就可以进行下一步的编译了。

## 编译和安装

编译的第一步是使用`make`命令。之后回答如下图所示的一些必要的问题。问题将取决于从哪个内核升级以及要升级到哪个内核。这一步可能会有大量的问题，将会耗费大量的时间。

```bash
make -j4
```

![img](https://lcom.static.linuxfound.org/sites/lcom/files/kernel_compile_2.jpg)

**由于我使用本机的内核配置文件，因此在这一步并没有遇到很多问题，遇到的问题与android相关，全部选择了N**

编译成功会出现如下的提示:

```bash
Kernel: arch/x86/boot/bzImage is ready  (#2)
```

编译完成后，你可以使用下面的命令安装你已开启的模块：

```bash
sudo make modules_install
```

之后使用下面的命令安装内核：

```bash
sudo make install
```

这一步的所用命令都是相当耗时的。

## 启用内核作为引导

`make install`命令执行完毕后，就需要启用该内核作为引导，使用下面的命令:

```bash
sudo update-initramfs -c -k 4.17-rc2
```

需要注意的是，最后的内核版本号需要根据实际的内核版本书写。当上面的命令执行完毕后，使用下面的命令更新grub

```bash
sudo update-grub
```

现在你可以重启系统并且选择新安装的内核了。



## 遇到的问题

1. 编译阶段的pem错误

```bash
make[1]: *** No rule to make target 'debian/canonical-certs.pem', needed by 'certs/x509_certificate_list'.  Stop.
```

修改.config，可能这个改完还会遇到类似的问题，需要把涉及的pem字段改为空。之后重新编译即可。

```bash
# 修改前 
CONFIG_SYSTEM_TRUSTED_KEYS="debian/canonical-certs.pem"
# 修改后
CONFIG_SYSTEM_TRUSTED_KEYS=""
```

2. pahole (pahole) is not available

```bash
BTF: .tmp_vmlinux.btf: pahole (pahole) is not available
Failed to generate BTF for vmlinux
Try to disable CONFIG_DEBUG_INFO_BTF
make: *** [Makefile:1211: vmlinux] Error 1
```

解决方法：

```bash
sudo apt-get install dwarves
```

3. zstd not found

```bash
/bin/sh: 1: zstd: not found
make[2]: *** [arch/x86/boot/compressed/Makefile:143: arch/x86/boot/compressed/vmlinux.bin.zst] Error 127
```

解决方式

```bash
sudo apt-get install zstd
```



