# 第1讲 预备知识

（1）SLAM定义

​		**SLAM**，即**S**imultaneous **L**ocation **a**nd **M**apping，同步定位与地图构建[1]，指搭载特定传感器的主体，在没有环境先验信息的情况下，于运动过程中建立环境的模型，同时估计自己的运动。

​		（个人观点）若**偏向定位**，则向轻量化的、快速跟踪自身位置的定位系统发展；若**偏向建图**，则向三维重建等注重地图、模型构建的方向发展。

（2）**SLAM相关书籍**

1. 《概率机器人》（*Probabilistic Robotics*）
2. 《计算机视觉中的多视图几何》（*Multiple View Geometry in Computer Vision*）
3. 《机器人学中的状态估计》（*State Estimation for Robotics: A Matrix-Lie-Group Approach*）
4. 《机器人感知-因子图在SLAM中的应用》（*Factor Graphs for Robot Perception*）

（3）SLAM**模块**划分

- 视觉里程计、后端优化、建图、后端优化、回环检测

（4）使用到的**库**

- Eigen, Sophus, OpenCV, PCL, g2o, Ceres, GTSAM

（5）**本书划分**

- 第一部分：**数学基础**篇

  - 第1讲：预备知识，基本介绍，自测习题

  - 第2讲：系统概述，组成模块，编程环境搭建过程

  - 第3讲：三维空间刚体运动，旋转矩阵、欧拉角、四元数，Eigen库

  - 第4讲：李群和李代数，定义及使用方法，Sophus库

  - 第5讲：针孔相机模型，计算机图像表示，OpenCV库

  - 第6讲：非线性优化，包括状态估计理论基础、最小二乘、梯度下降，Ceres和g2o库

- 第二部分：**实践应用**篇

  - 第7讲：特征点法的视觉里程计，包括特征点提取、对极几何约束计算、PnP和ICP等。
    - 实践：用这些方法估计两个图像之间的运动。
  - 第8讲：直接法的视觉里程计，包括光流和直接法原理。
    - 实践：一个简单的直接法运动估计。
  - 第9讲：后端优化，深入讨论Bundle Adjustment（BA）及利用稀疏性加速求解。
    - 实践：用Ceres和g2o分别书写一个BA程序。
  - 第10讲：后端优化中的位姿图，表达关键帧之间约束的一种更紧凑的形式。
    - 实践：SE(3)和Sim(3)的位姿图，以及使用g2o对一个位姿球进行优化。
  - 第11讲：回环检测，词袋方法。
    - 实践：使用DBoW3书写字典训练和回环检测程序。
  - 第12讲：地图构建，单目稠密深度图估计、RGB-D稠密地图构建。
    - 实践：书写极线搜索和块匹配程序，RGB-D构建点云地图和八叉树地图。
  - 第13讲：工程实践，搭建一个双目视觉里程计框架，会包含优化、关键帧选择等问题。
    - 实践：在Kitti数据集上测试性能，讨论一些改进手段。
  - 第14讲：介绍开源SLAM方案、未来发展方向等。

## （6）习题

1. **有线性方程 $Ax=b$ ，若已知 $A$，$b$，需要求解 $x$，该如何求解？这对 $A$ 和 $b$ 有哪些要求？**

   TODO。

2. **高斯分布是什么？它的一维形式是什么样子？它的高维形式是什么样子？**

   TODO。

3. **你知道C++中的类吗？你知道STL吗？你使用过它们吗？**

   ① 类是用于指定对象的形式，它包含了**数据表示法**和**用于处理数据的方法**；

   ② STL（Standard Templete Library），标准模板库，是一套功能强大的C++模板类，其核心包括容器、算法、迭代器三个组件，带有丰富的预定义函数，帮助我们通过简单的方式处理复杂的任务。

   ③ 使用过。

4. **你以前是怎样书写C++程序的？**

   在Visual Studio中直接建立工程书写，或在linux环境的clion/vscode中书写并用CMake编译运行。

5. **你知道C++11标准么？听说或使用过其中哪些新特性？有没有其他的标准？**

   ① 知道，是C++98发布13年之后的第一次重大修正。

6. **你知道Linux么？有没有至少使用过一种（不算安卓）操作系统？**

   知道，使用过Ubuntu。

7. **Linux的目录结构是什么样的？你知道哪些基本命令，如ls、cat等？**

   Linux的目录结构是：；我知道的基本命令有rm、cp、dpkg、mkdir等等。

8. **如何在Ubuntu系统中安装软件（不打开软件中心）？这些软件会被安装在什么地方？如果只知道模糊的软件名（如Eigen），应该如何安装？**

   ① 直接命令行安装/在网络上下载安装包后根据不同的安装包格式进行命令行安装；

   ② 会直接被安装在根目录中的默认位置；

   ③ 可以通过TAB补全的方式找到软件名称，一般依赖库多为`lib\<name>-dev`的名称形式。

9. **Vim学习。**

   TODO。



# 第2讲 初始SLAM

​		**定位**和**建图**是一个自主移动机器人感知的内外两面：一方面要明白自身的状态（位置），另一方面要明白外在的环境（地图），固定到环境中的传感器简单、直接地测量机器人位置信息但是束缚了外部环境，无法提供普遍的、通用的解决方案，而基于机载传感器间接推算自身位置，对环境不提出任何要求，适用于位置环境。

​		激光SLAM相对成熟，《概率机器人》中介绍了许多相关知识。

​		本书主题为视觉SLAM，包括单目相机、双目相机、RGB-D相机、全景相机、事件相机等等。

## 2.1 相机简介

### （1）单目相机

​		由于单目相机拍摄的图像只是三维空间的二维投影，如果真想恢复三维结构，必须改变相机视角，也即相机必须**运动**（motion），才能估计场景中物体的远近和大小（称之为**结构**，Structure）。相机通过运动中形成的图像间视差定量判断物体远近。

​		单目SLAM估计的轨迹和地图将与实际轨迹和地图相差一个因子——尺度。

### （2）双目相机和深度相机

​		通过某种手段测量物体与相机距离，场景三维结构可以在单个图片恢复，同时克服尺度不确定性。

​		双目相机两相机之间的距离称为基线（Baseline）。基线越大，可测距离越远，配置与标定较为复杂，深度量程和精度受到双目基线和分辨率所限，且视差计算非常消耗计算资源（主要问题之一）。

​		深度相机，通过红外结构光或Time-of-Flight（ToF）原理，直接测出物体与相机距离，节省大量计算资源，缺点是测量范围窄、噪声大、视野小、易受日光干扰、无法测量投射材质等，主要用于室内，室外较难应用。

## 2.2 经典视觉SLAM框架

​		SLAM不是某种简单的、只要输入数据就可以不断输出定位和地图信息的算法，它需要一个完善的代码框架。

​	以下是经典的视觉SLAM框架：

<img src="slambook_note.assets/飞书20220414-164142.png" alt="飞书20220414-164142" style="zoom: 80%;" />

**SLAM流程步骤**：

1. **传感器信息读取**：传感器信息的读取和预处理。
2. **前端视觉里程计**（Front End）：估计相邻图像间相机的运动及局部地图。
   - 与【计算机视觉】更相关，如图像的特征点提取与匹配
   - 会出现累计漂移，需要通过后端优化和回环检测进行校正。
3. **后端（非线性）优化**（Back End）：接收不同时刻里程计的相机位姿和回环检测信息，对其进行优化并获得全局一致的轨迹和地图。
   - 【滤波与非线性优化】算法
   - 从带有噪声的数据中估计整个系统状态（轨迹+地图），以及这个状态估计的不确定性（最大后验概率估计，Maximum-a-Posteriori，MAP）。
4. **回环检测**（Loop Closure）：检测回环，并向信息提供给后端。
   - 判断图像间相似性
5. **建图**（Mapping）：根据估计轨迹和其他等信息，建立与任务要求对应的地图。
   - 不同的应用对应不同的地图需求；地图可以分为度量地图（稀疏/稠密）、拓扑地图等等

如果把环境限定在<u>静态的、刚体的、光照变化不明显的、没有人为干扰的场景</u>，那么这种场景下的SLAM技术以及相对成熟了。

## 2.3 SLAM问题的数学表述

​		把一段连续时间的运动变成了离散时刻 $t=1,..,K$ 当中发生的事情，机器人在各时刻的位置记为 $x_1,...,x_K$ ，地图由许多个路标组成，用表示 $y_1,...,y_N$ 表示它们。

- **运动**： $k+1$ 时刻到 $k$ 时刻，机器人位置 $x$ 如何变化。

  - $$
    x_k=f(x_{k-1},u_k,w_k)
    $$


  - 参数：$k-1$ 时刻位置，运动传感器读数/输入，过程噪声

- **观测**：机器人在 $k$ 时刻于 $x_k$ 处探测到某一路标 $y_j$ 。

  - $$
    z_{k,j}=h(y_j,x_k,v_{k,j})
    $$

  - 参数：某一路标点，$k$ 时刻位置，观测噪声

运动和观测方程描述了最基本的SLAM问题，状态估计问题，状态估计问题的求解，总体上经历了从滤波方法到优化方法的转变。

- **滤波**方法：EKF为主的预测+更新，后开始使用离子滤波器
- **优化**方法：以图优化为代表的，当前主流

## 2.4 编程实践

C++文件的编译方式：

1. 使用g++命令行编译：

   ​		不适用于大规模程序的类丰富、依赖关系复杂的情况。

2. 使用cmake编译：

   - 先执行 `cmake` 处理了工程文件之间的关系，再执行 `make` 实际调用了 `g++` 来编译程序，将一串g++命令变成了维护若干个比较直观的`CMakeLists.txt`文件。

   - 最简单形式：最小版本，工程名，生成可执行文件

     ```cmake
     cmake_minimum_required(VERSION 2.8)
     project(HelloSLAM)
     add_executable(helloSLAM helloSLAM.cpp)
     ```

   - 引用库：生成库，生成可执行文件，链接库

     Linux中的库文件有静态库和共享库两种，静态库每次被调用都生成一个副本，而共享库只有一个副本，更节省空间。

     ```cmake
     # 创建静态库，生成文件后缀为.a
     add_library(hello libHelloSLAM.cpp)   # 生成 libhello.a
     # 创建共享库，生成文件后缀为.so
     add_library(hello_shared SHARED libHelloSLAM.cpp)  # 生成 libhello_shared.so
     
     add_executable(useHello useHello.cpp)
     target_link_libraries(useHello hello)
     ## target_link_libraries(useHello hello_shared)
     ```
   
   - 包含子文件夹
   
     ```cmake
     add_subdirectory(useEigen)
     ```
   
   - 指定C++标准
   
     ```cmake
     set(CMAKE_CXX_FLAGS "-std=c++11")
     ```

## 习题

1. **阅读文献[1]和[14]，你能看懂其中的内容么？**

   TODO。

2. **阅读SLAM的综述文献，例如[9，15-18]等。这些文献中关于SLAM的看法与本书有何异同？**

   TODO。

3. **g++命令有哪些参数？怎么填写参数可以更改生成的程序文件名字？**

   TODO。

4. **使用build文件夹来编译你的cmake工程，然后在KDevelop中试试。**

   TODO。

5. **可以在代码中添加一些语法错误，看看编译会生成什么样的信息。你能看懂g++的错误信息么？**

   TODO。

6. **如果忘了把库链接到可执行程序上，编译会报错么？报什么样的错误？**

   TODO。

7. **阅读《cmake 实践》，了解cmake的其他语法。**

   TODO。

8. **完善Hello SLAM小程序，把它做成一个小程序库，安装到本地硬盘中。然后新建一个工程，使用find_package找到这个库调用。**

   TODO。

9. **阅读cmake教学材料，例如：https://github.com/TheErk/CMake-tutorial。**

   TODO。

10. **找到KDevelop的官方网站，看看它还有哪些特性。你都用上了么？**

    TODO。

11. **如果在第1讲学习了Vim，那么请试试KDevelop的Vim编辑功能。**

    TODO。

# 第3讲 三维空间刚体运动

- 理解三维空间的刚体运动描述方式：**旋转矩阵**、**变换矩阵**、**四元数**和**欧拉角**。
- 掌握**Eigen库**的矩阵、几何模块的使用方法。

## 3.1 旋转矩阵

两个向量 $a$ 和 $b$ 的内积和外积运算：
$$
a{\cdot}b=a^Tb=\sum^3_{i=1}a_ib_i=|a||b|cos\langle a,b\rangle
$$

$$
a\times b=\begin{vmatrix}\begin{vmatrix}e_1&e_2&e_3\\a_1&a_2&a_3\\b_1&b_2&b_3\end{vmatrix}\end{vmatrix}
=\begin{bmatrix}a_2b_3-a_3b_2\\a_3b_1-a_1b_3\\a_1b_2-a_2b_1\end{bmatrix}
=\begin{bmatrix}0&-a_3&a_2\\a_3&0&-a_1\\-a_2&a_1&0\end{bmatrix}b
=a^{\wedge}b
$$

其中**反对称矩阵（Skew-symmetrix Matrix）**的符号定义为：
$$
a^{\wedge}=\begin{bmatrix}0&-a_3&a_2\\a_3&0&-a_1\\-a_2&a_1&0\end{bmatrix}
$$
将刚体从局部坐标系转换到世界坐标系，经历了一个**欧式变换（Euclidean Transform）**，它由旋转和平移组成。

首先考虑**旋转**，**旋转矩阵（Rotation Matrix）**各分量是两个坐标系基的内积，因此也叫**方向余弦矩阵（Direction Cosine Matrix）**，定义同一向量在两个坐标系下的坐标分别为 $a$ 和 $a'$ 。
$$
\begin{bmatrix}a_1\\a_2\\a_3\end{bmatrix}
= \begin{bmatrix}e_1^Te_1'&e_1^Te_2'&e_1^Te_3'\\e_2^Te_1'&e_2^Te_2'&e_2^Te_3'\\e_3^Te_1'&e_3^Te_2'&e_3^Te_3'\end{bmatrix}\begin{bmatrix}a_1'\\a_2'\\a_3'\end{bmatrix}
\equiv Ra'
$$
旋转矩阵具有一些特别的性质，如：它是行列式为1的正交矩阵（逆为自身转置）。$n$ 维旋转矩阵的集合定义如下，为**特殊正交群（Special Orthogonal Group）**：
$$
\ce{SO}(n)=\{R\in \mathbb {R}^{n\times n}\vert RR^T =I,\ce{det}(R)=1\}
$$
定义坐标系1、坐标系2，向量 $a$ 在两坐标系下的坐标为 $a_1$，$a_2$，它们之间的关系是：
$$
a_1=R_{12}a_2+t_{12}
$$
其中，$R_{12}$ 表示`把坐标系2的向量变换到坐标系1中`（乘在向量左边，所以从右往左读），$t_{12}$ 表示`从1到2的向量` 或 `2在1中的表示`。

可以将**旋转和平移变换合并**到一个齐次变换 $T$ 中，即**变换矩阵（Transform Matrix）**，其左上角为旋转矩阵，右上角为平移向量，左下角为 $0$ 向量，右下角为1，又称为**特殊欧式群（Special Euclidean Group）**：
$$
\ce{SE}(3)=\{T=\left[\matrix{R&t\\0^T&1}\right]\in \mathbb {R}^{4\times 4}\vert R\in \ce{SO}(3),t\in \mathbb {R}^3\}
$$
与 $\ce{SO}(3)$ 一样，求解该矩阵的逆表示一个反向的变换：
$$
T^{-1}=\left[\matrix{R^T&-R^Tt\\0^T&1}\right]
$$

## 3.2 实践：Eigen

Eigen是一个纯用头文件搭建起来的库，没有类似.so或.a的二进制文件。

- 示例程序中包含了**矩阵、向量**的**创建、指定值赋值、索引访问、随机赋值、数据类型转换，矩阵的转置**、**求和、求迹、数乘、求逆、求行列式**等。
- 在进行矩阵运算时，要注意不要出现**数据类型、矩阵维数**上的错误。
- 随机创建的实矩阵与它的转置相乘，可以保证结果的半正定。
- **半正定定义**：如果对任何非零向量 $X$，都有 $X'AX≥0 $ ，半正定矩阵的行列式非负。
- Eigen矩阵不支持自动类型提升。
- 更多Eigen知识：[查看Eigen官网教程](http://eigen.tuxfamily.org/dox-devel/modules.html)

## 3.3 旋转向量与欧拉角

### 3.3.1 旋转向量

​		我们希望有一种方式能够紧凑地描述旋转和平移。实际上，任何一个旋转都可以用一个旋转轴和一个旋转角来刻画，我们使用一个向量，方向与旋转轴一致，长度等于旋转角，称之为**旋转向量**（或者轴角，Axis-Angle）。

- 旋转向量 $\Rightarrow$ 旋转矩阵：

$$
R=\ce{cos}\theta I+(1-\ce{cos}\theta)nn^T+\ce{sin}\theta n^{\wedge}
$$

- 旋转矩阵 $\Rightarrow$ 旋转向量：

$$
\begin{align}
tr(R)
=&\ \ce{cos}\theta\ \ce{tr}(I)+(1-\ce{cos}\theta)\ \ce{tr}(nn^T)+\ce{sin}\theta \ \ce{tr}(n^{\wedge})\\
=&\ 3\ce{cos}\theta+(1-\ce{cos}\theta)\\
=&\ 1+2\ce{cos}\theta
\end{align}
$$

$$
\theta = \ce{arccos}\frac{\ce{tr}(R)-1}{2}
$$

而转轴 $n$ 满足 $Rn=n$ ，它是矩阵 $R$ 特征值1对应的特征向量。

### 3.3.2 欧拉角

​		旋转矩阵和旋转向量虽然能够描述旋转，但是对人类来说并不**直观**，而**欧拉角**提供了一种非常直观的描述旋转的方式——用3个分离转角把一个旋转分解成3次绕不同轴的旋转。欧拉角根据**依次绕轴的顺序**、**转轴是否固定**分为很多种不同的类型。

​		$ZYX$ 是较为常见的一种**欧拉角**定义方式，即使用“偏航-俯仰-滚转”（yaw-pitch-roll）3个角度 $[r,p,y]^T$ 来描述一个旋转。

​		欧拉角的一个常见问题是**万向锁问题**（[Gimbal Lock](https://en.wikipedia.org/wiki/Gimbal_lock)）。旋转向量也有奇异性，发生在转角 $\theta$ 超过 $2\pi$ 而产生周期性时。

## 3.4 四元数

​		**旋转向量**用9个向量描述3自由度的旋转，具有冗余性；**欧拉角和旋转向量**紧凑，但是具有奇异性。事实上，我们找不到不带奇异性的三维向量描述方式。

​		**四元数**是Hamilton找到的一种扩展的复数，既紧凑，也没有奇异性，缺点是不够直观、运算稍复杂些。
$$
q=q_0+q_1i+q_2j+q_3k=\left[s,{\upsilon}\right]^T
$$

- 四元数的**运算**：

  定义两个四元数
  $$
  q_a=s_a+x_ai+y_aj+z_ak=[s_a,{\upsilon}_a]^T\\
  q_b=s_b+x_bi+y_bj+z_bk=[s_b,{\upsilon}_b]^T
  $$

  - 加减法：

  - 乘法：
    $$
    \begin{align}
    q_aq_b=&s_as_b-x_ax_b-y_ay_b-z_az_b\\
    &+(s_ax_b+x_as_b+y_az_b-z_ay_b)i\\
    &+(s_ay_b-x_az_b+y_as_b+z_ax_b)j\\
    &+(s_az_b+x_ay_b-y_ax_b+z_as_b)k\\
    =&\left[s_as_b-{\upsilon}_a^T{\upsilon}_b,s_a{\upsilon}_b+s_b{\upsilon}_a+{\upsilon}_a\times {\upsilon}_b\right]
    \end{align}
    $$

  - 模长：$\Vert q_a\Vert=\sqrt{s_a^2+x_a^2+y_a^2+z_a^2}\ ,\Vert q_aq_b\Vert=\Vert q_a\Vert \Vert q_b\Vert$

  - 共轭：四元数共轭即把虚部取成相反数。

  - 逆：$q^{-1}=q^*/\Vert q\Vert^2$ ，四元数与自己的逆的乘积为实四元数1。

  - 数乘：

- 四元数**表示旋转**：假设三维空间点 $p=[x,y,z]\in\mathbb {R}^3$ 以及一个表示旋转的四元数 $q$ 。
  - 把三维空间点用一个虚四元数来描述：$p=[0,x,y,z]^T$
  - 旋转后的点可以表示为：$p'=qpq^{-1}$

- **四元数到其他旋转表示**的转换

  - 为描述旋转的四元数间的运算**定义符号** $^{+}$ 和 $^{\oplus}$ ：
    $$
    q^{+}=\left[\matrix{s&-{\upsilon}^T\\{\upsilon}&sI+{\upsilon}^{\wedge}}\right],
    q^{\oplus}=\left[\matrix{s&-{\upsilon}^T\\{\upsilon}&sI-{\upsilon}^{\wedge}}\right]
    $$

  - 四元数**乘法**
    $$
    q_1^{+}q_2=\left[\matrix{s_1&-{\upsilon}_1^T\\{\upsilon}_1&s_1I+{\upsilon}_1^{\wedge}}\right]
    \left[\matrix{s_2\\ \upsilon_2 }\right]
    =\left[\matrix{-{\upsilon}_1^T\upsilon_2+s_1s_2\\s_1{\upsilon}_2+s_2{\upsilon}_1+{\upsilon}_1^{\wedge}{\upsilon}_2}\right]
    =q_1q_2
    =q_2^{\oplus}q_1
    $$

  - 考虑用四元数**对空间点进行旋转**：
    $$
    p'=qpq^{-1}=q^+p^+q^{-1}=q^+{(q^{-1})}^{\oplus}p\\
    q^+{(q^{-1})}^{\oplus}=
    \left[\matrix{s&-{\upsilon}^T\\{\upsilon}&sI+{\upsilon}^{\wedge}}\right]
    \left[\matrix{s&{\upsilon}^T\\-{\upsilon}&sI+{\upsilon}^{\wedge}}\right]
    =\left[\matrix{1&0\\0^T&{\upsilon}{\upsilon}^T+s^2I+2s{\upsilon}^{\wedge}+({\upsilon}^{\wedge})^2}\right]
    $$

  - **四元数 $\Rightarrow$ 旋转矩阵**：
    $$
    R={\upsilon}{\upsilon}^T+s^2I+2s{\upsilon}^{\wedge}+({\upsilon}^{\wedge})^2
    $$

  - **四元数 $\Rightarrow$ 轴角**：
    $$
    \begin{align}
    \ce{tr}(R)
    & =\ce{tr}({\upsilon}{\upsilon}^T+s^2I+2s{\upsilon}^{\wedge}+({\upsilon}^{\wedge})^2)\\
    & = v_1^2+v_2^2+v_3^2+3s^2-2(v_1^2+v_2^2+v_3^2)\\
    & = 4s^2-1
    \end{align}
    $$

    $$
    \theta=\ce{arccos}\frac{\ce{tr}(R-1)}{2}=\ce{arccos}(2s^2-1)\\
    \ce{cos}\theta=2s^2-1=2\ \ce{cos}^2\frac{\theta}{2}-1\\
    \theta=2\ \ce{arccos}s
    $$

    $$
    \begin{align}
    & \theta=2\ \ce{arccos}\ q_0\\
    & \left[n_x,n_y,n_z\right]^T=\left[q_1,q_2,q_3\right]^T/\ce{sin}\frac{\theta}{2}
    \end{align}
    $$

## 3.5 相似、仿射、射影变换

1. **相似变换**

   7自由度，相较欧式变换，多了一个缩放因子 $s$ ，允许物体均匀缩放，三维相似变换的集合也叫**相似变换群**，记为 $\ce{Sim}(3)$ 。
   $$
   T_s=\left[\matrix{sR&t\\0^T&1}\right]
   $$

2. **仿射变换**

   12自由度，将欧式变换左上角的正交矩阵换成任意一个可逆矩阵，变换后保持平行性。
   $$
   T_A=\left[\matrix{A&t\\0^T&1}\right]
   $$

3. **射影变换**

   15自由度，保持接触平面的相交与相切关系。

   从真实世界到相机照片的变换可以看成一个射影变换（可以想象观察一个原本方形的地板砖，观察结果是一个不规则的四边形）。
   $$
   T_A=\left[\matrix{A&t\\a^T&1}\right]
   $$

## 3.6 实践：Eigen几何模块

| 几何表示方法 | 维度 |     Eigen变量名     |
| :----------: | :--: | :-----------------: |
|   旋转矩阵   | 3×3  |   Eigen::Matrix3d   |
|   旋转向量   | 3×1  |  Eigen::AngleAxisd  |
|    欧拉角    | 3×1  |   Eigen::Vector3d   |
|    四元数    | 4×1  | Eigen::Quaterniond  |
| 欧式变换矩阵 | 4×4  |  Eigen::Isometry3d  |
|   仿射变换   | 4×4  |   Eigen::Affine3d   |
|   射影变换   | 4×4  | Eigen::Projective3d |

进一步查看Eigen集合模块，可以[查看Eigen Geometry官网教程](http://eigen.tuxfamily.org/dox/group__TutorialGeometry.html)

**可视化轨迹**

将轨迹文件存储在`trajectory.txt`中，每一行的格式为：
$$
\ce{time},t_x,t_y,t_z,q_x,q_y,q_z,q_w
$$
如果只谈论机器人的位姿，可以使用 $T_{WR}$ 或 $T_{RW}$ ，它们互为逆，可以轻松根据一个求出另一个。

严格来说，机器人（相机）坐标系的原点 $O_R$ 为世界坐标系中的坐标 $O_W$ ，它们之间满足：
$$
O_W=T_{WR}O_R=t_{WR}
$$
因此，$T_{WR}$ 能够更加直观地描述相机位置，轨迹文件中存储的正是 $T_{WR}$ 而不是 $T_{RW}$ 。

**可视化轨迹的绘制**使用了基于OpenGL的Pangolin库，它在OpenGL的绘图操作之上还提供了一些GUI功能。

## 习题

1. 验证旋转矩阵是正交矩阵。

   TODO。

2. 寻找罗德里格斯公式的推导过程并加以理解。

   TODO。

3. 验证四元数旋转某个点后结果是一个虚四元数（实部为零），所以仍然对应到一个三维空间点，见式(3.33)。

   TODO。

4. 画表总结旋转矩阵、轴角、欧拉角、四元数的转换关系。

   TODO。

5. 假设有一个大的Eigen矩阵，想把它的左上角3×3的块取出来，然后赋值 $I_{3\times 3}$ 。请编写程序实现。

   TODO。

6. 一般线性方程 $Ax=b$ 有哪几种做法？你能在Eigen中实现么？

   TODO。

# 第4讲 李群与李代数

​		三维旋转矩阵构成了**特殊正交群** $\ce{SO}(3)$ ，变换矩阵构成了**特殊正交群** $\ce{SE}(3)$。
$$
\ce{SO}(3)=\{R\in \mathbb {R}^{3\times 3}\vert RR^T =I,\ce{det}(R)=1\}\\
\ce{SE}(3)=\{T=\left[\matrix{R&t\\0^T&1}\right]\in \mathbb {R}^{4\times 4}\vert R\in \ce{SO}(3),t\in \mathbb {R}^3\}
$$
这两个集合对乘法封闭，可以称之为**群**。

**群**是**一种集合加上一种运算的代数结构**，若把集合记为 $A$ ，运算记为 $\cdot$ ，那么群可以记为 $G=(A,\cdot)$，要求满足以下几个条件（“封结幺逆”）：

1. 封闭性：$\forall a_1,a_2\in A,a_1\cdot a_2\in A$.
2. 结合律：$\forall a_1,a_2,a_3\in A,(a_1\cdot a_2)\cdot a_3=a_1\cdot(a_2\cdot a_3)$.
3. 幺元：$\exists a_0\in A,\ce{s}.\ce{t}.\forall a\in A,a_0\cdot a=a\cdot a_0=a$.
4. 逆：$\forall\in A,\exists a^{-1}\in A,\ce{s}.\ce{t}.a\cdot a^{-1}=a_0$.

**李群**则表示**具有光滑性质的群**，像整数群 $\mathbb Z$ 那样离散的群没有离散性质，故不是李群。

## 习题

1. **验证 $\ce{SO}(3)$ 、$\ce{SE}(3)$ 和 $\ce{Sim}(3)$ 关于乘法成群。**

   

2. **验证 $(\mathbb R^3,\mathbb R,\times)$ 构成李代数。**

   

3. **验证 $\mathfrak {so}(3)$ 和 $\mathfrak {se}(3)$ 满足李代数要求的性质。**

   

4. **验证性质(4.20)和(4.21)。**

   

5. **证明：**
   $$
   Rp^{\wedge}R^T=(Rp)^{\wedge}
   $$
   

6. **证明：**
   $$
   R\exp(p^{\wedge})R^T=\exp ((Rp)^{\wedge})
   $$
   **该式称为 $\ce{SO}(3)$ 上的伴随性质。同样地，在 $\ce{SE}(3)$ 上也有伴随性质：**
   $$
   T\exp(\xi^{\wedge})T^{-1}=\exp((\ce{Ad}(T)\xi)^{\wedge})
   $$
   **其中：**
   $$
   \ce{Ad}(T)=\left[\matrix{R&t^\wedge R\\0&R}\right]
   $$
   

7. **仿照左扰动的推导，推导 $\ce{SO}(3)$ 和 $\ce{SE}(3)$ 在左右扰动下的导数。**

   

8. **搜索 cmake 的 find_package 指令是如何运作的。它有哪些可选的参数？为了让 cmake 找到某个库，需要哪些先决条件？**

   

# 第5讲 相机与图像

机器人是如何观测外部世界的，这个观测就是相机成像的过程。



## 习题

1. 寻找一部相机（你的手机或笔记本的摄像头即可），标定它的内参。你可能会用到标定板，或者自己打印一张标定用的棋盘格。
2. 叙述相机内参的物理意义。如果一部相机的分辨率变为原来的两倍而其他地方不变，那么它的内参将如何变化？
3. 搜索特殊相机（鱼眼或全景相机）的标定方法。它们与普通的针孔模型有何不同？
4. 调研全局快门（global shutter）相机与卷帘快门（rolling shutter）相机的异同。它们在SLAM中有何优缺点？
5. RGB-D相机是如何标定的？以Kinect为例，需要标定哪些参数？（参照https://github.com/code-iai/iai_kinect2）
6. 除了示例程序演示的遍历图像的方式，你还能举出哪些遍历图像的方法？
7. 阅读OpenCV官方教程，学习它的基本用法。



# 第6讲 非线性优化
