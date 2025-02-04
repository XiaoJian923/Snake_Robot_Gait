# 蛇形机器人步态

### 介绍
使用MuJoCo开源物理引擎，仿真实现蛇形机器人各种步态。

### 特色😆
包含MuJoCo模型、参考文献、中英文双语代码文件、中英文双语README.md文件、步态讲解和前置基础知识说明等。[English version of README.md](./README.md)

### 内容及进度
|    步态     |         模型          | 模型进度 |                       代码                       | 代码进度 |                            参考文献                            |
|:---------:|:-------------------:|:----:|:----------------------------------------------:|:----:|:----------------------------------------------------------:|
| **蜿蜒步态**  |                     |      |                                                |      |                                                            |
| **行波步态**  |                     |      |                                                |      |                                                            |
| **翻滚步态**  |                     |      |                                                |      |                                                            |
| **履带步态**  | Snake_Robot_o30.xml |  Y   | [CrawlerGait_zh.py](./codes/CrawlerGait_zh.py) |  Y   |                   [2016](./references/2016_Gait_Design_of_A_Snake_Robot_by_Connecting_Simple_Shapes.pdf)                    |
| **足波步态**  |                     |      |                                                |      |                                                            |

### 准备
- 知识
  - Python **[必要]**
  - 微分几何-曲线论 [b站视频课程](https://www.bilibili.com/video/BV1K54y1a7cf/?spm_id_from=333.1007.top_right_bar_window_custom_collection.content.click&vd_source=7a02bcb69ff53d02d5749b97cdd79fdc) [飞书文档笔记](./materials/Differential_Geometry_Study_Notes_Curve_Theory.pdf)
- 软件
  - Anaconda
  - Pycharm
> 1. 笔者的环境配置见**environment.yaml**。安装Anaconda后，可通过命令``conda env create -f environment.yaml``复现环境
> 
> 2单独安装MuJoCo，使用命令``pip install mujoco``


### 内容详解
- **蜿蜒步态**
> 前置基础知识：初等数学（三角函数）、微积分（尤其定积分的定义）和微分几何（曲线弧长参数、曲线曲率）。

[1] S.Hirose, Biologically Inspired Robots: Snake-Like Locomotors and Manipulators. New York, NY: Oxford University Press, 1993.

[2] Saito M, Fukaya M, Iwasaki T. Modeling, analysis, and synthesis of serpentine locomotion with a multilink robotic snake[J]. IEEE control systems magazine, 2002, 22(1): 64-81.
**************
>下述内容参考自文献[2]，文献[2]又参考自文献[1]。笔者未能找到文献[1]的电子版与纸质版😭

***Serpenoid Curve***

在x-y平面，若过原点的曲线满足
$$
x(s)=\int_0^scos(\xi_\sigma)d\sigma,\ \ \ \ y(s)=\int_0^ssin(\xi_\sigma)d\sigma,\ \ \ \ \xi_\sigma:=acos(b\sigma)+c\sigma \tag{1.1}
$$
其中a、b、c为标量，s为弧长（表示从原点到该点的曲线长度），则称该曲线为一条Serpenoid曲线。

参数a决定了曲线的波动程度，参数b决定了单位长度内的周期数，参数c决定了宏观的圆形形状。

文献[1]：Serpenoid曲线的曲率是个正弦曲线函数，可得
$$
\kappa(s)=\sqrt{(\frac{\mathrm{d}^2x}{\mathrm{d}s^2})^2+(\frac{\mathrm{d}^2y}{\mathrm{d}s^2})^2}=\vert absin(bs)-c\vert \tag{1.2}
$$
> 然而，文献[2]并没有给出Serpenoid曲线的推导过程。下面尝试对上式进行推导。注：下面内容部分生成于[DeepSeek](https://www.deepseek.com/)。

1. **定义曲率** 假设Serpenoid曲线的曲率$\kappa(s)$是弧长$s$的函数，且具有周期性。例如：
$$
\kappa(s)=Asin(\omega s)+c \tag{2.1}
$$
2. **建立微分方程** 设曲线的切线与某固定方向的夹角为$\theta(s)$，则曲线的切向量可表示为$\vec{T}(s)=(cos\theta(s), sin\theta(s))$。由微分几何曲线论知识可得：
$$
\frac{\mathrm{d}\theta(s)}{\mathrm{d}s}=\kappa(s)
$$
两边同时对$s$积分可得：
$$
\theta(s)=\int_0^s(Asin(\omega \tau)+c)d\tau
$$
$$
\theta(s)=-\frac{A}{\omega}cos(\omega s)+cs+\theta_0
$$
由现实意义可知：$\theta(0)=0$，即$\theta_0=0$。因此
$$
\theta(s)=-\frac{A}{\omega}cos(\omega s)+cs \tag{2.2}
$$
若记$\theta=\xi,s=\sigma,\omega=b,A=-ab$，对<font color=red>$(2.1)$</font>与<font color=red>$(2.2)$</font>进行部分替换，可得
$$
\kappa(s)=-absin(bs)+c
$$
$$
\xi(\sigma)=acos(b\sigma)+c\sigma
$$
这与$(1.1)$的夹角、$(1.2)$相一致。
3. **积分得到坐标** 对切向量积分可得到曲线的坐标：
$$
x(s)=\int_0^scos\theta(\tau)d\tau,\ \ \ \ y(s)=\int_0^ssin\theta(\tau)d\tau
$$
这与$(1.1)$的坐标公式相一致。
> 上述内容可加深对Serpenoid曲线的理解，<font color=red>但假设$\kappa(s)$为正弦曲线的理由</font>还需要进一步阅读论文 **[1]**。
> 
> 下述内容给出$n-1$个关节（joints），$n$个片段（links, segments），$n+1$个点（points）的蛇形机器人对Serppenoid曲线的近似 **（离散化）**。

设蛇形机器人总长度为1，则每个片段长$1/n$。$x(s),y(s),0\leq s\leq 1$可表示Serpenoid曲线。$s_i:=i/n(i=0,...,n)$可表示$n+1$个点，其中$s_i:=i/n(i=1,...,n-1)$表示$n-1$个**关节点**。由定义可知（需掌握定积分的几何意义）：
$$
x_i=\sum_{k=1}^i\frac{1}{n}cos(acos(\frac{kb}{n})+\frac{kc}{n}),\ \ \ \ y_i=\sum_{k=1}^i\frac{1}{n}sin(acos(\frac{kb}{n})+\frac{kc}{n})
$$
连接$n+1$个点$(x_i,y_i)$，可得n个直线片段对Serpenoid曲线的近似。

记第$i$个片段与$x$轴的逆时针方向夹角为$\theta_i$，由几何意义可得
$$
tan(\theta_i)=\frac{y_i-y_{i-1}}{x_i-x_{i-1}}=\frac{sin(acos(ib/n)+ic/n)}{cos(acos(ib/n)+ic/n)}
$$
$$
\theta_i=acos(\frac{ib}{n})+\frac{ic}{n},\ \ \ \ i=1,...,n
$$
决定离散Serpenoid曲线的相对角（**关节点**处的角度）由下式可得：
$$
\phi_i:=\theta_i-\theta_{i+1},\ \ \ \ i=1,...n-1
$$
$$
\phi_i:=a(cos(\frac{ib}{n})-cos(\frac{(i+1)b}{n}))-\frac{c}{n}
$$
由和差化积公式
$$
cos\alpha - cos\beta=-2sin(\frac{\alpha+\beta}{2})sin(\frac{\alpha-\beta}{2})
$$
可得
$$
\phi_i:=-2asin(\frac{ib}{n}+\frac{b}{2n})sin(-\frac{b}{2n})-\frac{c}{n}
$$
记$\alpha:=a\vert sin(\frac{\beta}{2})\vert,\beta:=\frac{b}{n},\gamma:=-\frac{c}{n}$则
$$
\phi_i:=-2asin(i\beta+\frac{\beta}{2})sin(-\frac{\beta}{2})+\gamma
$$
$$
\phi_i:=\alpha sin(i\beta+\frac{\beta}{2})+\gamma
$$
可知相邻相对角的相位差为$\beta$。
> 至此可得文献[2]中的表达式。下面给出蛇形机器人物理样机实现蜿蜒步态的函数表达式。

***Serpentine Locomotion***
$$
\phi_i(t)=\alpha sin(\omega t+(i-1)\beta)+\gamma,\ \ \ \ (i=1,...,n-1)
$$
-------------
> 后续内容：对文献[2]中



- **行波步态** 



- **翻滚步态**



- **履带步态**
> 前置基础知识：微分几何-曲线论（Frenet-Serret Frame）。

[1] Takemori T, Tanaka M, Matsuno F. Gait design of a snake robot by connecting simple shapes[C]//2016 IEEE International Symposium on Safety, Security, and Rescue Robotics (SSRR). IEEE, 2016: 189-194.

[2] Takemori T, Tanaka M, Matsuno F. Gait design for a snake robot by connecting curve segments and experimental demonstration[J]. IEEE Transactions on Robotics, 2018, 34(5): 1384-1391.
**************







- **足波步态**




## 联系我😊
邮箱: xjxf0923@gmail.com 3332407087@qq.com

微信: xjxf0923

***
> markdown语法 https://markdown.com.cn/basic-syntax/
> 
> Emoji表情 https://emojipedia.org/

