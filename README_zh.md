# TDRC-J2T 中文版readme

该项目是一个将关节空间转换为肌腱空间的TDRC（肌腱驱动连续机器人）机器人的演示。代码同时使用C++和Python实现，并使用NumPy库进行数值计算。

## 公式推导
该项目针对的连续体机器人是一个刚柔耦合连续体机械臂，具备一个刚性基座、直线单元和一个双段连续体段，其中连续体段由两个连续体单元组成，各具备两个正交的弯曲自由度。
靠近基座的连续体段称作近段连续体（Proximal Segment），远离基座的连续体段称作远段连续体（Distal Segment）。
在该项目中，我们使用基于恒曲率假设的连续体机器人运动学模型，来推导关节空间到肌腱空间的转换公式。

### 关节空间到肌腱空间的转换关系
连续体段的运动学分析基于恒曲率假设，即每个连续体段在弯曲时保持恒定的曲率。坐标系定义和参数化如图所示：
<img src="./fig/kine_ex.png" alt="Coordinate System and Parameters" width="60%" />
由于近段和远段连续体之间存在过渡段，因此在符号上将近段表示为$a$，远端表示为$c$（过渡段为$b$）。对于每个连续体段，我们定义了以下参数：
- $\phi_a$: 近段弯曲平面角度
- $\theta_a$: 近段弯曲角度
- $\phi_c$: 远段弯曲平面角度
- $\theta_c$: 远段弯曲角度
- $l_a$: 近段长度
- $l_c$: 远段长度

其中，$\phi_a$和$\phi_c$的定义是相对于离散关节圆心的局部坐标系的X轴的夹角，$\theta_a$和$\theta_c$的定义是连续体段末端相对于连续体段起始位置的弯曲角度。

图1(b)中展示了离散连续体关节驱动丝孔位与驱动丝弯曲平面角度和弯曲角度的关系，图中特别展示的是近段连续体的驱动丝孔位与$\phi_a$和$\theta_a$的关系，远段离散关节与其类似，但是缺少两对驱动丝孔位（因为远端连续体的驱动丝需要穿过近段连续体，因此近段连续体需要保留与远段连续体相关的驱动丝孔位）。

通过分析，容易得出，当$\phi_a$和$\theta_a$确定后，穿过各孔的驱动丝长度在该段连续体中的长度就确定了，其长度变化量的大小，与离散关节圆心到该孔的距离在弯曲平面上的的投影成正比，具体推导如下：

首先，将孔位进行标号，如图中所示，驱动近段连续体的孔位分别为$h_{a1}$、$h_{a2}$、$h_{a3}$、$h_{a4}$，远段连续体的孔位分别为$h_{c5}$、$h_{c6}$、$h_{c7}$、$h_{c8}$。其中，$h_{a5}$、$h_{a6}$、$h_{a7}$、$h_{a8}$分别为远段连续体的驱动丝孔位在近段连续体中的对应孔位。然后对每根驱动丝也进行编号，$t_1, t_2,...,t_8$分别与相应下标的孔位对应。

对于近段连续体，本文规定，将驱动丝的”收线量“定义为长度变化量$\Delta l$。约定当驱动丝长度缩短时, $\Delta l > 0$；当驱动丝长度变长时, $\Delta l < 0$.

针对本案例而言，$\Delta l_{ai}$（其中$i=1,2,3,4$）与离散关节圆心到孔位$h_{ai}$的距离在弯曲平面上的投影成正比，即：

$$\Delta l_{ai} = r \theta_a \cos(\gamma_{ai} - \phi_a), \qquad i\in\{1,2,3,4\}$$

其中，$r$为离散关节圆心到孔位的距离，$\gamma_{ai}$为孔位$h_{ai}$为离散关节圆心的极角，具体定义如下：

$$\gamma_{ai}=(i-1)\frac{\pi}{2}, \qquad i\in\{1,2,3,4\}.$$

此处很好理解：由于驱动孔是均匀、等半径分布，对于近段连续体，一共有8个驱动孔，其中4个用于近段连续体，4个用于远段连续体。对于近段连续体的驱动孔，相对于局部坐标系的X轴，孔位$h_{a1}$、$h_{a2}$、$h_{a3}$、$h_{a4}$分别位于离散关节圆心的0度、90度、180度和270度位置，因此其极角$\gamma_{ai}$分别为0、$\pi/2$、$\pi$和$3\pi/2$。对于远段连续体的驱动孔，孔位$h_{c5}$、$h_{c6}$、$h_{c7}$、$h_{c8}$分别位于离散关节圆心的45度、135度、225度和315度位置，因此其极角$\gamma_{ci}$分别为$\pi/4$、$3\pi/4$、$5\pi/4$和$7\pi/4$. 同理，孔位$h_{a5}$、$h_{a6}$、$h_{a7}$、$h_{a8}$分别位于离散关节圆心的45度、135度、225度和315度位置，因此其极角$\gamma_{ai}$分别为$\pi/4$、$3\pi/4$、$5\pi/4$和$7\pi/4$. 由于近段连续体的驱动孔与远段连续体的驱动孔在同一位置，因此它们的极角相同。

继续分析，近段连续体驱动丝长度的变化量$\Delta l_{aj}$,（其中 $j=5,6,7,8$）为：

$$\Delta l_{aj} = r \theta_a \cos(\gamma_ {aj} - \phi_a), \qquad j\in\{5,6,7,8\}$$

其中，$\gamma_{aj}$为孔位$h_{aj}$为离散关节圆心的极角，具体定义如下:
$$\gamma_{aj}=(j-5)\frac{\pi}{2}+\frac{\pi}{4}, \qquad j\in\{5,6,7,8\}.$$

至此，近段连续体的驱动丝长度变化量$\Delta l_{ai}$（其中$i=1,2,3,4$）和$\Delta l_{aj}$（其中$j=5,6,7,8$）与弯曲平面角度$\phi_a$和弯曲角度$\theta_a$的关系已经明确。

接下来推导远段连续体的驱动丝长度变化量，由于远段连续体的驱动丝需要穿过近段连续体，因此远段驱动丝长度的变化量由两部分组成：1）其穿过近段时因近段弯曲造成的长度变化 $\Delta l_{aj}$（其中$j=5,6,7,8$）和2）其在远段连续体中因远段弯曲造成的长度变化$\Delta l_{cj}$（其中$j=5,6,7,8$）。因此，总变化为两者之和。
首先计算远段连续体的驱动丝长度变化量$\Delta l_{cj}$（其中$j=5,6,7,8$）与远段连续体的弯曲平面角度$\phi_c$和弯曲角度$\theta_c$的关系为：

$$\Delta l_{cj} = r \theta_c \cos(\gamma_{cj} - \phi_c), \qquad j\in\{5,6,7,8\}$$

其中，$\gamma_{cj}$为孔位$h_{cj}$为离散关节圆心的极角，具体定义如下（在本案例中，$h_{cj}$与$h_{aj}$相对应, 因此$\gamma_{cj}$与$\gamma_{aj}$相同）：
$$\gamma_{cj}=(j-5)\frac{\pi}{2}+\frac{\pi}{4}, \qquad j\in\{5,6,7,8\}.$$

最终，控制近段和远段的每根驱动丝的长度变化量可以分别表示为：
$$\Delta l_i = \Delta l_{ai} = r \theta_a \cos(\gamma_{ai} - \phi_a), \qquad i\in\{1,2,3,4\}$$
$$\Delta l_j = \Delta l_{aj} + \Delta l_{cj} = r \theta_a \cos(\gamma_{aj} - \phi_a) + r \theta_c \cos(\gamma_{cj} - \phi_c), \qquad j\in\{5,6,7,8\}$$

#### 展开形式

为了便于理解，此处直接展开所有驱动丝的长度变化量与连续体关节参数的关系：

对于近段连续体的 4 根驱动丝，有：

$$\Delta l_1=r\theta_a\cos(0-\phi_a)=r\theta_a\cos\phi_a$$

$$\Delta l_2=r\theta_a\cos\!\left(\frac{\pi}{2}-\phi_a\right)=r\theta_a\sin\phi_a$$

$$\Delta l_3=r\theta_a\cos(\pi-\phi_a)=-r\theta_a\cos\phi_a=-\Delta l_1$$

$$\Delta l_4=r\theta_a\cos\!\left(\frac{3\pi}{2}-\phi_a\right)=-r\theta_a\sin\phi_a=-\Delta l_2$$

对于远段连续体对应的 4 根驱动丝，由于其需要先穿过近段连续体，再作用于远段连续体，因此总长度变化量由两部分组成：一部分是穿过近段连续体时因近段弯曲产生的长度变化，另一部分是在远段连续体中因远段弯曲产生的长度变化。于是有：

$$\Delta l_5=r\theta_a\cos\!\left(\frac{\pi}{4}-\phi_a\right)+r\theta_c\cos\!\left(\frac{\pi}{4}-\phi_c\right)$$

$$\Delta l_6=r\theta_a\cos\!\left(\frac{3\pi}{4}-\phi_a\right)+r\theta_c\cos\!\left(\frac{3\pi}{4}-\phi_c\right)$$

$$\Delta l_7=r\theta_a\cos\!\left(\frac{5\pi}{4}-\phi_a\right)+r\theta_c\cos\!\left(\frac{5\pi}{4}-\phi_c\right)=-\Delta l_5$$

$$\Delta l_8=r\theta_a\cos\!\left(\frac{7\pi}{4}-\phi_a\right)+r\theta_c\cos\!\left(\frac{7\pi}{4}-\phi_c\right)=-\Delta l_6$$

### 电机空间到肌腱空间的转换关系
为了建立电机空间到肌腱空间的转换关系，我们需要将上述驱动丝长度变化量与电机轴的旋转角度进行关联。假设电机旋转角度为$\alpha_k$（其中$k=1,2,...,8$），绕线轴的直径为$d$。若忽略多层缠绕、打滑以及驱动丝弹性伸长等因素，并假设绕线轴等效半径恒定，则电机转角与绕线长度之间满足：
$$\Delta l_{spool} = \frac{d}{2} \alpha_k, \qquad k\in\{1,2,3,4\}$$
其中，$\alpha_k$以弧度计。

在本案例中，控制同一个自由度的两根驱动丝分别绕在同一个电机绕线轴上，并按照相反方向旋转，为了更加直观地展示电机旋转与驱动丝变化的关系，我们对控制连续体部分的4个电机进行编号：
- 电机$m_1$控制驱动丝$t_1$和$t_3$
- 电机$m_2$控制驱动丝$t_2$和$t_4$
- 电机$m_3$控制驱动丝$t_5$和$t_7$
- 电机$m_4$控制驱动丝$t_6$和$t_8$. 

下面所有clockwise/counter-clockwise的参考均为从上往下看绕线轴。

在本案例中，对于每一对驱动丝，编号较低的驱动丝绕在电机绕线轴的下侧（以绕线轴为起点，逆时针缠绕），编号较高的驱动丝绕在电机绕线轴的上侧（以绕线轴为起点，顺时针缠绕）。规定电机 $z$ 轴向上为正方向，当电机正转（$\alpha_k > 0$ ，逆时针旋转）时：
- 编号较低的驱动丝几何长度增加，因此收线量为负；
- 编号较高的驱动丝几何长度减少，因此收线量为正。

因此，对于每一对驱动丝，我们可以得到以下关系：
$$\alpha_1=-\frac{2}{d}\Delta l_1=\frac{2}{d}\Delta l_3$$

$$\alpha_2=-\frac{2}{d}\Delta l_2=\frac{2}{d}\Delta l_4$$

$$\alpha_3=-\frac{2}{d}\Delta l_5=\frac{2}{d}\Delta l_7$$

$$\alpha_4=-\frac{2}{d}\Delta l_6=\frac{2}{d}\Delta l_8$$

### 关节空间到电机空间的转换关系

#### 关节空间到电机空间的变换
通过上述两个转换关系，我们可以将关节空间的参数（$\phi_a$、$\theta_a$、$\phi_c$、$\theta_c$）转换为电机空间的参数（$\alpha_1$、$\alpha_2$、$\alpha_3$、$\alpha_4$）。具体转换关系如下：

$$\alpha_1=-\frac{2r}{d}\theta_a\cos\phi_a$$

$$\alpha_2=-\frac{2r}{d}\theta_a\sin\phi_a$$

$$\alpha_3=-\frac{2r}{d}\left[\theta_a\cos\!\left(\frac{\pi}{4}-\phi_a\right)+\theta_c\cos\!\left(\frac{\pi}{4}-\phi_c\right)\right]$$

$$\alpha_4=-\frac{2r}{d}\left[\theta_a\cos\!\left(\frac{3\pi}{4}-\phi_a\right)+\theta_c\cos\!\left(\frac{3\pi}{4}-\phi_c\right)\right]$$


#### 电机空间到关节空间的变换

记
$$K = \frac{2r}{d}$$

则近段连续体的弯曲角度和弯曲平面角可由 \(\alpha_1,\alpha_2\) 直接求得：

\[
\theta_a=\frac{1}{K}\sqrt{\alpha_1^2+\alpha_2^2}
=\frac{d}{2r}\sqrt{\alpha_1^2+\alpha_2^2}
\]

\[
\phi_a=\operatorname{atan2}(-\alpha_2,-\alpha_1)
\]

接着定义中间变量：

\[
u=-\frac{\alpha_3}{K}-\theta_a\cos\!\left(\frac{\pi}{4}-\phi_a\right)
\]

\[
v=-\frac{\alpha_4}{K}-\theta_a\cos\!\left(\frac{3\pi}{4}-\phi_a\right)
\]

即

\[
u=-\frac{d}{2r}\alpha_3-\theta_a\cos\!\left(\frac{\pi}{4}-\phi_a\right)
\]

\[
v=-\frac{d}{2r}\alpha_4-\theta_a\cos\!\left(\frac{3\pi}{4}-\phi_a\right)
\]

则远段连续体满足：

\[
\theta_c\cos\phi_c=\frac{u-v}{\sqrt2}
\]

\[
\theta_c\sin\phi_c=\frac{u+v}{\sqrt2}
\]

从而可得远段连续体的弯曲角度和弯曲平面角：

\[
\theta_c=\sqrt{u^2+v^2}
\]

\[
\phi_c=\operatorname{atan2}(u+v,\;u-v)
\]

需要注意的是，当 \(\theta_a=0\) 时，\(\phi_a\) 无物理意义；当 \(\theta_c=0\) 时，\(\phi_c\) 无物理意义。因此在逆变换中，若某段弯曲角接近 0，则对应弯曲平面角会出现退化或数值不稳定。这一问题将在后续的版本中通过泰勒展开和关节参数化方法进行优化，当前版本中暂不处理这一问题。

## 代码实现

代码实现分为Python版本和C++版本，两个版本的功能和接口保持一致，均提供了关节空间到电机空间的转换函数和电机空间到关节空间的转换函数。用户可以根据需要选择使用Python版本或C++版本进行调用。具体代码实现请参见项目中的`src`目录下的相应文件。