#### ① 耦合机器人运动学
机器人的广义坐标被定义为：

$ \boldsymbol z=
\begin{bmatrix}
\boldsymbol q_{DH}^{\mathrm T}&\phi&\theta&\beta
\end{bmatrix}^{\mathrm T}\in\mathbb R^{10} $

+ $ \boldsymbol q_{DH}\in\mathbb R^7 $是刚性机械臂关节角
+ $ \theta,\phi

 $是柔性段弯曲角
+ $ \beta $用于确认轴线上 RCM 点位置的虚拟坐标，是 RCM 点在轴线上的插值参数



相机的末端位姿转化矩阵为：

$ \boldsymbol T_{09}(\boldsymbol z)=
\boldsymbol T_{08}(\boldsymbol q_{DH})
\boldsymbol T_{\mathrm{pcc}}(\phi,\theta)
\boldsymbol T_c $

+ $ \boldsymbol T_{08} $是机械臂到柔性段基座的位姿
+ $ \boldsymbol T_{\mathrm{pcc}} $是柔性段 PCC 变换
+ $ \boldsymbol T_c $是相机固定安装变换

刚性机械臂的雅可比矩阵表示为：

$ \boldsymbol{J}_r=
\begin{bmatrix}
\boldsymbol{J}_{r,p}^{\mathrm T}&
\boldsymbol{J}_{r,\omega}^{\mathrm T}
\end{bmatrix}^{\mathrm T}
\in\mathbb{R}^{6\times7} $

其中$ \boldsymbol J_{r,p},\boldsymbol J_{r,\omega}\in\mathbb R^{3\times7} $，对应机械臂关节角的变化$ \dot{\boldsymbol q}_{DH} $

相机坐标系相较于机械臂末端的雅可比矩阵为：$ \boldsymbol J_{\mathrm{tool}}(\phi,\theta)
 $，该雅可比通过$ \boldsymbol T_{89}(\boldsymbol z)=\boldsymbol T_{\mathrm{pcc}}(\phi,\theta)
\boldsymbol T_c $可获得，对应连续体的弯曲$ 
\begin{bmatrix}
\dot{\phi}\\
\dot{\theta}
\end{bmatrix} $

因此末端相机相较于世界坐标系的速度，可由刚性机械臂的速度加上连续体部分叠加：

$ {}^{0}\boldsymbol V_9
=\begin{bmatrix}
{}^{0}\boldsymbol v_{9}\\
{}^{0}\boldsymbol\omega_{9}
\end{bmatrix}=
{}^{0}\boldsymbol V_9^{\mathrm{rigid}}+
{}^{0}\boldsymbol V_9^{\mathrm{flexible}} $

根据速度的叠加，末端速度为机械臂线速度带来的变化，加上角速度带来的变化，可得：

$ {}^{0}\boldsymbol v_{9}^{\mathrm{rigid}}
=
\left[
\boldsymbol J_{r,p}
+
\widehat{
{}^{0}\boldsymbol p_{08}-{}^{0}\boldsymbol p_{09}
}
\boldsymbol J_{r,\omega}
\right]
\dot{\boldsymbol q}_{DH} $

              $ {}^{0}\boldsymbol\omega_{09}^{\mathrm{rigid}}
=
\boldsymbol J_{r,\omega}\dot{\boldsymbol q}_{DH} $

$ {}^{0}\boldsymbol V_{9}^{\mathrm{rigid}}
=
\left[
\boldsymbol J_r+
\boldsymbol S_p\boldsymbol J_{r,\omega}
\right]\dot{\boldsymbol q}_{DH} $

$ \boldsymbol R_p=
\begin{bmatrix}
\widehat{
{}^{0}\boldsymbol p_{08}-{}^{0}\boldsymbol p_{09}
}\\
0_{3 \times 3}
\end{bmatrix} $

而连续体的速度为：

$  {}^{0}\boldsymbol V_{9}^{\mathrm{flexible}}=
\boldsymbol R_p
\boldsymbol J_{\mathrm{tool}}
\begin{bmatrix}
\dot\phi\\
\dot\theta
\end{bmatrix} $

$ \boldsymbol R_p=
\begin{bmatrix}
{}^{0}\boldsymbol R_8&\boldsymbol0\\
\boldsymbol0&{}^{0}\boldsymbol R_8
\end{bmatrix} $

综上所述：

$ {}^{0}\boldsymbol V_9
=\begin{bmatrix}
{}^{0}\boldsymbol v_{9}\\
{}^{0}\boldsymbol\omega_{9}
\end{bmatrix}=\begin{bmatrix}
\boldsymbol J_g &0_{6\times1}\end{bmatrix}\dot{\boldsymbol z}=\begin{bmatrix}
\boldsymbol J_r+
\boldsymbol S_p\boldsymbol J_{r,\omega}&\boldsymbol R_p
\boldsymbol J_{\mathrm{tool}} & 0_{6\times1}\end{bmatrix}\dot{\boldsymbol z} $

#### ② 视觉伺服和 RCM 约束
###### （1）视觉伺服
单个图像点的交互矩阵(既 3 维空间点到成像平面点的映射)

$ \boldsymbol J_{\mathrm{img}}=
\begin{bmatrix}
\boldsymbol L_t&\boldsymbol L_\omega
\end{bmatrix}                   $

速度的映射：$ \boldsymbol L_t=
\begin{bmatrix}
-\bar f/Z&0&\bar u/Z\\
0&-\bar f/Z&\bar v/Z
\end{bmatrix} $

旋转的映射：$ \boldsymbol L_\omega=
\begin{bmatrix}
\bar u\bar v/\bar f&-(\bar f^2+\bar u^2)/\bar f&
\bar v\\(\bar f^2+\bar v^2)/\bar f&-\bar u\bar v/\bar f&-\bar u
\end{bmatrix} $

此处因为相机自身运动与成像平面内点运动的关系为：

$ \begin{bmatrix}
\dot{u}\\
\dot{v}
\end{bmatrix} = J_{\mathrm{img}}\begin{bmatrix}
{}^{c} \boldsymbol{v}_{c}\\
{}^{c} \boldsymbol{w}_{c}
\end{bmatrix} $

因此可以得到成像平面内点速度和各关节的关系：

$ \boldsymbol J_{v0}=
\begin{bmatrix}
\alpha_s
\boldsymbol J_{\mathrm{img}}
\boldsymbol X
\boldsymbol J_g&
\boldsymbol 0_{2\times1}
\end{bmatrix}
\in\mathbb R^{2\times10} $

+ $ \alpha_s $是恒定的缩放因子
+ $ \boldsymbol X =\begin{bmatrix}
{}^{0}\boldsymbol R_9^T&\boldsymbol0\\
\boldsymbol0&{}^{0}\boldsymbol R_9^T
\end{bmatrix} $

**<font style="color:#DF2A3F;">注意：虽然刚性机械臂的运动学校准良好，但是 PCC 模型下的连续体，以及制造导致的弯曲不确定性，导致整个视觉伺服的雅可比矩阵并非一个精确模型</font>**

###### （2）RCM 模型
定义 RCM 点：

$ \boldsymbol p_{\mathrm{rcm}}
=
\boldsymbol p_{07}
+
\beta
\left(
\boldsymbol p_{08}-\boldsymbol p_{07}
\right) $

+ $ \boldsymbol p_{07} $是近轴端点，$ \boldsymbol p_{08} $是远轴端点
+ $ \boldsymbol p_{t} $是固定套管的穿刺点
+ $ \beta $是 RCM 点在轴线上的插值参数

对应的 RCM 误差为：

$ \boldsymbol e_{\mathrm{rcm}}=
\boldsymbol p_{\mathrm{rcm}}-\boldsymbol p_t $

RCM 点和插值参数$ \beta $的变化可表示为（将表示的 RCM 对时间求点，再统一到关节角表示可得）：

$ \dot{\boldsymbol p}_{\mathrm{rcm}}=
\left[
\boldsymbol J_7+
\beta(\boldsymbol J_8-\boldsymbol J_7)
\right]
\dot{\boldsymbol q}_{DH}+
(\boldsymbol p_{08}-\boldsymbol p_{07})\dot\beta $

+ $ \boldsymbol J_7 $和$ \boldsymbol J_8 $分别是连个端点的的雅可比矩阵

进一步可得 RCM 的雅可比矩阵为

$ \boldsymbol J_{\mathrm{rcm}}=
\begin{bmatrix}
\boldsymbol J_7+
\beta(\boldsymbol J_8-\boldsymbol J_7)
&\boldsymbol 0_{3\times2}&
\boldsymbol p_{08}-\boldsymbol p_{07}
\end{bmatrix} $

#### ③ 仿真设计
##### （1）情况说明  
对于建模的运动学，因为主要是关心位置的变化情况，而不是位姿的变化，因此估计连续体产生的雅可比矩阵$ \boldsymbol J_{\mathrm{tool}}(\phi,\theta)
 $
采样对象：

$ \mathcal D_k=
\left\{q_{c,k},
{}^{B}p_{C,k},
{}^{B}R_{C,k},
s_k,t_k\right\} $

+ $ q_{c,k} $是连续体构型
+ $ {}^{B}p_{C,k} $相机相对位置
+ $ {}^{B}R_{C,k} $相对旋转矩阵
+ $ s_k $图像特征
+ $ t_k $时间戳

在 matlab 中建立一个高精度的 PCC 模型作为教师模型：

$ p_T=f_T(q_c)
\\
q_c=\begin{bmatrix}
\phi\\
\theta
\end{bmatrix} $

然后产生雅可比矩阵：

$ J_{p,T}(q_c)
=\frac{\partial f_T(q_c)}{\partial q_c}
\in\mathbb R^{3\times2} $

+ $ T $表示教师模型，并非转置

神经网络转而进行如下学习：

$ q_c\longmapsto
\widehat J_p(q_c)
\approx J_{p,T}(q_c)
 $

##### （2） 仿真设计
###### 1） 建立教师模型
**① 网络只学习 3x2 的位置雅可比**

变量定义：$ \phi $是弯曲平面角，$ \theta $是弯曲角

位置雅可比可表示为：

$ J_p(\phi,\theta)=
\begin{bmatrix}
\dfrac{\partial p_x}{\partial\phi}&
\dfrac{\partial p_x}{\partial\theta}
\\[6pt]
\dfrac{\partial p_y}{\partial\phi}
&
\dfrac{\partial p_y}{\partial\theta}
\\[6pt]
\dfrac{\partial p_z}{\partial\phi}
&\dfrac{\partial p_z}{\partial\theta}
\end{bmatrix}
 $

此时满足：

$ \dot p=J_p(q_c)\dot q_c $

此处的相机的光心位置在连续体基坐标系$ \{B\} $中可表示为：

$ p={}^Bp_C $

因此教师雅可比矩阵对整个表达式求导：

$ J_{p,T}=\frac{\partial p_C}{\partial q_c} $

**② 教师模型的计算方式**

对于此时没有额外末端偏置的单端，假设其柔性长度为$ L_f $，则可以写为：

$ p_T(\phi,\theta)=L_f
\begin{bmatrix}
a(\theta)\cos\phi\\
a(\theta)\sin\phi\\
b(\theta)
\end{bmatrix} $

+ $ a(\theta)=\frac{1-\cos\theta}{\theta}
 $
+ $ 
b(\theta)=\frac{\sin\theta}{\theta} $

直接求导可以得到：

$ J_{p,T}=L_f
\begin{bmatrix}
-a(\theta)\sin\phi & a'(\theta)\cos\phi\\
a(\theta)\cos\phi & a'(\theta)\sin\phi\\
0 & b'(\theta)
\end{bmatrix} $

$ a'(\theta)=\frac{\theta\sin\theta-(1-\cos\theta)}{\theta^2}
 $

$ b'(\theta)=\frac{\theta\cos\theta-\sin\theta}{\theta^2} $

###### 2）产生数据
**① 训练样本**

训练样本可表示为：

$ \mathcal D_i=
\left\{
q_{c,i},\,
p_{T,i},\,
J_{p,T,i}
\right\} $

位置$ p_{T,i} $ 主要用于后续运动预测检查、位置观测器和闭环控制，不是必须与雅可比一起作为网络输出

将雅可比按列排开，此时标签排列为：

$ y_i=
\begin{bmatrix}
J_{x\phi}\\
J_{y\phi}\\
J_{z\phi}\\
J_{x\theta}\\
J_{y\theta}\\
J_{z\theta}
\end{bmatrix}
\in\mathbb R^6 $

**② 输入、输出与采样**

输入仍然是两个角度，但是做特征编码

$ \xi(q_c)=
\begin{bmatrix}
\sin\phi\\
\cos\phi\\
\theta/\theta_s\\
1\end{bmatrix}
\in\mathbb R^4
 $

+ $ \theta_s>0 $ 是固定归一化尺度，例如工作范围中的最大弯曲角

对输出做统一尺度处理  
使用米和弧度，并选择固定长度尺度$ L_s $，例如柔性段长度：

$ \bar p=\frac{p}{L_s}
 $

$ \bar J_p=\frac{J_p}{L_s} $

网络学习$ \bar J_p $，控制器使用时恢复：

$ \widehat J_p=L_s\widehat{\bar J}_p $

这样后续位置观测误差和位置跟踪误差也统一使用归一化位置，避免离线训练和在线更新使用不同尺度  

然后在二维角度范围内生成上千条样本

###### 3）建立位置雅可比网络分支
**① 网路输出完整雅可比**

采用：

$ \widehat J_p(q_c)=L_s
\begin{bmatrix}
f_{\Theta_\phi}(\xi)&f_{\Theta_\theta}(\xi)
\end{bmatrix} $

+ $ f_{\Theta_\phi}(\xi)\in\mathbb R^3
 $
+ $ f_{\Theta_\theta}(\xi)\in\mathbb R^3 $

 教师 PCC 负责产生数据；正常控制时需要的雅可比由网络完整输出。  

**② 初始的主干结构**

每个分支采用：

$ 4\rightarrow32\rightarrow32\rightarrow16\rightarrow3
 $

三个隐藏层使用$ \tanh $，最后一层线性输出

对于分支$ j\in\{\phi,\theta\}

 $：

$ z_{0j}=\xi, $

$ z_{1j}=\tanh(W_{1j}z_{0j}), $

$ z_{2j}=\tanh(W_{2j}z_{1j}), $

$ z_{3j}=\tanh(W_{3j}z_{2j}), $

$ \widehat{\bar j}_j=W_{4j}z_{3j}. $

| 参数 | 维数 | 作用 |
| --- | ---: | --- |
| $ W_{1j} $ | $ 32\times4 $ | 第一隐藏层 |
| $ W_{2j} $ | $ 32\times32 $ | 第二隐藏层 |
| $ W_{3j} $ | $ 16\times32 $ | 最后隐藏层 |
| $ W_{4j} $ | $ 3\times16 $ | 最终位置雅可比列 |


**③ 给内部曾增加辅助输出**

四组主干权重分别对应：

$ \underbrace{W_1,W_2}_{\text{估计误差层}},
\underbrace{W_3,W_4}_{\text{跟踪误差层}} $

 对第一、第二隐藏层增加：  

$ \widehat{\bar j}^{(1)}_j=W^o_{1j}z_{1j}\\
\widehat{\bar j}^{(2)}_j=W^o_{2j}z_{2j} $

因此网络具备三个雅可比输出：

$ \widehat{\bar J}^{(1)}_p,\qquad
\widehat{\bar J}^{(2)}_p,\qquad
\widehat{\bar J}_p
 $

 前两个是辅助输出，最后一个才进入控制器

###### 4）离线学习教师雅可比
**① 训练最终输出**

最简单的损失为：

$ \mathcal L_{\mathrm{final}}=\frac1N
\sum_{i=1}^{N}
\left\|
\widehat{\bar J}_p(q_{c,i})
-\bar J_{p,T}(q_{c,i})
\right\|_F^2 $

 确认两列网络都能学准，再增加辅助输出监督：  

$ \mathcal L_{\mathrm{aux}}=\frac1N\sum_{i=1}^{N}
\sum_{l=1}^{2}
\left\|\widehat{\bar J}^{(l)}_p(q_{c,i})
-\bar J_{p,T}(q_{c,i})
\right\|_F^2 $

组合为：

$ \mathcal L
=
\mathcal L_{\mathrm{final}}
+
\lambda_{\mathrm{aux}}\mathcal L_{\mathrm{aux}}
+
\lambda_W\|\Theta\|^2 $

进而让后续位置观测器从一个合理的速度映射开始，而不是完全随机初始化  

**② 检查是否正确预测位置变化**

位置变化为：

$ \Delta p_T=f_T(q_c+\Delta q_c)-f_T(q_c) $

$ \Delta\widehat p=\widehat J_p(q_c)\Delta q_c $

比较

$ E_{\Delta p}=\left\|
\Delta p_T-\Delta\widehat p
\right\| $

 由于雅可比是局部线性映射，即使使用准确教师雅可比，也存在有限步长的线性化误差。因此，应同时计算：

$ E_{\Delta p,T}=\left\|
\Delta p_T-J_{p,T}(q_c)\Delta q_c
\right\| $  

进而分清误差来自网络还是线性化本身  

###### 5）冻结网络，完成位置控制闭环
**① 将被控对象和控制器分开**

 被控对象使用教师正运动学：  

$ p_k=f_T(q_{c,k}) $

 控制器使用神经网络：  

$ \widehat J_{p,k}=f_\Theta(q_{c,k}) $

执行运动后:

$ q_{c,k+1}=q_{c,k}+T_su_k $

$ p_{k+1}=f_T(q_{c,k+1}) $

**② 采用阻尼最小二乘验证**

采用阻尼最小二乘验证，定义

$ e_p=p-p_d $

采用如下调试控制器，同时世家角速度和构型范围限制：

$ u=\left(
\widehat J_p^T\widehat J_p+\mu^2I_2
\right)^{-1}
\widehat J_p^T
\left(
\dot p_d-K_pe_p
\right)
 $

然后设计一个可达轨迹进行验证

###### 6）加入位置观测和分层在线更新
**① 两个内部层分别建立一个位置观测器**

使用归一化位置：

$ \bar p_T=\frac{f_T(q_c)}{L_s}
 $

对内部层$ l=1,2 $定义

$ r_l=\bar p_T-\widehat{\bar p}_l $

观测器为：

$ \dot{\widehat{\bar p}}_l=
\widehat{\bar J}^{(l)}_p(q_c)\dot q_c
+K_{pl}r_l
+K_{sl}\operatorname{sat}(r_l) $

初始化可设为：

$ \widehat{\bar p}_l(0)=\bar p_T(0)
 $

**② 用统一形式实现各层更新**

对于隐藏层$ l=1,2,3 $和分支$ j $，定义

$ \Phi_{lj}=z_{lj}\dot q_{c,j} $

$ D_{lj}=
\dot q_{c,j}
\operatorname{diag}
\left[
1-z_{lj}\odot z_{lj}
\right] $

此处采用$ \tanh'(a)=1-\tanh^2(a) $

| 隐藏层$ l $ | 对应输出权重$ G_{lj} $ | 学习误差$ \varepsilon_l $ |
| --- | --- | --- |
| 1 | $ W^o_{1j} $ | $ r_1=\bar p_T-\widehat{\bar p}_1 $ |
| 2 | $ W^o_{2j} $ | $ r_2=\bar p_T-\widehat{\bar p}_2 $ |
| 3 | $ W_{4j} $ | $ e=\bar p_T-\bar p_d $ |


可统一写为：

$ 
\dot W_{lj}=
\alpha_lD_{lj}^{T}G_{lj}^{T}
\varepsilon_l z_{l-1,j}^{T}
-\alpha_l\beta_l
\|\varepsilon_l\|W_{lj} $

以及

$ \begin{aligned}
\dot G_{lj}={}&
a_l\varepsilon_l
\left(
\Phi_{lj}^{T}
-
z_{l-1,j}^{T}W_{lj}^{T}D_{lj}^{T}
\right)-
a_lb_l\|\varepsilon_l\|G_{lj}
\end{aligned}
 $

 $ a_l,b_l $表示对应输出权重的适应增益和泄漏系数  

当$ l=1,2 $时，它更新内部主干与辅助输出；当$ l=3 $时，它更新最后隐藏层与最终输出层

注意：$ G_{3j} $就是$ W_{4j} $，不是额外增加一个第五层

