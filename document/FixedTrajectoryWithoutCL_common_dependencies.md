# `Reference/Visual_IBVS_Baseline.m` 对 `matlab/common` 的依赖清单

## 说明

本清单依据 `Reference/Visual_IBVS_Baseline.m` 及其在 `matlab/common` 中的函数调用关系整理。这里的“调用”包括：

1. `Visual_IBVS_Baseline.m` 中直接调用的文件；
2. 这些直接调用的公共函数内部继续调用的文件。

因此，列表同时标出直接调用和间接调用。`cfg.output.plot` 为真且有有效样本时才调用绘图函数；其余控制链函数在对应流程执行时调用。

## 一、实际调用的文件

### 1. `Visual_IBVS_Baseline.m` 直接调用

| 文件 | 调用位置/关系 | 作用 |
|---|---|---|
| [`defaultConfig.m`](../matlab/common/defaultConfig.m) | 初始化阶段直接调用 | 建立默认配置 `cfg`，包括仿真步长、机器人与连续体几何参数、相机参数、轨迹、RCM 约束、控制器、神经求解器、Remote API 和输出设置。入口随后覆盖步长、保存路径，并合并用户配置。 |
| [`resolveModelGeometry.m`](../matlab/common/resolveModelGeometry.m) | 配置合并后直接调用 | 校验并分离控制器模型长度 `cfg.model.flexLen` 与场景几何长度 `cfg.geometry.flexLen`，同时检查 9 个盘片长度的有效性和递增关系。 |
| [`referenceTrajectory.m`](../matlab/common/referenceTrajectory.m) | 连接场景前验证一次；跟踪循环中每步调用 | 根据当前时间生成期望图像点 `desiredPixels` 和期望图像速度 `desiredPixelVelocity`。轨迹是以 `cfg.trajectory.center` 为中心的四叶形像素轨迹。 |
| [`robotKinM.m`](../matlab/common/robotKinM.m) | 初始化和跟踪循环中直接调用 | 按 Panda 的 Modified DH 参数计算从基座到轴端/连续体起点的齐次变换矩阵；输入 `d8` 后可得到包含刚性轴段长度的变换。 |
| [`softH.m`](../matlab/common/softH.m) | 跟踪循环中直接调用 | 根据连续体姿态 `[φ; θ]` 和长度计算常曲率连续体齐次变换，用于计算相机位姿 `H09`。`θ=0` 时使用极限形式。 |
| [`Jtool.m`](../matlab/common/Jtool.m) | 跟踪循环中直接调用 | 计算连续体末端/相机相关的 6×2 几何雅可比 `Jinst`，对应连续体变量 `φ`、`θ`。 |
| [`buildSceneState.m`](../matlab/common/buildSceneState.m) | 跟踪循环中直接调用 | 使用场景几何长度生成发送给 CoppeliaSim 的状态数据：7 个关节量、9 个盘片变换和 1 个相机变换，共 127 个数值；该函数使用 `cfg.geometry.flexLen`，不使用控制器模型长度。 |
| [`Jimage.m`](../matlab/common/Jimage.m) | 获得视觉点后直接调用 | 根据图像点、主点、焦距、像素尺寸和深度 `Z` 计算点特征的 2×6 图像雅可比/交互矩阵。 |
| [`JrobotM.m`](../matlab/common/JrobotM.m) | 跟踪循环中直接调用 | 计算 Panda 轴端/轴末端的空间雅可比；入口使用 `J7` 构造 RCM 雅可比，使用 `Jr` 的平移部分构造视觉系统雅可比。 |
| [`hat.m`](../matlab/common/hat.m) | 构造 `Sp` 时直接调用 | 将 3×1 向量转换为 3×3 反对称叉乘矩阵，用于把相机与轴端之间的位移项纳入空间雅可比变换。 |
| [`neuralVelocitySolver.m`](../matlab/common/neuralVelocitySolver.m) | 每个跟踪步直接调用 | 求解带任务等式、关节位置限位和速度限位的广义速度 `dq`。同时维护乘子状态 `mu0`、执行投影迭代、施加 `φ` 居中参考速度，并返回残差、迭代次数和投影信息。这里的“neural”指求解器结构；本入口已将学习参数更新关闭。 |
| [`plotFixedTrajectory.m`](../matlab/common/plotFixedTrajectory.m) | 结果保存后按配置条件调用 | 绘制 4 个结果窗口：像素轨迹、像素误差、9 个关节/连续体角度和 RCM 位置误差。`saveFile` 参数仅为兼容接口，函数本身不导出图像文件。 |

### 2. 由上述函数间接调用

| 文件 | 间接调用关系 | 作用 |
|---|---|---|
| [`gFun.m`](../matlab/common/gFun.m) | `neuralVelocitySolver.m` 内部调用 | 对求解器的对偶变量/中间量执行上下界截断，把结果限制在 `zL` 与 `zU` 给出的范围内。 |
| [`actFun.m`](../matlab/common/actFun.m) | `neuralVelocitySolver.m` 内部调用 | 对投影误差执行非线性激活；具体形式由 `cfg.neural.caseFun` 选择，结果参与乘子迭代更新。 |


## 二、汇总

`matlab/common` 目录当前共有 14 个 `.m` 文件：

- 实际使用：14 个，其中 12 个由 `Visual_IBVS_Baseline.m` 直接调用，2 个由求解器间接调用；

`Visual_IBVS_Baseline.m` 不使用在线视觉雅可比学习配置；当前目录中的公共函数链不包含在线视觉雅可比学习更新。求解器使用的 `cfg.neural.gamma` 属于求解器迭代参数，不是 `piHat` 的学习率。
