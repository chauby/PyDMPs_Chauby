# NGGD模仿学习算法

## 概述

NGGD（Neural Gaussian Mixture Model with Gaussian Mixture Regression and Dynamic Movement Primitives）模仿学习算法是一个结合了高斯混合模型（GMM）、高斯混合回归（GMR）和改进动态运动基元（DMP）的完整模仿学习框架。

## 算法流程

该算法的完整流程如图1所示：

1. **示教数据获取**: 通过手动示教方式获取多条手写示教轨迹
2. **数据预处理**: 对轨迹数据进行时间对齐和添加高斯噪声等预处理操作
3. **运动轨迹建模**: 使用GMM学习示教轨迹中的特征信息，对轨迹进行建模
4. **运动轨迹回归**: 通过GMR进行轨迹回归，得到概率最优的回归轨迹，实现运动轨迹技能的复现
5. **基于DMP的轨迹泛化**: 将复现的轨迹数据输入到改进的DMP算法中，实现对运动轨迹的泛化

## 文件结构

```
NGGD/
├── README.md                           # 说明文档
├── data_preprocessing.py               # 数据预处理模块
├── gmm_trajectory_modeling.py          # GMM轨迹建模模块
├── gmr_trajectory_regression.py        # GMR轨迹回归模块
├── improved_dmp.py                     # 改进的DMP算法模块
├── nggd_imitation_learning.py          # NGGD主程序
├── demo_nggd_ur5.py                    # UR5机器人演示程序
└── results/                            # 结果保存目录
```

## 安装依赖

```bash
pip install numpy scipy matplotlib scikit-learn pandas
```

## 使用方法

### 1. 基本使用

```python
from nggd_imitation_learning import NGGDImitationLearning

# 创建NGGD系统
nggd_system = NGGDImitationLearning(
    n_gmm_components=10,    # GMM组件数量
    n_dmp_bfs=100,          # DMP基函数数量
    noise_std=0.01          # 噪声标准差
)

# 示教轨迹数据 (n_trajectories, n_timesteps, n_dims)
demonstration_trajectories = [...]  # 你的示教轨迹数据

# 执行学习
learning_results = nggd_system.learn_from_demonstrations(
    demonstration_trajectories,
    alignment_method='linear',  # 对齐方法: 'linear', 'dtw', 'uniform'
    add_noise=True,            # 是否添加噪声
    normalize=True,            # 是否归一化
    plot_results=True          # 是否绘制结果
)

# 轨迹泛化
generalized_trajectory = nggd_system.generalize_trajectory(
    new_initial=[0.5, 0.5, 0.2],  # 新的初始位置
    new_goal=[-0.5, -0.5, 0.8],   # 新的目标位置
    tau=1.0                       # 时间缩放因子
)
```

### 2. UR5机器人演示

```python
from demo_nggd_ur5 import NGGDUR5Demo

# 创建演示系统
demo = NGGDUR5Demo()

# 运行完整演示
demo.run_complete_demo()
```

### 3. 单独使用各个模块

#### 数据预处理

```python
from data_preprocessing import TrajectoryPreprocessor

preprocessor = TrajectoryPreprocessor(target_length=100, noise_std=0.01)

# 时间对齐
aligned_trajectories, timestamps = preprocessor.time_alignment(
    trajectories, method='linear'
)

# 添加噪声
noisy_trajectories = preprocessor.add_gaussian_noise(aligned_trajectories)

# 归一化
normalized_trajectories, norm_params = preprocessor.normalize_trajectories(
    noisy_trajectories, method='minmax'
)
```

#### GMM轨迹建模

```python
from gmm_trajectory_modeling import GMMTrajectoryModeling

gmm_modeler = GMMTrajectoryModeling(n_components=10)
gmm_modeler.fit(trajectories, timestamps)

# 可视化GMM组件
gmm_modeler.visualize_gmm_components()

# 采样轨迹
sampled_traj, sampled_times = gmm_modeler.sample_trajectory()
```

#### GMR轨迹回归

```python
from gmr_trajectory_regression import GMRTrajectoryRegression

gmr_regressor = GMRTrajectoryRegression(gmm_modeler)

# 回归轨迹
query_times = np.linspace(0, 1, 100)
regressed_trajectory = gmr_regressor.regress(query_times)

# 带置信区间的回归
regressed_traj, lower_bound, upper_bound = gmr_regressor.regress_with_confidence(
    query_times, confidence_level=0.95
)
```

#### 改进的DMP

```python
from improved_dmp import ImprovedDMP

improved_dmp = ImprovedDMP(n_dmps=3, n_bfs=100)

# 从GMR轨迹学习
improved_dmp.learning_from_gmr(gmr_trajectory, timestamps)

# 复现轨迹
reproduced_trajectory, velocity, acceleration = improved_dmp.reproduce()

# 泛化轨迹
generalized_trajectory = improved_dmp.generalize(
    new_initial=[0.2, 0.8, 0.1],
    new_goal=[0.8, 0.2, 0.9],
    tau=1.0
)
```

## 参数说明

### NGGDImitationLearning参数

- `n_gmm_components`: GMM组件数量，默认10
- `n_dmp_bfs`: DMP基函数数量，默认100
- `noise_std`: 高斯噪声标准差，默认0.01

### 数据预处理参数

- `target_length`: 目标轨迹长度，默认None（使用最长轨迹长度）
- `noise_std`: 噪声标准差，默认0.01
- `alignment_method`: 对齐方法，可选'linear', 'dtw', 'uniform'

### GMM参数

- `n_components`: GMM组件数量，默认10
- `random_state`: 随机种子，默认42

### DMP参数

- `n_dmps`: 数据维度数，默认1
- `n_bfs`: 基函数数量，默认100
- `dt`: 时间步长，默认0.01
- `alpha_y`: 阻尼参数，默认60
- `beta_y`: 刚度参数，默认alpha_y/4

## 可视化功能

算法提供了丰富的可视化功能：

1. **预处理比较**: 显示原始轨迹和处理后轨迹的对比
2. **GMM组件**: 可视化GMM的高斯组件分布
3. **GMR回归**: 显示回归轨迹和置信区间
4. **DMP比较**: 比较GMR参考轨迹、DMP复现轨迹和泛化轨迹
5. **3D可视化**: 支持3D轨迹的可视化

## 示例结果

运行算法后，会在`results/`目录下生成以下可视化结果：

- `preprocessing_comparison.png`: 预处理比较图
- `gmm_components.png`: GMM组件分布图
- `gmr_regression.png`: GMR回归结果图
- `complete_pipeline.png`: 完整流程比较图

## 注意事项

1. 确保示教轨迹数据格式正确：`(n_trajectories, n_timesteps, n_dims)`
2. 轨迹数据应该包含足够的变化以学习到有效的运动模式
3. 根据具体应用调整GMM组件数量和DMP基函数数量
4. 对于高维轨迹，建议先进行降维处理

## 引用

如果您使用了本算法，请引用相关论文：

```
@article{nggd_imitation_learning,
  title={NGGD: A Novel Imitation Learning Algorithm Combining GMM, GMR and DMP},
  author={Your Name},
  journal={Journal Name},
  year={2024}
}
```

## 许可证

本项目采用MIT许可证，详见LICENSE文件。

## 联系方式

如有问题或建议，请联系：
- 邮箱: your.email@example.com
- GitHub: https://github.com/yourusername/nggd-imitation-learning
