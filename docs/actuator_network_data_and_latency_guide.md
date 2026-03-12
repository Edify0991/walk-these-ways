# Actuator Network 训练与延迟辨识最小方案

本文给出这个仓库下可直接落地的最小流程：
1) 先做延迟辨识（得到 constant action delay）
2) 再做 actuator network 数据采集与训练

---

## 1. 延迟辨识最小实验脚本方案

### 1.1 采集什么实验
建议最小化做两段（总计约 3~5 分钟）：

- **段A：站立抬起（无接触冲击）的小幅关节激励**
  - 每条腿依次给关节目标加小幅正弦（例如髋/大腿/小腿），频率 0.5~3 Hz 扫频。
  - 目标：估计指令到响应的时移，降低地面冲击干扰。

- **段B：低速原地踏步/慢走**
  - 目标：验证段A得到的时延在真实闭环工况是否仍成立。

日志请用部署链路保存 `log.pkl`（`hardware_closed_loop`）。

### 1.2 需要字段
延迟辨识脚本使用：
- `torques`（命令力矩/PD理想力矩）
- `joint_pos`
- `joint_pos_target`
- `joint_vel`
- 可选 `tau_est`（推荐，有则更稳）

### 1.3 运行脚本
新增脚本：`scripts/actuator_net/identify_latency.py`

示例：

```bash
python scripts/actuator_net/identify_latency.py \
  --log logs/<exp>/<date>/<time>/log.pkl \
  --dt 0.02 \
  --max-lag-steps 12 \
  --signal torques_to_tau_est
```

如果日志没有 `tau_est`：

```bash
python scripts/actuator_net/identify_latency.py \
  --log logs/<exp>/<date>/<time>/log.pkl \
  --dt 0.02 \
  --max-lag-steps 12 \
  --signal torques_to_pos_err
```

输出会给出每个关节 lag（step）、中位数 lag，以及建议 `lag_timesteps`。

### 1.4 如何写回配置
仿真中 action delay 是通过 `lag_timesteps` 实现的，建议先设置为辨识结果中位数，再做小范围调参（±1 step）：

- `Cfg.domain_rand.randomize_lag_timesteps = True`
- `Cfg.domain_rand.lag_timesteps = <identified_steps>`

---

## 2. actuator network 训练采集方案

### 2.1 训练前做什么实验
推荐三类数据混合采集（总计 10~20 分钟）：

1. **单关节/多关节扫频（低风险）**
   - 覆盖低速、小力矩、换向。
2. **中幅周期运动**
   - 覆盖中速段。
3. **真实任务片段（慢走、转向、启停）**
   - 覆盖实际分布。

### 2.2 必采字段（每时刻）
训练脚本读取并使用：
- `tau_est`（监督标签）
- `joint_pos`
- `joint_pos_target`
- `joint_vel`
- `torques`（只用于画图比对）

### 2.3 如果用 PD 控制器采集，PD 怎么设
目标是“安全 + 激励充分 + 不饱和”：

- 从稳定站立参数起步。
- `Kp`：先中等（能跟踪，不剧烈振荡），再逐步加到略偏硬。
- `Kd`：用于抑制震荡；先低后中，不宜过高（会放大噪声）。
- 检查 `tau_est`/`torques` 是否频繁饱和；若饱和，降低激励幅值或 Kp。
- 先在离地/吊起条件做扫频，再落地做低速步态。

实操上可先沿用当前部署配置中的 PD 参数，先拿到一批“干净可用”数据，再逐步扩展覆盖范围。

---

## 3. 本仓库中数据放哪里、从哪里训练、哪里看结果

### 3.1 数据放哪里
默认训练脚本从：
- `log_dir_root = "../../logs/"`
- `log_dir = "example_experiment/2022/11_01/16_01_50_0"`
拼接并读取 `<log_dir_root><log_dir>/log.pkl`。

因此你只需把真机 `log.pkl` 放到 `logs/<你的实验路径>/`，然后改 `scripts/actuator_net/train.py` 的 `log_dir`。

### 3.2 从哪里启动训练
在仓库根目录运行：

```bash
python scripts/actuator_net/train.py
```

你需要在 `scripts/actuator_net/train.py` 设置：
- `load_pretrained_model = False`
- `actuator_network_path`（例如 `../../resources/actuator_nets/unitree_go1_new.pt`）
- `log_dir_root`、`log_dir`

### 3.3 从哪里看结果
- 终端会打印每个 epoch 的 `loss / test loss / mae`。
- 脚本会弹出 12 关节对比图：
  - idealized torque（`torques`）
  - true torque（`tau_est`）
  - actuator model predicted torque（模型输出）

如果只看已有模型效果，可运行：

```bash
python scripts/actuator_net/eval.py
```

---

## 4. 推荐最小执行清单

1. 用 PD 进行 3~5 分钟延迟辨识采集，跑 `identify_latency.py`，得到 `lag_timesteps`。
2. 把 `lag_timesteps` 写入训练/测试配置。
3. 再采 10~20 分钟 actuator 训练数据（含扫频 + 低速步态）。
4. 运行 `python scripts/actuator_net/train.py` 训练并看曲线。
5. 若预测滞后或幅值偏差明显：
   - 先补数据覆盖（速度区间、换向、负载）
   - 再微调 `lag_timesteps`（±1 step）

