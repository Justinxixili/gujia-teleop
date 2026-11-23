# 骨架摇操（gujia-teleop）

基于马文（Marvin）双臂机器人 SDK 的骨架遥操作（摇操）系统：操作人员穿戴骨架，骨架上的 STS 编码器实时测量 14 个手臂关节（A/B 双臂 × 7）和 2 个夹爪的开合，控制脚本把骨架角度映射为机械臂关节指令，实现"人怎么动、机械臂怎么动"的遥操作。

仓库地址：<https://github.com/Justinxixili/gujia-teleop>

> ⚠️ 遥操会使机械臂真实运动。使用前请确认急停可用、工作区域无人，并先在低速空载状态下测试。

---

## 主要功能

- **图形化控制台**：启动/停止控制脚本，配置机器人 IP、编码器串口、波特率。
- **两步对齐闸门**：第一步机械臂回到起始点，第二步骨架逐关节对齐；全部通过后才允许开启遥操。
- **实时对齐反馈**：14 个关节的起始误差、骨架误差实时显示，20Hz 刷新。
- **遥操**：右踏板开启/停止，骨架角度经一阶低通滤波后下发，支持启动插值过渡和漂移超限自动要求重新对齐。
- **回 Home**：左踏板一键回起始点位（插值 + 到位检测 + 超时保护）。
- **夹爪跟随**：按 `gripper_config.ini` 的张开/闭合端点线性映射，MIT 位置控制。
- **RealSense 三路相机**：头部 D455/D435 + 左右腕 D405，拼接预览、左右腕交换、按帧采集（含 `metadata.json`）。
- **舵机原点标定工具**：GUI 逐关节记录正/负限位，按实际运动路径自动计算中点，支持行程超过 180° 的关节，人工确认后导出。
- **全部参数可配置**：`teleop_config.ini` / `gripper_config.ini` / `servo_config.ini`。
- **便携版 exe**：PyInstaller 打包，复制到其他 Windows 电脑解压即用。

---

## 快速开始

### 方式一：便携版 exe（推荐给现场电脑）

1. 解压 `GujiaTeleop_release.zip` 到任意文件夹；
2. 保持 `GujiaTeleop.exe`、`OriginCalibration.exe`、三个 ini、两个 dll 在同一目录；
3. 双击 `GujiaTeleop.exe` 启动摇操控制台，或 `OriginCalibration.exe` 启动标定工具。

### 方式二：源码运行

```bat
cd GUJIA\SDK_PYTHON
pip install -r requirements.txt
start_gujiayaocao_frontend.bat
```

或直接：

```bat
python gujiayaocao_0817_frontend.py
```

---

## 目录结构

```
GUJIA/                                  ← 本仓库根目录
├─ README.md
├─ .gitignore
├─ .gitattributes
└─ GUJIA/
   ├─ FD1.9.6/                         第三方舵机调试工具（可选）
   └─ SDK_PYTHON/
      ├─ gujiayaocao_0817_frontend.py  入口：Tkinter 图形控制台
      ├─ gujiayaocao_0817.py           控制脚本：对齐 / 遥操 / 回 Home / 夹爪
      ├─ CalibrationOfTheOrigin.py     舵机原点标定工具（GUI，独立运行）
      ├─ fx_robot.py                   机器人控制 SDK 封装
      ├─ fx_kine.py                    运动学计算接口
      ├─ KM_CAN.py                     CAN 电机与夹爪控制
      ├─ footkeyboard.py               脚踏键盘测试脚本（独立运行）
      ├─ teleop_config.ini             摇操参数
      ├─ gripper_config.ini            夹爪张开/闭合端点映射
      ├─ servo_config.ini              编码器零点/方向/ID 映射（标定工具只读）
      ├─ requirements.txt              Python 依赖
      ├─ libMarvinSDK.dll / .so        机器人控制动态库
      ├─ libKine.dll / .so             运动学动态库
      ├─ start_gujiayaocao_frontend.bat
      ├─ camera_dataset/               相机采集输出（不入库）
      ├─ packaging/                    exe 打包脚本与使用说明
      │  ├─ teleop_launcher.py
      │  ├─ build_exe.py
      │  └─ 使用说明.txt
      └─ release/                      打包产物（不入库）
```

---

## 运行环境

| 项目 | 要求 |
|---|---|
| 操作系统 | Windows 10/11 64 位（源码也兼容 Linux，DLL 换 `.so`） |
| Python（源码运行） | 3.13（64 位） |
| 机器人 | 马文 Marvin 双臂机器人，网络可达（默认 `192.168.1.190`） |
| 骨架编码器 | 16 路 STS 舵机编码器，串口 1000000 波特率 |
| 夹爪电机 | 达妙 DM4310 × 2，经机器人 CAN 网关 |
| 相机（可选） | Intel RealSense D455/D435 + D405 × 2，需安装 RealSense 驱动 |

Python 依赖（`requirements.txt`）：

```text
pynput        # 键盘/脚踏监听与按键发送
numpy         # CAN 电机通信数组运算
pyserial      # 编码器串口
Pillow        # 相机画面显示/保存
pyrealsense2  # RealSense 相机（仅相机功能需要）
```

---

## 配置说明

优先级：**环境变量 `GUJIAYAOCAO_*` > 配置文件 > 代码默认值**。前端界面填写的 IP/串口/波特率通过环境变量传给控制脚本，所以界面上的值优先。

### 1. `teleop_config.ini` —— 摇操参数

修改后需重启控制程序（配置只在启动时读取）。

| 段 | 键 | 默认值 | 说明 |
|---|---|---|---|
| `[robot]` | `ip` | `192.168.1.190` | 机器人 IP |
| | `start_pose_a` / `start_pose_b` | 见文件 | 起始/回 Home 点位（7 个关节，度） |
| | `start_tol_deg` | `5.0` | 起始点允许误差 |
| | `skeleton_align_tol_deg` | `10.0` | 骨架对齐允许误差 |
| | `skeleton_realign_tol_deg` | `30.0` | 已对齐后允许的最大漂移 |
| | `vel_ratio` / `acc_ratio` | `100` | 遥操速度/加速度百分比 |
| `[encoder]` | `port` | 空 | 串口；留空按平台自动选择 |
| | `baudrate` | `1000000` | 串口波特率 |
| | `send_frequency` | `100` | 编码器读取频率（Hz） |
| | `filter_alpha` | `0.9` | 一阶低通系数（0~1，越小越平滑） |
| | `resolution` | `4096` | 编码器分辨率 |
| | `enable_checksum` | `true` | 是否校验帧校验和 |
| `[control]` | `loop_period_sec` | `0.03` | 主循环周期（对齐阶段约 33Hz） |
| | `gate_status_interval_sec` | `0.05` | 对齐状态推送间隔（20Hz） |
| | `gate_print_interval_sec` | `2.0` | 控制台文字打印间隔 |
| | `teleop_start_step_deg` | `0.2` | 遥操启动插值每周期最大步长 |
| | `home_step_deg` | `0.2` | 回 Home 插值每周期最大步长 |
| | `home_wait_timeout_sec` | `10.0` | 回 Home 到位等待超时 |
| | `home_interp_extra_sec` | `5.0` | 回 Home 插值超时余量 |
| | `ctrl_hz` | `100` | 速度前馈控制频率 |
| `[gripper]` | `teleop_enable` | `true` | 夹爪是否跟随 |
| | `command_hz` / `recv_hz` | `50` / `10` | 夹爪指令/反馈频率 |
| | `left_motor_id` / `right_motor_id` | `1` / `2` | 夹爪电机 CAN ID |
| | `kp` / `kd` / `tau_ff` | `3.0` / `0.12` / `0.0` | MIT 控制参数 |
| | `qmin` / `qmax` | `0.0` / `1.3` | 夹爪电机机械限位（rad） |
| | `enc_min_deg` / `enc_max_deg` | `0` / `90` | 编码器摆动角度范围 |
| `[impedance]` | `k` / `d` | 见文件 | 关节阻抗 K/D |
| `[tool]` | `a_kine` / `a_dyn` / `b_kine` / `b_dyn` | 见文件 | 工具负载偏移与动力学参数 |

### 2. `servo_config.ini` —— 编码器零点/映射（**只读**）

- 保存每个编码器的 `zero_offset_raw`（零点原始值）、`direction`（方向）和 `[encoder_map]`（ID 与关节的对应关系）；
- **摇操和标定程序都不会修改它**，标定结果需人工核对后合并；
- 默认映射：A 臂 1–7、左夹爪 8、B 臂 9–15、右夹爪 16（接线不同时按实际修改）。

```ini
[encoder_map]
arm_a = 1,2,3,4,5,6,7
left_gripper = 8
arm_b = 9,10,11,12,13,14,15
right_gripper = 16
```

### 3. `gripper_config.ini` —— 夹爪端点映射

夹爪不使用关节中点原点，而是按"完全闭合 / 完全张开"两个编码器原始值做线性映射：

```ini
[right]
closed_raw = 836      # 完全闭合时的编码器原始值
open_raw = 652        # 完全张开时的编码器原始值
target_closed = 1.1   # 闭合对应的 MIT 位置目标（rad）
target_open = -0.15   # 张开对应的 MIT 位置目标（rad）
```

- 留空或两端相同的夹爪退回旧的 `abs(角度)/9` 归一化；
- 方向反了就把 `target_closed` / `target_open` 互换；
- 左夹爪需要按实测填写 `[left]`。

### 环境变量

| 变量 | 说明 | 默认 |
|---|---|---|
| `GUJIAYAOCAO_ROBOT_IP` | 机器人 IP（优先于配置） | 配置文件值 |
| `GUJIAYAOCAO_ENCODER_PORT` | 编码器串口（优先于配置） | 配置文件/自动 |
| `GUJIAYAOCAO_ENCODER_BAUDRATE` | 串口波特率 | 配置文件值 |
| `GUJIAYAOCAO_SKELETON_REALIGN_TOL_DEG` | 重新对齐阈值 | 配置文件值 |
| `GUJIAYAOCAO_TELEOP_START_STEP_DEG` | 遥操启动步长 | 配置文件值 |
| `GUJIAYAOCAO_HOME_STEP_DEG` | 回 Home 步长 | 配置文件值 |
| `GUJIAYAOCAO_GRIPPER_CMD_HZ` | 夹爪指令频率 | 配置文件值 |
| `GUJIAYAOCAO_GRIPPER_RECV_HZ` | 夹爪反馈频率 | 配置文件值 |

---

## 摇操使用流程

### 界面

- **侧边栏**：折叠、设置、深色/浅色模式；
- **顶部**：运行状态、PID、刷新串口、打开目录；
- **准备开始**：启动程序 / 回 Home / 开启遥操 / 退出程序；
- **连接参数**：机器人 IP、编码器串口、波特率；
- **对齐状态**：A/L、B/R 两行共 14 个关节，显示 `起 xx°`（起始点误差）和 `骨 xx°`（骨架误差），绿色=已对齐，红色=未对齐；
- **相机采集**：启动/停止、采集一帧、交换左右腕、保存目录；
- **运行日志**：控制脚本输出和界面操作记录（20Hz 对齐 JSON 不刷屏）。

### 操作步骤

1. **连接设备**：机器人上电、网络连通；编码器串口插好；在前端选择串口和波特率。
2. **启动程序**：点「启动程序」，控制脚本开始运行，机械臂进入 state=3 拖动模式（可手动拖动）。
3. **第一步：手臂对齐**
   - 手动拖动机械臂到起始点位（A/B 各自 7 个关节，允许 ±5°）；
   - 前端第一阶段的误差格子变绿；
   - 点「回 Home」也可以自动插值回到起始点位（回 Home 期间和到位后不可拖动）。
4. **第二步：骨架对齐**
   - 穿戴骨架，逐个关节摆到与机械臂相同的姿态；
   - 每个骨架关节误差进入 ±10° 后会**锁存**为已对齐，不需要一直保持；
   - 前端 14 个格子全部变绿后，日志提示"可以按右踏板开启摇操"。
5. **开启遥操**
   - 踩右踏板（或点「开启遥操」）：先以 0.2°/周期插值到骨架姿态，然后进入跟随；
   - 遥操中机械臂跟随骨架，夹爪跟随夹爪编码器；
   - 已对齐后若某个关节漂移超过 30°，会要求重新对齐超限关节。
6. **停止遥操**：再踩一次右踏板，机械臂恢复可拖动（state=3）。
7. **回 Home**：踩左踏板，自动插值回起始点位并等待到位信号（10 秒超时保护）。
8. **退出**：踩 ESC（或点「退出程序」），机械臂切到 state=0 抱闸后退出。

> 对齐阶段主循环约 33Hz 实时检查，状态 20Hz 推送到前端；**开始遥操后不再 subscribe/检查**，不占用遥操通信。

### 相机采集

- 三路相机自动分配：头部优先 D455/D435，左右腕优先 D405；
- 画面拼接为 2×2（头部跨两列）；
- 点「采集一帧」保存到 `camera_dataset/<时间戳>_<序号>/`：
  - `head.jpg` / `left_wrist.jpg` / `right_wrist.jpg`
  - `mosaic.jpg`（拼接图）
  - `metadata.json`（相机序列号、分辨率、时间等）
- 左右腕装反时点「交换左右腕」。

---

## 舵机原点标定

```bat
python CalibrationOfTheOrigin.py
```

或双击便携版 `OriginCalibration.exe`。

### 流程

1. 选择编码器串口并连接（只读读取 `servo_config.ini` 的 `[encoder_map]` 用于关节命名）；
2. 把某个关节转到**机械正限位**，点该行「记正限位」；
3. 转到**机械负限位**，点「记负限位」；
4. 程序按**实际运动路径的中点**计算原点，状态变为「待确认」；
5. 人工核对后点「确认」，该行锁定（可「取消确认」重测）；
6. 全部确认后「导出结果...」，生成独立的 ini 文件。

### 界面规则

- 记录任一限位后关节名称变**蓝**，正负限位都记录后变**绿**；
- 每个记录按钮再点一次 = 清空该限位，可反复重测；
- 记录完成后，正/负限位以**相对标定原点的角度**显示（如 `+164.1° / −164.2°`），便于和额定关节范围对照；
- 记录前有稳定性检查：最近约 0.24 秒读数波动超过 0.7° 会拒绝记录，提示"等关节停稳"。

### 计算原理

- 连续跟踪相邻帧编码器增量并累计展开位置，按实际运动路径取中点；
- 因此 **行程超过 180° 的关节不会算成补角**，恰好 180° 的关节也没有二选一歧义；
- 记录时请让关节从一个硬限位**连续**移动到另一个硬限位，中途不要绕圈。

### 注意事项

- **`servo_config.ini` 只读**：标定程序不会修改它；导出时如果选择了它也会被拒绝；
- 导出的 `zero_offset_raw` 需要人工核对后合并进 `servo_config.ini`；
- 夹爪不需要中点原点，见下文"夹爪端点标定"。

---

## 夹爪端点标定与映射

夹爪是一个电机，闭合/张开是电机的正/反向旋转。摇操按端点线性映射：

1. 用标定工具观察夹爪编码器（ID8 / ID16）；
2. 记录**完全闭合**和**完全张开**时的原始值；
3. 填入 `gripper_config.ini`：

```ini
[right]
closed_raw = 836
open_raw = 652
target_closed = 1.1
target_open = -0.15
```

4. 如果张开/闭合方向反了，互换 `target_closed` / `target_open`；
5. 没有配置的夹爪退回旧的 `abs(角度)/9` 归一化。

端点映射与编码器零点无关：无论 `servo_config.ini` 里夹爪的 `zero_offset_raw` 是中点还是端点，结果一致。

---

## 内部实现

### 进程与数据流

```
gujiayaocao_0817_frontend.py（GUI）
   │  启动子进程，传入环境变量（IP/串口/波特率）
   ▼
gujiayaocao_0817.py（控制）
   ├─ 主线程：脚踏事件、对齐检查、状态推送、回 Home、遥操启停
   ├─ 编码器线程：100Hz 读串口 → 解析 → 低通滤波 → （遥操时）下发机械臂/夹爪
   └─ pynput 监听：s=左踏板，d=右踏板，ESC=退出
```

打包后前端通过 `GujiaTeleop.exe --run-control` 启动同一个 exe 的控制模式。

### 对齐状态协议

控制脚本每 20Hz 输出一行：

```text
__ALIGN_STATUS__{"start_tol":5.0,"skeleton_tol":10.0,...,"stage":"arm|skeleton|ready","A":{...},"B":{...}}
```

前端解析后刷新 14 个关节格子，`__ALIGN_STATUS__` 行不写入日志。

### 编码器帧

- 读取命令一次读取 16 路 STS 编码器，每帧 8 字节：`FF FF ID LEN ERR POS_L POS_H CHECKSUM`；
- 控制脚本按帧头 + 校验和**重同步**，容忍噪声、粘连和残缺帧；
- 角度按 `servo_config.ini` 的 `[encoder_map]` 按 ID 映射，不依赖帧内顺序。

### 时序

| 环节 | 频率/周期 |
|---|---|
| 编码器读取 | 100Hz |
| 对齐检查（仅对齐阶段） | 约 33Hz（`loop_period_sec`） |
| 对齐状态推送 | 20Hz（`gate_status_interval_sec`） |
| 控制台打印 | 2 秒 |
| 遥操中主循环 | 约 33Hz，只轮询踏板，不 subscribe |
| 夹爪指令 / 反馈 | 50Hz / 10Hz |

---

## 打包成便携版 exe

在 `SDK_PYTHON` 下执行：

```bat
python packaging/build_exe.py
```

生成 `release/`：

- `GujiaTeleop.exe` —— 摇操控制台（前端 + 控制脚本同一个 exe）
- `OriginCalibration.exe` —— 舵机原点标定
- `servo_config.ini` / `gripper_config.ini` / `teleop_config.ini`
- `libMarvinSDK.dll` / `libKine.dll`
- `使用说明.txt`

同时生成 `release/GujiaTeleop_release.zip`（在 release 文件夹内）。复制到其他 Windows 10/11 64 位电脑解压即可使用。

说明：

- exe 是 PyInstaller **onefile**，首次启动需要解压运行库，会慢几秒；
- 打包产物（`release/`、zip）不入库；
- 打包后配置和相机输出目录都在 **exe 所在文件夹**；
- 前端停止控制进程时使用 `taskkill /T` 结束整个进程树，避免 onefile 引导进程残留。

---

## 常见问题排查

| 现象 | 可能原因 / 处理 |
|---|---|
| 点「启动程序」提示找不到控制脚本 | 源码模式下脚本不在同目录；确认 `gujiayaocao_0817.py` 与前端同级 |
| 串口列表为空 | 未安装 pyserial、USB 驱动未装、串口被占用 |
| 打开串口失败 | 串口号错误、被其它程序占用、波特率不匹配 |
| 启动后立即退出 | `servo_config.ini` 缺失；机器人 IP 不可达；SDK DLL 缺失或位数不匹配 |
| 机器人连接失败 | 检查 IP、网线/网络、防火墙；控制脚本会打印连接信息后退出 |
| 对齐格子一直红 | 机械臂未到起始点（±5°）或骨架未进入 ±10°；看日志和 `起/骨` 数值 |
| 骨架编码器未初始化 | 串口未连、波特率错、接线/ID 映射不对；用标定工具看实时原始值 |
| 遥操拒绝 | 对齐未完成；按日志提示先完成第一步/第二步 |
| 遥操中关节漂移超限 | 漂移超过 30° 会要求重新对齐超限关节，重新摆好骨架即可 |
| 夹爪反向/行程不对 | 检查 `gripper_config.ini` 端点；方向反了互换 `target_closed/target_open` |
| 相机启动失败 | 未装 Pillow 或 pyrealsense2；RealSense 未连接/驱动未装/USB 带宽不足 |
| 相机画面黑屏 | 相机被其它程序占用（如 RealSense Viewer），关闭后重试 |
| 标定无法导出 | 没有已确认的关节；导出只包含已确认关节 |
| exe 被 SmartScreen 拦截 | 更多信息 → 仍要运行；或对 exe 解除锁定 |
| 提示缺少 VC++ 运行库 | 安装 Microsoft Visual C++ 2015–2022 Redistributable (x64) |
| 关闭程序后进程仍在 | 使用界面「退出程序」触发 ESC；打包版会自动 `taskkill /T` |

---

## 安全注意事项

1. 操作前确认**急停开关**可用，工作区域内无人；
2. 首次使用先空载、低速（降低 `vel_ratio/acc_ratio`）验证方向；
3. 对齐阶段机械臂处于拖动模式，注意不要让机械臂自由下落或碰撞；
4. 回 Home、开启遥操都会产生整臂运动，操作人员应站稳并留出运动空间；
5. 修改 `servo_config.ini` / `teleop_config.ini` 后先重启程序，再重新对齐测试；
6. 不要在有负载或人员靠近时调试限位、阻抗、速度参数。

---

## 版本与仓库说明

- 仓库为公开仓库，`libMarvinSDK` / `libKine` 等厂商动态库随仓库分发，请确认厂商许可后再对外分发；
- `camera_dataset/`、`release/`、`*.zip`、`packaging/_build/` 不纳入版本管理；
- 主要更新：
  - 对齐检查改为实时（33Hz 检查 / 20Hz 刷新），遥操期间停止检查；
  - 摇操参数集中到 `teleop_config.ini`；
  - 编码器按 ID 映射，帧解析支持噪声重同步；
  - 舵机标定改为按实际运动路径计算，支持 >180°/180° 行程；
  - 夹爪改为端点线性映射，修复只用到约 9° 量程的问题；
  - 新增便携版 exe 打包。
