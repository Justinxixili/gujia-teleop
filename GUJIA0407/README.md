# GUJIA0407 —— 适配 Gento SDK（00040701）的骨架摇操工程

本目录是原工程 `GUJIA/SDK_PYTHON` 的**独立副本**，用于适配另一版机器人 SDK：

- SDK 来源：`D:\摇操\骨架摇操\GUJIA\00040701\PYTHON_SDK`
- Python 接口：`GentoRobot.py`（类 `GentoRobot`）
- 动态库：`libGentoSDKPY.dll` / `.so`

> 原工程 `GUJIA/` 未做任何修改。本副本只用于新 SDK 版本。

---

## 1. 目录结构

```
GUJIA0407/
└─ SDK_PYTHON/
   ├─ GentoRobot.py              Gento Python SDK（从 00040701 复制）
   ├─ libGentoSDKPY.dll/.so      Gento SDK 动态库
   ├─ gujiayaocao_0817.py        骨架摇操后台控制（已适配 Gento）
   ├─ gujiayaocao_0817_frontend.py 前端界面（未改）
   ├─ CalibrationOfTheOrigin.py  舵机原点标定（未改，只用串口）
   ├─ KM_CAN.py                  达妙夹爪 CAN（已适配 Gento terminal 接口）
   ├─ servo_config.ini           编码器零点/方向/映射
   ├─ teleop_config.ini          摇操参数（含 PD / 拖动参数）
   ├─ gripper_config.ini         夹爪端点
   ├─ requirements.txt
   └─ packaging/                 PyInstaller 打包脚本（已改 DLL 名）
```

原工程里的 `fx_robot.py`、`fx_kine.py`、`libMarvinSDK*`、`libKine*` **没有**复制过来；
`FD1.9.6` 也未复制（与摇操运行无关，需要时单独拷）。

---

## 2. 接口适配对照

| 原 Marvin SDK | 新 Gento SDK |
|---|---|
| `Marvin_Robot()` | `GentoRobot()` |
| `connect(ip_str)` | `link(a,b,c,d, log_level=...)` + `check_link_state()` |
| `subscribe(dcss)` | `get_rt_dict()` / `get_sg_dict()`，或直接读 `robot.rt` / `robot.sg` |
| `set_tool(arm, kine, dyn)` | `runtime_set_tool_kd(obj, kine[6], dyn[10])` |
| `set_state(3) + set_impedance_type(1)` | `switch_to_pd_mode(obj, timeout, vel, acc, k, d)`（遥操） |
| `set_drag_space(arm, 1)` | `switch_to_drag_joint(obj, timeout, k, d)`（对齐/暂停） |
| `set_vel_est_step(arm, ms)` | `config_set_pd_cmd_cycle_time(ms)` |
| `set_joint_cmd_pose(arm, joints)` | `runtime_set_joint_pos_pd_cmd(obj, joints)`（PD 模式） |
| `clear_error('A')` | `reset_error(obj, timeout)` |
| `log_switch / local_log_switch` | `set_log_level(FXLogMask...)` |
| `release_robot()` | `unlink()` + `cleanup()` |
| `soft_stop('AB')` | `emergency_stop(FXObjMask.OBJ_ALL_FLAG)` |
| `outputs[i]['fb_joint_pos']` | `rt_dict['arms'][i]['fb']['fb_pos']` / `robot.rt.m_ARMS[i].m_ARM_OUT.m_ARM_FBK_Joint_Pos` |
| `outputs[i]['low_speed_flag']` | `robot.sg.m_ARMS[i].m_ARM_GET.m_ARM_FBK_LowSpdFlag`（1=到位） |
| `outputs[i]['frame_serial']` | `rt_dict['frame_serial']` |
| `get_485_data/set_485_data` | `terminal_get(obj, timeout)` / `terminal_set(obj, FXChnType.CHN_CANFD, data)` |

### A/B 与 OBJ_ARM0/1

代码里默认：

- A（左）→ `FXObjType.OBJ_ARM0`
- B（右）→ `FXObjType.OBJ_ARM1`

如果新 SDK/控制器左右定义相反，改 `gujiayaocao_0817.py` 开头的：

```python
ARM_A_OBJ = FXObjType.OBJ_ARM0
ARM_B_OBJ = FXObjType.OBJ_ARM1
```

---

## 3. 控制模式策略

| 阶段 | 旧模式 | 新工程模式 |
|---|---|---|
| 启动 / 人工对齐 | 扭矩 + 关节阻抗 + 拖动 | `switch_to_drag_joint` |
| 遥操 | 扭矩 + 关节阻抗 + 100Hz 位置流 | `switch_to_pd_mode` + `runtime_set_joint_pos_pd_cmd` |
| 暂停 | 关节拖动 | `switch_to_drag_joint` |
| 回 Home | 阻抗 + 位置流 | PD 模式 + 位置流（保持 PD） |
| 退出 | state=0 | `switch_to_idle` + `unlink` + `cleanup` |

PD 模式是 Gento README 明确推荐用于遥操的模式（低延迟、兼具关节阻抗柔顺性），对应旧工程的“阻抗 + 速度前馈”。

`teleop_config.ini` 的 `[impedance]`：

```ini
# Gento PD 模式刚度/阻尼（常用值）
k = 14, 14, 14, 10.5, 5.6, 5.6, 5.6
d = 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3
# 拖动模式（对齐/暂停）
drag_k = 3, 3, 3, 2, 1, 1, 1
drag_d = 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2
```

> PD 模式的刚度有范围限制（最大 20,20,20,15,8,8,8），旧配置的 `9,9,7` 在 5~7 轴超出上限，因此改成了 Gento README 的常用值。

---

## 4. 现场必须确认的项

1. **PD 模式使能条件**：控制器 `robot.ini` 中 `JointPIDCtlType=1`，否则 `switch_to_pd_mode` 会失败。
2. **左右臂映射**：确认 A/B 与 `OBJ_ARM0/1` 的对应关系（见上）。
3. **PD 前馈周期**：代码按 `ctrl_hz=100` 设为 10 ms；若控制器建议不同，改 `teleop_config.ini` 的 `ctrl_hz`。
4. **PD 刚度/阻尼**：按现场手感微调，注意 5~7 轴上限（8/8/8）。
5. **低速到位标志**：`low_speed_flag == 1` 是否表示到位；如果语义不同，改 `_arm_low_speed_flag` 的判断。
6. **夹爪 CAN 通道**：`KM_CAN.py` 用 `FXChnType.CHN_CANFD`（对应旧 SDK 的 `com=1`，CAN 端）。若夹爪实际走 485A/485B，改 `recv()` / `__send_data_can()` 的 `chn_type`。
7. **关节角约定**：新控制器与原控制器的关节零点/方向如果不同，需要重新核对 `servo_config.ini` 的 zero/direction 和 `start_pose_*`。
8. **工具参数**：`teleop_config.ini` 的 `[tool]` 通过 `runtime_set_tool_kd` 下发；确认单位（mm/deg、kg/mm²）。

---

## 5. 运行

### 源码运行

```bat
cd /d D:\摇操\骨架摇操\GUJIA\GUJIA0407\SDK_PYTHON
python gujiayaocao_0817.py
```

或启动前端：

```bat
python gujiayaocao_0817_frontend.py
```

### 打包

```bat
cd /d D:\摇操\骨架摇操\GUJIA\GUJIA0407\SDK_PYTHON
python packaging\build_exe.py
```

产物输出到 `SDK_PYTHON/release/`，动态库已改为 `libGentoSDKPY.dll`。
`GentoRobot.py` 里对 DLL 的查找做了打包兼容（源码目录 / `_MEIPASS` / `_MEIPASS/SDK_PYTHON`）。

### 原点标定

`CalibrationOfTheOrigin.py` 不依赖机器人 SDK，仍可单独运行/打包，用法不变。

---

## 6. 已知差异 / 待验证

- 本工程**未上实机验证**，只做了静态编译和 SDK 离线加载检查；
- `runtime_set_joint_pos_pd_cmd` 的 100Hz 流式下发、`switch_to_drag_joint ↔ switch_to_pd_mode` 切换、`terminal` 夹爪收发都需要现场跑通；
- 新 SDK 自带几何干涉检测（`config_set_interference_enable` / `interf_*`），本工程暂未接入；
- 碰撞检测（关节外力 `fb_ext_torque`）暂未接入，可后续按原计划补充。
