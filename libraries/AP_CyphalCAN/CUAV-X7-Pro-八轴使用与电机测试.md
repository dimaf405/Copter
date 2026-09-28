# CUAV X7 Pro：CyphalCAN 八轴 Copter 烧录与电机测试指南

本文适用于本工程的 CUAV-X7 Copter 自定义固件，以及符合《CyphalCAN协议-V1.0.2-2024》的八台电调。这里的“八轴”指 **八根独立机臂、每根机臂一台电机**。如果实物是四根机臂、每根机臂上下同轴两台电机，请选 OctaQuad，不能照搬本文的 Octa X 电机顺序。

本工程已有构建文件 `build/CUAV-X7/bin/arducopter.apj`，其固件包标识为 `CUAV-X7`、板卡 ID 为 `1010`。以下是**待在实物上执行的步骤**；编译通过或仿真通过均不代表电调、线束和失联行为已经通过台架验证。

## 操作顺序速览

1. 拆下全部螺旋桨，备份现有参数，使用 Mission Planner 的 **Load custom firmware** 刷入上述 `.apj`。
2. 八根独立机臂的 X 布局设 `FRAME_CLASS=3`、`FRAME_TYPE=1` 并重启；四根同轴机臂的 X8 应设 `FRAME_CLASS=4`，不能使用本文的 Octa X 电机位置表。
3. 设置 `SERVO_32_ENABLE=1` 并重启，将虚拟 `SERVO17`～`SERVO24` 分别设为 `Motor1`～`Motor8`（功能 `33`～`40`），清除实体 PWM 通道中重复的电机功能，再重启。
4. 将八台电调逐一设为节点 `0x10`～`0x17`，计划接在专用 CAN2 总线；首次测试只接 `0x10` 一台。设置 `CAN_P2_DRIVER=2`、`CAN_D2_PROTOCOL=15`、`CAN_P2_BITRATE=500000`，按第 4 节的阶段要求重启。
5. 先以 `CAN_D2_CY_ESC_BM=1` 仅测试 Motor1，**此阶段禁止常规解锁**；确认后改为 `255` 并重启。在无桨状态用 Motor Test 依次检查 A–H 的实际位置、转向和反馈。
6. 最后完成常规预解锁、停机、急停和 CAN 失联台架验证。Motor Test 通过不代表具备飞行条件。

## 1. 烧录前准备

1. **卸下八副螺旋桨**，记录每台电调和电机的实际安装位置。准备能独立切断电调主电源的开关或插头；调参和烧录时先断开电调主电源。
2. 在地面站导出并保存飞控现有的完整参数，再记录当前遥控器、传感器、电源监测和安全开关配置。升级后按需恢复这些配置，不要把其他机型的整套参数直接导入。
3. 核对飞控确为 CUAV X7 Pro，并已有兼容 ArduPilot 的引导程序。在 Mission Planner 的 **SETUP → Install Firmware → Load custom firmware** 中选择 `build/CUAV-X7/bin/arducopter.apj`。该入口安装的是本地自定义固件；页面上的在线 Copter 固件按钮会安装对应的在线版本。
4. 等待烧录、校验及重启完成。重新连接后确认车辆类型为 Copter、姿态显示随飞控转动，且完整参数表中允许将 `CAN_D2_PROTOCOL` 设为 `15`。`15` 是本工程新增的 CyphalCAN 驱动编号，普通上游固件没有这个驱动。

若地面站找不到 **Load custom firmware**，在 Mission Planner 的 **Config → Planner** 将布局切换为 **Advanced**。上述 `.apj` 方法以已有兼容引导程序为前提；不要把 `arducopter_with_bl.hex` 当作普通 `.apj` 上传。

## 2. 切换到八轴八电机

机架类型由参数控制，**不需要再编译一份“八轴固件”**。在 Mission Planner 完整参数表中设置：

| 实际机架 | `FRAME_CLASS` | `FRAME_TYPE` | 说明 |
| --- | ---: | ---: | --- |
| 八根独立机臂，X 布局 | `3` | `1` | 本文后续电机顺序采用这一行 |
| 八根独立机臂，+ 布局 | `3` | `0` | 必须改用官方 Octa + 的位置图核对 |
| 四根机臂，上下同轴八电机 | `4` | 按实际布局选择 | 属于 OctaQuad；不能使用下方 Octa X 的 A–H 位置表 |

设置后重启，在地面站启动消息中确认机架为 `OCTA/X`（选择 X 时）。只有实际机臂方位与所选布局一致，后续的控制分配和转向表才成立。

### 电机输出功能与物理 PWM 引脚

CyphalCAN 驱动读取 ArduPilot 的 `Motor1`～`Motor8` 输出功能，功能值依次为 `33`～`40`。这些功能必须各有一个通道分配，否则常规预解锁会报告 `motor N function missing`。本机 CAN-only 安装可使用 X7 Pro 没有实体 PWM 引脚的 `SERVO17`～`SERVO24` 作为虚拟通道，使实体 PWM1～PWM8 不同时承载电机输出：

1. 设置 `SERVO_32_ENABLE=1`，重启。确认完整参数表已出现 `SERVO17_FUNCTION`～`SERVO24_FUNCTION`。如果没有出现，暂停此步骤并核对固件及板卡，不要清除现有电机功能。
2. 依次设置 `SERVO17_FUNCTION=33`、`SERVO18_FUNCTION=34`、…、`SERVO24_FUNCTION=40`；保存并重新读取，确认八项均存在。
3. 检查全部 `SERVOx_FUNCTION`。将原来占用实体 PWM 通道的 `Motor1`～`Motor8` 功能清除，例如默认的 `SERVO1_FUNCTION`～`SERVO8_FUNCTION` 设为 `0`。不要清除刚设置的 `SERVO17`～`SERVO24`，也不要改动承载其他设备的通道。
4. 重启后再次核对：功能 `33`～`40` 各仅分配给对应的 `SERVO17`～`SERVO24`，实体 PWM 通道没有重复的电机功能。电调的 PWM 信号线也应与飞控 PWM 输出**物理断开**。

虚拟通道仅向飞控内部的电机功能提供输出值；X7 Pro 板级定义只有 14 路实体 PWM。`SERVO_32_ENABLE` 和 `SERVO17`～`SERVO24` 参数是否可见，以刷入后的实际参数表为准。若采用其他通道映射，仍需满足每个 Motor 功能恰好对应一台电调，并重新核对下文节点表。

## 3. CAN2 接线与电调节点

建议把这八台电调接在独立的 **CAN2** 总线上，CAN1 继续用于已有的 DroneCAN 电源模块等设备。当前 CUAV-X7 板级定义将 CAN2 与 USB 高速模式列为复用关系，且当前定义选择普通 USB FS 模式；使用 CAN2 不意味着普通 USB 连接必须停用。接插件针脚、电源线和线序仍须按 [CUAV X7 Pro 官方接线资料](https://doc.cuav.net/controller/x7-plus-pro/quick-start-x7-pro.html)及电调实物标识核对，不要仅凭线色判断。

- CAN_H 与 CAN_L 使用双绞线；总线两端各需一个 120 Ω 终端。先查飞控、电调及分线板是否自带终端，避免重复安装。断电测量 CAN_H 与 CAN_L，两个 120 Ω 终端并联时通常约为 60 Ω。确认电调另有符合其规格的主电源。
- 飞控与电调均使用 **500 kbit/s**。本驱动是 Cyphal/CAN v1 的这套定制电调消息，不与 DroneCAN v0 设备共用一条物理总线；Mission Planner 的 DroneCAN/UAVCAN 配置窗口也不能给这些电调分配节点 ID。
- 按下文的只读日志法或用独立 CAN 适配器**逐台**确认现有节点 ID，再标记电调对应的机臂；需要改号时才使用厂家工具。默认起始 ID `0x10`（十进制 `16`），Motor1～Motor8 需要连续使用 `0x10`～`0x17`。若厂家实际 ID 是另一组连续值，可用 `CAN_D2_CY_ESC_ID` 改起始值，但本驱动只接受起始 ID **十进制 16～40**（`0x10`～`0x28`），后七台依次递增；超出范围或 ID 不连续时，先重新编号。飞控源节点 `CAN_D2_CY_NODE` 默认是 `10`，不应与电调节点重复。本驱动不实现协议中的节点 ID 设置命令。不要在八台电调同时接入时使用厂家的广播改号命令。
- 上电后先用分析仪确认每台电调返回 6160、6161 和 7509，且未报告故障、油门为零、控制源为 CAN。若电调尚未启用 CAN 油门模式，应先按厂家工具完成配置。

### 只读查看电调现有的节点 ID

协议中的 **CAN-ID** 是每帧的 29 位报文标识；电调可配置的地址叫 **Node-ID**。7509 心跳以及 6160、6161 反馈帧的 CAN-ID 都包含发送方 Node-ID。心跳在上电后约每秒主动发送一次，因此通常无需发送查询命令。协议的 6145 是**设置新地址**，不能用来读取原地址。

1. 保持无桨、飞控未解锁。若 CAN2 尚未启用，先按第 4 节阶段 A 设置 `CAN_P2_DRIVER=2`、`CAN_D2_PROTOCOL=15` 并重启。再设置 `CAN_P2_BITRATE=500000`、`CAN_D2_CY_ESC_BM=0`、`CAN_P2_OPTIONS=1`、`LOG_DISARMED=1`，保存并重启。掩码为 `0` 时本驱动不发送油门帧，仍可接收电调的主动上报。
2. **每次只给一台电调上电**，等待至少 2 秒，下载飞控 DataFlash `.BIN` 日志。查看 `CANF` 记录中 `Bus=1`（CAN2）的扩展、非服务帧，选出 Subject-ID 为 `7509`、`6160` 或 `6161` 的报文。可用 Mission Planner 日志浏览器，或在仓库根目录执行 `python3 modules/mavlink/pymavlink/tools/mavlogdump.py --types CANF --format csv 日志文件.BIN` 导出原始记录。
3. 对选中的 `CANF.Id` 计算：

   ```text
   raw_can_id = CANF.Id & 0x1FFFFFFF
   subject_id = (raw_can_id >> 8) & 0x1FFF
   node_id    = raw_can_id & 0x7F
   ```

   例如 `CANF.Id=0x907D5510` 是节点 `0x10` 发出的 7509 心跳。逐台记录节点 ID 与机臂位置；全部检查完毕后，将 `CAN_P2_OPTIONS` 和 `LOG_DISARMED` 恢复到测试前的值。

若心跳及反馈均未出现，不能据此断定节点不存在；应先检查电源、500 kbit/s、总线接线和日志设置。协议 6144 的 `command=10` 可触发指定或全部电调发送一次心跳，但当前驱动没有发送此指令的功能，需使用支持该协议的 CAN 工具。两台电调使用相同 Node-ID 时，仅凭在线报文无法可靠区分；逐台供电才能确认。协议 430 `GetInfo` 可在**已知目标 Node-ID** 后读取设备唯一码，它不是未知地址的直接查询命令。

## 4. CAN 参数：按阶段设置并重启

建议按下面三个阶段操作，避免尚未确认节点位置时同时驱动八台电机。参数可在 Mission Planner 的完整参数表中逐项输入。

**阶段 A：启用专用 CAN2 驱动。**

```text
CAN_P2_DRIVER     2
CAN_D2_PROTOCOL   15
FRAME_CLASS       3
FRAME_TYPE        1        # 仅适用于八根独立机臂的 X 布局
SERVO_32_ENABLE   1
```

保存并重启。`CAN_P2_DRIVER=2` 表示物理 CAN2 使用第二个软件驱动；`CAN_D2_PROTOCOL=15` 表示该驱动运行本工程的 CyphalCAN 协议。重启后确认 `CAN_D2_CY_*` 参数组已出现，并完成第 2 节的虚拟电机功能映射。

**阶段 B：先仅启用 Motor1，且只接入已确认节点为 `0x10`、安装在前右近机头位置的一台电调。**

```text
CAN_P2_BITRATE    500000
CAN_D2_CY_NODE    10
CAN_D2_CY_ESC_ID  16
CAN_D2_CY_ESC_RT  200
CAN_D2_CY_ESC_BM  1
CAN_D2_CY_POLES   0
```

保存并重启。`CAN_D2_CY_NODE` 是飞控发送帧的源节点 ID；`ESC_ID` 是 Motor1 的电调节点 ID；`ESC_BM` 是 Motor1～Motor8 的位掩码，`1` 只选 Motor1。`POLES=0` 表示暂不把电调电频换算成机械转速；如需 RPM，填写实物电机的**磁极对数**，不能把磁极总数直接填入。此时只执行第 5 节中的 **A 单台无桨测试，禁止常规解锁**。驱动只检查掩码选中的电调，因此一台健康电调可能使其自身的预解锁检查通过；`ESC_BM=1` 不是禁止整机解锁的开关。

**阶段 C：单台无桨测试通过，且其余七台的节点 ID 与安装位置均已核对后，接入全部八台。** 先断开电调主电源，再连接其余七台的 CAN 线和电源线，并复查总线终端与线序；参数保存、飞控重启完成后，才给电调上电。

```text
CAN_D2_CY_ESC_BM  255
```

保存并重启，随后执行第 5 节全部 A–H 测试，逐一确认其余七台的实际转向。位掩码按 Motor1～Motor8 分别对应 `1、2、4、8、16、32、64、128`，`255` 即全选。`NODE`、`ESC_ID`、`ESC_BM` 改动在运行中**不会立即生效**；它们不是急停手段，修改后必须重启。启动时 `ESC_BM=0` 时驱动不发送油门帧。

## 5. 无桨逐台电机测试

整个测试阶段都保持**螺旋桨拆除**、机体固定、人员远离电机，并准备独立切断电调主电源。释放飞控安全开关前再确认一次。Mission Planner 路径为 **SETUP → Optional Hardware → Motor Test**。先尝试约 `5%` 油门、`1 s` 持续时间；若电机不转，在确认其他设置无误后再小幅增加，例如到 `10%`。一次只测试一个字母，**等待该次测试完全停止后再测试下一个字母**。观察到异常立即停止并断开电调电源。

下表仅适用于 `FRAME_CLASS=3`、`FRAME_TYPE=1` 的标准 **Octa X**。位置按俯视、机头向前；转向也按俯视判断。测试字母按机体外圈顺时针排列，但**不按 Motor 编号顺序排列**。

| Motor Test | 电机功能 | 默认电调节点 | 预期机臂位置 | 预期转向 |
| --- | --- | --- | --- | --- |
| A | Motor1 (`33`) | `0x10` | 前右，靠近机头 | CW |
| B | Motor3 (`35`) | `0x12` | 右前 | CCW |
| C | Motor8 (`40`) | `0x17` | 右后 | CW |
| D | Motor4 (`36`) | `0x13` | 后右 | CCW |
| E | Motor2 (`34`) | `0x11` | 后左 | CW |
| F | Motor6 (`38`) | `0x15` | 左后 | CCW |
| G | Motor7 (`39`) | `0x16` | 左前 | CW |
| H | Motor5 (`37`) | `0x14` | 前左，靠近机头 | CCW |

请在实物记录中为每一行填写“实际机臂位置、实际转向、实际节点 ID、是否只有一台转动、反馈电压/电流/温度”。A 只能驱动前右近机头的 Motor1；依次测试到 H。若位置不符，先查电调节点 ID 与安装位置，不要只靠改变 `SERVOx_FUNCTION` 来掩盖节点接错。若转向不符，按电调厂家方式改变电机方向，或在完全断电后对调该电机的任意两根相线，再重复测试。

**Motor Test 能转不等于可以正常解锁。** Copter 的 Motor Test 不执行本驱动的完整预解锁遥测检查；即使某台电调反馈丢失，测试命令仍可能让其他电机转动。必须继续完成下节检查。

Motor Test 运行期间，Copter 会临时停用油门及地面站失联保护，并将 EKF 故障动作改为只报告。测试异常时不要依赖这些常规保护停机；使用已准备好的独立电调主电源断开手段。

## 6. 常规使用前的台架验收

1. 保持无桨、固定机体，确认 `CAN_D2_CY_ESC_BM=255` 已保存并重启、八台电调节点唯一、总线速率一致、A–H 的位置和转向全部正确；检查每台电调的供电及反馈状态。
2. 进行**常规预解锁**检查，保持 `ARMING_CHECK` 正常启用，不要用 `ARMING_CHECK=0` 绕过。被 `ESC_BM` 选中的每台电调须持续提供新鲜的 6160（不超过 250 ms）、6161（不超过 500 ms）、7509（不超过 3 s）反馈；还须报告 CAN 控制源、零油门、正常心跳、无故障且未运行。
3. 无桨验证常规解锁、低油门与上锁；检查上锁、安全开关、急停和输出更新停止时飞控发送零油门。本驱动在未解锁、安全开关未释放、急停或电机输出超过 200 ms 未更新时发送零油门，但 CAN 线断开后零帧无法送达电调。**必须在台架上实测电调自身的 CAN 命令丢失保护及停转延迟**，不能把飞控的 200 ms 逻辑当作电调断线保护。
4. 可临时设置 `CAN_P2_OPTIONS=1` 记录总线 `CANF` 日志，核对 6152/6153 油门帧及 6160/6161/7509 反馈；测试后可关闭全帧日志以减少日志量。若记录到 `ESC` 日志，其 `Instance` 按内部索引 `0`～`7` 对应 Motor1～Motor8。`POLES=0` 时无机械 RPM 更新，不能把“没有 RPM 数值”直接判定为没有 6160 反馈。
5. 检查遥控器、飞控姿态、罗盘、惯导、电池监测、失控保护及 Copter 常规飞前项目。上述 CAN 测试只能证明台架上的接口行为；首飞和调参仍需按整机质量、桨、电机、电池及安全措施单独评估。

本驱动目前只处理油门 6152/6153、反馈 6160/6161 和心跳 7509；不会通过 CAN 设置电调 ID、更新固件或配置电调寄存器。**飞行中反馈丢失不会自动触发独立的整机故障保护动作**。在确认电调自身的命令超时保护、总线负载、断线停转和整机失效处置之前，不应把无桨 Motor Test 结果当作飞行放行依据。

## 7. 常见问题

| 现象 | 优先检查 |
| --- | --- |
| `CAN_D2_PROTOCOL=15` 不可选，或没有 `CAN_D2_CY_*` | 是否刷入本工程 `.apj`；`CAN_P2_DRIVER=2` 和协议号是否保存并重启。 |
| 没有油门 6152/6153 帧 | `CAN_D2_CY_ESC_BM` 是否仍为 `0`；CAN2 驱动、500 kbit/s、供电及线束是否正确。 |
| `ESC N telemetry missing` | 对应节点 ID 是否唯一且连续；6160、6161、7509 是否都在发送；检查 CAN_H/CAN_L、终端和速率。 |
| `ESC N not ready` | 检查电调故障位、是否选用 CAN 油门源、心跳状态、是否报告非零输出油门或仍在运行。 |
| `motor N function missing` | 核对 `SERVO_32_ENABLE=1`、`SERVO17_FUNCTION`～`SERVO24_FUNCTION=33`～`40`，并确认没有被清除或覆盖。 |
| Motor Test 不转 | 确认安全开关、急停状态、电调主电源、节点 ID、CAN 油门模式及所测电机对应的 `ESC_BM` 位。 |
| Motor Test 转错位置 | 按上方 **A–H → Motor → 节点** 表查节点分配和电调实际安装位置。 |
| `reboot required after parameter change` | `CAN_D2_CY_NODE`、`ESC_ID` 或 `ESC_BM` 已改变；保存后断开测试并重启。 |

## 参考资料

- 本工程驱动范围和构建测试：[AP_CyphalCAN README](README.md)。协议字段以用户提供的《CyphalCAN协议-V1.0.2-2024》及本工程实际代码为准。
- [ArduPilot：机架类型配置](https://ardupilot.org/copter/docs/frame-type-configuration.html)、[电机顺序及 Mission Planner Motor Test](https://ardupilot.org/copter/docs/connect-escs-and-motors.html)。
- [ArduPilot：CAN 总线配置](https://ardupilot.org/copter/docs/common-canbus-setup-advanced.html)、[Mission Planner：加载本地 `.apj` 固件](https://ardupilot.org/planner/docs/common-loading-firmware-onto-pixhawk.html)。
- [CUAV X7 Pro 官方快速布线](https://doc.cuav.net/controller/x7-plus-pro/quick-start-x7-pro.html)。
