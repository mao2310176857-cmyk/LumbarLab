# Lumbar Lab v0.1 接口约定

本项目为 Python 3.10+ 标准库本地服务 + 无构建依赖网页，默认模拟，真实串口只读。

## HTTP

- `GET /api/state` 获取下述状态；前端约 200 ms 轮询。
- `POST /api/action` JSON `{action: ..., ...}`，成功返回最新状态，错误 400 JSON `{error: 中文原因}`。
- `GET /api/sessions` 返回 `{sessions:[{id,started_at,ended_at,samples,feedback_count}]}`。
- `GET /api/sessions/<id>` 返回 `{schema_version:1,id,started_at,ended_at,parameters,reference,frames:[],events:[]}`。frames 中为记录时的 state 快照（不含 history）；events 为 `{t,type,detail}`。
- `GET /api/sessions/<id>/download` 下载同一 JSON。
- `GET /api/ports` 返回 `{ports:[{device,description}],available:bool}`。

动作：`manual` + `direction:advance|retract`（一次受限小步）；`stop`；`calibrate`（稳定采集 3 秒）；`auto_start`；`auto_pause`；`scenario` + `name:neutral|forward|backward|left|away|contact_loss|dropout|invalid`；`record_start`；`record_stop`；`feedback` + `value:too_much|comfortable|too_little`；`connect` + `source:simulation|serial,port?,baud?`；`zero`（串口模式需 confirmed_empty:true；使用 raw 在电脑端归零，不发送板端指令）；`parameters` + `values`（控制参数）。

状态结构（字段全部提供，未准备好时 null）：
```json
{"time":0,"source":"simulation","connected":true,"scenario":"neutral","zones":[0,0,0,0,0,0,0,0],"raw_zones":[0,0,0,0,0,0,0,0],"matrix":null,"raw_matrix":null,"frame_seq":0,"device_time_ms":0,"frame_age_ms":0,"signal_valid":true,"occupied":true,"contact":true,"mode":"manual","action":"hold","reason":"手动模式","position":45,"reference":null,"calibration":{"active":false,"progress":0,"message":"尚未记住支撑"},"metrics":{"lumbar":0,"target":null,"lower":null,"upper":null},"parameters":{"tolerance":0.18,"persistence_s":0.6,"speed":5.0,"settle_s":0.6},"recording":{"active":false,"id":null,"samples":0,"elapsed_s":0},"events":[],"history":[],"hardware":{"actuation_enabled":false,"mapping_ready":false,"port":null,"message":"模拟设备"}}
```
reference: `{zones:[8 values],noise:[8 values],created_at:ISO,position:number|null}`；history: 最近 60 秒 `{t,z5,z6,z7,lumbar,position,action,lower,upper}`；events: 最近 30 条 `{t,type,detail}`。
mode 为 manual|auto|paused，action 为 advance|retract|hold|stop，position 为模拟行程百分比（串口 null）。time/t 为服务启动后秒。

## 设备适配接口（lumbar/devices.py）

`SimulationDevice` 由主代理实现。
`SerialSensor(port, baud=2000000, mapping_path=...)`：connect(), close(), read() -> `SensorFrame | None`（非阻塞获取最近新帧），zero()（仅电脑端 raw 基线，缺 raw 报错）；字段 mapping_ready, port, message。串口读取线程只有一处，绝不 serial.write。
`SensorFrame` dataclass：zones:list[float] | None, raw_zones:list[float] | None, matrix:list[int] | None, raw_matrix:list[int] | None, seq:int, device_time_ms:int, received_at:float（time.monotonic）, valid:bool=True。
`parse_frame(line)` -> dict | None；忽略空行和 # 注释，JSON 内容严格检查（数组 256 个 0..255 整数、base 0/1、seq uint32、t uint32），错误 ValueError。base=0 应标记数据未标定、不可控制。
`load_mapping(path)` 和 `map_zones(values,mapping)`：config/zones.json 中键 1..8，各值为矩阵索引列表；初始全部空，不猜映射。聚合用均值，相对单位 0..255。
`list_serial_ports()` -> list[dict]。pyserial 按需导入，模拟不依赖。

## 硬件扩展入口

`python run.py --actuator-factory integrations.hardware_adapter:create_actuator` 显式加载接手方实现的 Python 工厂。模板为 `integrations/hardware_adapter_template.py`，先复制后实现；模板的 serial 分支会抛出未实现错误。未提供参数时保留默认只读串口行为。启动始终先进入模拟模式；切换串口时才创建真实适配器。

接手文件分工与验收见 [HANDOFF.md](../HANDOFF.md) 和 [硬件接入与自动驱动实现指南](HARDWARE_INTEGRATION.md)。

## 执行器与输出状态

控制器 → `AppService._dispatch_control()` → `ActuatorDispatcher` → 适配器。`AppService(..., actuator_factory=None)` 默认工厂返回模拟执行器或禁用执行器；确认实际硬件后可注入工厂 `(source, sensor_device) -> actuator`，无需修改自动决策算法。默认串口仍没有真实驱动协议。

新增 `state.actuator`：
```json
{"requested_action":"hold","command":{"seq":1,"action":"hold","issued_at":100.0,"speed":0,"duration_s":0},"sent":true,"acknowledged":true,"message":"模拟执行器已接收指令；不代表真实电机动作","fault":null,"is_hardware":false,"enabled":true}
```

`action` 是控制器决策；`actuator.command` 是最后实际派发给适配器的命令。禁用输出时，派发器只传递不会写硬件的 stop 意图，算法仍可生成建议。`seq` 为单个派发器生命周期内的序号。`issued_at` 来自电脑端单调时钟；保持/停止的 speed 和 duration 为 0。同一次动作的签发时刻和时限不会被 tick 延长。

`sent/acknowledged` 只表示适配器返回的发送/接收状态，不表示机械动作完成。`hardware.actuation_enabled` 为真实适配器 enabled 且无锁存故障；不能据此判断限位或运动是否完成。默认状态始终为 false。旧记录缺少 actuator 字段时，界面展示未记录输出状态。

执行异常和未确认接收会暂停控制并请求停止，故障保留到检查并重新连接；每次用户停止都会重新调用停止接口。归零与切换数据源前必须成功完成适配器停止请求。实际部署的物理限位、驱动反馈与独立超时停机仍需在适配器及板端实现，当前没有对应硬件实现。
