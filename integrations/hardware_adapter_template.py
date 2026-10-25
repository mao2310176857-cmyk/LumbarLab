"""给硬件同伴的接入模板：本文件不会打开端口、发送字节或驱动电机。

1. 复制为 integrations/hardware_adapter.py。
2. 根据已确认的驱动程序、接线和协议实现下方 TODO，完成台架验证。
3. 将 create_actuator 的 serial 分支改为返回你的适配器。
4. 启动：python run.py --actuator-factory integrations.hardware_adapter:create_actuator
   启动仍为模拟模式；界面连接串口时才调用 serial 分支。

execute 收到 ActuatorCommand，没有界面派发器的 seq；适配器必须自行生成
wire command_id，并核对设备回执的 ID、状态和时限。这里的自增 ID 仅为示例，
实际协议还需处理重新连接、板端重启、重复包及旧回执。

同一个串口不能被两个对象重复打开。sensor_device 当前只读，不能直接借用其
私有 _serial 写命令；同板采集/驱动需实现一个共同拥有串口并分流数据/回执的
传输层。独立驱动板可由适配器持有自己的另一个端口。
"""
import time

from lumbar.actuators import ActuatorCommand, ActuatorResult, SimulationActuator


def create_actuator(source, sensor_device):
    """AppService 使用的工厂；未完成的模板明确拒绝启用真实输出。"""
    if source == "simulation":
        return SimulationActuator(sensor_device)
    if source == "serial":
        # 完成下面的真实驱动、ACK、限位及板端超时后才可改为：
        # return HardwareActuator(sensor_device)
        raise NotImplementedError("电机模板尚未实现：请先补齐驱动协议、速度标定、回执和板端停止，再启用 serial 工厂")
    raise ValueError("未知数据来源")


class HardwareActuator:
    """可替换的最小驱动骨架；所有物理输出方法均未实现。

    适配器构造时应确认设备身份、限位与初始停止状态；只有完成握手才允许
    enabled=True。此模板工厂不会创建此类，直接创建也不会打开设备。
    """

    enabled = False
    is_hardware = True

    def __init__(self, sensor_device):
        self.sensor_device = sensor_device
        self._wire_id = 0
        # TODO: 建立已确认的传输/握手；失败应抛异常，不得返回假 ACK。
        # 完成设备身份与初始停止确认后，才设置 self.enabled = True。

    def execute(self, command):
        if not isinstance(command, ActuatorCommand):
            raise ValueError("execute 需要 ActuatorCommand")
        self._wire_id += 1
        command_id = self._wire_id
        if command.action in ("hold", "stop"):
            return self._stop_and_confirm(command_id)
        remaining_s = command.expires_at - time.monotonic()
        if remaining_s <= 0:
            # 延迟到达的移动命令不能启动；请求停止并返回真实停止回执。
            return self._stop_and_confirm(command_id)
        return self._move_and_confirm(command_id, command.action, command.speed, remaining_s)

    def _move_and_confirm(self, command_id, direction, speed_intent, remaining_s) -> ActuatorResult:
        # TODO: speed_intent 目前采用模拟行程百分比/秒，必须经过实测标定
        # 转成该机构的速度/脉冲/位置策略，绝不能直接当 PWM 或毫米/秒。
        # TODO: 将 direction + 真实速度 + 剩余时限写入已确认的协议。
        # 板端独立执行时限/看门狗/限位，重复命令不得重新延长运动期限。
        # TODO: 等待并校验对应 command_id 的设备 ACK，错误/超时应抛异常
        # 或返回 sent/acknowledged 的真实结果。写入成功不等于电机已运动。
        raise NotImplementedError("尚未实现真实移动协议、速度映射和设备回执")

    def _stop_and_confirm(self, command_id) -> ActuatorResult:
        # TODO: 明确该机构的停止语义（制动/释放/保持位置），发送停止并验证
        # 对应回执。不能无条件 return ActuatorResult(True, True, ...)。
        raise NotImplementedError("尚未实现真实停止指令及确认")

    def close(self):
        # AppService 会先请求 stop，再调用 close。此处仍需按驱动设计确保
        # 停止/释放连接，且断电/拔线/电脑崩溃由板端独立保护。
        raise NotImplementedError("尚未实现驱动连接关闭")
