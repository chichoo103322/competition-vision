"""任务状态机：控制视觉上位机当前处于哪个识别阶段。"""

from __future__ import annotations

from enum import Enum, auto


class MissionState(Enum):
    """视觉上位机任务状态。"""

    IDLE = auto()
    READ_TASK_CODE = auto()
    READ_MATERIALS = auto()
    READ_RINGS = auto()
    VERIFY_PLACEMENT = auto()
    ERROR = auto()


class MissionController:
    def __init__(self) -> None:
        self.state = MissionState.IDLE

    def start_mission(self) -> None:
        """收到开始指令后进入任务码识别阶段。"""
        self.state = MissionState.READ_TASK_CODE

    def on_task_code_received(self) -> None:
        """任务码发送成功后进入物料识别阶段。"""
        self.state = MissionState.READ_MATERIALS

    def request_materials(self) -> None:
        """主控请求物料列表。"""
        self.state = MissionState.READ_MATERIALS

    def request_rings(self) -> None:
        """主控请求圆环列表。"""
        self.state = MissionState.READ_RINGS

    def verify_placement(self) -> None:
        """主控请求放置结果确认。"""
        self.state = MissionState.VERIFY_PLACEMENT

    def set_error(self) -> None:
        """进入错误状态。"""
        self.state = MissionState.ERROR

    def reset(self) -> None:
        """回到待机状态。"""
        self.state = MissionState.IDLE
