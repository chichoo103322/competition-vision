"""视觉任务的数据模型与通用状态常量。"""

from __future__ import annotations

from dataclasses import dataclass

STATUS_OK = 0
ERROR_TASK_CODE = 1
ERROR_CAMERA = 2
ERROR_MATERIALS = 3
ERROR_RINGS = 4


@dataclass(frozen=True)
class TaskCode:
    """由四组三位数组成的搬运任务码。"""

    group1: str
    group2: str
    group3: str
    group4: str

    @classmethod
    def parse(cls, text: str) -> "TaskCode":
        """从形如 156+123+516+231 的字符串解析任务码。"""
        parts = text.strip().split("+")
        if len(parts) != 4:
            raise ValueError("任务码必须包含四组三位数")
        for part in parts:
            if len(part) != 3 or not part.isdigit():
                raise ValueError(f"任务码分组格式错误: {part}")
        return cls(group1=parts[0], group2=parts[1], group3=parts[2], group4=parts[3])

    def __str__(self) -> str:
        return f"{self.group1}+{self.group2}+{self.group3}+{self.group4}"


@dataclass(frozen=True)
class Material:
    """一个被识别出的物料。"""

    index: int
    color_id: int
    shape_id: int
    center_x: int
    center_y: int
    theta_deg: float
    confidence: int
    track_id: int | None = None
    tracking_status: str = 'new'
    point_kind: str = 'contour_bounding_box_center'

    def to_tuple(self):
        """转成串口协议要求的元组。"""
        return (
            self.index,
            self.color_id,
            self.shape_id,
            self.center_x,
            self.center_y,
            self.theta_deg,
            self.confidence,
        )


@dataclass(frozen=True)
class Ring:
    """一个被识别出的放置圆环。"""

    ring_no: int
    center_x: int
    center_y: int
    confidence: int
    radius: float = 0.0

    def to_tuple(self):
        """转成串口协议要求的元组。"""
        return (self.ring_no, self.center_x, self.center_y, self.confidence)
