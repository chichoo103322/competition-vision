#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
树莓派 <-> STM32 通信协议编解码（帧格式见 PROTOCOL.md v3.2 · 智能搬运赛项版）

架构：树莓派只做视觉识别 + 二维码解析，任务规划在 STM32。
  RPi -> STM32 : 视觉结果上报（任务码 / 识别到 / 未识别到 / 放置验证 / 障碍物 / 心跳）
                 + 对主控控制帧的 ACK 应答（0x81）
  STM32 -> RPi : 扫描控制（开始(带模式)/停止/设目标色/重扫）

帧格式: [0xAA][LEN][CMD][参数...][CHK]   CHK = 载荷逐字节求和，取低 8 位

用法：
    frame = cmd_taskcode('156+123+516+231')   # 二维码解析 → 任务码帧
    ser.write(frame)
    for cmd, params in splitter.feed(ser.read(64)):   # 流式拆帧，容忍粘包/断帧
        ...                                           # 主控指令/ACK 分发
"""

# ---------- 命令码 ----------
# RPi -> STM32：视觉结果
CMD_TASKCODE  = 0x10   # 任务码：4组×3位数字，共 12 字节
CMD_FOUND     = 0x11   # 识别到物料：颜色 + center_x/y + 偏移角(0.1°) + 距离(cm)
CMD_NOTFOUND  = 0x12   # 未识别到物料（读码阶段=读码失败，触发主控重扫）
CMD_PLACE_CHK = 0x13   # (可选) 放置验证：区域 + 环号(1~6) + 结果
CMD_OBSTACLE  = 0x14   # (可选) 障碍物：偏移角 + 距离
CMD_HEARTBEAT = 0x15   # 心跳：RPi 1Hz 上报，主控据此判断链路存活
CMD_RING_CENTER = 0x16 # 识别到圆环中心：环号 + center_x + center_y

# STM32 -> RPi：扫描控制
CMD_SCAN      = 0x21   # 开始扫描（带 1 字节模式参数，见 SCAN_MODE_*）
CMD_STOP_SCAN = 0x22   # 停止扫描
CMD_SET_COLOR = 0x23   # 设定目标颜色
CMD_RESCAN    = 0x24   # 按当前模式重试（重扫/重读二维码）

# 双向：应答
CMD_ACK       = 0x81   # 应答：原CMD + 状态。RPi 收到 0x2x 控制帧后必须回 ACK；
                       # 主控对 RPi 上报帧（0x1x）不应答

# ---------- 扫描模式（0x21 参数） ----------
SCAN_MODE_QR       = 0   # 读二维码板 → 上报 0x10
SCAN_MODE_MATERIAL = 1   # 找目标颜色物料（配合 0x23 设色）→ 上报 0x11/0x12
SCAN_MODE_VERIFY   = 2   # 放置验证 → 上报 0x13

# ---------- 编码约定 ----------
COLOR = {1: '红', 2: '黄', 3: '蓝', 4: '绿', 5: '黑', 6: '浅蓝', 7: '底座'}  # 赛题颜色（7=底座，非夹取物料）
ST_OK, ST_ERR, ST_BUSY = 0, 1, 2                                 # ACK 状态
MAX_FRAME = 16            # 本协议最大帧长（任务码帧 16 字节）


# ---------- 帧层 ----------
def build_frame(cmd, params=b''):
    """组帧：AA LEN CMD 参数... CHK"""
    payload = bytes([cmd]) + params
    return b'\xAA' + bytes([len(payload)]) + payload + bytes([sum(payload) & 0xFF])


def parse_frame(frame):
    """拆帧 → (cmd, 参数bytes)；帧头/长度/校验任一不符返回 None"""
    if len(frame) < 3 or frame[0] != 0xAA:
        return None
    length = frame[1]
    if len(frame) < length + 3:
        return None
    payload = frame[2:2 + length]
    if sum(payload) & 0xFF != frame[2 + length]:
        return None
    return payload[0], payload[1:]


class FrameSplitter:
    """流式拆帧器：容忍粘包（多帧一次到达）、断帧（一帧分多次到达）、垃圾字节。
    每个串口接收方应持有唯一实例，把每次读到的字节喂给 feed()，
    取回本次新解析出的 (cmd, params) 列表。"""

    def __init__(self):
        self.buf = bytearray()

    def feed(self, data):
        self.buf += data
        out = []
        while True:
            idx = self.buf.find(b'\xAA')
            if idx < 0:
                self.buf.clear()
                break
            if idx > 0:                       # 丢弃帧头前的垃圾字节
                del self.buf[:idx]
            if len(self.buf) < 3:
                break
            total = self.buf[1] + 3
            if self.buf[1] == 0 or total > MAX_FRAME:   # LEN 非法 → 跳过该帧头重新同步
                del self.buf[0]
                continue
            if len(self.buf) < total:         # 断帧：等更多字节
                break
            parsed = parse_frame(bytes(self.buf[:total]))
            if parsed:
                out.append(parsed)
                del self.buf[:total]
            else:                             # 校验失败 → 只丢帧头，重新同步
                del self.buf[0]
        return out


# ---------- 输入校验 ----------
def _check_color(color):
    if color not in COLOR:
        raise ValueError(f'非法颜色码 {color}（应为 1~6：{list(COLOR)}）')


def _check_range(name, v, lo, hi):
    if not lo <= v <= hi:
        raise ValueError(f'{name}={v} 超出范围 [{lo}, {hi}]')


def _s16(v): return v.to_bytes(2, 'big', signed=True)   # 有符号 16 位
def _u16(v): return v.to_bytes(2, 'big')                # 无符号 16 位


def _off(deg):
    """度 → 0.1° int16，整数化；超出 int16 表示范围抛 ValueError"""
    v = round(deg * 10)
    _check_range('偏移角(0.1°)', v, -32768, 32767)
    return _s16(v)


def _dist(cm):
    _check_range('距离(cm)', cm, 0, 65535)
    return _u16(cm)


# ---------- 编码（RPi -> STM32）----------
def cmd_taskcode(task):
    """任务码帧。task 支持 "156+123+516+231" / [156,123,516,231] / 12 位数字列表。
    每位数字必须 1~6（赛题颜色/环号取值），非法输入抛 ValueError。"""
    if isinstance(task, str):
        digits = [int(c) for c in task if c.isdigit()]
    elif len(task) == 4:                       # 4 组三位数 → 逐位展开
        digits = [d for g in task for d in (g // 100, g % 100 // 10, g % 10)]
    else:                                      # 12 位单数字
        digits = list(task)
    if len(digits) != 12:
        raise ValueError(f'任务码需 12 位数字，实际 {len(digits)}')
    if any(d < 1 or d > 6 for d in digits):
        raise ValueError(f'任务码每位须为 1~6，实际 {digits}')
    return build_frame(CMD_TASKCODE, bytes(digits))


def cmd_found(color, center_x, center_y, offset_deg, dist_cm):
    """识别到物料：颜色码(1~6) + 中心x/y + 偏移角(度, 正=右偏) + 距离(cm)"""
    _check_color(color)
    _check_range('center_x', center_x, 0, 65535)
    _check_range('center_y', center_y, 0, 65535)
    return build_frame(
        CMD_FOUND,
        bytes([color])
        + _u16(center_x)
        + _u16(center_y)
        + _off(offset_deg)
        + _dist(dist_cm),
    )


def cmd_ring_center(ring_no, center_x, center_y):
    """识别到圆环中心：环号(1~6) + 中心x/y（图像像素）"""
    _check_range('环号', ring_no, 1, 6)
    _check_range('center_x', center_x, 0, 65535)
    _check_range('center_y', center_y, 0, 65535)
    return build_frame(
        CMD_RING_CENTER,
        bytes([ring_no]) + _u16(center_x) + _u16(center_y),
    )


def cmd_not_found():
    """未识别到物料（读码阶段发送表示读码失败，主控将重扫）"""
    return build_frame(CMD_NOTFOUND)


def cmd_place_check(zone, pos, ok):
    """(可选) 放置验证：区域(1粗加工 2暂存) + 环号(1~6) + 结果(0错 1对)"""
    _check_range('区域', zone, 1, 2)
    _check_range('环号', pos, 1, 6)
    _check_range('结果', ok, 0, 1)
    return build_frame(CMD_PLACE_CHK, bytes([zone, pos, ok]))


def cmd_obstacle(offset_deg, dist_cm):
    """(可选) 障碍物：偏移角 + 距离"""
    return build_frame(CMD_OBSTACLE, _off(offset_deg) + _dist(dist_cm))


def cmd_heartbeat():
    """心跳：RPi 视觉主循环以约 1Hz 周期发送，主控 3s 未收到任何 0x1x 帧判链路故障"""
    return build_frame(CMD_HEARTBEAT)


def cmd_ack(orig_cmd, status=ST_OK):
    """ACK 应答：RPi 收到主控控制帧(0x2x)后回发。orig_cmd=原命令码，status=0/1/2"""
    _check_range('状态码', status, 0, 2)
    return build_frame(CMD_ACK, bytes([orig_cmd, status]))


# ---------- 解码（STM32 -> RPi）----------
def parse_control(frame):
    """解析主控指令 → (cmd, value)；无效返回 None。
    value：0x21=(模式)，0x22/0x24=None，0x23=颜色码，0x81=(原CMD, 状态码)
    """
    parsed = parse_frame(frame)
    if not parsed:
        return None
    cmd, p = parsed
    if cmd == CMD_SCAN and len(p) == 1:
        return cmd, p[0]
    if cmd in (CMD_STOP_SCAN, CMD_RESCAN) and not p:
        return cmd, None
    if cmd == CMD_SET_COLOR and len(p) == 1:
        return cmd, p[0]
    if cmd == CMD_ACK and len(p) == 2:
        return cmd, (p[0], p[1])
    return None


# ---------- 自测 ----------
if __name__ == '__main__':
    cases = [
        (cmd_taskcode('156+123+516+231'), 'aa0d1001050601020305010602030134'),
        (cmd_taskcode([156, 123, 516, 231]), 'aa0d1001050601020305010602030134'),
        (cmd_found(5, 100, 200, 12.3, 45), 'aa0a1105006400c8007b002dea'),
        (cmd_ring_center(3, 100, 200), 'aa061603006400c845'),
        (cmd_not_found(), 'aa011212'),
        (cmd_place_check(2, 3, 1), 'aa041302030119'),
        (cmd_obstacle(-5.0, 30), 'aa0514ffce001eff'),
        (cmd_heartbeat(), 'aa011515'),
        (cmd_ack(0x21, ST_OK), 'aa03812100a2'),
        (build_frame(CMD_SCAN, bytes([SCAN_MODE_MATERIAL])), 'aa02210122'),
        (build_frame(CMD_SCAN, bytes([SCAN_MODE_QR])), 'aa02210021'),
        (build_frame(CMD_SET_COLOR, b'\x05'), 'aa02230528'),
        (build_frame(CMD_RESCAN), 'aa012424'),
    ]
    for frame, expect in cases:
        assert frame.hex() == expect, f'编码不符: {frame.hex()} != {expect}'
        assert parse_frame(frame), f'解析失败: {expect}'

    # parse_control 分发
    assert parse_control(build_frame(CMD_SCAN, bytes([SCAN_MODE_MATERIAL]))) == (CMD_SCAN, SCAN_MODE_MATERIAL)
    assert parse_control(build_frame(CMD_SET_COLOR, b'\x06')) == (CMD_SET_COLOR, 6)
    assert parse_control(build_frame(CMD_ACK, b'\x21\x00')) == (CMD_ACK, (0x21, 0))

    # 坏校验必须被拒绝
    bad = bytearray(cmd_found(5, 100, 200, 12.3, 45)); bad[-1] ^= 0xFF
    assert not parse_frame(bytes(bad)), '坏校验未被拒绝'

    # 非法输入必须抛 ValueError（而非静默发帧）
    for fn in (lambda: cmd_taskcode('789+123+516+231'),   # 数字越界
               lambda: cmd_taskcode('156+123+516+23'),     # 位数不足
               lambda: cmd_found(7, 0, 0, 10, 10),         # 颜色越界
               lambda: cmd_found(5, 0, 0, 4000.0, 10),     # 偏移角溢出 int16
               lambda: cmd_found(5, 0, 0, 0, -1),          # 距离为负
               lambda: cmd_place_check(2, 7, 1),           # 环号越界(1~6)
               lambda: cmd_ring_center(7, 0, 0)):          # 圆环号越界(1~6)
        try:
            fn()
            raise AssertionError(f'非法输入未被拒绝: {fn}')
        except ValueError:
            pass

    # FrameSplitter：粘包 / 断帧 / 垃圾字节 / 坏帧重同步
    sp = FrameSplitter()
    f1, f2 = cmd_found(5, 100, 200, 12.3, 45), cmd_heartbeat()
    assert sp.feed(f1[:4]) == []                  # 断帧前半
    r = sp.feed(f1[4:] + f2)                      # 断帧后半 + 粘包
    assert r == [parse_frame(f1), parse_frame(f2)], f'粘包/断帧: {r}'
    r = sp.feed(b'\x00\xff' + bytes(bad) + f1)    # 垃圾前缀 + 坏帧 + 好帧
    assert r == [parse_frame(f1)], f'重同步: {r}'
    assert sp.buf == bytearray(), '拆帧器缓冲应已清空'

    print('自测通过：编解码/校验/拆帧与协议示例帧完全匹配')
