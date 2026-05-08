"""
XY3606B 可调电源模块 Modbus RTU 驱动
ESP32 MicroPython
UART: TX=GPIO18, RX=GPIO19, 波特率=115200

寄存器映射 (FC03 读保持寄存器 / FC06 写单个寄存器):
  0x0000  电压设定值      RW  0.01V
  0x0001  电流设定值      RW  0.001A
  0x0002  输出电压实测    RO  0.01V
  0x0003  输出电流实测    RO  0.001A
  0x0004  输出功率实测    RO  0.01W
  0x0005  输入电压实测    RO  0.01V
  0x0006  输出电量累计    RO  0.001Ah
  0x0008  输出能量累计    RO  0.001Wh
  0x000A  运行时间 (时)   RO
  0x000B  运行时间 (分)   RO
  0x000C  运行时间 (秒)   RO
  0x000D  实时温度        RO  0.1℃
  0x0012  输出开关        RW  1=开, 0=关
  0x0013  温度单位        RW  1=华氏, 0=摄氏
  0x0014  液晶亮度        RW  1~5
  0x0016  品牌            RO  ASCII "YT"
  0x0017  固件版本        RO  例: 0x0088 → V1.36
  0x001C  蜂鸣器          RW  1=开, 0=关
  0x001D  数据组          RW  0~9
  0x001F  MPPT开关        RW  1=开, 0=关
  0x0020  MPPT系数        RW  0.75~0.85 (×100存储)
"""

import struct
import time
from machine import UART


# ─── 寄存器地址 ───────────────────────────────────────────────────────
REG_V_SET       = 0x0000   # 电压设定值       RW  0.01V
REG_I_SET       = 0x0001   # 电流设定值       RW  0.001A
REG_V_OUT       = 0x0002   # 输出电压实测     RO  0.01V
REG_I_OUT       = 0x0003   # 输出电流实测     RO  0.001A
REG_P_OUT       = 0x0004   # 输出功率实测     RO  0.01W
REG_V_IN        = 0x0005   # 输入电压实测     RO  0.01V
REG_AH_OUT      = 0x0006   # 输出电量累计     RO  0.001Ah
REG_WH_OUT      = 0x0008   # 输出能量累计     RO  0.001Wh
REG_TIME_H      = 0x000A   # 运行时间 时      RO
REG_TIME_M      = 0x000B   # 运行时间 分      RO
REG_TIME_S      = 0x000C   # 运行时间 秒      RO
REG_TEMP        = 0x000D   # 实时温度         RO  0.1℃
REG_OUTPUT      = 0x0012   # 输出开关         RW  1=开, 0=关
REG_TEMP_UNIT   = 0x0013   # 温度单位         RW  1=华氏, 0=摄氏
REG_BRIGHTNESS  = 0x0014   # 液晶亮度         RW  1~5
REG_BRAND       = 0x0016   # 品牌             RO  ASCII
REG_VERSION     = 0x0017   # 固件版本         RO
REG_BUZZER      = 0x001C   # 蜂鸣器           RW  1=开, 0=关
REG_DATA_GROUP  = 0x001D   # 数据组           RW  0~9
REG_MPPT_EN     = 0x001F   # MPPT开关         RW  1=开, 0=关
REG_MPPT_COEF   = 0x0020   # MPPT系数         RW  ×100存储

# ─── Modbus 功能码 ────────────────────────────────────────────────────
FC_READ_HOLD    = 0x03     # 读保持寄存器
FC_WRITE_SINGLE = 0x06     # 写单个寄存器


# ─── CRC16 ───────────────────────────────────────────────────────────
def _crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc


# ─── 驱动主类 ─────────────────────────────────────────────────────────
class XY3606B:
    """XY3606B Modbus RTU 驱动"""

    def __init__(self, slave_id=1, tx_pin=18, rx_pin=19,
                 baudrate=115200, timeout_ms=300):
        """
        :param slave_id:   Modbus 从机地址 (默认 1)
        :param tx_pin:     UART TX 引脚号  (默认 GPIO18)
        :param rx_pin:     UART RX 引脚号  (默认 GPIO19)
        :param baudrate:   波特率          (默认 115200)
        :param timeout_ms: 响应超时毫秒数  (默认 300)
        """
        self.slave_id   = slave_id
        self.timeout_ms = timeout_ms
        self.uart = UART(1, baudrate=baudrate, tx=tx_pin, rx=rx_pin,
                         bits=8, parity=None, stop=1)

    # ── 底层帧处理 ────────────────────────────────────────────────────

    def _build_frame(self, func: int, payload: bytes) -> bytes:
        raw = bytes([self.slave_id, func]) + payload
        return raw + struct.pack('<H', _crc16(raw))

    def _send_recv(self, frame: bytes, expected: int) -> bytes | None:
        """发送帧并等待 expected 字节的响应，失败返回 None"""
        self.uart.read()           # 清空接收缓冲
        self.uart.write(frame)
        deadline = time.ticks_add(time.ticks_ms(), self.timeout_ms)
        buf = b''
        while len(buf) < expected:
            if time.ticks_diff(deadline, time.ticks_ms()) <= 0:
                return None
            chunk = self.uart.read(expected - len(buf))
            if chunk:
                buf += chunk
        return buf

    def _check_crc(self, frame: bytes) -> bool:
        if len(frame) < 4:
            return False
        return _crc16(frame[:-2]) == struct.unpack('<H', frame[-2:])[0]

    # ── 寄存器读写 ────────────────────────────────────────────────────

    def read_registers(self, addr: int, count: int = 1) -> list | None:
        """
        读取连续保持寄存器 (FC03)
        :return: 寄存器值列表，失败返回 None
        """
        payload = struct.pack('>HH', addr, count)
        frame   = self._build_frame(FC_READ_HOLD, payload)
        resp    = self._send_recv(frame, 5 + count * 2)
        if resp is None:
            return None
        if not self._check_crc(resp):
            return None
        if resp[1] != FC_READ_HOLD:
            return None
        n = resp[2]
        return list(struct.unpack(f'>{n // 2}H', resp[3:3 + n]))

    def write_register(self, addr: int, value: int) -> bool:
        """
        写单个保持寄存器 (FC06)
        :return: 成功 True，失败 False
        """
        payload = struct.pack('>HH', addr, value)
        frame   = self._build_frame(FC_WRITE_SINGLE, payload)
        resp    = self._send_recv(frame, 8)
        if resp is None:
            return False
        if not self._check_crc(resp):
            return False
        if resp[1] != FC_WRITE_SINGLE:
            return False
        return True

    # ── 电压 / 电流 / 输出开关 ────────────────────────────────────────

    def set_voltage(self, voltage_v: float) -> bool:
        """设置输出电压 (V)，分辨率 0.01V"""
        return self.write_register(REG_V_SET, round(voltage_v * 100))

    def set_current(self, current_a: float) -> bool:
        """设置输出电流 (A)，分辨率 0.001A"""
        return self.write_register(REG_I_SET, round(current_a * 1000))

    def set_output(self, enable: bool) -> bool:
        """开启 / 关闭输出"""
        return self.write_register(REG_OUTPUT, 1 if enable else 0)

    def get_output(self) -> bool | None:
        """读取输出开关状态，True=开，False=关，失败返回 None"""
        vals = self.read_registers(REG_OUTPUT)
        return None if vals is None else bool(vals[0])

    # ── 实时测量值 ────────────────────────────────────────────────────

    def read_status(self) -> dict | None:
        """
        一次读取输出电压/电流/功率及输入电压
        :return: {'v_out', 'i_out', 'p_out', 'v_in'}，单位 V / A / W
        """
        vals = self.read_registers(REG_V_OUT, 4)
        if vals is None:
            return None
        return {
            'v_out': vals[0] / 100,
            'i_out': vals[1] / 1000,
            'p_out': vals[2] / 100,
            'v_in' : vals[3] / 100,
        }

    def read_energy(self) -> dict | None:
        """
        读取累计电量与能量
        :return: {'ah_out' (Ah), 'wh_out' (Wh)}
        """
        ah = self.read_registers(REG_AH_OUT)
        wh = self.read_registers(REG_WH_OUT)
        if ah is None or wh is None:
            return None
        return {
            'ah_out': ah[0] / 1000,
            'wh_out': wh[0] / 1000,
        }

    def read_runtime(self) -> dict | None:
        """
        读取运行时间
        :return: {'hours', 'minutes', 'seconds'}
        """
        vals = self.read_registers(REG_TIME_H, 3)
        if vals is None:
            return None
        return {
            'hours'  : vals[0],
            'minutes': vals[1],
            'seconds': vals[2],
        }

    def read_temperature(self) -> float | None:
        """读取模块温度 (℃ 或 ℉，取决于温度单位设置)"""
        vals = self.read_registers(REG_TEMP)
        return None if vals is None else vals[0] / 10

    # ── 设定值读取 ────────────────────────────────────────────────────

    def read_setpoints(self) -> dict | None:
        """
        读取电压/电流设定值
        :return: {'v_set' (V), 'i_set' (A)}
        """
        vals = self.read_registers(REG_V_SET, 2)
        if vals is None:
            return None
        return {
            'v_set': vals[0] / 100,
            'i_set': vals[1] / 1000,
        }

    # ── 设备信息 ──────────────────────────────────────────────────────

    def read_device_info(self) -> dict | None:
        """
        读取品牌与固件版本
        :return: {'brand' (str), 'version_raw' (int)}
        """
        vals = self.read_registers(REG_BRAND, 2)
        if vals is None:
            return None
        brand = chr(vals[0] >> 8) + chr(vals[0] & 0xFF)
        return {
            'brand'      : brand.strip(),
            'version_raw': vals[1],
        }

    # ── 显示与蜂鸣器设置 ──────────────────────────────────────────────

    def set_brightness(self, level: int) -> bool:
        """设置液晶亮度，level: 1~5"""
        level = max(1, min(5, level))
        return self.write_register(REG_BRIGHTNESS, level)

    def set_buzzer(self, enable: bool) -> bool:
        """开启 / 关闭蜂鸣器"""
        return self.write_register(REG_BUZZER, 1 if enable else 0)

    def set_temp_unit(self, fahrenheit: bool = False) -> bool:
        """设置温度单位，False=摄氏(默认)，True=华氏"""
        return self.write_register(REG_TEMP_UNIT, 1 if fahrenheit else 0)

    # ── MPPT ──────────────────────────────────────────────────────────

    def set_mppt(self, enable: bool, coef: float = 0.80) -> bool:
        """
        设置 MPPT
        :param enable: True=开启
        :param coef:   MPPT系数 0.75~0.85
        """
        ok = self.write_register(REG_MPPT_EN, 1 if enable else 0)
        if ok and enable:
            coef_raw = round(max(75, min(85, coef * 100)))
            ok = self.write_register(REG_MPPT_COEF, coef_raw)
        return ok

    # ── 数据组 ────────────────────────────────────────────────────────

    def set_data_group(self, group: int) -> bool:
        """切换预设数据组，group: 0~9"""
        group = max(0, min(9, group))
        return self.write_register(REG_DATA_GROUP, group)

    # ── 调试工具 ──────────────────────────────────────────────────────

    def scan(self, start=0x0000, end=0x0020) -> None:
        """扫描并打印指定范围内所有有效寄存器的值"""
        print(f"{'地址':^8} {'十进制':>8} {'十六进制':>8}")
        print("-" * 28)
        for addr in range(start, end + 1):
            vals = self.read_registers(addr, 1)
            if vals is not None:
                print(f"  0x{addr:04X}   {vals[0]:>8d}   0x{vals[0]:04X}")
            time.sleep_ms(80)


# ─── 使用示例 ──────────────────────────────────────────────────────────
def demo():
    psu = XY3606B(slave_id=1, tx_pin=18, rx_pin=19, baudrate=115200)

    # 设备信息
    info = psu.read_device_info()
    if info:
        print(f"品牌: {info['brand']}  固件版本原始值: {info['version_raw']:#06x}")

    # 读取设定值
    sp = psu.read_setpoints()
    if sp:
        print(f"设定 → 电压: {sp['v_set']:.2f} V  电流: {sp['i_set']:.3f} A")

    # 配置输出参数
    psu.set_voltage(12.00)
    psu.set_current(2.000)

    # 开启输出
    psu.set_output(True)
    time.sleep_ms(500)

    # 循环读取状态
    for _ in range(5):
        st = psu.read_status()
        if st:
            print(f"V={st['v_out']:.2f}V  I={st['i_out']:.3f}A  "
                  f"P={st['p_out']:.2f}W  Vin={st['v_in']:.2f}V")
        time.sleep(1)

    # 读取温度与运行时间
    print(f"温度: {psu.read_temperature()} ℃")
    rt = psu.read_runtime()
    if rt:
        print(f"运行时间: {rt['hours']}h {rt['minutes']}m {rt['seconds']}s")

    # 关闭输出
    psu.set_output(False)


if __name__ == '__main__':
    demo()
