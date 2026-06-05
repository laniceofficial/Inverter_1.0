#!/usr/bin/env python3
"""
DebugCapture 二进制转 CSV 工具

用法:
  python capture_convert.py capture_data.bin [output.csv]

输入: J-Link savebin 导出的二进制文件 (或任意 DebugCapture::Sample 数组的原始内存 dump)
输出: 可直接用 Excel / Python pandas 分析的 CSV 文件

struct Sample (36 bytes, little-endian):
  offset  0: uint32_t tick         (DWT->CYCCNT)
  offset  4: float    vRaw         (ADC raw 电压码值)
  offset  8: float    iRaw         (ADC raw 电流码值)
  offset 12: float    vFilt        (滤波后电压)
  offset 16: float    iFilt        (滤波后电流)
  offset 20: float    power        (发射功率)
  offset 24: float    iRawAvg      (电流滑动窗口均值)
  offset 28: uint16_t askPtr       (ASK bitBufferPointer_)
  offset 30: uint8_t  askValid     (ASK valid_)
  offset 31: uint8_t  askConnected (ASK connected_)
  offset 32: uint32_t mcmp2        (ADC 触发比较值)
"""

import struct
import sys
import os

# struct format: little-endian, no alignment padding
SAMPLE_FMT = "<IffffffHBBI"
SAMPLE_SIZE = struct.calcsize(SAMPLE_FMT)  # should be 36
assert SAMPLE_SIZE == 36, f"Unexpected struct size: {SAMPLE_SIZE}, expected 36"

HEADER = [
    "index", "tick", "vRaw", "iRaw", "vFilt", "iFilt",
    "power", "iRawAvg", "askPtr", "askValid", "askConnected", "mcmp2"
]


def convert(bin_path: str, csv_path: str) -> int:
    file_size = os.path.getsize(bin_path)
    count, remainder = divmod(file_size, SAMPLE_SIZE)

    if remainder != 0:
        print(f"Warning: file size {file_size} is not a multiple of {SAMPLE_SIZE}. "
              f"Truncating {remainder} bytes. (count={count})")

    if count == 0:
        print("Error: no complete samples in file.")
        return 1

    with open(bin_path, "rb") as f_in, open(csv_path, "w", newline="") as f_out:
        f_out.write(",".join(HEADER) + "\n")

        for i in range(count):
            raw = f_in.read(SAMPLE_SIZE)
            if len(raw) < SAMPLE_SIZE:
                break

            (
                tick, vRaw, iRaw, vFilt, iFilt, power, iRawAvg,
                askPtr, askValid, askConnected, mcmp2
            ) = struct.unpack(SAMPLE_FMT, raw)

            f_out.write(f"{i},{tick},{vRaw:.6f},{iRaw:.6f},{vFilt:.6f},{iFilt:.6f},"
                        f"{power:.6f},{iRawAvg:.6f},{askPtr},{askValid},{askConnected},{mcmp2}\n")

    print(f"Converted {count} samples → {csv_path}")
    return 0


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    bin_file = sys.argv[1]
    csv_file = sys.argv[2] if len(sys.argv) > 2 else bin_file.replace(".bin", ".csv")

    sys.exit(convert(bin_file, csv_file))
