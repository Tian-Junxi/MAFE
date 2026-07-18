#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
生成一个 200 Hz、6 列 IMU 示例数据：
第 1~3 列：陀螺仪三轴；第 4~6 列：加速度计三轴。

示例中叠加了 4 个振动频率，用于测试 C++ FFT + Gaussian fitting 代码。
"""
import csv
import math
import random
from pathlib import Path

fs = 200.0
T = 12.0
n = int(fs * T)
freqs = [23.0, 37.0, 58.0, 76.0]
random.seed(42)

out = Path(__file__).resolve().parents[1] / "examples" / "example_imu_200hz_6cols.csv"
out.parent.mkdir(parents=True, exist_ok=True)

with out.open("w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["gyro_x", "gyro_y", "gyro_z", "acc_x", "acc_y", "acc_z"])
    for i in range(n):
        t = i / fs
        gx = 0.02 * random.gauss(0, 1)
        gy = 0.02 * random.gauss(0, 1)
        gz = 0.02 * random.gauss(0, 1)
        ax = 0.03 * random.gauss(0, 1)
        ay = 0.03 * random.gauss(0, 1)
        az = 0.03 * random.gauss(0, 1)
        for k, fr in enumerate(freqs):
            phase = 0.4 * k
            gx += (0.10 + 0.02*k) * math.sin(2*math.pi*fr*t + phase)
            gy += (0.08 + 0.01*k) * math.sin(2*math.pi*fr*t + phase + 0.5)
            gz += (0.06 + 0.015*k) * math.sin(2*math.pi*fr*t + phase + 1.0)
            ax += (0.18 + 0.03*k) * math.sin(2*math.pi*fr*t + phase + 0.2)
            ay += (0.16 + 0.02*k) * math.sin(2*math.pi*fr*t + phase + 0.7)
            az += (0.14 + 0.02*k) * math.sin(2*math.pi*fr*t + phase + 1.2)
        w.writerow([gx, gy, gz, ax, ay, az])

print(out)
