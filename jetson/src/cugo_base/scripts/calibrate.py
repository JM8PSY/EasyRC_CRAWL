#!/usr/bin/env python3
"""cugo_calibrate のラッパー（インストール後は `ros2 run cugo_base cugo_calibrate`）。

既定はドライラン。実際に動かすには --yes（要立ち会い）。詳細は cugo_base/calibrate.py。
"""
import sys

from cugo_base.calibrate import main

if __name__ == '__main__':
    sys.exit(main())
