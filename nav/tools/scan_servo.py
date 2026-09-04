#!/usr/bin/env python3
"""서보 버스를 스캔한다.  scan_servo.py [포트...]

바퀴는 ID 7·8·9, 팔은 1~6.  응답 없는 ID 는 배선/전원 문제다.
"""
import sys
from scservo_sdk import PacketHandler, PortHandler
ports = sys.argv[1:] or ["/dev/ttyACM0", "/dev/ttyACM1"]
for pname in ports:
    p = PortHandler(pname); h = PacketHandler(0)
    if not p.openPort():
        print("  %s  열 수 없음" % pname); continue
    p.setBaudRate(1000000)
    hit, miss = [], []
    for i in range(1, 13):
        _, res, err = h.ping(p, i)
        (hit if res == 0 else miss).append(i)
    p.closePort()
    print("  %-14s 응답 %-22s 무응답 %s"
          % (pname, ",".join(map(str, hit)) or "없음", ",".join(map(str, miss))))
