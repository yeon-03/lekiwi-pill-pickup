#!/usr/bin/env python3
"""각 시리얼 포트에 무엇이 붙어 있는지 알아낸다.

라이다: 230400 에서 시작 명령(A5 60)을 보내면 데이터가 쏟아진다.
서보  : 1 Mbps 에서 ping 에 응답한다.
"""
import sys, time, glob
import serial

ports = sys.argv[1:] or sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))

for p in ports:
    print("=== %s" % p)
    # 1) 라이다인가
    try:
        s = serial.Serial(p, 230400, timeout=0.5)
        s.reset_input_buffer()
        s.write(bytes([0xA5, 0x60])); s.flush()
        time.sleep(1.2)
        n = s.in_waiting
        d = s.read(min(n, 64)) if n else b""
        s.write(bytes([0xA5, 0x65])); s.flush()
        s.close()
        print("   라이다 시험(230400): %d 바이트  %s"
              % (n, d[:16].hex(" ") if d else "-"))
    except Exception as e:
        print("   라이다 시험 실패: %s" % e)
    # 2) 그냥 열어두면 뭐가 오나 (자발 송신 장치 탐지)
    for baud in (1000000, 115200):
        try:
            s = serial.Serial(p, baud, timeout=0.5)
            s.reset_input_buffer(); time.sleep(0.6)
            n = s.in_waiting; s.close()
            print("   무입력 수신(%d): %d 바이트" % (baud, n))
        except Exception as e:
            print("   무입력 수신(%d) 실패: %s" % (baud, e))
