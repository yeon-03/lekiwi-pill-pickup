"""LeKiwi Pi 온보딩용 네트워크/SSH 계층.

순수 로직(`kit_rules` 등)은 Qt 없이 stdlib 만 쓰고 단위 테스트 대상이다.
프로세스/워커 계층은 `services/` 최상위의 Qt 모듈이 담당한다.

설계 전문: `.agent/LEKIWI.md` §6.
"""
