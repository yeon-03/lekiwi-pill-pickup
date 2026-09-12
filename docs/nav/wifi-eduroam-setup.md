# LeKiwi 무선(eduroam) 연결 절차

랜선으로 연결된 상태에서 로봇을 학교 Wi-Fi(eduroam)에 붙이는 방법이다.
붙고 나면 랜선을 뽑아도 되므로, 로봇이 자유롭게 움직일 수 있다.

**왜 랜선이 먼저 필요한가**: 무선 주소는 DHCP로 받는다. 로봇이 직접 알려주기
전에는 그 주소를 알 방법이 없다. 랜선은 "네 주소가 뭐냐"고 물어볼 통로다.

---

## 0단계 — 노트북에서 자기 eduroam 설정 확인

로봇에 넣을 값을 **자기 노트북의 실제 동작 설정에서** 가져온다. 추측하면 안 된다.

```bash
nmcli con show eduroam | grep 802-1x
```

이런 값이 나온다.

```
802-1x.eap:          peap
802-1x.identity:     2131139@hansung.ac.kr
802-1x.phase2-auth:  gtc
```

> ⚠️ **반드시 본인 학번·비밀번호를 쓸 것.** 로봇 SD카드에 평문으로 저장되고,
> 그 카드는 사람들 사이를 오간다. 남의 계정을 넣으면 안 된다.
> 작업이 끝나면 6단계 아래의 "정리" 를 볼 것.

---

## 1단계 — 유선으로 로봇 찾기

노트북에서 인터넷 공유가 켜져 있어야 한다
(Settings → Network → Wired → ⚙️ → IPv4 → `Shared to other computers`).
그러면 노트북이 `10.42.0.1` 이 되고 로봇은 `10.42.0.x` 를 받는다.

```bash
nmap -sn 10.42.0.0/24
```

`10.42.0.1` 은 노트북 자신이고, **다른 하나가 로봇**이다 (예: `10.42.0.142`).

---

## 2단계 — SSH 접속

```bash
ssh-keygen -R 10.42.0.142        # 호스트키 경고 예방 (SD카드를 바꿨으면 반드시 뜬다)
ssh roboseasy@10.42.0.142        # 비밀번호: 스페이스 한 칸
```

---

## 3단계 — 기존 설정 백업 (로봇 안에서)

```bash
sudo cp /etc/netplan/50-cloud-init.yaml /etc/netplan/50-cloud-init.yaml.bak
```

---

## 4단계 — eduroam 설정 쓰기

**`<학번>` 과 `<비밀번호>` 두 곳만 본인 것으로 바꾼다.**

```bash
sudo tee /etc/netplan/50-cloud-init.yaml > /dev/null <<'EOF'
network:
  version: 2
  ethernets:
    eth0:
      optional: true
      dhcp4: true
  wifis:
    wlan0:
      optional: true
      dhcp4: true
      dhcp6: false
      dhcp4-overrides:
        route-metric: 50
      regulatory-domain: "KR"
      access-points:
        "eduroam":
          auth:
            key-management: eap
            method: peap
            identity: "<학번>@hansung.ac.kr"
            password: "<비밀번호>"
EOF

sudo chmod 600 /etc/netplan/50-cloud-init.yaml
```

### `route-metric: 50` 이 왜 중요한가

이게 없으면 **케이블이 꽂혀 있는 동안 무선 주소로 접속이 안 된다.**

기본 경로가 둘이면 metric 이 낮은 쪽이 이긴다. 유선이 기본 100 이라 그냥 두면
유선이 이기고, 이런 일이 생긴다.

```
상대 ──SYN──▶ 로봇 wlan0           도착은 한다
로봇 ──SYN/ACK──▶ 유선(eth0) 으로 나감  →  엉뚱한 데로 가서 버려진다
```

들어온 길과 나가는 길이 달라서 연결이 성립하지 않는다. 실제로 이것 때문에
노트북 두 대가 한 로봇에 붙었을 때 한쪽이 로봇을 아예 못 찾은 적이 있다.
`50` 으로 두면 무선이 이겨서, 케이블이 꽂힌 채로도 무선 접속이 유지된다.

`phase2-auth: gtc` 를 netplan 에 적는 항목은 없지만, wpa_supplicant 가
서버가 제시하는 방식을 스스로 협상하므로 이대로 붙는다 (실측 확인).

---

## 5단계 — 문법 검사 후 적용

```bash
sudo netplan generate     # 오류가 없으면 아무것도 출력하지 않는다
sudo netplan apply
```

30초쯤 기다린다. 유선 설정은 건드리지 않았으므로 SSH 는 유지된다.

---

## 6단계 — 붙었는지 확인

```bash
iw dev wlan0 link         # SSID: eduroam 이 보여야 한다
hostname -I               # 주소가 두 개 나온다
```

```
10.42.0.142  223.194.138.173
             ↑ 이것이 무선 주소
```

**이 무선 주소를 기록해서 공유할 것.** 재부팅하면 바뀔 수 있다.

---

## 7단계 — 무선으로 접속

노트북에서:

```bash
ssh roboseasy@223.194.138.173
```

접속되면 **랜선을 뽑아도 된다.**

---

## 정리 (작업이 끝나면)

로봇에 남은 개인 계정을 지운다.

```bash
sudo cp /etc/netplan/50-cloud-init.yaml.bak /etc/netplan/50-cloud-init.yaml
sudo netplan apply
```

---

## 잘 안 될 때

| 증상 | 조치 |
|---|---|
| `nmap` 에 로봇이 안 잡힘 | 첫 부팅 2~3분 대기. 노트북 인터넷 공유가 켜져 있는지 확인 |
| 호스트키 경고 | `ssh-keygen -R <IP>` — SD카드를 바꿨으면 정상이다 |
| `iw dev wlan0 link` → `Not connected` | 학번·비밀번호 오타. `sudo journalctl -u wpa_supplicant -n 30` 확인 |
| 주소가 하나만 나옴 | 인증은 됐는데 DHCP 실패. `sudo netplan apply` 한 번 더 |
| 무선 주소로 접속 안 됨 | `route-metric: 50` 이 빠졌는지 확인 (4단계) |
| 무선 스캔 결과 없음 | `sudo rfkill unblock wifi` 후 재시도 |
| 되돌리기 | 위 "정리" 참고 |

---

## 노트북 두 대가 한 로봇에 붙을 때

| | 동시 사용 | 이유 |
|---|---|---|
| SSH 접속, 로그 보기 | ✅ 괜찮다 | 읽기만 한다 |
| ZMQ 카메라 스트림 | ❌ 안 된다 | 한 host 에 클라이언트 하나만. 둘이 붙으면 프레임을 나눠 가져 둘 다 끊긴다 |
| 서보 버스 제어 | ❌ 안 된다 | 반이중이라 두 쪽이 열면 통신이 깨진다 |
| ROS2 노드 | ⚠️ 조건부 | 구독·모니터링은 괜찮지만 두 곳에서 `/cmd_vel` 을 쏘면 서로 싸운다 |
| 네트워크 설정 변경 | ❌ 위험 | 한 쪽만 할 것 |

**보는 것은 같이, 조종하는 것은 한 쪽만.**

---

## 참고: 고정 IP 는 왜 안 쓰는가

킷 기본 설정은 `192.168.0.20N` 고정 IP 다. 이것은 `192.168.0.x` 대역
(연구실 공유기)에서만 동작한다. eduroam 같은 다른 망에 고정 모드로 붙으면
**연결은 되지만 통신이 안 된다.** 그래서 이 문서는 `dhcp4: true` 를 쓴다.

연구실로 돌아가면 백업으로 되돌리거나 `setip <킷번호>` 로 고정 IP 를 되살린다.
