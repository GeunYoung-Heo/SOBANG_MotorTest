# ASMR_ActuatorTest 작업 메모 (Claude Code 세션용)

- 결과 보고와 중간 진행 메모는 모두 **한국어**로 한다.
- 팀원용 문서는 노션에 있다. 루트 README.md는 노션 링크만 둔다.

## Radxa (로봇 온보드 컴퓨터) 접속

| 항목 | 값 (2026-11-12 확인) |
|---|---|
| 보드 | Radxa ROCK3 Model C (ROCK 3C), aarch64, 4코어, RAM 1.9 GB, 저장공간 59 GB |
| OS | Debian 12 (bookworm), 커널 6.1.84-10-rk2410-nocsf, Python 3.11.2 |
| 호스트 이름 | `rock-3c` → 노트북에서 `rock-3c.local`(mDNS)로 찾아짐 |
| 계정 | 사용자 `radxa`. 이 노트북에서는 SSH 키로 비밀번호 없이 접속됨. 비밀번호는 저장소 루트의 `radxa.local.env`에 있음 (git에 안 올라가는 파일) |
| 호스트 키 | ed25519 `SHA256:rJpJZSgPHMewFTAYWLRMeWG6x0xYA1CkbkcEvsRYIwc` (다르면 다른 기기) |

공개 저장소이므로 비밀번호는 이 파일에 쓰지 않는다. `radxa.local.env`에만 둔다.

### 연결 구성

- 노트북 유선랜(Windows 어댑터 이름 `이더넷`)과 Radxa(`end1`)를 랜선으로 직접 연결했다.
- 노트북에서 **인터넷 연결 공유(ICS)** 를 켜 두었다 (Wi-Fi를 이더넷으로 공유).
  - 노트북: 192.168.137.1
  - Radxa: **고정 IP 192.168.137.188/24** (게이트웨이·DNS 192.168.137.1). 2026-11-20에 NetworkManager 프로필 `Wired connection 1`을 manual로 바꿨다. 노트북 ICS의 DHCP가 응답하지 않아 Radxa가 주소를 못 받았기 때문이다.
  - 다른 네트워크(학교 공유기 등)에 꽂으려면 DHCP로 되돌린다: `sudo nmcli con mod "Wired connection 1" ipv4.method auto ipv4.addresses "" ipv4.gateway "" ipv4.dns "" && sudo nmcli con up "Wired connection 1"`
- Radxa는 노트북을 거쳐 인터넷과 DNS를 쓴다. 2026-11-20에 한때 Wi-Fi가 "공유됨"으로 표시되는데도 ICS의 NAT·DNS·DHCP가 모두 응답하지 않았다. 사용자가 공유를 껐다 켜니 다시 됐다 (ping 8.8.8.8, DNS, HTTPS, NTP 동기화 확인). 노트북을 재부팅한 뒤 Radxa 인터넷이 안 되면 같은 방법을 쓴다 (아래 '연결이 안 될 때' 2). 고정 IP라 이때도 Radxa 접속 자체는 된다.

### 접속 (PowerShell, Git Bash 둘 다)

이 노트북에는 SSH 키 인증이 설정되어 있다 (2026-11-12). 비밀번호를 묻지 않는다.

```powershell
ssh radxa                          # 대화형 접속 (별칭: radxa, rock-3c, rock-3c.local)
ssh radxa "uname -a; ip -br addr"  # 명령 하나 실행하고 끝
```

파일 복사 예:

```powershell
scp -r "$env:USERPROFILE\Desktop\ASMR_ActuatorTest\MotorTestGUI" radxa:~/
scp radxa:/etc/os-release .
```

노트북 쪽 설정:
- 전용 키: `~\.ssh\id_ed25519_radxa`. 암호 없음. 지문은 `SHA256:Jc6Uudnb3lv5dwLEP8S9lDSQEbEmutz9UqNkMFT7pto`이고, Radxa `~/.ssh/authorized_keys`에 등록되어 있다.
- `~\.ssh\config`에 `Host radxa rock-3c rock-3c.local` 항목이 있다 (HostName **192.168.137.188**, User radxa, IdentitiesOnly yes). 처음 추가하기 전 파일은 `config.bak_before_radxa`, HostName을 rock-3c.local에서 고정 IP로 바꾸기 전 파일은 `config.bak_before_static_ip`로 백업했다 (2026-11-20, mDNS 이름 찾기가 자주 실패해서).
- `~\.ssh\known_hosts`에 `rock-3c.local`과 `192.168.137.188`의 호스트 키가 들어 있다.
- 다른 PC에서는 그 PC의 키를 따로 등록해야 한다.

### Claude 세션에서 명령을 실행할 때

- `ssh -o BatchMode=yes -o ConnectTimeout=8 radxa "명령"`을 쓴다. BatchMode 덕분에 비밀번호 창에서 멈추지 않는다. 명령마다 새로 접속해도 된다.
- 길거나 여러 줄인 작업은 `ssh radxa 'bash -s' < 스크립트파일`로 보낸다.
- Windows에 들어 있는 `ssh-keyscan`(OpenSSH_for_Windows 9.5)은 Radxa의 키 교환 방식(sntrup761)을 몰라서 실패한다. 호스트 키를 다시 받아야 하면 Git Bash의 `ssh-keyscan`(OpenSSH 10.2)을 쓴다. `ssh`/`scp`는 Windows 것도 문제없다.
- 키 인증이 안 될 때만 paramiko + 비밀번호로 접속한다.
  - paramiko는 세션 scratchpad에 별도 venv를 만들어 설치한다. `odrv051` 가상환경에는 설치하지 않는다.
  - 접속 정보는 `radxa.local.env`에서 읽는다.
  - 한 연결에서 `exec_command`는 한 번만 쓴다. 두 번째 채널을 열면 연결이 끊겼다 (2026-11-12에 두 번 재현). 여러 명령은 `bash -s`로 한 번에 보낸다.
- Radxa에서 패키지 설치, 네트워크·시스템 설정 변경, 재부팅은 **사용자에게 먼저 묻는다**. 상태를 읽는 명령만 바로 실행해도 된다.
- Radxa `~/.ssh/authorized_keys`에는 이 노트북 키 말고도 2026-11-11에 등록된 키가 하나 더 있다. 지우지 않는다.

### 연결이 안 될 때

1. 노트북 이더넷 주소를 확인한다.

   ```powershell
   Get-NetIPAddress -InterfaceAlias "이더넷" -AddressFamily IPv4
   ```

   192.168.137.1이 아니라 169.254.x.x라면 공유가 이더넷에 적용되지 않은 것이다. 2026-11-12에 처음 설정했을 때도 그랬고, 다시 적용해서 해결했다.
2. 공유를 다시 적용한다 (노트북 설정 변경이라 사용자가 직접 한다).
   1. `Win + R`을 누르고 `ncpa.cpl` 입력 → Enter.
   2. **Wi-Fi**를 오른쪽 클릭 → **속성** → **공유** 탭.
   3. "다른 네트워크 사용자가 이 컴퓨터의 인터넷 연결을 통해 연결할 수 있도록 허용"의 체크를 풀고 **확인**.
   4. 같은 곳에서 다시 체크하고, "홈 네트워킹 연결"에서 **이더넷**을 고른 뒤 **확인**.
   5. 랜선을 뺐다 꽂거나 Radxa를 재부팅한다.
3. Radxa 주소를 찾는다. `Resolve-DnsName rock-3c.local`을 실행하거나, 192.168.137.2~254로 ping을 돌린다. Radxa 유선 MAC은 `9E-35-EF-B1-FC-E1`이다.
4. 공유 없이 랜선만 꽂으면 DHCP가 없어서 Radxa가 IPv6 링크 로컬 주소(`fe80::5507:7849:6547:982`)로 잠깐씩만 보인다. 2026-11-20부터는 고정 IP라 DHCP와 상관없이 192.168.137.188로 보여야 한다.
5. IPv4가 안 될 때의 비상 접속: IPv6 링크 로컬로 접속된다 (Radxa가 보이는 순간에만). `ssh -i ~/.ssh/id_ed25519_radxa -o IdentitiesOnly=yes -o HostKeyAlias=rock-3c.local radxa@fe80::5507:7849:6547:982%12` (12 = 노트북 '이더넷' ifIndex, `(Get-NetAdapter -Name "이더넷").ifIndex`로 확인).

### 현재 상태 (2026-11-12)

- CAN 인터페이스 없음. USB 장치도 없음 (CANable은 노트북에 꽂혀 있음).
- python-can 설치 안 됨.
- 시간은 NTP로 맞춰져 있고, 시간대는 UTC.

### MotorTestGUI의 Radxa 경유 모드 (2026-11-20 추가)

- GUI 위쪽 **Radxa 경유**를 켜고 연결하면, GUI가 `MotorTestGUI/radxa/can_bridge.py`를 Radxa `~/motor_bridge/`에 복사(`ssh radxa "cat > ..."`)하고 `ssh -L`로 실행한다. 중계는 Radxa의 `/tmp/motortestgui_*.sock` 유닉스 소켓에서만 받는다 (포트를 열지 않음).
- 중계는 표준 라이브러리만 쓴다 (설치 없음). CANable은 `/dev/ttyACM*` 중 USB 16d0:117e를 자동으로 찾아 slcan으로 연다. `--socketcan can0`도 지원.
- 안전: GUI에서 0.5 s 동안 아무것도 안 오거나, 연결·SSH가 끊기면 본 적 있는 모든 node에 IDLE을 보낸다.
- 2026-11-20: radxa 계정을 `dialout` 그룹에 넣었다 (`/dev/ttyACM*` 접근용, 사용자 허락받음). 고정 IP로 바꾼 뒤 실제 Radxa에서 SSH 경로를 시험했다 (CAN 장치 없이 `--allow-empty`): 복사 0.4 s, 왕복 1.2 ms, 끊으면 중계 프로그램과 소켓 정리됨. 같은 날 CANable 두 개를 Radxa에 꽂고 실기 확인: `/dev/ttyACM0` = 일련번호 …6C4845 (오른다리, 노트북에서는 COM17), `/dev/ttyACM1` = …884845 (왼다리, COM16). Radxa 경유로 12개 heartbeat 10 Hz 누락 없음, GUI 방식 폴링 50 Hz, 엔코더 요청→응답 평균 4.7 ms (직결 약 0.5 ms), 버스 점유 15%씩, 송신 실패 0.

### IMU (2026-11-23)

- 로봇의 IMU는 **BMI088** (Grove 6-Axis)이다. 헤더 핀 1 (3.3 V), 3 (I2C3 SDA), 5 (I2C3 SCL), 6 (GND)에 꽂혀 있다. `/dev/i2c-3`에서 가속도 0x19, 자이로 0x69.
- 사용자 허락을 받고 바꾼 것: 오버레이 `rk3568-i2c3-m0` 켬 (`sudo rsetup enable_overlays rk3568-i2c3-m0.dtbo` + 재부팅), smbus2 0.6.1을 radxa 계정 `~/.local`에 설치 (인터넷이 없어 노트북에서 받은 wheel로). numpy 1.24.2는 원래 있었고 radxa는 원래 i2c 그룹이었다. i2c-tools(i2cdetect)는 없다.
- sudo는 비밀번호가 필요하다. `radxa.local.env`의 값을 화면에 내지 않고 stdin으로 넘긴다: `grep '^RADXA_PASSWORD=' radxa.local.env | cut -d= -f2- | tr -d '\r\n' | ssh radxa 'sudo -S -p "" 명령'`.
- I2C3는 기본 100 kHz라 `imu_reader.py`의 BMI088 필터가 약 300 Hz로 돈다. MotorTestGUI의 IMU 사용법은 `MotorTestGUI/README.md` 11절.
- 같은 날 Radxa 인터넷이 안 됐다 (노트북 ICS). 시계도 NTP를 못 맞춰 3일 늦었다 (Nov 20으로 표시).
