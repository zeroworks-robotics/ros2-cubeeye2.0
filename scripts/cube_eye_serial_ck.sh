#!/bin/bash -i

# CubeEye2.0 (ROS2) 상/하단 depth 카메라의 시리얼 넘버 확인 스크립트.
# 두 대가 모두 연결된 경우, 한쪽을 USB unbind 해 격리한 상태로 노드를 잠깐 띄워
# 남은 장치의 시리얼을 로그에서 파싱한다. 한 대만 연결된 경우엔 unbind 없이 바로
# 파싱한다. (ROS1 cubeEye_I200D/scripts 버전을 ROS2 로 이식)

# ROS2 워크스페이스 setup.bash (필요시 CUBEEYE_SETUP_BASH 로 덮어쓰기)
SETUP_BASH="${CUBEEYE_SETUP_BASH:-/etc/coga-robotics/cona/setup.bash}"
source "$SETUP_BASH"

# 노드 실행 후 파싱까지 대기할 시간(초)
RUN_SECONDS="${CUBEEYE_RUN_SECONDS:-10}"

# 노드 출력을 받아 파싱할 임시 로그 파일
LOG_FILE="$(mktemp /tmp/cube_eye_serial_ck.XXXXXX.log)"
trap 'rm -f "$LOG_FILE"' EXIT

# 노드를 잠깐 실행해 로그를 받아 시리얼을 파싱한다.
# 호출: run_and_parse <연관배열이름>
run_and_parse() {
    local -n serial_map="$1"
    : > "$LOG_FILE"

    # 별도 프로세스 그룹으로 실행 → 종료 시 그룹 전체에 SIGINT 를 보내 노드를 정리한다.
    # 시리얼 출력(scan)은 autorun 경로에서만 찍히므로 더미 serialnumber 를 넘겨
    # autorun → scan() 을 강제한다. (connect 는 더미라 실패하지만 scan 은 이미 끝남)
    setsid ros2 launch cubeeye_camera cubeeye_camera_auto_launch.py \
        serialnumber:=scan frametype:=6 \
        > "$LOG_FILE" 2>&1 &
    local launch_pid=$!
    sleep "$RUN_SECONDS"
    kill -INT -- "-${launch_pid}" 2>/dev/null
    wait "$launch_pid" 2>/dev/null

    while IFS= read -r line; do
        if [[ "$line" =~ ([0-9]+)\)\ source\ name.*serialNumber\ *:\ *([A-Za-z0-9]+) ]]; then
            local index="${BASH_REMATCH[1]}"
            local serial="${BASH_REMATCH[2]}"
            serial_map["$index"]="$serial"
        fi
    done < "$LOG_FILE"
}

# 지정한 USB 커널 장치를 unbind/bind. 커널 이름이 비어있으면(미연결) 아무것도 안 함.
usb_unbind() { [ -n "$1" ] && echo "$1" | sudo tee /sys/bus/usb/drivers/usb/unbind > /dev/null; }
usb_bind()   { [ -n "$1" ] && echo "$1" | sudo tee /sys/bus/usb/drivers/usb/bind   > /dev/null; }

# 장치 존재 확인 및 KERNEL 이름 추출
if ls -al /dev/bot_upper_depth &> /dev/null; then
    upper_ck=1
    upper_depth_kernel=$(udevadm info -a -n /dev/bot_upper_depth | grep KERNEL== | awk -F'"' '{print $2}')
else
    upper_ck=0
fi

if ls -al /dev/bot_lower_depth &> /dev/null; then
    lower_ck=1
    lower_depth_kernel=$(udevadm info -a -n /dev/bot_lower_depth | grep KERNEL== | awk -F'"' '{print $2}')
else
    lower_ck=0
fi

declare -A upper_serial_map
declare -A lower_serial_map

# lower 시리얼 확인: upper 를 격리(연결돼 있으면 unbind)한 뒤 노드 실행
if [ "$lower_ck" -eq 1 ]; then
    usb_unbind "$upper_depth_kernel"
    run_and_parse lower_serial_map
    usb_bind "$upper_depth_kernel"
fi

# upper 시리얼 확인: lower 를 격리(연결돼 있으면 unbind)한 뒤 노드 실행
if [ "$upper_ck" -eq 1 ]; then
    usb_unbind "$lower_depth_kernel"
    run_and_parse upper_serial_map
    usb_bind "$lower_depth_kernel"
fi

# 연결 상태 출력
if [ "$lower_ck" -eq 1 ] && [ "$upper_ck" -eq 1 ]; then
    echo "Both devices are connected."
elif [ "$lower_ck" -eq 1 ]; then
    echo "Only lower device is connected."
elif [ "$upper_ck" -eq 1 ]; then
    echo "Only upper device is connected."
else
    echo "Neither device is connected."
fi

# 결과 출력
if [ "$upper_ck" -eq 1 ]; then
    for key in $(printf "%s\n" "${!upper_serial_map[@]}" | sort -n); do
        echo "Upper device serials(DEPTH_ONLY_A_SN): ${upper_serial_map[$key]}"
    done
fi

if [ "$lower_ck" -eq 1 ]; then
    for key in $(printf "%s\n" "${!lower_serial_map[@]}" | sort -n); do
        echo "Lower device serials(DEPTH_ONLY_B_SN): ${lower_serial_map[$key]}"
    done
fi
