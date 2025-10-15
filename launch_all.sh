#!/bin/bash
# FireDrone 전체 시스템 실행 스크립트

# 색상 정의
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}🔥 FireDrone 시스템 시작${NC}"
echo -e "${GREEN}========================================${NC}"

# ROS 환경 설정
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://localhost:11311

# 1. ROS Master 확인
echo -e "\n${YELLOW}[1/4] ROS Master 확인 중...${NC}"
if ! pgrep -x rosmaster > /dev/null; then
    echo -e "${RED}ROS Master가 실행되지 않았습니다.${NC}"
    echo -e "${YELLOW}새 터미널에서 'roscore'를 실행하세요.${NC}"
    exit 1
fi
echo -e "${GREEN}✓ ROS Master 실행 중${NC}"

# 2. MAVROS 확인
echo -e "\n${YELLOW}[2/4] MAVROS 확인 중...${NC}"
if ! rostopic list 2>/dev/null | grep -q "/mavros"; then
    echo -e "${RED}MAVROS가 실행되지 않았습니다.${NC}"
    echo -e "${YELLOW}새 터미널에서 다음 명령을 실행하세요:${NC}"
    echo -e "roslaunch mavros px4.launch fcu_url:=/dev/ttyACM0:921600"
    exit 1
fi
echo -e "${GREEN}✓ MAVROS 연결됨${NC}"

# 3. YOLO 브릿지 실행 (백그라운드)
echo -e "\n${YELLOW}[3/4] YOLO 브릿지 실행 중...${NC}"
python3 yolo_bridge.py &
YOLO_PID=$!
echo -e "${GREEN}✓ YOLO 브릿지 실행 (PID: $YOLO_PID)${NC}"
sleep 2

# 4. 메인 드롭 컨트롤러 실행
echo -e "\n${YELLOW}[4/4] 메인 드롭 컨트롤러 실행 중...${NC}"
python3 main.py &
MAIN_PID=$!
echo -e "${GREEN}✓ 메인 컨트롤러 실행 (PID: $MAIN_PID)${NC}"

echo -e "\n${GREEN}========================================${NC}"
echo -e "${GREEN}🚁 시스템 실행 완료!${NC}"
echo -e "${GREEN}========================================${NC}"
echo -e "\n${YELLOW}종료하려면 Ctrl+C를 누르세요${NC}"
echo -e "${YELLOW}PID: YOLO=$YOLO_PID, Main=$MAIN_PID${NC}\n"

# Ctrl+C 핸들러
trap "echo -e '\n${RED}시스템 종료 중...${NC}'; kill $YOLO_PID $MAIN_PID 2>/dev/null; exit" INT

# 프로세스가 살아있는 동안 대기
wait

