import rclpy
import DR_init
from std_msgs.msg import Float64MultiArray
import threading


# 로봇 설정 상수 (필요에 따라 수정)
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight" #워크셀에 등록된 툴 무게
ROBOT_TCP = "GripperDA_v1" #워크셀에 등록된 그리퍼 이름

# TOPIC SUBSCRIBE를 위한 변수
latest_force = None     # [fx, fy, fz, mx, my, mz]
latest_posx  = None     # [x, y, z, rx, ry, rz]

# TOPIC SUBSCRIBE를 위한 함수
def force_callback(msg):
    global latest_force
    latest_force = msg.data

def posx_callback(msg):
    global latest_posx
    latest_posx = msg.data

def spin_thread(node):
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    executor.spin()

# DR_init 설정
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

def initialize_robot():
    """로봇의 Tool과 TCP를 설정"""
    from DSR_ROBOT2 import set_tool, set_tcp  # 필요한 기능만 임포트

    # Tool과 TCP 설정
    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)

    # 설정된 설정값 출력
    # print("#"*50)
    print("Initializing robot with the following settings:")
    print(f"ROBOT_ID: {ROBOT_ID}")
    print(f"ROBOT_MODEL: {ROBOT_MODEL}")
    print(f"ROBOT_TCP: {ROBOT_TCP}")
    print(f"ROBOT_TOOL: {ROBOT_TOOL}")
    print("#"*50)
    
# 리퍼 열기 함수
def open_gripper():
    from DSR_ROBOT2 import(set_digital_output,ON,OFF,wait)
    set_digital_output(1, OFF)
    wait(1.00)
    set_digital_output(2, ON)

def insert_card_slot():
    from DSR_ROBOT2 import (
    movel,
    posx,
    wait,
    task_compliance_ctrl,
    release_compliance_ctrl,
    set_stiffnessx,
    set_desired_force,
    release_force,
    DR_MV_MOD_REL,
    DR_FC_MOD_ABS,
    DR_QSTOP,
    drl_script_stop
)

    FZ_CONTACT = 6.0          # 바닥 접촉 판단
    SLIDE_DIST = 60.0         # 최대 +X 이동 거리 (mm)
    SLIDE_STEP = 5.0          # 한 번에 x축으로 아동할 거리
    FZ_DELTA_TH = 8.0         # 슬롯 입구 감지 힘 변화
    TY_DELTA_TH = 0.2         # 슬롯 입구 감지 토크 변화
    TILT_ANGLE = 10.0         # 그리퍼 기울기 (deg),-
    PIVOT_LEN = 5.0        # PIVOT_LEN = TCP_Z_offset + 카드 두께/2,
    X_SLIDE = 6.1         # sin10(deg) * 6.1(mm)((물체 길이 - 그리퍼가 잡고 있는 부분)
    START_POSE = posx(315.058,-241.698,128.128,146.081,-179.672,146.604)
    
    
        # 1. 시작점 도착
    movel(START_POSE, v=50, a=50)
    movel([0,0,-20,0,0,0],v =20,a=20,mod=DR_MV_MOD_REL)

    # 2. 힘제어 시작
    task_compliance_ctrl()
    set_stiffnessx([1000,1000,300,100,100,100])

    set_desired_force(
        [0, 0, -20, 0, 0, 0],
        [0, 0, 1, 0, 0, 0],
        mod=DR_FC_MOD_ABS
    )
    wait(0.2)

   
    # 3. Z 하강 → fz ≥ 6N

    while True:
        if latest_force is None:
            wait(0.2)

        fz = latest_force[2]

        if fz >= FZ_CONTACT:
            # 즉시 정지
            drl_script_stop(DR_QSTOP)
            break
                
    # 4.힘 제어 해제
    release_force(time=0.1)
    release_compliance_ctrl()
    wait(1.0)
    
    
    # 5. 그리퍼 기울임 (+X 방향)
    movel(posx(0, 0, PIVOT_LEN, 0, 0, 0), v=10, a=10, mod=DR_MV_MOD_REL)
    movel(posx(0, 0, 0, 0, TILT_ANGLE, 0), v=1, a=1, mod=DR_MV_MOD_REL)
    movel(posx(X_SLIDE, 0, -PIVOT_LEN, 0, 0, 0), v=10, a=10, mod=DR_MV_MOD_REL)

    wait(0.1)
    
    # 힘 제어
    task_compliance_ctrl()
    set_stiffnessx([1000,1000,300,100,100,100])

    set_desired_force(
        [0, 0, -10, 0, 0, 0],
        [0, 0, 1, 0, 0, 0],
        mod=DR_FC_MOD_ABS
    )
    wait(0.01)
    
    # 6. +X 방향 이동 (슬롯 탐색)
    while latest_force is None or latest_posx is None:
        wait(0.01)
        prev_fz = latest_force[2]
        prev_ty = latest_force[4]
        prev_z  = latest_posx[2]

    moved = 0.0
    while moved < SLIDE_DIST:

        movel(posx(SLIDE_STEP, 0, 0, 0, 0, 0),
              v=10, a=10, mod=DR_MV_MOD_REL)

        moved += SLIDE_STEP

        fz = latest_force[2]
        ty = latest_force[4]
        z  = latest_posx[2]

        # 슬롯 입구 감지 (힘 또는 토크 급변)
        if abs(fz - prev_fz) > FZ_DELTA_TH or \
        abs(ty - prev_ty) > TY_DELTA_TH or \
        abs(z - prev_z)  >= 5:
            drl_script_stop(DR_QSTOP)
            break

        prev_fz = fz
        prev_ty = ty
        prev_z  = z

    # 7. 그리퍼 다시 세움
    movel(posx(0, 0, PIVOT_LEN, 0, 0, 0), v=5, a=5, mod=DR_MV_MOD_REL)
    movel(posx(0, 0, 0, 0, -TILT_ANGLE, 0), v=5, a=5, mod=DR_MV_MOD_REL)
    movel(posx(-X_SLIDE, 0, -PIVOT_LEN, 0, 0, 0), v=5, a=5, mod=DR_MV_MOD_REL)
    wait(1.0)
    
    # 8. 삽입
    set_desired_force(
        [0, 0, -15, 0, 0, 0],
        [0, 0, 1, 0, 0, 0],
        mod=DR_FC_MOD_ABS
    )
    wait(0.2)

    # 완잔히 바닥면에 닿을 때 까지 지속
    while True:
        if latest_force is None:
            wait(0.01)

        if latest_force[2] >= 8:
            break

    release_force(time=0.1)
    release_compliance_ctrl()

    # z상승
    movel(posx(0,0,20,0,0,0),v =50,a=50,mod=DR_MV_MOD_REL)

    # 그리퍼 열기
    open_gripper()
    

def main(args=None):
    """메인 함수: ROS2 노드 초기화 및 동작 수행"""
    rclpy.init(args=args)
    # 두 개의 노드 생성
    # 1. 제어 명령용 클라이언트 노드 (DSR_ROBOT2 내부에서 사용)
    node_client = rclpy.create_node("insert_card_slot_client", namespace=ROBOT_ID)
    
    # 2. Subscribe용 노드 (백그라운드 스레드에서 spin)
    node_sub = rclpy.create_node("insert_card_slot_sub", namespace=ROBOT_ID)

    # DR_init에 클라이언트 노드 설정 (DSR 라이브러리 함수들이 사용함)
    DR_init.__dsr__node = node_client

    # 힘 topic subscribe (Sub 노드에 등록)
    node_sub.create_subscription(
        Float64MultiArray,
        "/tool_force",
        force_callback,
        10
        )
    # 위치 값 subscribe (Sub 노드에 등록)
    node_sub.create_subscription(
        Float64MultiArray,
        "/current_posx",
        posx_callback,
        10
    )

    # Subscribe 노드를 별도 스레드에서 spin
    threading.Thread(
        target=spin_thread,
        args=(node_sub,),
        daemon=True
    ).start()


    # 초기화는 한 번만 수행
    initialize_robot()
    # 작업 수행
    try:
        insert_card_slot()
    except Exception as e:
        print(f"Error during insertion: {e}")
    finally:
        # 종료 처리
        node_client.destroy_node()
        node_sub.destroy_node()
        rclpy.shutdown()
