import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Float64MultiArray, Float64, String, Int32
from dsr_msgs2.srv import MovePause, MoveResume, MoveStop

from random import random
import threading
import os
import yaml
import time
from ament_index_python.packages import get_package_share_directory

import DR_init

# =========================================================
# 0. Global Setup & Initialization
# =========================================================

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP  = "GripperDA_v1"

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# Global Config Storage
CONFIG = {}

def load_config():
    """Loads the pose_config.yaml file into the global CONFIG dictionary."""
    global CONFIG
    try:
        config_path = os.path.expanduser("./src/m0609_semicon_task_handler/config/pose_config.yaml")
        
        if not os.path.exists(config_path):
            print(f"Warning: Source config not found at {config_path}. Trying package share...")
            pkg_path = get_package_share_directory('m0609_semicon_task_handler')
            config_path = os.path.join(pkg_path, 'config', 'pose_config.yaml')

        print(f"Loading configuration from: {config_path}")
        with open(config_path, 'r') as f:
            CONFIG.update(yaml.safe_load(f))
        print("Configuration loaded successfully.")
    except Exception as e:
        print(f"ERROR: Failed to load configuration: {e}")
        # Initialize with empty or defaults to prevent crash, but operations will fail
        CONFIG.clear()

def initialize_robot():
    from DSR_ROBOT2 import set_tool, set_tcp, set_robot_mode, ROBOT_MODE_AUTONOMOUS
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)

    print("#" * 50)
    print("Initializing robot with the following settings:")
    print(f"ROBOT_ID: {ROBOT_ID}")
    print(f"ROBOT_MODEL: {ROBOT_MODEL}")
    print(f"ROBOT_TCP: {ROBOT_TCP}")
    print(f"ROBOT_TOOL: {ROBOT_TOOL}")
    print("#" * 50)

# =========================================================
# 1. Global Variables & Callbacks (for Sensing)
# =========================================================

latest_force = None     # [fx, fy, fz, mx, my, mz]
latest_posx  = None     # [x, y, z, rx, ry, rz]

def force_callback(msg):
    global latest_force
    latest_force = msg.data

def posx_callback(msg):
    global latest_posx
    latest_posx = msg.data

# ========== Task Control Globals ==========
task_running = False
task_paused = False
motion_paused = False  # Track if robot motion is paused

# Threading event for pause/resume synchronization
pause_event = threading.Event()
pause_event.set()  # Initially not paused (set = can proceed)

# Service clients (initialized in main)
_move_pause_cli = None
_move_resume_cli = None
_move_stop_cli = None
_task_node = None
_move_recover_cli = None

def call_move_pause():
    """로봇 모션 일시정지 (move_pause 서비스 호출 - 비동기)"""
    global _move_pause_cli, _task_node, motion_paused
    if _move_pause_cli is None or _task_node is None:
        print("[WARN] move_pause client not initialized")
        return False
    try:
        req = MovePause.Request()
        future = _move_pause_cli.call_async(req)
        # callback에서는 spin_until_future_complete 사용 불가 (spin 충돌)
        # 비동기로 호출하고 바로 상태 변경
        motion_paused = True
        print("[MOTION] Pause requested")
        return True
    except Exception as e:
        print(f"[MOTION] Pause error: {e}")
        return False

def call_move_resume():
    """로봇 모션 재개 (move_resume 서비스 호출 - 비동기)"""
    global _move_resume_cli, _task_node, motion_paused
    if _move_resume_cli is None or _task_node is None:
        print("[WARN] move_resume client not initialized")
        return False
    try:
        req = MoveResume.Request()
        future = _move_resume_cli.call_async(req)
        motion_paused = False
        print("[MOTION] Resume requested")
        return True
    except Exception as e:
        print(f"[MOTION] Resume error: {e}")
        return False

def call_move_stop(stop_mode=1):
    """로봇 모션 정지 (move_stop 서비스 호출 - 비동기, stop_mode: 0=SLOW, 1=QUICK)"""
    global _move_stop_cli, _task_node, motion_paused
    if _move_stop_cli is None or _task_node is None:
        print("[WARN] move_stop client not initialized")
        return False
    try:
        req = MoveStop.Request()
        req.stop_mode = stop_mode  # 0: SLOW_STOP, 1: QUICK_STOP
        future = _move_stop_cli.call_async(req)
        motion_paused = False
        print(f"[MOTION] Stop (mode={stop_mode}) requested")
        return True
    except Exception as e:
        print(f"[MOTION] Stop error: {e}")
        return False

def recover_chip():
    RECOVER_ZONE = [528.21, 12.64, 196.22, 82.62, 179.73, 82.97]
    from DSR_ROBOT2 import movel
    """로봇 복구 모드 (move_recover 서비스 호출 - 비동기)"""
    global _move_recover_cli, _task_node, motion_paused
    
    if _move_recover_cli is None or _task_node is None:
        print("[WARN] move_recover client not initialized")
        return False
        
    try:
        req = MoveRecover.Request()
        future = _move_recover_cli.call_async(req)
        motion_paused = False
        print("[MOTION] Recover requested via MoveRecover service")
        
        # 복구 행동
        movel(RECOVER_ZONE, vel=50, acc=50)
        gripper_open()
        print("✅ 칩 복구 위치 도달")
        call_move_resume()
        return True
    except Exception as e:
        print(f"[MOTION] Recover error: {e}")
        return False


def task_command_callback(msg):
    global task_running, task_paused, motion_paused, pause_event
    cmd = msg.data.upper()
    print(f"[{time.strftime('%H:%M:%S')}] Command received: {cmd}", flush=True)
    
    if cmd == "START":
        # START가 눌렸을 때: 일시정지 상태면 resume, 아니면 task 시작
        if motion_paused or task_paused:
            print("[CMD] Motion was paused, resuming...")
            call_move_resume()
            task_paused = False
            pause_event.set()  # Unblock waiting threads
        else:
            task_running = True
            task_paused = False
            pause_event.set()
    elif cmd == "STOP":
        # STOP: 모션 즉시 정지 (QUICK_STOP)
        call_move_stop(1)
        task_running = False
        task_paused = False
        pause_event.set()  # Unblock to allow exit
    elif cmd == "PAUSE":
        # PAUSE: 모션 일시정지
        call_move_pause()
        task_paused = True
        pause_event.clear()  # Block waiting threads
    elif cmd == "RESUME":
        # RESUME: 모션 재개
        call_move_resume()
        task_paused = False
        pause_event.set()  # Unblock waiting threads
    elif cmd == "EMERGENCY_STOP":
        # EMERGENCY: 즉시 정지
        call_move_stop(1)
        task_running = False
        task_paused = False
        pause_event.set()  # Unblock to allow exit

def drl_stop_cmd_callback(msg):
    """외부 정지 명령(/drl_stop_cmd) 수신 시 DRL QSTOP 전송"""
    from DSR_ROBOT2 import (
        drl_script_stop,
        get_drl_state,
        DRL_PROGRAM_STATE_PLAY,
        DRL_PROGRAM_STATE_HOLD,
    )

    if msg.data == 1:
        state = get_drl_state()
        if state in [DRL_PROGRAM_STATE_PLAY, DRL_PROGRAM_STATE_HOLD]:
            print("[WATCHER] DRL 실행 중 → 강제 정지 명령 전송")
            drl_script_stop(1)  # 1: QSTOP
        else:
            print(f"[WATCHER] DRL 정지 상태 (현재: {state})")

def check_task_status():
    """Checks pause/stop state inside task loops using threading.Event."""
    global task_running, task_paused, pause_event
    
    if task_paused:
        print("[CHECK] Task paused, waiting for resume...", flush=True)
    
    # Event.wait() releases GIL, allowing spin_thread to process callbacks
    while not pause_event.wait(timeout=0.1):
        if not task_running:  # Stop requested while paused
            print("[CHECK] Stop requested while paused", flush=True)
            return False
            
    if not task_running:
        print("[CHECK] Task not running", flush=True)
        return False
        
    return True

def spin_thread(node):
    print("[SPIN] spin_thread started with MultiThreadedExecutor")
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    print("[SPIN] executor spinning...")
    executor.spin()
    print("[SPIN] executor stopped")


# =========================================================
# 2. Motion Constants & Functions (from motion_control)
# =========================================================

# --- Speed Settings ---
VEL_LINEAR_FAST = 70
ACC_LINEAR_FAST = 70
VEL_LINEAR_APPROACH = 20
ACC_LINEAR_APPROACH = 20

VEL_JOINT_DEFAULT = 70
ACC_JOINT_DEFAULT = 70

# --- Poses & Waypoints (MAPPED FROM CONFIG) ---
# Accessors will be used inside functions to ensure CONFIG is loaded

def get_config_val(path_str, default=None):
    """Helper to retrieve nested keys 'a.b.c'."""
    keys = path_str.split('.')
    val = CONFIG
    try:
        for k in keys:
            val = val[k]
        return val
    except (KeyError, TypeError):
        print(f"Config Key Missing: {path_str}")
        return default

# --- Helpers for Motion ---

def gripper_open():
    if not check_task_status(): return
    from DSR_ROBOT2 import set_digital_output, wait, ON, OFF
    set_digital_output(2, ON)
    wait(1.00)
    set_digital_output(2, OFF)

def gripper_close():
    if not check_task_status(): return
    from DSR_ROBOT2 import set_digital_output, wait, ON, OFF
    set_digital_output(1, ON)
    wait(1.00)
    set_digital_output(1, OFF)

def move_joint(q):
    if not check_task_status(): return
    from DSR_ROBOT2 import movej, posj
    movej(posj(q), vel=VEL_JOINT_DEFAULT, acc=ACC_JOINT_DEFAULT)

def move_home():
    from DSR_ROBOT2 import movel, posx
    home_cart = get_config_val('poses.home_cartesian')
    if home_cart:
        HOME_P = posx(home_cart)
        movel(HOME_P, vel=VEL_LINEAR_FAST, acc=ACC_LINEAR_FAST)
    else:
        print("Error: Home Cartesian Pose not defined.")

def move_home_joint():
    """Moves to the home position using DSR move_home function."""
    from DSR_ROBOT2 import move_home, DR_HOME_TARGET_MECHANIC
    print("[HOME] Moving to mechanical home position...")
    move_home(DR_HOME_TARGET_MECHANIC)

def move_to_pose(pose_list):
    """Moves to a specific Cartesian pose."""
    from DSR_ROBOT2 import movel, posx
    target = posx(pose_list)
    movel(target, vel=VEL_LINEAR_FAST, acc=ACC_LINEAR_FAST)

def _extract_posx6(p):
    # Case A: p is [x, y, z, rx, ry, rz] (Flat list of floats)
    # Check if p has 6 elements AND the first element is a number (float/int), not a list/tuple
    try:
        if len(p) >= 6 and isinstance(p[0], (int, float)):
             return p[0], p[1], p[2], p[3], p[4], p[5]
    except Exception: pass

    # Case B: p is [[x, y, z, rx, ry, rz], solution_space] (Nested structure)
    try:
        if len(p) >= 1 and hasattr(p[0], '__len__') and len(p[0]) >= 6:
             return p[0][0], p[0][1], p[0][2], p[0][3], p[0][4], p[0][5]
    except Exception: pass
    
    raise RuntimeError(f"Cannot extract 6D pose from get_current_posx(). Data: {p}")

def _wrap_deg(a):
    while a > 180.0: a -= 360.0
    while a < -180.0: a += 360.0
    return a

def _pose_from_current():
    from DSR_ROBOT2 import get_current_posx
    p = get_current_posx()
    x, y, z, A, B, C = _extract_posx6(p)
    return (x, y, z, A, B, C)

def _pose_same(p6):
    from DSR_ROBOT2 import posx
    x, y, z, A, B, C = p6
    return posx([x, y, z, A, B, C])

def _pose_tilt_from_pose(p6, pitch_delta):
    from DSR_ROBOT2 import posx
    x, y, z, A, B, C = p6
    B2 = _wrap_deg(B + pitch_delta)
    return posx([x, y, z, A, B2, C])

def move_rel_x(dx, vel=VEL_LINEAR_APPROACH, acc=ACC_LINEAR_APPROACH):
    if not check_task_status(): return
    from DSR_ROBOT2 import get_current_posx, movel, posx
    p = get_current_posx()
    x, y, z, A, B, C = _extract_posx6(p)
    movel(posx([x + dx, y, z, A, B, C]), vel=vel, acc=acc)


# =========================================================
# 3. Insertion Logic (from insert_card_slot)
# =========================================================

def execute_insertion_at(target_pose_list):
    """
    Moves to target_pose_list and performs insertion.
    Args:
        target_pose_list: [x, y, z, rx, ry, rz] coordinates where insertion starts.
    """
    from DSR_ROBOT2 import (
        movel, posx, wait, task_compliance_ctrl, release_compliance_ctrl,
        set_stiffnessx, set_desired_force, release_force,
        DR_MV_MOD_REL, DR_FC_MOD_ABS, DR_QSTOP, drl_script_stop
    )

    # 0. Constants from Config
    ins_cfg = get_config_val('insertion', {})
    
    FZ_CONTACT = ins_cfg.get('fz_contact', 6.0)
    SLIDE_DIST = ins_cfg.get('slide_dist', 60.0)
    SLIDE_STEP = ins_cfg.get('slide_step', 5.0)
    FZ_DELTA_TH = ins_cfg.get('fz_delta_th', 8.0)
    TY_DELTA_TH = ins_cfg.get('ty_delta_th', 0.2)
    TILT_ANGLE = ins_cfg.get('tilt_angle', 10.0)
    PIVOT_LEN = ins_cfg.get('pivot_len', 5.0)
    X_SLIDE = ins_cfg.get('x_slide', 6.1)
    
    STIFFNESS = ins_cfg.get('stiffness', [1000,1000,300,100,100,100])
    FORCE_DES_CONTACT = ins_cfg.get('force_des_contact', [0, 0, -20, 0, 0, 0])      
    FORCE_DES_SLIDE = ins_cfg.get('force_des_slide', [0, 0, -10, 0, 0, 0])
    FORCE_DES_FINAL = ins_cfg.get('force_des_final', [0, 0, -15, 0, 0, 0])

    print(f"Starting Insertion at {target_pose_list}...")

    # 1. Move to Start Pose
    target_p = posx(target_pose_list)
    movel(target_p, v=50, a=50)   
    
 # Approach (Z -20 relative)
    movel([0,0,-20,0,0,0], v=20, a=20, mod=DR_MV_MOD_REL)

    # 2. Force Control Start
    task_compliance_ctrl()
    set_stiffnessx(STIFFNESS)
    set_desired_force(FORCE_DES_CONTACT, [0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_ABS)
    wait(0.2)

    # 3. Z Descent (Contact Detection)
    # Using global 'latest_force' updated by callback 
    while True:
        if latest_force is None:
            wait(0.2)
            continue

        fz = latest_force[2]
        if fz >= FZ_CONTACT:
            drl_script_stop(DR_QSTOP) # Stop immediately
            break
    
    # 4. Release Force
    release_force(time=0.1)
    release_compliance_ctrl()
    wait(1.0)
    
    # 5. Tilt Gripper
    movel(posx(0, 0, PIVOT_LEN, 0, 0, 0), v= 10, a= 10, mod=DR_MV_MOD_REL)
    movel(posx(0, 0, 0, 0, TILT_ANGLE, 0), v= 1, a= 1, mod=DR_MV_MOD_REL)
    movel(posx(X_SLIDE, 0, -PIVOT_LEN, 0, 0, 0), v= 10, a= 10, mod=DR_MV_MOD_REL)
    wait(0.1)
    
    # 6. Slide & Insert loop
    task_compliance_ctrl()
    set_stiffnessx(STIFFNESS)
    set_desired_force(FORCE_DES_SLIDE, [0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_ABS)
    wait(0.01)

    while latest_force is None or latest_posx is None:
        wait(0.01)
    
    prev_fz = latest_force[2]
    prev_ty = latest_force[4]
    prev_z  = latest_posx[2]

    
    moved = 0.0
    while moved < SLIDE_DIST:
        movel(posx(SLIDE_STEP, 0, 0, 0, 0, 0), v= 10, a= 10, mod=DR_MV_MOD_REL)
        moved += SLIDE_STEP
        
        fz = latest_force[2]
        ty = latest_force[4]
        z  = latest_posx[2]        
        if abs(fz - prev_fz) > FZ_DELTA_TH or abs(ty - prev_ty) > TY_DELTA_TH:
            drl_script_stop(DR_QSTOP)
            break
        prev_fz = fz
        prev_ty = ty

    # 7. Restore & Final Push
    movel(posx(0, 0, PIVOT_LEN, 0, 0, 0), v=5, a=5, mod=DR_MV_MOD_REL)
    movel(posx(0, 0, 0, 0, -TILT_ANGLE, 0), v=5, a=5, mod=DR_MV_MOD_REL)
    movel(posx(-X_SLIDE, 0, -PIVOT_LEN, 0, 0, 0), v=5, a=5, mod=DR_MV_MOD_REL)
    wait(1.0)
    
    set_desired_force(FORCE_DES_FINAL, [0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_ABS)
    wait(0.2)
    
    # Wait for full contact
    while True:
        if latest_force and latest_force[2] >= 8:
            break
        wait(0.1)

    release_force(time=0.1)
    release_compliance_ctrl()

    # Move Up & Open
    movel(posx(0,0,20,0,0,0),v=50, a=50, mod=DR_MV_MOD_REL)
    gripper_open()
    print("Insertion Completed.")


# =========================================================
# 4. Custom Sequence (Mixing Motion & Insert)
# =========================================================

def log_info(step, msg):
    """Helper for consistent logging."""
    print(f"[INFO] [{step}] {msg}")

def perform_task():
    """
    Replicates the original motion_control sequence, BUT using the integrated task_node structure.
    """
    from DSR_ROBOT2 import wait, movel, movej, get_current_posx, get_current_posj, posj # Ensure imports available locally if needed
    
    log_info("Init", "Ready. Waiting for START command...")
    while not task_running:
        time.sleep(1)
        
    log_info("Init", "Starting Task Sequence...")
    
    # Config Variables
    POSJ_HOME = get_config_val('position.posj_home', [])

    JOINT_POSES_SHELF_1 = get_config_val('poses.shelf_1.joint_poses', [])
    OFFSET_S1_APPROACH_X = get_config_val('poses.shelf_1.approach_x_offset', 20.0)
    OFFSET_S1_RETRACT_X = get_config_val('poses.shelf_1.retract_x_offset', -10.0)
    WAIT_GRIPPER_STABILIZE_SEC = get_config_val('poses.shelf_1.wait_stabilize_sec', 0.5)

    JOINT_POSES_SHELF_2 = get_config_val('poses.shelf_2.joint_poses', [])
    WAIT_INSPECTION_SEC = get_config_val('poses.shelf_3_tilt.wait_inspection_sec', 2.0)
    
    ANGLE_S3_TILT_DEG = get_config_val('poses.shelf_3_tilt.angle_deg', 30.0)
    DIR_S3_TILT_SIGN = get_config_val('poses.shelf_3_tilt.direction_sign', -1.0)
    
    Q_WAYPOINT_TRANSITION = get_config_val('poses.transition_waypoint')
    INSERT_TARGET = get_config_val('poses.insert_a_target') # Load Insertion Target

   # --- 1. Homing ---
    log_info("Home", "Moving to Home Joint Pose...")
    movej(posj(POSJ_HOME), v=100, a=100)
    wait(0.3)

    log_info("Init", "Ready. Waiting for START command...")
    while not task_running:
        time.sleep(0.5)
        
    log_info("Init", "Starting Task Sequence...")
    
    # --- 1. Homing ---
    log_info("Home", "Moving to Home Joint Pose...")
    movej(POSJ_HOME, v=100, a=100)
    wait(0.3)
    
    # --- 2. Stage A: Shelf 1 -> Shelf 2 Roundtrip (3 Cycles) ---
    cycle_count = min(len(JOINT_POSES_SHELF_1), len(JOINT_POSES_SHELF_2), 3)
    log_info("Stage A", f"Starting Shelf 1 <-> Shelf 2 Loop ({cycle_count} cycles)")
    
    for i in range(cycle_count):
        if not check_task_status(): return

        step_id = f"A-{i+1}"
        
        # 2.1 Pick from Shelf 1
        log_info(step_id, f"Opening Gripper & Moving to Shelf 1 Pose {i+1}")
        gripper_open()
        wait(0.1)
        
        log_info(step_id, f"Joint Move -> {JOINT_POSES_SHELF_1[i]}")
        move_joint(JOINT_POSES_SHELF_1[i])
        entry_point = get_current_posx()
        if cycle_count == 0:
            entry_point[0][0]= entry_point[0][0] +10
        elif cycle_count == 1:
            entry_point[0][0]= entry_point[0][0] +12
        else:
            entry_point[0][0]= entry_point[0][0] +12

        if cycle_count == 2:
            entry_point[0][1]= entry_point[0][1] -133
        elif cycle_count == 0:
            entry_point[0][1]= entry_point[0][1] -60
        else: 
            entry_point[0][1]= entry_point[0][1] -75
        entry_point[0][2]= entry_point[0][2] -25

        movel(entry_point[0], v=50, a=50)
        wait(0.2)

        log_info(step_id, f"Approach X ({OFFSET_S1_APPROACH_X})")
        move_rel_x(OFFSET_S1_APPROACH_X)
        wait(WAIT_GRIPPER_STABILIZE_SEC)
        
        log_info(step_id, "Gripper CLOSE")
        gripper_close()
        wait(0.2)
        
        log_info(step_id, f"Retract X ({OFFSET_S1_RETRACT_X})")
        move_rel_x(OFFSET_S1_RETRACT_X)
        wait(0.1)
        
        # 2.2 Move to Shelf 2 & Place
        log_info(step_id, f"Moving to Shelf 2 Pose {i+1}")
        log_info(step_id, f"Joint Move -> {JOINT_POSES_SHELF_2[i]}")

    # 위치 1번에서 2번 가는 함수
        point_away = posj(-29, 18, 69, 0, 92, -29)
        movej(point_away, vel=100, acc=100)
        JOINT_POSES_SHELF_1[i][2] = JOINT_POSES_SHELF_1[i][2] + 30

        # 위치 2번에서 슬롯으로 가는 함수
        movej(JOINT_POSES_SHELF_2[i], vel=100, acc=100)
        wait(0.2)

        ### insertion (Place Sequence)
        # 1. Approach/Insert (Relative: x+1, y+32, z-127)
        from DSR_ROBOT2 import posx, DR_MV_MOD_REL

    # 위치 2번에서 슬롯으로 삽입하는 함수
    # 알고리즘
    # 삽입확인...
        #알고리즘 생기면 이 함수 삭제.
        movel(posx(1, 0, -30, 0, 0, 0), vel=VEL_LINEAR_APPROACH, acc=ACC_LINEAR_APPROACH, mod=DR_MV_MOD_REL)
        wait(0.5)
        
        log_info(step_id, "Gripper OPEN (Place)")
        gripper_open()
        
        # 2. Retract (Relative: z+50, then z+50 -> Total z+100)
    # 삽입된 위치에서 들어올리는 함수
    
        movel(posx(0, 0, 100, 0, 0, 0), vel=VEL_LINEAR_FAST, acc=ACC_LINEAR_FAST, mod=DR_MV_MOD_REL)
        
        wait(0.5)
        log_info(step_id, "Cycle Completed")

    # --- 3. Stage B: Shelf 2 -> Mid -> Tilt -> Place (3 Cycles) ---
    log_info("Stage B", f"Starting Shelf 2 -> Tilt -> Place Loop ({cycle_count} cycles)")
    
    import random

# 3회 중 2회 성공, 1회 실패 (순서는 랜덤)
    results = [True, True, False]
    random.shuffle(results)

    for i in range(cycle_count):
        if not check_task_status(): return

        step_id = f"B-{i+1}"
        is_success = results[i]
        # 3.1 Pick from Shelf 2
        log_info(step_id, f"Moving to Shelf 2 Pose {i+1} (Pick)")
        move_joint(JOINT_POSES_SHELF_2[i])
        wait(0.2)
        
        movel(posx(0, 0, -50, 0, 0, 0), vel=VEL_LINEAR_FAST, acc=ACC_LINEAR_FAST, mod=DR_MV_MOD_REL)
        log_info(step_id, "Gripper CLOSE")
        gripper_close()
        wait(0.2)

        movel(posx(0, 0,  50, 0, 0, 0), vel=VEL_LINEAR_FAST, acc=ACC_LINEAR_FAST, mod=DR_MV_MOD_REL)
        
        # 3.2 Move to Transition Point
        if Q_WAYPOINT_TRANSITION:
            log_info(step_id, "Moving to Transition Waypoint")
            move_joint(Q_WAYPOINT_TRANSITION)
            wait(0.2)
        
        if is_success:
            cur6 = list(_pose_from_current())
            cur6[1] = cur6[1] + 200

            p_work = _pose_same(cur6)
            p_tilt = _pose_tilt_from_pose(cur6, DIR_S3_TILT_SIGN * ANGLE_S3_TILT_DEG)

            log_info(step_id, f"[SUCCESS] Move to Y+120 then Tilt {DIR_S3_TILT_SIGN * ANGLE_S3_TILT_DEG} deg")
            movel(p_tilt, vel=VEL_LINEAR_FAST, acc=ACC_LINEAR_FAST)
            wait(0.1)

            log_info(step_id, "Gripper OPEN (Drop)")
            gripper_open()
            wait(0.2)

            log_info(step_id, "Restoring Orientation")
            movel(p_work, vel=VEL_LINEAR_FAST, acc=ACC_LINEAR_FAST)
            wait(0.1)

        else:
            log_info(step_id, "[FAIL] Drop at TRANSITION waypoint (OPEN only)")
            if Q_WAYPOINT_TRANSITION:
                wait(0.1)

            gripper_open()
            wait(0.2)
    # --- 4. Final Homing ---

    log_info("End", "Moving Home")
    movej(posj(POSJ_HOME), v=100, a=100)
    log_info("End", "All tasks completed successfully.")

# =========================================================
# X. Main Execution
# =========================================================

def main(args=None):
    rclpy.init(args=args)
    load_config()
    node = rclpy.create_node("task_node", namespace=ROBOT_ID)
    callback_node = rclpy.create_node("task_callback_node", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    # QoS Profile for Robot State (Reliable + Transient Local)
    qos_profile = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1
    )

    # QoS Profile for task command (Publisher와 호환되도록 VOLATILE 사용)
    command_qos_profile = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10
    )

    callback_node.create_subscription(Float64MultiArray, "/tool_force", force_callback, qos_profile)
    callback_node.create_subscription(Float64MultiArray, "/current_posx", posx_callback, qos_profile)
    callback_node.create_subscription(String, "/dsr01/task_command", task_command_callback, command_qos_profile)
    callback_node.create_subscription(Int32, "/drl_stop_cmd", drl_stop_cmd_callback, command_qos_profile)
    
    # 4. Setup Service Clients for motion control (on callback node)
    global _move_pause_cli, _move_resume_cli, _move_stop_cli, _task_node
    _task_node = callback_node
    
    # 서비스 경로: dsr_controller2는 namespace 아래에 서비스 생성
    svc_prefix = f"/{ROBOT_ID}"  # /dsr01
    _move_pause_cli = callback_node.create_client(MovePause, f"{svc_prefix}/motion/move_pause")
    _move_resume_cli = callback_node.create_client(MoveResume, f"{svc_prefix}/motion/move_resume")
    _move_stop_cli = callback_node.create_client(MoveStop, f"{svc_prefix}/motion/move_stop")
    
    print(f"[MAIN] Waiting for motion services at {svc_prefix}/motion/...")
    if not _move_pause_cli.wait_for_service(timeout_sec=2.0):
        print("[WARN] move_pause service not available")
    if not _move_resume_cli.wait_for_service(timeout_sec=2.0):
        print("[WARN] move_resume service not available")
    if not _move_stop_cli.wait_for_service(timeout_sec=2.0):
        print("[WARN] move_stop service not available")
    print("[MAIN] Motion services ready")
    
    try:
        # 5. Robot Initialization (BEFORE spin_thread to avoid spin conflict)
        # DSR_ROBOT2 functions use spin_until_future_complete internally
        initialize_robot()
        
        # 6. Start Background Spin for CALLBACK NODE - 완전히 분리된 노드
        t = threading.Thread(target=spin_thread, args=(callback_node,), daemon=True)
        t.start()
        
        # Give spin thread time to start
        time.sleep(0.5)
        print("[MAIN] Spin thread started for callback_node, proceeding to task...")
        
        # 7. Execute Task Sequence
        if 'perform_task' in globals():
            perform_task()
        else:
            print("Warning: perform_task function is not yet defined.")

    except KeyboardInterrupt:
        print("\nShutting down task node...")
        try:
            # Safety: Return to Start Position (Home) on Interrupt
            from DSR_ROBOT2 import movej, posj
            POSJ_HOME = get_config_val('position.posj_home', [])
            print("Safety Homing Triggered...")
            movej(posj(POSJ_HOME), v=100, a=100)
        except Exception as home_err:
            print(f"Failed to safety home: {home_err}")
    except Exception as e:
        print(f"Unexpected error: {e}")
    finally:
        callback_node.destroy_node()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
