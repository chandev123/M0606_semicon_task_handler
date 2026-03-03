import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float64
import DR_init
import time
# from .get_posx import GetPosx

ROBOT_ID='dsr01'
ROBOT_MODEL='m0609'
ROBOT_TOOL="Tool Weight"
ROBOT_TCP="GripperDA_v1"

VELOCITY=60
ACC=60

DR_init.__dsr__id=ROBOT_ID
DR_init.__dsr__model=ROBOT_MODEL

def initialize_robot():
    from DSR_ROBOT2 import set_tool,set_tcp, set_robot_mode, ROBOT_MODE_AUTONOMOUS
    
    # Set robot mode to autonomous to ensure commands are accepted
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)

    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)
    
    print("Robot initialized with the following settings:")
    print("#"*50)
    print(f"ROBOT_ID: {ROBOT_ID}")
    print(f"ROBOT_MODEL: {ROBOT_MODEL}")
    print(f"ROBOT_TCP: {ROBOT_TCP}")
    print(f"ROBOT_TOOL: {ROBOT_TOOL}")
    print(f"VELOCITY: {VELOCITY}")
    print(f"ACC: {ACC}")
    print("#"*50)

def perform_task(node, publisher, publish_and_log=True):
    from DSR_ROBOT2 import get_tool_force
    tool_force = get_tool_force()

    if publish_and_log:
        node.get_logger().info(f"Tool Force: {tool_force}")

        # tool_force는 6개의 값 [fx, fy, fz, mx, my, mz]을 가집니다.
        # Float64는 값 1개만 보낼 수 있어서 에러가 납니다.
        # 전체를 보내려면 Float64MultiArray를 써야 합니다.
        msg = Float64MultiArray()
        msg.data = list(tool_force)
        publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("hw_node", namespace=ROBOT_ID) # Renamed node to hw_node to avoid conflict/confusion with get_posx class
    # 6개 값을 모두 보내기 위해 MultiArray 사용
    publisher_force = node.create_publisher(Float64MultiArray, '/my_robot/get_tool_force', 10)
    node.get_logger().info("Performing task...")
    DR_init.__dsr__node = node
    count = 0
    try:
        initialize_robot()
        while rclpy.ok():
            count += 1
            perform_task(node, publisher_force, publish_and_log=True)
            time.sleep(1)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt, shutting down...")
    except Exception as e:
        node.get_logger().info(f'An unexpected error occurred: {e}')
    finally:
        node.get_logger().info("Shutting down...")
        rclpy.shutdown()

if __name__ == '__main__':
    main()    
