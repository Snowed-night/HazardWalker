#!/usr/bin/env python3
import argparse, rospy
from building_generator_interfaces.srv import SetDoorState

def check():
    rospy.wait_for_service('/set_door_state', timeout=10)
    proxy = rospy.ServiceProxy('/set_door_state', SetDoorState)
    resp = proxy('main_entrance', False)
    print(f'大门状态: accepted={resp.accepted}, state={resp.state}')
    is_open = str(resp.state).lower() == 'open'
    print(f'大门: [开启]' if is_open else '大门: [关闭] <<< 这就是问题！')
    return is_open

def open_door():
    rospy.wait_for_service('/set_door_state', timeout=10)
    proxy = rospy.ServiceProxy('/set_door_state', SetDoorState)
    resp = proxy('main_entrance', True)
    print(f'开门结果: accepted={resp.accepted}, state={resp.state}')
    print(f'大门: 已打开！' if str(resp.state).lower()=='open' else '开门失败')

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--open', action='store_true')
    parser.add_argument('--close', action='store_true')
    parser.add_argument('--list', action='store_true')
    args = parser.parse_args()
    rospy.init_node('check_door', anonymous=True)
    if args.open: open_door()
    elif args.close:
        rospy.wait_for_service('/set_door_state', timeout=10)
        rospy.ServiceProxy('/set_door_state', SetDoorState)('main_entrance', False)
        print('已发送关门命令')
    elif args.list:
        rospy.wait_for_service('/set_door_state', timeout=10)
        proxy = rospy.ServiceProxy('/set_door_state', SetDoorState)
        for did in ['main_entrance', 'entrance']:
            try:
                r = proxy(did, False)
                print(f'  {did}: state={r.state}, accepted={r.accepted}')
            except Exception as e:
                print(f'  {did}: 查询失败 ({e})')
    else: check()

if __name__ == '__main__':
    main()
