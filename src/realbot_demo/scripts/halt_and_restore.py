import subprocess

def halt():
    # 运行紧急停机的指令
    halt_command = "rosservice call /arm_emergency_stop\n \
        rosnode kill /wpb_home_grab_action\n \
        rosservice call /emergency_stop"

    subprocess.Popen(halt_command, shell=True)

def restore():
    # 运行恢复的指令
    restore_command = "rosservice call /arm_zero\n \
        rosrun wpb_home_behaviors wpb_home_grab_action"

    subprocess.Popen(restore_command, shell=True)