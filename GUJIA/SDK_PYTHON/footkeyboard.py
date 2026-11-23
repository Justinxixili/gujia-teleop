from pynput import keyboard
# 你的脚踏输出：左=s，中间=q，右=d
KEY_LEFT  = keyboard.KeyCode(char='s')
KEY_MID   = keyboard.KeyCode(char='q')
KEY_RIGHT = keyboard.KeyCode(char='d')

left_pedal = False
mid_pedal  = False
right_pedal= False

def on_key_press(key):
    global left_pedal,mid_pedal,right_pedal
    if key == KEY_LEFT:
        left_pedal=True
        print("【左踏板 s】按下 ✅")
    if key == KEY_MID:
        mid_pedal=True
        print("【中踏板 q】按下 ✅")
    if key == KEY_RIGHT:
        right_pedal=True
        print("【右踏板 d】按下 ✅")

    if key == keyboard.Key.esc:
        return False

def on_key_release(key):
    global left_pedal,mid_pedal,right_pedal
    if key == KEY_LEFT:
        left_pedal=False
        print("【左踏板 s】松开 ⏹")
    if key == KEY_MID:
        mid_pedal=False
        print("【中踏板 q】松开 ⏹")
    if key == KEY_RIGHT:
        right_pedal=False
        print("【右踏板 d】松开 ⏹")


print("脚踏监听已启动，踩踏板查看状态，按ESC退出")
with keyboard.Listener(on_press=on_key_press, on_release=on_key_release) as listener:
    listener.join()