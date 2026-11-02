#!/usr/bin/env python3
"""Small ROS client; closing the window does not terminate simulation."""

import json
import math
import time
import tkinter as tk
from tkinter import ttk

import rclpy
from std_msgs.msg import String
from std_srvs.srv import SetBool


def main():
    rclpy.init()
    node = rclpy.create_node('sim_gimbal_panel')
    root = tk.Tk()
    root.title('RM27 云台控制')
    root.geometry('360x180')
    root.resizable(False, False)
    frame = ttk.Frame(root, padding=16)
    frame.pack(fill='both', expand=True)
    state = dict(spinning=False, want_upright=False, upright=False, connected=False)
    received_at = [0.0]
    pending = [None]
    status = tk.StringVar(value='等待仿真连接')
    labels = {'Waiting for joint feedback': '等待关节反馈', 'Rotating': '持续旋转中',
              'Returning to forward': '正在回正', 'Upright': '已立头', 'Folded': '已倒头',
              'Folding': '正在倒头', 'Raising': '正在立头',
              'Waiting for fresh joint feedback': '等待关节反馈恢复'}
    angles = tk.StringVar(value='Yaw: --    Roll: --')
    ttk.Label(frame, textvariable=status).pack(anchor='w', pady=(0, 8))
    ttk.Label(frame, textvariable=angles).pack(anchor='w', pady=(0, 14))
    clients = {key: node.create_client(SetBool, '/sim/gimbal/set_'+key)
               for key in ('spin', 'upright')}

    def command(key, value):
        if not clients[key].service_is_ready():
            status.set('Control service unavailable')
            return
        pending[0] = clients[key].call_async(SetBool.Request(data=value))

    spin = ttk.Button(frame, text='开启旋转',
                      command=lambda: command('spin', not state['spinning']))
    spin.pack(side='left', fill='x', expand=True, padx=(0, 8))
    head = ttk.Button(frame, text='立头',
                      command=lambda: command('upright', not state['want_upright']))
    head.pack(side='left', fill='x', expand=True)

    def update(msg):
        state.update(json.loads(msg.data))
        received_at[0] = time.monotonic()
        status.set(labels.get(state['status'], state['status']))
        angles.set(f"Yaw: {math.degrees(state['yaw']) % 360:6.1f} deg    "
                   f"Roll: {math.degrees(state['roll']):5.1f} deg")

    node.create_subscription(String, '/sim/gimbal/state', update, 1)

    def poll():
        rclpy.spin_once(node, timeout_sec=0)
        if pending[0] and pending[0].done():
            try:
                response = pending[0].result()
                if not response.success:
                    status.set(response.message)
            except Exception as error:
                status.set(str(error))
            pending[0] = None
        active = state['connected'] and time.monotonic()-received_at[0] < 1 and not pending[0]
        spin.configure(text='停止并回正' if state['spinning'] else '开启旋转',
                       state='normal' if not pending[0] and clients['spin'].service_is_ready() and
                             (state['spinning'] or (active and state['upright'] and
                              state['want_upright'])) else 'disabled')
        head.configure(text='倒头' if state['want_upright'] else '立头',
                       state='normal' if active else 'disabled')
        if time.monotonic()-received_at[0] >= 1:
            status.set('等待仿真连接')
        root.after(40, poll)

    root.after(0, poll)
    try:
        root.mainloop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
