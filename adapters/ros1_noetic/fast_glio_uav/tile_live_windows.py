#!/usr/bin/env python3
"""Place Gazebo and RViz side by side on the current X11 desktop."""
import ctypes
import re
import subprocess
import time


class ClientMessage(ctypes.Structure):
    _fields_ = [('type', ctypes.c_int), ('serial', ctypes.c_ulong),
                ('send_event', ctypes.c_int), ('display', ctypes.c_void_p),
                ('window', ctypes.c_ulong), ('message_type', ctypes.c_ulong),
                ('format', ctypes.c_int), ('data', ctypes.c_long * 5)]


class Event(ctypes.Union):
    _fields_ = [('client', ClientMessage), ('pad', ctypes.c_long * 24)]


def windows():
    tree = subprocess.check_output(['xwininfo', '-root', '-tree'], text=True)
    found = {}
    for line in tree.splitlines():
        match = re.search(r'^\s*(0x[0-9a-f]+) "([^"]+)": \("([^"]+)"', line)
        if not match:
            continue
        window_id, title, app = match.groups()
        if app == 'gazebo' and title == 'Gazebo':
            found['gazebo'] = int(window_id, 16)
        elif app == 'rviz' and title.endswith(' - RViz'):
            found['rviz'] = int(window_id, 16)
    return found


def screen_size():
    output = subprocess.check_output(['xrandr', '--current'], text=True)
    match = re.search(r'current (\d+) x (\d+)', output)
    if not match:
        raise RuntimeError('cannot read X11 screen size')
    return tuple(map(int, match.groups()))


def main():
    deadline = time.monotonic() + 35
    while time.monotonic() < deadline:
        found = windows()
        if len(found) == 2:
            break
        time.sleep(0.5)
    else:
        raise RuntimeError('Gazebo or RViz window did not appear')
    x11 = ctypes.CDLL('libX11.so.6')
    x11.XOpenDisplay.restype = ctypes.c_void_p
    x11.XDefaultRootWindow.argtypes = [ctypes.c_void_p]
    x11.XDefaultRootWindow.restype = ctypes.c_ulong
    x11.XInternAtom.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int]
    x11.XInternAtom.restype = ctypes.c_ulong
    x11.XSendEvent.argtypes = [ctypes.c_void_p, ctypes.c_ulong, ctypes.c_int,
                              ctypes.c_long, ctypes.POINTER(Event)]
    x11.XFlush.argtypes = [ctypes.c_void_p]
    display = x11.XOpenDisplay(None)
    if not display:
        raise RuntimeError('cannot open X11 display')
    root = x11.XDefaultRootWindow(display)
    state_atom = x11.XInternAtom(display, b'_NET_WM_STATE', 0)
    resize_atom = x11.XInternAtom(display, b'_NET_MOVERESIZE_WINDOW', 0)
    max_h = x11.XInternAtom(display, b'_NET_WM_STATE_MAXIMIZED_HORZ', 0)
    max_v = x11.XInternAtom(display, b'_NET_WM_STATE_MAXIMIZED_VERT', 0)

    def send(window, atom, data):
        event = Event()
        event.client.type = 33
        event.client.display = display
        event.client.window = window
        event.client.message_type = atom
        event.client.format = 32
        event.client.data[:] = data
        x11.XSendEvent(display, root, 0, (1 << 20) | (1 << 19),
                       ctypes.byref(event))
        x11.XFlush(display)

    for window in found.values():
        send(window, state_atom, [0, max_h, max_v, 1, 0])
    time.sleep(0.5)
    width, height = screen_size()
    left_width = width // 2
    y = 75
    usable_height = height - y - 30
    send(found['gazebo'], resize_atom,
         [15 << 8, 0, y, left_width, usable_height])
    send(found['rviz'], resize_atom,
         [15 << 8, left_width, y, width - left_width, usable_height])
    print(f'Gazebo left, RViz right: {width}x{height}', flush=True)


if __name__ == '__main__':
    main()
