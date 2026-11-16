"""Bounded Windows wait for the WiLoR simulation control thread.

Uses the documented Windows 10 1803+ high-resolution waitable timer.
No process priority, global timer policy, or busy waiting is changed.
"""
import math
import os
import time


def next_deadline(previous, started, finished, period=.01):
    """Never compress two fixed simulation steps into a catch-up burst."""
    due = max(previous + period, started + period)
    missed = finished >= due
    return (finished + period if missed else due), missed


class ControlWait:
    def __init__(self):
        self.handle = None
        self.info = dict(backend='event_wait', initialization_error=None)
        if os.name != 'nt':
            return
        import ctypes as c
        from ctypes import wintypes as w
        self.c = c
        self.kernel = c.WinDLL('kernel32', use_last_error=True)
        self.kernel.CreateWaitableTimerExW.argtypes = [c.c_void_p, w.LPCWSTR, w.DWORD, w.DWORD]
        self.kernel.CreateWaitableTimerExW.restype = w.HANDLE
        self.kernel.SetWaitableTimer.argtypes = [w.HANDLE,c.POINTER(c.c_longlong),w.LONG,c.c_void_p,c.c_void_p,w.BOOL]
        self.kernel.SetWaitableTimer.restype = w.BOOL
        self.kernel.WaitForSingleObject.argtypes = [w.HANDLE,w.DWORD]
        self.kernel.WaitForSingleObject.restype = w.DWORD
        self.kernel.CloseHandle.argtypes = [w.HANDLE]
        self.kernel.CloseHandle.restype = w.BOOL
        # Unnamed, auto reset; TIMER_MODIFY_STATE | SYNCHRONIZE.
        self.handle = self.kernel.CreateWaitableTimerExW(None,None,0x2,0x100002)
        if self.handle:
            self.info['backend'] = 'windows_high_resolution_waitable_timer'
        else:
            self.info['initialization_error'] = c.get_last_error()

    def wait(self, deadline, stopping):
        delay = deadline-time.perf_counter()
        if delay <= 0 or stopping.is_set():
            return
        # This helper is only for <= one 10ms control period; bounded stop delay.
        if delay > .011:
            raise ValueError('Control wait exceeds one bounded cycle')
        if self.handle is None:
            stopping.wait(delay)
            return
        due = self.c.c_longlong(-max(1,math.ceil(delay*1e7)))
        if not self.kernel.SetWaitableTimer(self.handle,self.c.byref(due),0,None,None,False):
            raise self.c.WinError(self.c.get_last_error())
        status = self.kernel.WaitForSingleObject(self.handle,1000)
        if status != 0:
            raise RuntimeError(f'Control timer wait failed: {status}')

    def close(self):
        if self.handle:
            handle,self.handle=self.handle,None
            if not self.kernel.CloseHandle(handle):
                raise self.c.WinError(self.c.get_last_error())
