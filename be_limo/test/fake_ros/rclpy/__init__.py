"""Tiny in-process stand-in for rclpy: topics are delivered synchronously."""
_ok = False
BUS = None


def init(args=None):
    global _ok
    _ok = True


def ok():
    return _ok


def shutdown():
    global _ok
    _ok = False


def spin_once(node, timeout_sec=None):
    pass


def spin(node):
    raise KeyboardInterrupt
