import time


def wait_until(pred, timeout=2.0, interval=0.01):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if pred():
            return True
        time.sleep(interval)
    return pred()


class DeadCapture:
    """開けない USB カメラの代役。"""

    def isOpened(self):
        return False

    def read(self):
        return False, None

    def release(self):
        pass

    def set(self, *a):
        pass


def dead_opener():
    return DeadCapture()
