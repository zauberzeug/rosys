from multiprocessing import Event, Process, Queue
from queue import Empty


class CpuIntensiveProcess(Process):
    '''Example code which uses a lot of CPU and hence should be run in a separate process.

    The computation happens in the `run()` function.
    Through the _send() function the latest data point is sent back to the main process.
    This is done by first removing all existing elements from the queue and then adding back the latest.
    Therefore the main process is sure to always receive the latest computed value and none of the obsolete ones.
    '''

    def __init__(self, queue: Queue):
        super().__init__()
        self.queue = queue
        self.stop_requested = Event()

    def run(self):
        x = 0
        try:
            while not self.stop_requested.is_set():
                x += 1
                self._send(x)
        except KeyboardInterrupt:
            self.stop()

    def _send(self, value: int):
        try:
            while True:
                self.queue.get_nowait()
        except Empty:
            pass
        self.queue.put_nowait(value)

    def stop(self):
        self.stop_requested.set()
