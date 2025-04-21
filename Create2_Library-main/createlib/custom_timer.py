from threading import Timer, Lock
import time

####################
#  Sources:
#    https://stackoverflow.com/questions/2398661/schedule-a-repeating-event-in-python-3/18942977
#    https://stackoverflow.com/questions/474528/what-is-the-best-way-to-repeatedly-execute-a-function-every-x-seconds
#####################
class CustomTimer:
    """A flexible timer that supports single-shot or repeating fixed-rate execution."""

    def __init__(self, interval, function, *args, autostart=True, repeat=True, **kwargs):
        self._lock = Lock()
        self.function = function
        self.interval = interval
        self.args = args
        self.kwargs = kwargs
        self.repeat = repeat
        self._timer = None
        self._stopped = True
        self.next_call = None
        if autostart:
            self.start()

    def start(self, from_run=False):
        """
        Starts the timer. If repeating, uses fixed-rate scheduling.
        """
        with self._lock:
            if from_run or self._stopped:
                now = time.time()
                if not self.next_call:
                    self.next_call = now + self.interval
                else:
                    self.next_call += self.interval

                delay = max(0, self.next_call - now)
                self._timer = Timer(delay, self._run)
                self._stopped = False
                self._timer.start()

    def _run(self):
        """
        Executes the function. Repeats if repeat=True; otherwise stops after one shot.
        """
        try:
            self.function(*self.args, **self.kwargs)
        finally:
            if self.repeat:
                self.start(from_run=True)
            else:
                self.stop()

    def stop(self):
        """
        Stops the timer and clears internal state.
        """
        with self._lock:
            if self._timer:
                self._timer.cancel()
                self._timer = None
            self._stopped = True
            self.next_call = None