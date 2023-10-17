import time
from collections import defaultdict, deque


class Profile:
    history: dict[str, deque[float]] = defaultdict(deque)

    def __init__(self, histsize: int = 30) -> None:
        """Usage:

        @Profile()
        def func(...):
            ...

        or

        @Profile(histsize=40)
        def func(...):
            ...
        """
        self.histsize = histsize

    def __call__(self, fn):
        def wrapper(*args, **kwargs):
            start = time.time()
            retval = fn(*args, **kwargs)
            end = time.time()

            self.update_history(fn.__name__, end - start)

            return retval

        return wrapper

    def update_history(self, key: str, runtime: float) -> None:
        v = self.history[key]
        if len(v) == self.histsize:
            v.popleft()
        v.append(runtime)

    @classmethod
    def summary(cls) -> str:
        lines = []

        for k, v in cls.history.items():
            if len(v) != 0:
                mean = sum(v) / len(v)
                hz = 1 / mean
                lines.append(f"{k}: n={len(v)}; mean={mean*1000:.1f} ms | {hz:.2f} hz")
            else:
                lines.append(f"{k}: not called")

        return "\n".join(lines)
