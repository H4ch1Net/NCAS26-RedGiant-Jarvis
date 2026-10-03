from world import world


def wait(ms):
    world.advance(ms)


class StopWatch:
    def __init__(self):
        self._t0 = world.t_ms

    def reset(self):
        self._t0 = world.t_ms

    def time(self):
        return int(world.t_ms - self._t0)
