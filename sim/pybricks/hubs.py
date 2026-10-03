from world import world


class _Speaker:
    def beep(self, *a, **k):
        world.event("beep")
        world.advance(100)

    def play_file(self, path):
        world.event("play_file", path)

    def set_volume(self, v, *a):
        world.event("volume", v)


class _Light:
    def on(self, color):
        world.event("light", color)

    def off(self):
        world.event("light", None)


class _Screen:
    def clear(self):
        world.event("screen", "<clear>")

    def print(self, *args):
        world.event("screen", " ".join(str(a) for a in args))


class _Buttons:
    def pressed(self):
        return []


class EV3Brick:
    def __init__(self):
        self.speaker = _Speaker()
        self.light = _Light()
        self.screen = _Screen()
        self.buttons = _Buttons()
