"""Hold-a-button-combination detector (no ROS dependency, so it is unit-testable)."""


class ChordDetector:
    """Fires once when every button in `buttons` has been held for `hold_s` seconds.

    It re-arms only after the chord has been fully released, so holding it for a
    long time triggers exactly once. Any button in the chord going up restarts
    the timer.
    """

    def __init__(self, buttons, hold_s):
        self._buttons = tuple(buttons)
        self._hold_s = float(hold_s)
        self._since = None      # time the chord became fully pressed
        self._fired = False

    def held_for(self, now):
        """Seconds the chord has been held, or 0.0 if it isn't down."""
        return 0.0 if self._since is None else now - self._since

    def reset(self):
        self._since = None
        self._fired = False

    def update(self, button_states, now):
        """Feed the current Joy `buttons` array; True exactly when the hold completes."""
        down = all(i < len(button_states) and button_states[i] for i in self._buttons)
        if not down:
            self.reset()
            return False
        if self._since is None:
            self._since = now
        if not self._fired and now - self._since >= self._hold_s:
            self._fired = True
            return True
        return False


class DoubleTap:
    """Fires when a button is pressed twice within `window_s` seconds.

    Counts presses (released -> pressed edges); the press that fires starts
    over, so a triple tap fires once rather than twice.
    """

    def __init__(self, button, window_s=0.4):
        self._button = button
        self._window_s = float(window_s)
        self._was_down = False
        self._last_press = None

    def reset(self):
        self._was_down = False
        self._last_press = None

    def update(self, button_states, now):
        """Feed the current Joy `buttons` array; True on the second tap."""
        down = self._button < len(button_states) and bool(button_states[self._button])
        pressed = down and not self._was_down
        self._was_down = down
        if not pressed:
            return False
        if self._last_press is not None and now - self._last_press <= self._window_s:
            self._last_press = None
            return True
        self._last_press = now
        return False


class Repeat:
    """Edge + auto-repeat for one held button (like a keyboard): True on the
    press, then after `delay_s` every `rate_s` while it stays down."""

    def __init__(self, delay_s=0.4, rate_s=0.12):
        self._delay_s = delay_s
        self._rate_s = rate_s
        self._next = None

    def update(self, down, now):
        if not down:
            self._next = None
            return False
        if self._next is None:
            self._next = now + self._delay_s
            return True
        if now >= self._next:
            self._next = max(self._next + self._rate_s, now)
            return True
        return False


class TapCounter:
    """Counts quick taps of one button: after the button has stayed up for
    `window_s` since the last press, update() returns how many presses there
    were (1, 2, 3, ...) once; otherwise 0. cancel() drops the presses counted
    so far (e.g. the button was used as part of a combo instead)."""

    def __init__(self, button, window_s=0.4):
        self._button = button
        self._window_s = float(window_s)
        self._was_down = False
        self._count = 0
        self._last_press = 0.0

    def reset(self):
        self._was_down = False
        self._count = 0

    def cancel(self):
        self._count = 0

    def update(self, button_states, now):
        down = self._button < len(button_states) and bool(button_states[self._button])
        if down and not self._was_down:
            self._count += 1
            self._last_press = now
        self._was_down = down
        if self._count and not down and now - self._last_press > self._window_s:
            n, self._count = self._count, 0
            return n
        return 0
