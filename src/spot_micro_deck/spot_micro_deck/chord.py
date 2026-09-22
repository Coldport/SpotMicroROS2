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
