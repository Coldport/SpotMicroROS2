from spot_micro_deck.chord import ChordDetector

L1, R1 = 4, 5


def pad(*down, n=21):
    return [1 if i in down else 0 for i in range(n)]


def test_fires_once_after_the_hold():
    c = ChordDetector([L1, R1], 3.0)
    assert not c.update(pad(L1, R1), 0.0)
    assert not c.update(pad(L1, R1), 2.9)
    assert c.update(pad(L1, R1), 3.0)
    # still held: must not fire again
    assert not c.update(pad(L1, R1), 3.5)
    assert not c.update(pad(L1, R1), 30.0)


def test_one_button_alone_never_fires():
    c = ChordDetector([L1, R1], 3.0)
    for t in (0.0, 1.0, 5.0, 10.0):
        assert not c.update(pad(L1), t)
    for t in (11.0, 15.0):
        assert not c.update(pad(R1), t)


def test_releasing_either_button_restarts_the_timer():
    c = ChordDetector([L1, R1], 3.0)
    c.update(pad(L1, R1), 0.0)
    assert not c.update(pad(L1), 2.0)          # R1 let go -> reset
    assert not c.update(pad(L1, R1), 2.5)      # re-pressed, timer restarts here
    assert not c.update(pad(L1, R1), 5.25)     # only 2.75 s since the restart
    assert c.update(pad(L1, R1), 5.5)          # exactly 3.0 s


def test_rearms_after_full_release():
    c = ChordDetector([L1, R1], 3.0)
    c.update(pad(L1, R1), 0.0)
    assert c.update(pad(L1, R1), 3.0)
    assert not c.update(pad(), 4.0)
    c.update(pad(L1, R1), 5.0)
    assert c.update(pad(L1, R1), 8.0)


def test_other_buttons_do_not_matter_and_short_arrays_are_safe():
    c = ChordDetector([L1, R1], 1.0)
    c.update(pad(L1, R1, 0, 7, 19), 0.0)
    assert c.update(pad(L1, R1, 0, 7, 19), 1.0)
    d = ChordDetector([L1, R1], 1.0)
    assert not d.update([1, 1], 0.0)           # array too short to contain L1/R1
    assert not d.update([], 5.0)


def test_held_for_reports_progress():
    c = ChordDetector([L1, R1], 3.0)
    assert c.held_for(0.0) == 0.0
    c.update(pad(L1, R1), 10.0)
    assert c.held_for(12.0) == 2.0
    c.update(pad(), 12.5)
    assert c.held_for(13.0) == 0.0


Y = 3


def test_double_tap_fires_on_the_second_press_only():
    from spot_micro_deck.chord import DoubleTap
    d = DoubleTap(Y, 0.4)
    assert not d.update(pad(Y), 0.0)
    assert not d.update(pad(Y), 0.1)      # still held: not a new press
    assert not d.update(pad(), 0.15)
    assert d.update(pad(Y), 0.3)
    assert not d.update(pad(), 0.35)
    assert not d.update(pad(Y), 0.5), 'third tap starts a new pair'


def test_double_tap_too_slow_does_not_fire():
    from spot_micro_deck.chord import DoubleTap
    d = DoubleTap(Y, 0.4)
    d.update(pad(Y), 0.0)
    d.update(pad(), 0.1)
    assert not d.update(pad(Y), 0.6)
    d.update(pad(), 0.7)
    assert d.update(pad(Y), 0.9), 'the slow tap counts as the first of a new pair'


def test_repeat_steps_on_press_then_after_delay():
    from spot_micro_deck.chord import Repeat
    r = Repeat(0.4, 0.1)
    assert r.update(True, 0.0)
    assert not r.update(True, 0.2)
    assert r.update(True, 0.4)
    assert not r.update(True, 0.45)
    assert r.update(True, 0.5)
    assert not r.update(False, 0.6)
    assert r.update(True, 0.7)


def test_tap_counter_counts_quick_taps_after_the_window():
    from spot_micro_deck.chord import TapCounter
    X = 2
    t = TapCounter(X, 0.4)
    assert t.update(pad(X), 0.0) == 0
    assert t.update(pad(), 0.1) == 0
    assert t.update(pad(X), 0.2) == 0
    assert t.update(pad(), 0.3) == 0
    assert t.update(pad(), 0.55) == 0      # still inside the window of the 2nd press
    assert t.update(pad(), 0.65) == 2
    assert t.update(pad(), 2.0) == 0       # reported once


def test_tap_counter_waits_for_release_and_cancel_drops_the_count():
    from spot_micro_deck.chord import TapCounter
    X = 2
    t = TapCounter(X, 0.4)
    t.update(pad(X), 0.0)
    assert t.update(pad(X), 1.0) == 0      # held: not a tap yet
    t.cancel()                             # used in a combo
    assert t.update(pad(), 1.1) == 0
    assert t.update(pad(), 3.0) == 0
