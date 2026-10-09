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
