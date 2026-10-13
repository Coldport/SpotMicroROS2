import os

import pytest

from spot_micro_deck import audio

CARDS = """\
 0 [C200           ]: USB-Audio - Anker PowerConf C200
                      Anker PowerConf C200 Anker PowerConf C200 at usb-xhci-hcd.1-1, high speed
 1 [vc4hdmi0       ]: vc4-hdmi - vc4-hdmi-0
                      vc4-hdmi-0
 2 [Codec          ]: PCH - Some Codec
                      Some Codec at 0x1234
"""


@pytest.fixture
def asound(tmp_path):
    (tmp_path / 'cards').write_text(CARDS)
    for i, pcms in ((0, ['pcm0c']), (1, ['pcm0p']), (2, ['pcm0p', 'pcm0c'])):
        d = tmp_path / f'card{i}'
        d.mkdir()
        for n in pcms:
            (d / n).mkdir()
    return str(tmp_path)


def test_lists_only_cards_that_record(asound):
    cards = audio.list_capture_cards(asound)
    assert [(c[0], c[1]) for c in cards] == [(0, 'C200'), (2, 'Codec')]


def test_auto_prefers_usb_and_hints_match_names(asound):
    assert audio.find_device('auto', asound) == 'plughw:0,0'
    assert audio.find_device('codec', asound) == 'plughw:2,0'
    assert audio.find_device('Anker', asound) == 'plughw:0,0'
    assert audio.find_device('hw:5,1', asound) == 'hw:5,1'
    with pytest.raises(audio.AudioError):
        audio.find_device('nope', asound)


def test_no_cards(tmp_path):
    with pytest.raises(audio.AudioError):
        audio.find_device('auto', str(tmp_path))


def test_check_access(tmp_path):
    with pytest.raises(audio.AudioError, match='gone'):
        audio.check_access('plughw:3,0', str(tmp_path))
    node = tmp_path / 'pcmC3D0c'
    node.write_bytes(b'')
    os.chmod(node, 0o600)
    audio.check_access('plughw:3,0', str(tmp_path))     # ours: fine
    audio.check_access('default', str(tmp_path))        # not hw: left to ffmpeg


def test_ffmpeg_command_mixes_to_mono_mulaw():
    cmd = audio.ffmpeg_command('plughw:0,0', 16000, 6.0)
    assert cmd[cmd.index('-i') + 1] == 'plughw:0,0'
    assert ['-ac', '1', '-ar', '16000', '-f', 'mulaw'] == cmd[cmd.index('-ac'):cmd.index('-f', cmd.index('-ac')) + 2]
    assert 'volume=6dB' in cmd
