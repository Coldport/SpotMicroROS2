"""
Write changed parameter values back into a ROS params YAML file, in place.

The Deck's settings menu changes parameters live; deck_bridge calls this so the
same values are in the params files the next launch reads. Edits are line-based
so the comments, ordering and alignment of the hand-written file survive: only
the value on a `key: value` line under `ros__parameters:` is replaced (a key not
in the file yet is appended to that block). No ROS imports, so it can be
unit-tested.
"""

import os
import re
import tempfile

import yaml

_KEY_LINE = re.compile(r'^(?P<indent>\s+)(?P<key>[A-Za-z_][\w.]*):(?P<sp>\s*)(?P<value>.*?)(?P<comment>\s+#.*)?$')


def format_value(value):
    if isinstance(value, bool):
        return 'true' if value else 'false'
    if isinstance(value, int):
        return str(value)
    if isinstance(value, float):
        return repr(value)
    if isinstance(value, str):
        return '"' + value.replace('\\', '\\\\').replace('"', '\\"') + '"'
    if isinstance(value, (list, tuple)):
        return '[' + ', '.join(format_value(v) for v in value) + ']'
    raise TypeError(f'cannot write {type(value).__name__} to YAML')


def update_text(text, values):
    """Return `text` with each key in `values` set to its new value."""
    lines = text.split('\n')
    params_at = next((i for i, ln in enumerate(lines) if ln.strip() == 'ros__parameters:'), None)
    if params_at is None:
        raise ValueError('no ros__parameters: block')
    params_indent = len(lines[params_at]) - len(lines[params_at].lstrip())

    remaining = dict(values)
    key_indent = None
    end = len(lines)          # one past the last line of the ros__parameters block
    for i in range(params_at + 1, len(lines)):
        line = lines[i]
        if line.strip() and not line.lstrip().startswith('#'):
            indent = len(line) - len(line.lstrip())
            if indent <= params_indent:
                end = i
                break
        m = _KEY_LINE.match(line)
        if not m:
            continue
        if key_indent is None:
            key_indent = m['indent']
        if m['indent'] != key_indent or m['key'] not in remaining:
            continue
        value = remaining.pop(m['key'])
        try:
            unchanged = yaml.safe_load(m['value']) == value
        except yaml.YAMLError:
            unchanged = False
        if unchanged:           # leave e.g. 0.50 as written
            continue
        new = format_value(value)
        comment = m['comment'] or ''
        if comment:
            # Keep the comment in its column when the value's width changes.
            gap = len(m['value']) + len(comment) - len(comment.lstrip()) - len(new)
            comment = ' ' * max(gap, 1) + comment.lstrip()
        lines[i] = f"{m['indent']}{m['key']}:{m['sp'] or ' '}{new}{comment}"

    if remaining:
        indent = key_indent or ' ' * (params_indent + 2)
        while end > params_at + 1 and not lines[end - 1].strip():
            end -= 1
        lines[end:end] = [f'{indent}{k}: {format_value(v)}' for k, v in remaining.items()]
    return '\n'.join(lines)


def update_file(path, values):
    """Set `values` in the params file at `path` (atomically; symlinks followed)."""
    path = os.path.realpath(path)
    with open(path, encoding='utf-8') as f:
        text = f.read()
    new_text = update_text(text, values)
    if new_text == text:
        return
    fd, tmp = tempfile.mkstemp(dir=os.path.dirname(path), prefix='.' + os.path.basename(path))
    try:
        with os.fdopen(fd, 'w') as f:
            f.write(new_text)
        os.chmod(tmp, os.stat(path).st_mode & 0o7777)
        os.replace(tmp, path)
    except BaseException:
        if os.path.exists(tmp):
            os.unlink(tmp)
        raise
