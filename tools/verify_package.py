#!/usr/bin/env python3
"""Check the delivered SHA-256 inventory without touching ports or hardware."""
import hashlib
from pathlib import Path
import sys


def verify(root):
    root = Path(root).resolve()
    manifest = root / 'MANIFEST.sha256'
    if not manifest.is_file():
        raise ValueError('MANIFEST.sha256 is missing; run this in the extracted handoff package')
    checked = 0
    failures = []
    for line in manifest.read_text(encoding='utf-8').splitlines():
        digest, separator, relative = line.partition('  ')
        if not separator or len(digest) != 64:
            raise ValueError('Invalid manifest line')
        target = (root / relative).resolve()
        if not target.is_relative_to(root):
            raise ValueError('Manifest contains a path outside the package')
        if not target.is_file() or hashlib.sha256(target.read_bytes()).hexdigest() != digest:
            failures.append(relative)
        checked += 1
    return checked, failures


def main():
    root = Path(__file__).resolve().parent.parent
    try:
        count, failures = verify(root)
    except (OSError, ValueError) as exc:
        print(str(exc), file=sys.stderr)
        return 2
    if failures:
        print('Changed or missing files:')
        print('\n'.join(failures))
        return 1
    print(f'PASS: {count} delivered files match MANIFEST.sha256')
    print('This checks file integrity, not hardware readiness; extra local files are not checked.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
