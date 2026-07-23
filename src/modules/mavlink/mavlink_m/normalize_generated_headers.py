#!/usr/bin/env python3

"""Normalize volatile pymavlink build metadata for reproducible headers."""

from pathlib import Path
import re
import sys


PINNED_BUILD_DATE = "Fri Jul 17 2026"
BUILD_DATE_PATTERN = re.compile(
    r'(#define MAVLINK_BUILD_DATE ")[^"]+("$)', re.MULTILINE
)


def main() -> int:
    if len(sys.argv) != 2:
        print(f"usage: {sys.argv[0]} <generated-header-root>", file=sys.stderr)
        return 2

    root = Path(sys.argv[1])
    version_headers = sorted(root.rglob("version.h"))

    if not version_headers:
        print(f"no MAVLink version headers found below {root}", file=sys.stderr)
        return 1

    for path in version_headers:
        source = path.read_text(encoding="utf-8")
        normalized, replacements = BUILD_DATE_PATTERN.subn(
            rf'\g<1>{PINNED_BUILD_DATE}\g<2>', source, count=1
        )

        if replacements != 1:
            print(f"unable to normalize MAVLINK_BUILD_DATE in {path}", file=sys.stderr)
            return 1

        path.write_text(normalized, encoding="utf-8")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
