#!/usr/bin/env python3
"""Print one line per release asset: name<TAB>size<TAB>download-url.

A separate file rather than `python3 -c` inside a shell string: the nested
quoting of an f-string inside a single-quoted shell argument is what made the
first version of this silently parse nothing and report "0 assets" for a release
that had 17.
"""
import json
import sys

data = json.load(sys.stdin)
for asset in data.get("assets", []):
    print("\t".join([
        asset.get("name", ""),
        str(asset.get("size", "")),
        asset.get("browser_download_url", ""),
    ]))
