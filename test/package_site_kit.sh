#!/bin/bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
bash "$ROOT/test/build_site_snapshot.sh"
KIT="$ROOT/build/m29-site-kit-$(date +%Y%m%d-%H%M%S)"
mkdir -- "$KIT"
cp -- "$ROOT/build/rt_site_snapshot" "$ROOT/test/capture_site.sh" \
    "$ROOT/test/M29_OFFLINE_AUDIT_20261014.md" "$KIT/"
mkdir -- "$KIT/layout-headers"
cp -- "$ROOT/lib/cncshm/private/ncdata.h" "$ROOT/lib/cncshm/api/laddata.h" \
    "$ROOT/lib/cncshm/api/regdata.h" "$KIT/layout-headers/"
git -C "$ROOT" rev-parse HEAD > "$KIT/source-commit.txt"
git -C "$ROOT" status --short > "$KIT/source-worktree.txt"
ldd "$KIT/rt_site_snapshot" > "$KIT/runtime-libraries.txt"
(
    cd -- "$KIT"
    find . -type f ! -name SHA256SUMS -print0 | sort -z | xargs -0 -r sha256sum > SHA256SUMS
)
tar -C "$(dirname -- "$KIT")" -czf "$KIT.tar.gz" -- "$(basename -- "$KIT")"
printf 'Ready: %s.tar.gz\n' "$KIT"
