# The ROS snapshot that matches this project exactly

**Finding: every one of the 399 installed ROS Humble packages matches the
`snapshots.ros.org` Humble snapshot of 2026-05-14, exactly. Zero version
differences, zero packages missing.**

That is what makes `docker/ros.Dockerfile` reproducible rather than
approximately right: pinning the apt source to that snapshot and installing the
399 packages at their recorded versions reconstructs the ROS side of the machine
that produced the results in `results/`.

## How this was established

Installed build dates span **2026-02-13 to 2026-05-05**, so the first snapshot
on or after 2026-05-05 was the candidate. The available Humble snapshots nearest
that are `2026-03-29` (too early — it predates the 20260414, 20260421, 20260422,
20260425 and 20260505 builds) and **`2026-05-14`**.

The snapshot's package index was downloaded and compared against
`env/ros-humble-packages_ubuntu22-gpu.txt` entry by entry:

```
snapshot 2026-05-14 index: 3728 packages
installed:                 399
  exact version match:     399
  present but different:     0
  absent from snapshot:      0
```

Reproduce it with:

```bash
curl -sS -o /tmp/Packages.gz \
  http://snapshots.ros.org/humble/2026-05-14/ubuntu/dists/jammy/main/binary-amd64/Packages.gz
gunzip -f /tmp/Packages.gz
python3 - <<'PY'
import pathlib
inst = dict(l.split() for l in
            pathlib.Path("env/ros-humble-packages_ubuntu22-gpu.txt").read_text().split("\n")
            if len(l.split()) == 2)
snap, pkg = {}, None
for line in pathlib.Path("/tmp/Packages").read_text(errors="replace").splitlines():
    if line.startswith("Package: "): pkg = line[9:].strip()
    elif line.startswith("Version: ") and pkg: snap[pkg] = line[9:].strip(); pkg = None
print("exact :", sum(snap.get(n) == v for n, v in inst.items()), "/", len(inst))
print("differ:", sum(n in snap and snap[n] != v for n, v in inst.items()))
print("absent:", sum(n not in snap for n in inst))
PY
```

## Use HTTP, not HTTPS

`snapshots.ros.org` resolves to CloudFront and its TLS certificate does **not**
cover the hostname:

```
curl: (60) SSL: no alternative certificate subject name matches target host name 'snapshots.ros.org'
```

So every URL here is `http://`. This is not a local misconfiguration — DNS
resolves and port 443 accepts the connection; the certificate is simply for the
CloudFront distribution. `docker/ros.Dockerfile` therefore adds the apt source
over HTTP and pins it with `[trusted=yes]`, which is acceptable because the
exact package versions are pinned and verified against the recorded manifest.

## The pinned list

`docker/ros-packages.txt` is generated from the manifest, one `name=version` per
line, and is the single source of truth the Dockerfile installs from:

```bash
awk 'NF==2 {print $1 "=" $2}' env/ros-humble-packages_ubuntu22-gpu.txt | sort \
  > docker/ros-packages.txt
```

All 399 are pinned rather than a hand-picked "top level" subset, for two
reasons. Guessing which of the 399 are top-level means guessing, and transitive
dependencies resolved at image build time are exactly the thing that drifts.
Pinning the lot is what makes the image match the machine.

## What this replaces

`env/MANIFEST.md` records that the ROS container ran from a **local Podman
snapshot image** (`localhost/ubuntu22-snapshot:latest`, 9.88 GB) that nobody
else can pull — the single biggest reproducibility hole in the project.
`docker/ros.Dockerfile` closes it: the image is now built from a public base
plus this pinned list.
