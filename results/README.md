# results/

Everything recorded while the system ran. `DATA_PACK.md` is the written-up
version — start there.

| Path | Files | What |
|---|---|---|
| `DATA_PACK.md` | 1 | the collated measurements and their interpretation |
| `evidence/` | 7 | the packet captures that settled the Wi-Fi storm, plus the decoders |
| `measurements/` | 5 | ROS graph dump, topic rates, TF tree, DDS port map (12 Sep) |
| `screenshots/` | 12 | RViz, the GUI and the robot during runs |
| `logs/` | 241 | agent, demo, Nav2, YOLO, voice and GUI logs |

## evidence/ — the Fast DDS storm

This is the most important evidence in the project, because it overturned two
earlier diagnoses.

| File | Shows |
|---|---|
| `01_pc_to_pi_nav2_storm.pcap` | Nav2 launched **without** the `/parameter_events` and `/rosout` remaps: ~7,400 packets/s, 2 MB/s, PC → Pi |
| `02_pi_to_pc_camera_storm.pcap` | the return direction while camera frames were stalling |
| `03_pc_to_pi_after_partial_remap.pcap` | the same bring-up with the remaps in place |
| `rtps_decode.py` | decodes the RTPS submessages, to show the traffic is heartbeats rather than data |
| `cam_probe.py` | timestamps camera frames to measure real age |
| `qos_override_attempt.xml` | an XML QoS override that **does not work** — `rmw_fastrtps` on Humble ignores it. Kept so nobody retries it. |
| `SUMMARY.txt` | what each capture was taken under |

What these captures replaced:

- the belief that the **Raspberry Pi was thermally weak** — it was not; our own
  Nav2 was saturating its Wi-Fi link, and the heat and throttling followed from
  that;
- the belief that a **4.3 s camera lag** was a clock-synchronisation problem —
  it was the depth pipeline plus this same flood. Colour-only, frames arrive
  47–85 ms old.

One thing still unexplained, and recorded as open rather than glossed: **why the
Fast DDS writer ignores valid ACKNACKs once its receive socket backs up.** Worth
reporting upstream with these captures.

## logs/

Named `<component>_<date>.log`, 241 files:

| Prefix | Count | Component |
|---|---|---|
| `vla_run_` | 135 | the agent |
| `demo_` | 48 | `vla_demo.sh` stage-by-stage bring-up |
| `yolo_` | 13 | detector |
| `voice_` | 13 | transcription |
| `gui_` | 13 | operator UI |
| `nav2_hwc_` / `nav2_hw_` | 16 | Nav2, composed and one-process-per-server |

The `nav2_hwc_*` logs are the ones that show 7/7 lifecycle bonds activating in
~30 s, which is the behaviour that only became reliable after the remaps.
