#!/bin/bash
# voice_xterm.sh -- start the speech front-end (voice_command.py) for HARDWARE.
# Launch:  xterm -T "VOICE hw" -e bash ~/vla_tools/voice_xterm.sh   (inside ubuntu22-gpu)
#
# WHY THIS EXISTS (11 Sep): faster-whisper loads libcublas.so.12 / libcudnn*.so.9
# lazily, at the FIRST transcription, not when the model loads. They are pip
# packages under ~/.local, not on the system loader path, so on 10 Sep the
# model "loaded in 4.1 s" and then EVERY transcription failed with
# "Library libcublas.so.12 is not found or cannot be loaded". Putting the two
# directories on LD_LIBRARY_PATH fixes it. voice_command.py itself is unchanged
# (it is shared with the simulation).
source /home/danyalaziz/robot_env.sh
SITE=/home/danyalaziz/.local/lib/python3.10/site-packages
export LD_LIBRARY_PATH="$SITE/nvidia/cublas/lib:$SITE/nvidia/cudnn/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
cd /home/danyalaziz
# stdin from /dev/null => the node runs HEADLESS (GUI hold-to-talk on
# /vla/voice/trigger is the only trigger), while its output stays visible here.
# VLA_MIC overrides the device; "pulse" follows the system default input.
# 14 Sep: the earbuds' recordings peak at full scale (1.000) at 100 % AND at 60 %
# PC-side gain -- the clipping happens inside the headset's own mic processing,
# so no PulseAudio setting cures it. Both models still transcribed the demo
# phrases correctly from that audio; hold the button until the phrase is finished.
# 14 Sep: medium.en -- same four test phrases transcribed exactly by both models,
# medium.en with higher confidence; 0.4 s per 30 s of audio on the A4000 once warm.
# 14 Sep: vocabulary hint (voice_command.py reads VLA_VOICE_PROMPT; empty = old behaviour)
# and a stricter confidence gate: real commands scored -0.25..-0.56 on 14 Sep, the
# junk ("welcome to our dressing room", "go to the beach") -0.92..-1.08.
export VLA_VOICE_PROMPT="${VLA_VOICE_PROMPT:-Robot commands: go to the chair. go to the person. go to the nearest person. go to the farthest person. go to the box. what do you see. find a chair. turn left. turn right. move forward one meter. move back. stop. cancel. dock. undock. patrol the area. explore the area.}"
python3 -u /home/danyalaziz/voice_command.py --mode ptt --mic "${VLA_MIC:-pulse}" --model "${VLA_WHISPER:-medium.en}" --min-logprob "${VLA_MIN_LOGPROB:--0.75}" < /dev/null 2>&1 \
  | tee -a "/home/danyalaziz/vla_logs/voice_$(date +%Y%m%d).log"
echo; echo "[voice exited -- window kept open]"; exec bash
