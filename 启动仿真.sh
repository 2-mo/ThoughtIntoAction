./myTeleop/run.sh simulate-official-x5 \
  --control-mode position \
  --scale-factor 1.0 \
  --max-joint-speed-rad-s 0.2 \
  --initial-q 0 0 0.0025 0.3302 0.0185 0.0559


  ./run.sh teleop-official-x5 \
  --control-mode pose \
  --scale-factor 0.5 \
  --max-joint-speed-rad-s 0.5 \
  --preview-time-s 0.05 \
  --actuate