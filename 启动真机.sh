cd ~/work/code/QuestArmTeleop

./myTeleop/run.sh teleop-official-x5 \
  --control-mode position \
  --scale-factor 0.03 \
  --max-joint-speed-rad-s 0.05 \
  --preview-time-s 0.05 \
  --actuate


./myTeleop/run.sh teleop-official-x5 \
  --control-mode pose \
  --scale-factor 0.03 \
  --max-joint-speed-rad-s 0.05 \
  --preview-time-s 0.05 \
  --actuate


cd myTeleop

./run.sh teleop-official-x5 \
  --control-mode pose \
  --scale-factor 0.5 \
  --max-joint-speed-rad-s 0.5 \
  --preview-time-s 0.05 \
  --actuate