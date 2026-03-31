#!/bin/bash
#!/bin/bash

cleanup() {
    echo "Ctrl+C detected. Cleaning up..."
    #killall -9 python3
    killall -9 janus
    exit 1
}

trap cleanup SIGINT

#killall -9 python3
killall -9 janus

source ~/serial/venv/bin/activate

/opt/janus/bin/janus &
sleep 1

cd ~/lidar
#python3 lidar2.py --port /dev/ttyUSB0 --video_out /dev/video24 --fps 10 --out_w 640 --out_h 640 &


cd ~/video
#python3 red5.py --serial /dev/serial0 --motor-hz 12 --deadband 0.02 --kp 0.45 --maxturn 0.30 --maxcmd 0.30 &

python3 red12.py \
  --in-dev 0 \
  --out-dev /dev/video10 \
  --width 640 \
  --height 480 \
  --fps 30 \
  --min-area 800 \
  --kp 0.3 \
  --kd 0.10 \
  --deadband 0.08 \
  --smooth-alpha 0.35 \
  --min-turn 0.15 \
  --maxturn 0.3 \
  --maxcmd 0.45 \
  --aruco --aruco-draw-ids --aruco-status \
  --aruco-pose \
  --aruco-draw-axes \
  --aruco-marker-size 0.105 \
  --aruco-every-n 3 \
  --aruco-pose-first-only \
  --target-mode aruco \
  --aruco-approach \
  --aruco-stop-dist 0.20 \
  --aruco-approach-align 0.18 \
  --aruco-forward-min 0.4 \
  --aruco-forward-speed 0.4 \
  --udp-host 127.0.0.1 \
  --udp-port 5010 \
  --cmd-rate-hz 12 &

sleep 5
#sudo v4l2-ctl -d /dev/video24 --set-parm=10

gst-launch-1.0 -v \
  v4l2src device=/dev/video10 ! video/x-raw,format=YUY2,width=640,height=480,framerate=30/1 ! \
  videoconvert ! video/x-raw,format=I420 ! \
  x264enc tune=zerolatency speed-preset=ultrafast bitrate=3800 key-int-max=30 bframes=0 ! \
  video/x-h264,profile=baseline ! h264parse ! \
  rtph264pay pt=96 config-interval=1 ! \
  tee name=t \
    t. ! queue ! udpsink host=127.0.0.1 port=8004 sync=false async=false &

#gst-launch-1.0 -v   v4l2src device=/dev/video24 !   video/x-raw,format=YUY2,width=640,height=640,framerate=10/1 !   videoconvert ! video/x-raw,format=I420 !   x264enc tune=zerolatency speed-preset=ultrafast bitrate=1200 key-int-max=20 bframes=0 !   video/x-h264,profile=baseline ! h264parse !   rtph264pay pt=96 config-interval=1 !   tee name=t     t. ! queue ! udpsink host=127.0.0.1 port=8005 sync=false async=false    


echo "Stack running..."

while true; do
    sleep 1
done
