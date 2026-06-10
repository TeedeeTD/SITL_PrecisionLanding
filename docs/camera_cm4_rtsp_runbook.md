# Camera CM4 RTSP H264 Runbook

Working setup for Raspberry Pi CM4 hardware H264 video streaming on Ubuntu Server 22.04.

Both camera paths are fully working with hardware encode/decode.

## Environment

Pi:

```text
Board: Raspberry Pi CM4 (8GB RAM)
OS: Ubuntu Server 22.04.5 LTS
Kernel: 5.15.0-1102-raspi aarch64
Pi RTSP/relay IP: 172.20.50.60
Pi IP on camera network: 192.168.168.100
```

Host:

```text
Host IP: 172.20.50.133
VLC: /snap/bin/vlc
```

CSI camera:

```text
Model: Raspberry Pi Camera V2
Sensor: IMX219
```

IP camera:

```text
IP: 192.168.168.14
RTSP port: 8554/tcp
Working RTSP URL: rtsp://192.168.168.14:8554/main.264
Codec: H264
Resolution: 1280x720
FPS: 30
```

MediaMTX:

```text
Version: v1.19.0 linux arm64
Pi listener: rtsp://172.20.50.60:8554
```

Hardware codec nodes:

```text
/dev/video10 = bcm2835-codec-decode (hardware H264 decoder)
/dev/video11 = bcm2835-codec-encode (hardware H264 encoder)
```

## Prerequisites

### 1. Boot Config (Critical For Hardware Encode)

The hardware encoder requires sufficient CMA memory. Default Ubuntu only allocates 64MB which causes the encoder to silently fail or hang.

Edit:

```bash
sudo nano /boot/firmware/config.txt
```

Required lines:

```text
camera_auto_detect=0
# display_auto_detect=1        # comment this out to avoid conflict
dtoverlay=imx219,cam1           # CSI camera on CAM1 port

[cm4]
dtoverlay=dwc2,dr_mode=host

[all]
dtoverlay=vc4-kms-v3d,cma-256  # CMA 256MB for hardware encoder
```

Reboot:

```bash
sudo reboot
```

Verify CMA after reboot:

```bash
cat /proc/meminfo | grep -i Cma
```

Expected:

```text
CmaTotal:     262144 kB
CmaFree:      ~200000+ kB
```

If CmaTotal is still 65536 kB (64MB), the dtoverlay line is wrong or conflicting. Fix before proceeding.

### 2. Install Build Dependencies

```bash
sudo apt update

sudo apt install -y \
  git build-essential pkg-config cmake ninja-build \
  python3 python3-pip python3-yaml python3-ply python3-jinja2 \
  libboost-dev libgnutls28-dev openssl libtiff5-dev pybind11-dev \
  libboost-program-options-dev libdrm-dev libexif-dev \
  libjpeg-dev libpng-dev ffmpeg v4l-utils

python3 -m pip install --user --upgrade meson
sudo python3 -m pip install --upgrade meson

echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc
source ~/.bashrc
```

Check Meson:

```bash
meson --version
sudo meson --version
```

Both should be at least `1.0.1`.

### 3. Build Libcamera

```bash
cd ~
git clone https://github.com/raspberrypi/libcamera.git
cd libcamera

meson setup build --buildtype=release \
  -Dpipelines=rpi/vc4 \
  -Dipas=rpi/vc4 \
  -Dv4l2=true \
  -Dgstreamer=disabled \
  -Dtest=false \
  -Dlc-compliance=disabled \
  -Dcam=disabled \
  -Dqcam=disabled \
  -Ddocumentation=disabled \
  -Dpycamera=disabled

ninja -C build
sudo meson install -C build
sudo ldconfig
```

### 4. Build Rpicam-Apps Headless

```bash
cd ~
git clone https://github.com/raspberrypi/rpicam-apps.git
cd rpicam-apps

meson setup build \
  -Denable_libav=disabled \
  -Denable_drm=enabled \
  -Denable_egl=disabled \
  -Denable_qt=disabled \
  -Denable_opencv=disabled \
  -Denable_tflite=disabled \
  -Denable_hailo=disabled

meson compile -C build
sudo meson install -C build
sudo ldconfig
```

### 5. Device Permissions

```bash
sudo usermod -aG video,render pis2
```

Create udev rule:

```bash
sudo nano /etc/udev/rules.d/99-rpicam.rules
```

Content:

```text
SUBSYSTEM=="dma_heap", GROUP="video", MODE="0660"
SUBSYSTEM=="video4linux", GROUP="video", MODE="0660"
KERNEL=="media*", GROUP="video", MODE="0660"
```

Apply and reboot:

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo reboot
```

Verify:

```bash
groups
rpicam-hello -n --list-cameras
```

Expected:

```text
0 : imx219 [3280x2464 10-bit RGGB]
```

### 6. Install MediaMTX

```bash
cd ~
wget https://github.com/bluenviron/mediamtx/releases/download/v1.19.0/mediamtx_v1.19.0_linux_arm64.tar.gz
tar -xzf mediamtx_v1.19.0_linux_arm64.tar.gz
```

---

## Part 1: CSI Camera — Hardware H264 RTSP Stream

Pipeline:

```text
IMX219 CSI → rpicam-vid [HW encode /dev/video11] → H264 → ffmpeg -c copy → MediaMTX → host VLC
```

### Terminal 1 on Pi: MediaMTX

```bash
cd ~ && ./mediamtx mediamtx.yml
```

### Terminal 2 on Pi: rpicam-vid + ffmpeg

```bash
rpicam-vid -n -t 0 \
  --width 1280 --height 720 \
  --framerate 30 \
  --bitrate 2000000 \
  --codec h264 \
  --profile baseline \
  --inline \
  -o - | \
ffmpeg -re -f h264 -i - \
  -c copy \
  -f rtsp \
  -rtsp_transport tcp \
  rtsp://127.0.0.1:8554/main
```

If the camera is upside down, add `--hflip --vflip` after `--inline`.

### Host: View stream

```bash
/snap/bin/vlc rtsp://172.20.50.60:8554/main
```

### Verified Performance

```text
mediamtx    ~3.46% CPU
rpicam-vid  ~15.70% CPU
ffmpeg      ~2.41% CPU
Total       ~21.57% CPU
```

---

## Part 2: IP Camera — MediaMTX Relay (No Transcode)

This relays the original 1280x720 H264 stream without decode or re-encode.

### MediaMTX Config

Edit `~/mediamtx.yml`, add under `paths:`:

```yaml
paths:
  ipcam:
    source: rtsp://192.168.168.14:8554/main.264
    rtspTransport: tcp
    sourceOnDemand: yes

  all_others:
```

### Terminal 1 on Pi: MediaMTX

```bash
cd ~ && ./mediamtx mediamtx.yml
```

### Host: View stream

```bash
/snap/bin/vlc rtsp://172.20.50.60:8554/ipcam
```

---

## Part 3: IP Camera — Hardware Decode + Scale + Hardware Encode RTSP Stream

This is the full transcode pipeline: decode the IP camera H264 stream using hardware, scale to a lower resolution, re-encode using hardware, and stream via RTSP.

Pipeline:

```text
IP camera H264 1280x720
→ [v4l2h264dec]   HW decode via /dev/video10
→ [v4l2convert]   HW scale + format convert via ISP (dmabuf-import)
→ [v4l2h264enc]   HW encode via /dev/video11
→ pipe stdout
→ ffmpeg -c copy → MediaMTX → host VLC
```

### Terminal 1 on Pi: MediaMTX

```bash
cd ~ && ./mediamtx mediamtx.yml
```

### Terminal 2 on Pi: GStreamer HW transcode + ffmpeg RTSP publish

320x240 stream:

gst-launch-1.0 -e   rtspsrc location="rtsp://192.168.168.14:8554/main.264" protocols=tcp latency=200 !   rtph264depay ! h264parse !   v4l2h264dec !   v4l2convert output-io-mode=dmabuf-import !   video/x-raw,width=320,height=240,format=I420 !   v4l2h264enc extra-controls="controls,video_bitrate=400000;" !   video/x-h264,level='(string)3' !   h264parse config-interval=1 !   fdsink fd=1 | ffmpeg -f h264 -i -   -c copy   -f rtsp   -rtsp_transport tcp   rtsp://127.0.0.1:8554/ipcam_320



To change resolution, modify three things:

```text
# To change resolution, modify the v4l2convert caps line and bitrate/level:
#
# 640x480:
#   video/x-raw,width=640,height=480,format=I420 !
#   v4l2h264enc extra-controls="controls,video_bitrate=800000;" !
#   video/x-h264,level='(string)3.1' !
#
# 1280x720 (same as source, only re-encode):
#   video/x-raw,width=1280,height=720,format=I420 !
#   v4l2h264enc extra-controls="controls,video_bitrate=2000000;" !
#   video/x-h264,level='(string)3.1' !
#
# Resolution guide:
#   320x240  → bitrate=400000   level=3     (low bandwidth)
#   640x480  → bitrate=800000   level=3.1   (medium quality)
#   1280x720 → bitrate=2000000  level=3.1   (full HD re-encode)
```

### Host: View stream

```bash
/snap/bin/vlc rtsp://172.20.50.60:8554/ipcam_320
```

### Verified Hardware Usage

```bash
sudo fuser -v /dev/video10 /dev/video11
```

Confirmed:

```text
/dev/video10: pis2 ... gst-launch-1.0   (HW decode)
/dev/video11: pis2 ... gst-launch-1.0   (HW encode)
```

### Verified Performance (320x240)

```bash
pidstat 1 5 | grep -E "ffmpeg|gst-launch|mediamtx"
```

Measured (with videoscale, before v4l2convert optimization):

```text
gst-launch-1.0  ~74% CPU  (HW decode + SW scale + HW encode)
ffmpeg           ~1% CPU   (-c copy only, no re-encode)
mediamtx         ~1.6% CPU
Total            ~77% CPU
```

With `v4l2convert output-io-mode=dmabuf-import` (hardware ISP scaling), CPU should be significantly lower since scaling is also offloaded to hardware.

For comparison, full software encode with libx264 uses ~156% CPU.

---

## Troubleshooting

### Hardware encoder hangs or outputs 0 bytes

Root cause is almost always insufficient CMA memory.

Check:

```bash
cat /proc/meminfo | grep -i Cma
```

If CmaTotal is below 262144 kB, fix `/boot/firmware/config.txt`:

```text
[all]
dtoverlay=vc4-kms-v3d,cma-256
```

Make sure `display_auto_detect=1` is commented out. Reboot.

### GStreamer v4l2h264enc "Failed to process frame"

Two causes after CMA fix:

1. Missing format conversion before encoder — use `v4l2convert output-io-mode=dmabuf-import` with explicit `format=I420` in caps.
2. Missing explicit H264 level cap — add `video/x-h264,level='(string)3' !` after `v4l2h264enc`.

### VLC shows "no data received in 10s"

Try forcing TCP transport on VLC:

```bash
/snap/bin/vlc --rtsp-tcp rtsp://172.20.50.60:8554/ipcam_320
```

### Debug driver issues

```bash
# Enable bcm2835-codec debug logging
sudo sh -c 'echo 5 > /sys/module/bcm2835_codec/parameters/debug'
dmesg -w

# Disable after debugging
sudo sh -c 'echo 0 > /sys/module/bcm2835_codec/parameters/debug'
```
