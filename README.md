# Object Detection in Autonomous Driving System

[![CI](https://github.com/RoshFps/Object-Detection-in-Autonomous-Driving-System/actions/workflows/ci.yml/badge.svg)](https://github.com/RoshFps/Object-Detection-in-Autonomous-Driving-System/actions/workflows/ci.yml)
![Python](https://img.shields.io/badge/python-3.8%2B-blue)
![YOLOv5](https://img.shields.io/badge/detector-YOLOv5-00FFFF)
![Arduino](https://img.shields.io/badge/hardware-Arduino-00979D)

A small-scale self-driving car prototype. A camera feeds several vision modules running on a laptop, and each module decides whether the car should stop, go forward or turn. It sends that decision as a single byte over USB serial to an Arduino, which drives the motors.

## Modules

| Script | What it detects | Method | Commands sent |
| --- | --- | --- | --- |
| `objects.py` | People, vehicles and animals in the path | YOLOv5s (COCO) | `S` if any are present, else `F` |
| `pathole_hump.py` | Potholes and speed humps | Custom-trained YOLOv5 (`pathole_hump.pt`) | `S` / `F` |
| `traffic_light.py` | Red / green traffic lights | Custom-trained YOLOv5 (`traffic_light.pt`) | `S` on any red light, else `F` |
| `road_sign.py` | Stop and direction signs | Hough circles + k-means colour analysis | `S`, `L`, `R`, `F` |
| `LaneDetection.py` | Lane curvature | Undistort → threshold → perspective warp → sliding-window polynomial fit | `L`, `R`, `F` (with `--arduino`) |

```
camera ─► vision module ─► decision (S/F/L/R) ─► serial_test.Send ─► Arduino ─► motor driver
```

### Serial protocol

| Byte | Meaning |
| --- | --- |
| `S` | Stop |
| `F` | Forward |
| `L` | Turn left |
| `R` | Turn right |

A command is only sent when it changes, so the serial line isn't flooded every frame. On exit the car is sent `S`.

## Setup

The YOLO scripts reuse the `models/` and `utils/` packages from [ultralytics/yolov5](https://github.com/ultralytics/yolov5). Clone it and copy this repository's files into it, or add it to `PYTHONPATH`:

```bash
git clone https://github.com/ultralytics/yolov5.git
git clone https://github.com/RoshFps/Object-Detection-in-Autonomous-Driving-System.git
cp Object-Detection-in-Autonomous-Driving-System/*.py Object-Detection-in-Autonomous-Driving-System/*.pt \
   Object-Detection-in-Autonomous-Driving-System/camera_calibration.json yolov5/
cd yolov5 && pip install -r ../Object-Detection-in-Autonomous-Driving-System/requirements.txt
```

**Arduino connection.** The port is detected automatically. To choose it yourself, set it before running:

```bash
export ARDUINO_PORT=/dev/ttyUSB0      # Windows: set ARDUINO_PORT=COM11
```

With no board connected the scripts run in **dry-run mode** and log the commands, which is useful for testing on a laptop.

## Usage

```bash
python objects.py --source 0                 # webcam
python pathole_hump.py --source road.mp4
python traffic_light.py --source 0 --conf-thres 0.8
python road_sign.py --camera 0
python LaneDetection.py --video project_video.mp4
python LaneDetection.py --camera 1 --arduino
```

Press `q` to quit the OpenCV windows.

## Hardware

- Arduino Uno (or compatible) with a motor driver (e.g. L298N)
- USB webcam
- Ultrasonic sensors and a chassis with DC motors

## Reliability and safety fixes in this version

- The stop decision no longer depends on whether frames are being drawn or saved. Previously, `--nosave` meant the car never received a stop.
- The traffic-light module decides once per frame, instead of sending a command for every detected box.
- Video output uses one writer per stream. Before, a new file was created on every frame, overwriting the video and leaking handles.
- Lane detection exits cleanly at the end of a video, and the road-sign module no longer crashes on current SciPy versions.
- The camera calibration is stored as JSON and loaded without `pickle`, since unpickling a file can execute arbitrary code. It is also loaded once instead of on every frame.
- The serial link no longer hard-codes `COM11` or crashes when no board is attached.

## Tests

```bash
pip install numpy opencv-python-headless pyserial
python -m unittest discover -s tests -t . -v
```

## Credits

- Sai Gokul KP ([@saigokul290](https://github.com/saigokul290))
- Roshan Immanuel ([@RoshFps](https://github.com/RoshFps))
- Rohan S

`train.py`, `val.py`, `export.py` and the detection-script scaffolding are adapted from [Ultralytics YOLOv5](https://github.com/ultralytics/yolov5), which is licensed under **AGPL-3.0**. Those files remain under AGPL-3.0; see the notice at the top of each file.
