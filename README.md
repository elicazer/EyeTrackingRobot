# EyeTrackingRobot

A computer vision-based eye tracking robot system that uses face detection and distance estimation to control robotic eye movements with realistic blinking behavior.

## 🎬 Demo

[![Eye Tracking Robot Demo](https://img.youtube.com/vi/D7Y-rvqnFdY/0.jpg)](https://youtube.com/shorts/D7Y-rvqnFdY?si=tRScbJB0Ge1I85Ub)

*Click the image above to watch the Eye Tracking Robot in action!*

## 🎯 Features

- **Real-time Face Detection**: Uses OpenCV's Haar Cascade classifier for accurate face detection
- **Distance Estimation**: Calculates distance to faces using focal length and known face dimensions
- **Servo Control**: Controls multiple servos for realistic eye movements and eyelid animations
- **Automatic Blinking**: Random blinking behavior with configurable intervals
- **Speed Calculation**: Estimates movement speed based on distance changes over time
- **Multi-Platform Support**: Works on desktop, Raspberry Pi, and Jetson Nano
- **Reference Image Capture**: Tools for capturing calibration images

## 🏗️ Project Structure

```
EyeTrackingRobot/
├── distance.py              # Main distance estimation with xArm control
├── facerec.py              # Advanced face tracking with servo control
├── Updated_distance.py      # Enhanced distance measurement
├── Capture_Reference_image/ # Reference image capture tools
├── Raspberry_pi/           # Raspberry Pi specific implementations
├── Speed/                  # Speed calculation modules
├── requirements.txt        # Python dependencies
└── haarcascade_frontalface_default.xml # Face detection model
```

## 🔧 Hardware Requirements

### Servos
- **SERVO_LREX** (Channel 0): Left/Right Eye X-axis
- **SERVO_LREY** (Channel 1): Left/Right Eye Y-axis  
- **SERVO_LUPLID** (Channel 2): Left Upper Eyelid
- **SERVO_RUPLID** (Channel 3): Right Upper Eyelid
- **SERVO_LLOLID** (Channel 4): Left Lower Eyelid
- **SERVO_RLOLID** (Channel 5): Right Lower Eyelid

### Supported Platforms
- **Desktop**: Standard USB webcam
- **Raspberry Pi**: Pi Camera or USB webcam
- **Jetson Nano**: CSI camera with GStreamer support

## 📦 Installation

### Dependencies
```bash
pip install -r requirements.txt
```

### Raspberry Pi Setup
```bash
sudo apt install ffmpeg python3-opencv
sudo apt install libxcb-shm0 libcdio-paranoia-dev libsdl2-2.0-0 libxv1 libtheora0 libva-drm2 libva-x11-2 libvdpau1 libharfbuzz0b libbluray2 libatlas-base-dev libhdf5-103 libgtk-3-0 libdc1394-22 libopenexr23
```

## 🚀 Usage

### Basic Face Tracking
```bash
python facerec.py
```

### Distance Estimation Only
```bash
python distance.py
```

### Speed Calculation
```bash
cd Speed/
python speed.py
```

### Capture Reference Images
```bash
cd Capture_Reference_image/
python Capture_Reference_Image.py
```

## ⚙️ Configuration

### Distance Calibration
- **KNOWN_DISTANCE**: 76.2 cm (measured distance for reference)
- **KNOWN_WIDTH**: 14.3 cm (average face width)
- **ref_image_face_width**: 182 pixels (face width in reference image)

### Video Settings
- **Resolution**: 640x480 (configurable)
- **FPS**: 20 (adjustable based on hardware)

### Servo Angles
- **Eye Y Range**: 52° - 112°
- **Eye X Range**: 57° - 145°

## 🎮 Controls

- **'q'**: Quit application
- **'c'**: Capture reference images (in capture mode)

## 🔬 Technical Details

### Distance Calculation
The system uses the pinhole camera model:
```
Distance = (Real_Width × Focal_Length) / Object_Width_in_Image
Focal_Length = (Object_Width_in_Image × Known_Distance) / Real_Width
```

### Eye Movement Mapping
Face coordinates are mapped to servo angles using linear interpolation:
```python
servo_angle = map_value(face_position, 0, video_dimension, servo_max, servo_min)
```

## 📁 Module Descriptions

- **distance.py**: Core distance estimation with xArm robotic arm integration
- **facerec.py**: Advanced face recognition with ServoKit control and blinking
- **Updated_distance.py**: Enhanced distance measurement with visual feedback
- **Speed/**: Speed calculation based on distance changes over time
- **Raspberry_pi/**: Platform-specific implementations for Pi
- **Capture_Reference_image/**: Tools for capturing calibration images

## 🤝 Contributing

Contributions are welcome! Please feel free to submit pull requests or open issues for bugs and feature requests.

## 📄 License

This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

This project demonstrates the power of computer vision and distance estimation using simple mathematical principles. Special thanks to the OpenCV community for providing excellent tools for computer vision applications.

- Original concept by AiPhile (Asadullah Dal)
- OpenCV community for computer vision tools
- Adafruit for servo control libraries
