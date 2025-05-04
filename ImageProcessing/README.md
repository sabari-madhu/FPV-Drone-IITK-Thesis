# ImageProcessing

This folder contains all image processing experiments and prototype implementations used during the development and testing phases of the thesis. These experiments cover a range of topics such as object detection, tracking, depth estimation, template matching, and custom video frame processing using both classical and deep learning-based methods.

Each Jupyter notebook demonstrates a self-contained experiment or use-case designed to test specific visual processing capabilities that were later integrated into the autonomous drone navigation system or supporting analysis.

---

## 📁 Notebooks Overview

### 1. **DPT_Model_Experimentation.ipynb**
- **Objective**: Estimate per-pixel depth from an RGB image using the DPT (Depth Prediction Transformer) model.
- **Techniques**: Vision Transformers, PyTorch inference, depth map post-processing.
- **Usage**: Used to experiment with monocular depth estimation capabilities for potential 3D awareness.

---

### 2. **HaarCascade_and_GroPro.ipynb**
- **Objective**: Detect heads or faces in fisheye images (captured using a GoPro camera) using classical Haar Cascade classifiers.
- **Techniques**: Frame splitting, distortion handling, Haar detection on vertical image slices.
- **Usage**: Evaluates how classical CV tools perform under wide-angle lens distortions.

---

### 3. **PatternChecker.ipynb**
- **Objective**: Detect a specific visual pattern in a target image using multi-scale template matching and feature-based methods.
- **Techniques**: Template Matching, ORB/SIFT-based feature matching, confidence score computation.
- **Usage**: Used to validate visual pattern recognition and matching schemes for static references in the environment.

---

### 4. **YOLO_DeepSort_Tracking.ipynb**
- **Objective**: Combine YOLO for person detection with DeepSORT for persistent multi-object tracking in video feeds.
- **Techniques**: Object detection, tracking, motion scoring, ID association.
- **Usage**: Forms the foundation for person-following behavior or crowd movement estimation logic.

---

### 5. **YOLO_Experiments.ipynb**
- **Objective**: Test Ultralytics YOLO models for object detection and segmentation on both images and real-time camera streams.
- **Techniques**: YOLOv8 detection/segmentation APIs, real-time video processing.
- **Usage**: Explores baseline model performance and segmentation capabilities in diverse scenarios.

---

## 🔧 Dependencies

Make sure to install the following Python libraries to run the notebooks:

```bash
pip install torch torchvision opencv-python matplotlib ultralytics deep_sort_realtime
```

For template matching and feature-based methods, `opencv-contrib-python` may be required:

```bash
pip install opencv-contrib-python
```

## 🧪 Purpose

This directory was primarily used for visual experimentation, model validation, and algorithmic trials. It supports the broader system developed for autonomous navigation in GPS-denied environments. Each notebook here served as a standalone prototype to benchmark specific techniques before integrating them into the full drone system.

## 📌 Notes

*   Some notebooks use live camera feeds. Ensure a webcam or video input is available when executing those cells.
*   Pretrained model weights (e.g., `yolov8n.pt`, `yolov8n-seg.pt`) may be downloaded automatically by the respective APIs.
*   The notebooks are modular and can be recombined or adapted into ROS2 nodes or integrated pipelines as needed.
