# Self_Driving_Car
Lane and Obstacle Detection on Pre-Recorded Dataset

# Real-Time Lane Detection and Driving Decision System

This project implements a **real-time lane detection and driving decision system** using classical computer vision techniques in OpenCV. It processes video input to detect road lanes, estimate the drivable path, and infer turn directions and obstacles using methods like polynomial curve fitting and Hough Line detection.

## Features

* **Lane Line Detection**: Identifies left and right lane lines using pixel scanning from the bottom up.
* **Drivable Path Estimation**: Uses midpoints and curve fitting to compute the optimal center driving path.
* **Steering Angle Estimation**: Calculates a normalized turn angle using the first derivative of the center path.
* **Center Offset Calculation**: Measures vehicle's lateral deviation from lane center.
* **Obstacle Detection**: Detects potential road obstacles using edge detection and Hough lines.
* **Frame Skipping**: Improves performance by processing every Nth frame.

## Technologies Used

* Python 3.x
* OpenCV
* NumPy
* Math

##  Directory Structure

```
.
├── lane_detection.py     # Main Python file with all processing logic
├── video/                # Folder for input road videos (modify path accordingly)
└── README.md             # This file
```

---

## How to Run

1. **Install Dependencies**

```bash
pip install opencv-python numpy
```

2. **Add Your Video**

Place your test road video in an accessible path. Update the video path in the script:

```python
video_path = "D:\\sem6\\dip\\projects\\DIP Project Videos\\PXL_20250325_044603023.TS.mp4"
```

3. **Run the Script**

```bash
python lane_detection.py
```

You will see a visualization window with:

* Blue lane boundary midpoints
* Red center driving path
* Green lane-fill polygon
* Yellow arrows indicating turning direction
* Detected obstacle lines (if present)

---

## Parameters

* `frame_skip`: Controls how many frames to skip to maintain real-time performance.
* `lookahead_distance_pixels`: Determines how far ahead the system looks to calculate turn angle.
* `scale_down_factor`: Resize factor for performance optimization.
* `dilate_kernel`: Kernel used to dilate lane lines and improve visibility.

---

## How It Works

1. **Preprocessing**

   * Converts frame to grayscale
   * Applies thresholding, Gaussian blur, and dilation
   * Converts binary image to 3-channel for visualization

2. **Lane Detection**

   * Bottom-up scan to find left and right lane edges
   * Calculates midpoints
   * Fills missing segments using extrapolation

3. **Path Planning**

   * Fits a polynomial curve to midpoints (left, right, center)
   * Highlights center path in red
   * Computes turn angle using curve derivative

4. **Obstacle Detection**

   * Applies CLAHE and Canny edge detection
   * Uses Hough Transform to highlight possible obstacles
   * Marks detection zones (Left, Center, Right)

---

## Sample Output
![Screenshot (208)](https://github.com/user-attachments/assets/5671cde5-c83a-4621-bcec-353ee8673de2)
![Screenshot (209)](https://github.com/user-attachments/assets/67749245-1026-44b2-af30-8ce78713c40b)
![Screenshot (210)](https://github.com/user-attachments/assets/ba43964e-1a12-4434-b698-0d776efbac31)



##  Notes

* This system is intended for educational and research purposes.
* Works well on videos with visible lanes and moderate lighting.
* Consider using more robust techniques (e.g., deep learning) for deployment in uncontrolled environments.

## Took help from other upload Github codes for understanding and logic-building

##  Contributing

Pull requests are welcome. For major changes, please open an issue first to discuss what you'd like to change or improve.



