import cv2
import numpy as np
import pyrealsense2 as rs
import matplotlib.pyplot as plt
from thermal.flir_camera import FLIRCamera
import argparse
import time




def main():
    try:
        with FLIRCamera("/dev/video52").configure_gain(temp_min=15.0, temp_max=45, auto_gain=False) as flir_cam:
            while True:
                ret_th, flir_frame = flir_cam.read()

                if not ret_th:
                    continue

                thermal_resized = cv2.resize(flir_frame, (640, 480))
                thermal_gray = (
                    cv2.cvtColor(thermal_resized, cv2.COLOR_RGB2GRAY)
                    if thermal_resized.ndim == 3 else thermal_resized
                )

                cv2.imshow('Thermal stream', thermal_gray)

                key = cv2.waitKey(1) & 0xFF
                if key in (ord('q'), 27):
                    break

    finally:
        cv2.destroyAllWindows()
        print('Camera closed.')

if __name__ == "__main__":
    main()