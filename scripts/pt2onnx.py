import sys
from ultralytics import YOLO
YOLO(sys.argv[1]).export(format="onnx")
