#!/bin/bash
# Download required models for Barry the Greeter demo
# These models are too large for git and are downloaded on-demand
#
# Usage: ./scripts/download_models.sh
#
# Models downloaded:
#   - OpenCV DNN face detector (res10_300x300_ssd) ~10MB

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

# Face detection model directory
FACE_MODEL_DIR="$PROJECT_ROOT/models/face_detection"
mkdir -p "$FACE_MODEL_DIR"

echo "=== Downloading Face Detection Models ==="

# OpenCV DNN face detector prototxt
PROTOTXT_URL="https://raw.githubusercontent.com/opencv/opencv/master/samples/dnn/face_detector/deploy.prototxt"
PROTOTXT_FILE="$FACE_MODEL_DIR/deploy.prototxt"

if [ ! -f "$PROTOTXT_FILE" ]; then
    echo "Downloading deploy.prototxt..."
    curl -L -o "$PROTOTXT_FILE" "$PROTOTXT_URL"
    echo "  -> Saved to $PROTOTXT_FILE"
else
    echo "  deploy.prototxt already exists, skipping"
fi

# OpenCV DNN face detector weights (Caffe model)
MODEL_URL="https://raw.githubusercontent.com/opencv/opencv_3rdparty/dnn_samples_face_detector_20170830/res10_300x300_ssd_iter_140000.caffemodel"
MODEL_FILE="$FACE_MODEL_DIR/res10_300x300_ssd_iter_140000.caffemodel"

if [ ! -f "$MODEL_FILE" ]; then
    echo "Downloading res10_300x300_ssd_iter_140000.caffemodel (~10MB)..."
    curl -L -o "$MODEL_FILE" "$MODEL_URL"
    echo "  -> Saved to $MODEL_FILE"
else
    echo "  res10_300x300_ssd_iter_140000.caffemodel already exists, skipping"
fi

echo ""
echo "=== Model Download Complete ==="
echo "Face detection models are ready at: $FACE_MODEL_DIR"
ls -lh "$FACE_MODEL_DIR"
