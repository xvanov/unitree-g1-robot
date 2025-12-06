#!/bin/bash
# Create mock inspection session data for testing report generation
# Usage: ./scripts/create_test_report_data.sh [session_id]

SESSION_ID="${1:-test_insp}"
BASE_DIR="data/inspections/${SESSION_ID}"

echo "Creating mock inspection data in ${BASE_DIR}..."

# Create directories
mkdir -p "${BASE_DIR}/images"
mkdir -p "${BASE_DIR}/annotated"

# Create mock analysis_results.json (matches Story 1-8 output format)
cat > "${BASE_DIR}/analysis_results.json" << 'EOF'
{
  "session_id": "test_insp",
  "images_processed": 5,
  "total_defects": 3,
  "defects_by_image": [
    {
      "image": "img_00000001.jpg",
      "defects": [
        {
          "id": "def_001",
          "type": "QUALITY_ISSUE",
          "description": "Scratch on tile surface near entrance",
          "confidence": 0.87,
          "severity": "high",
          "trade": "finishes",
          "image_location": {"x": 320, "y": 240},
          "bounding_box": {"x": 280, "y": 200, "width": 80, "height": 80},
          "plan_location": {"x": 2.5, "y": 1.2}
        }
      ]
    },
    {
      "image": "img_00000003.jpg",
      "defects": [
        {
          "id": "def_002",
          "type": "LOCATION_ERROR",
          "description": "Outlet misaligned by 5cm from plan",
          "confidence": 0.92,
          "severity": "medium",
          "trade": "mep",
          "image_location": {"x": 150, "y": 300},
          "bounding_box": {"x": 120, "y": 270, "width": 60, "height": 60},
          "plan_location": {"x": 4.1, "y": 3.8}
        },
        {
          "id": "def_003",
          "type": "SAFETY_HAZARD",
          "description": "Exposed electrical wiring",
          "confidence": 0.95,
          "severity": "high",
          "trade": "mep",
          "image_location": {"x": 400, "y": 180},
          "bounding_box": {"x": 370, "y": 150, "width": 60, "height": 60},
          "plan_location": {"x": 4.5, "y": 3.5}
        }
      ]
    }
  ]
}
EOF

# Create a simple test plan image (gray rectangle)
# Uses ImageMagick if available, otherwise creates empty placeholder
if command -v convert &> /dev/null; then
    convert -size 800x600 xc:white \
        -fill lightgray -draw "rectangle 50,50 750,550" \
        -fill black -draw "rectangle 100,100 200,200" \
        -fill black -draw "rectangle 600,400 700,500" \
        "${BASE_DIR}/plan.png"
    echo "Created plan.png with ImageMagick"
else
    # Create minimal valid PNG (1x1 white pixel) as placeholder
    echo -n -e '\x89PNG\r\n\x1a\n\x00\x00\x00\rIHDR\x00\x00\x00\x01\x00\x00\x00\x01\x08\x02\x00\x00\x00\x90wS\xde\x00\x00\x00\x0cIDATx\x9cc\xf8\xff\xff?\x00\x05\xfe\x02\xfe\xa7V\xcf\x00\x00\x00\x00IEND\xaeB`\x82' > "${BASE_DIR}/plan.png"
    echo "Created placeholder plan.png (ImageMagick not available)"
fi

# Create mock image metadata files
for i in 1 2 3 4 5; do
    PADDED=$(printf "%08d" $i)
    # Create placeholder image (or use convert if available)
    if command -v convert &> /dev/null; then
        convert -size 640x480 xc:gray "${BASE_DIR}/images/img_${PADDED}.jpg"
    else
        touch "${BASE_DIR}/images/img_${PADDED}.jpg"
    fi

    # Create metadata JSON
    cat > "${BASE_DIR}/images/img_${PADDED}.json" << EOF
{
  "image_path": "images/img_${PADDED}.jpg",
  "timestamp_ms": $((1701705600000 + i * 1000)),
  "robot_pose": {"x": $((i * 2)).0, "y": 1.5, "theta": 0.0},
  "plan_coords": {"x": $((i * 2)).0, "y": 1.5},
  "camera_yaw": 0.0,
  "sequence_number": $i
}
EOF
done

echo "Created ${BASE_DIR}:"
echo "  - analysis_results.json (3 defects)"
echo "  - plan.png"
echo "  - images/img_*.jpg + .json (5 images)"
echo ""
echo "Run report generation with:"
echo "  ./build/g1_inspector --generate-report --inspection ${SESSION_ID}"
