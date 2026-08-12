# End-to-End Execution Guide for 4D Radar Feature Extraction & Classification

This guide describes how to run the data extraction and classification pipeline from start to finish.

## Prerequisites
Make sure you have `numpy` installed. If you haven't, run:
```bash
pip install numpy
```

## Step 1: Converting Raw Binary PCD to 5D Feature Text
We first need to extract the raw `.pcd` fields (including Doppler and RCS) into a clean, human-readable `.txt` file containing 5 columns: `X, Y, Z, Doppler, RCS`.

Run the custom binary parser script `pcd_to_xyz.py` and pass the path to your raw PCD folder and your desired output text file.

```bash
# Example syntax:
python3 pcd_to_xyz.py \
    --input "/media/minhduc/TOSHIBA EXT1/4dradarslam/NTU4Dradlm/cp/cp/loop_true_fullpose/000900/cloud.pcd" \
    --output "/media/minhduc/TOSHIBA EXT1/C++ hoc/Python-AI-exercise/results_900_1200/output_000900.txt"
```
**Outcome**: A new text file is created where each row possesses the 5 critical spatio-kinematic features. You can now reliably analyze Doppler velocity and intensity.

## Step 2: Go-RIO SOTA Classification (Detecting Dynamic Objects)
With the fully populated 5D text file, we can classify point clouds by splitting dynamic objects (cars, pedestrians) from static background elements (walls, road) using the Go-RIO GP Velocity Integration mechanism.

Run the `classify_4d_radar.py` script. You must specify the `--vx`, `--vy`, `--vz` arguments depending on the Ego (car) velocity sampled from the vehicle's IMU at that frame.

```bash
# Example syntax (assuming the car is moving forward at 1.5 m/s):
python3 classify_4d_radar.py \
    --input "/media/minhduc/TOSHIBA EXT1/C++ hoc/Python-AI-exercise/results_900_1200/output_000900.txt" \
    --output "/media/minhduc/TOSHIBA EXT1/C++ hoc/Python-AI-exercise/results_900_1200/classified_000900.txt" \
    --vx 1.5 \
    --vy 0.0 \
    --vz 0.0
```

**Outcome**: A classified text file is generated where a 6th column (`Label`) is added. 
- `Label = 0`: Static Background
- `Label = 1`: Dynamic Object (Moving)

## Step 3 (Optional): Batch Processing
If you have thousands of frame directories (e.g. `000000` to `002000`), you can simply loop through them in bash:

```bash
for dir in /media/minhduc/TOSHIBA\ EXT1/4dradarslam/NTU4Dradlm/cp/cp/loop_true_fullpose/*; do
  frame=$(basename "$dir")
  
  # Export to 5D TxT
  python3 pcd_to_xyz.py \
    --input "$dir/cloud.pcd" \
    --output "results_batch/output_${frame}.txt"
    
  # Classify (assumes vx=1.5 for all frames here as placeholder)
  python3 classify_4d_radar.py \
    --input "results_batch/output_${frame}.txt" \
    --output "results_batch/classified_${frame}.txt" \
    --vx 1.5
done
```

That's it! Your pipeline is fully functional and operates seamlessly end to end.
