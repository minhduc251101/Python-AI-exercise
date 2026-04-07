#!/usr/bin/env python3
import os
import sys
import argparse
import numpy as np

def inspect_pcd_header(pcd_path, max_lines=30):
    if not os.path.exists(pcd_path):
        print(f"[ERROR] PCD file not found: {pcd_path}")
        sys.exit(1)

    header_lines = []
    with open(pcd_path, "rb") as f:
        for _ in range(max_lines):
            line = f.readline()
            if not line:
                break
            try:
                s = line.decode("utf-8", errors="ignore").strip()
            except Exception:
                s = str(line)
            header_lines.append(s)
            if s.startswith("DATA"):
                break

    print("\n[PCD HEADER]")
    for line in header_lines:
        print(" ", line)

    header_text = "\n".join(header_lines).lower()

    keywords = ["time", "timestamp", "stamp", "t", "fields"]
    found = {k: (k in header_text) for k in keywords}

    print("\n[PCD TIMESTAMP CHECK]")
    for k, v in found.items():
        print(f"  contains '{k}': {v}")

    if "fields" in header_text:
        for line in header_lines:
            if line.lower().startswith("fields"):
                print(f"  Parsed FIELDS line: {line}")
                break

def inspect_gt(gt_path):
    if not os.path.exists(gt_path):
        print(f"[ERROR] GT file not found: {gt_path}")
        sys.exit(1)

    for delim in [',', None]:
        try:
            data = np.loadtxt(gt_path, delimiter=delim, comments='#')
            if data.ndim == 1:
                data = data.reshape(1, -1)
            break
        except Exception:
            continue
    else:
        print(f"[ERROR] Could not parse GT file: {gt_path}")
        sys.exit(1)

    nrows, ncols = data.shape
    print("\n[GT FILE CHECK]")
    print(f"  Rows: {nrows}")
    print(f"  Cols: {ncols}")

    first_col = data[:, 0]
    print(f"  First 5 values in column 0: {first_col[:5]}")

    large_vals = np.sum(first_col > 1e9)
    print(f"  Count of col0 > 1e9: {large_vals}/{nrows}")

    if large_vals > 0:
        print("  Column 0 looks timestamp-like.")
    else:
        print("  Column 0 may be pose data, frame index, or something else.")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--map", required=True, help="Path to PCD map file")
    parser.add_argument("--gt_poses", required=True, help="Path to GT pose file")
    args = parser.parse_args()

    print("=" * 70)
    print("CHECK MAP / GT TIMESTAMP POSSIBILITY")
    print("=" * 70)

    inspect_pcd_header(args.map)
    inspect_gt(args.gt_poses)

    print("\n[INTERPRETATION]")
    print("  - If GT col0 is timestamp-like, GT likely contains timestamps.")
    print("  - If PCD FIELDS includes something like time/timestamp/t, the PCD may store time info.")
    print("  - If the map PCD has only x y z (or x y z intensity), it is probably just a fused map.")
    print("  - In that case, you cannot directly verify 'same timestamps' between map and GT.")

if __name__ == "__main__":
    main()