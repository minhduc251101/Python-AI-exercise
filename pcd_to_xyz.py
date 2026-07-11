import numpy as np
import os
import argparse
import sys

def pcd_to_5d_txt(pcd_path, txt_path):
    if not os.path.exists(pcd_path):
        print(f"Error: {pcd_path} not found.")
        sys.exit(1)

    with open(pcd_path, 'rb') as f:
        header_lines = []
        while True:
            line = f.readline().decode('ascii', errors='ignore').strip()
            header_lines.append(line)
            if line.startswith('DATA '):
                if line.split()[1] != 'binary':
                    print("Error: Script currently only supports binary DATA format PCD.")
                    sys.exit(1)
                break
        
        raw_data = f.read()
        
        # 4D Radar Header Fields: x y z rcs normal_x normal_y normal_z doppler
        # 32 bytes per point setup
        dt = np.dtype([
            ('x', np.float32), ('y', np.float32), ('z', np.float32), 
            ('rcs', np.float32), 
            ('nx', np.float32), ('ny', np.float32), ('nz', np.float32),
            ('doppler', np.float32)
        ])
        
        cloud_data = np.frombuffer(raw_data, dtype=dt)
        
        # Extract features (X, Y, Z, Doppler, RCS) to match our classification pipeline requirements
        out_data = np.vstack([
            cloud_data['x'], cloud_data['y'], cloud_data['z'],
            cloud_data['doppler'], cloud_data['rcs']
        ]).T
        
        os.makedirs(os.path.dirname(txt_path), exist_ok=True)
        # Using comma delimiter or space delimiter. Savetxt defaults to space.
        np.savetxt(txt_path, out_data, fmt="%.6f", header="X Y Z Doppler RCS", comments="")
        print(f"Successfully converted {pcd_path} -> {txt_path}")
        print(f"Points extracted: {len(out_data)}. Fields: [X, Y, Z, Doppler, RCS]")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert binary Radar PCD to 5-column txt")
    parser.add_argument("--input", default="/media/minhduc/TOSHIBA EXT1/4dradarslam/NTU4Dradlm/cp/cp/loop_true_fullpose/000900/cloud.pcd", help="Path to input .pcd")
    parser.add_argument("--output", default="/media/minhduc/TOSHIBA EXT1/C++ hoc/Python-AI-exercise/results_900_1200/output_000900.txt", help="Path to output .txt")
    
    args = parser.parse_args()
    pcd_to_5d_txt(args.input, args.output)
