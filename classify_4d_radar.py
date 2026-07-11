import numpy as np
import argparse
import os
import matplotlib.pyplot as plt

def classify_4d_radar(input_txt, output_txt, v_mean, v_cov, auto_ego, plot):
    """
    Implements Go-RIO Feature Classification (Static vs. Dynamic) for 4D Radar.
    """
    if not os.path.exists(input_txt):
        print(f"Error: {input_txt} not found.")
        return

    print(f"Loading data from {input_txt}...")
    data = np.loadtxt(input_txt, skiprows=1)
    if len(data) == 0:
        print("Empty file.")
        return
        
    points_xyz = data[:, 0:3]
    doppler_obs = data[:, 3]
    rcs = data[:, 4]
    
    # -------------------------------------------------------------------
    # Step 1: Ground Noise Filter
    # -------------------------------------------------------------------
    z_threshold = -1.2
    rcs_noise_threshold = 2.0
    non_ground_mask = ~((points_xyz[:, 2] < z_threshold) & (rcs < rcs_noise_threshold))
    
    filtered_xyz = points_xyz[non_ground_mask]
    filtered_doppler = doppler_obs[non_ground_mask]
    filtered_rcs = rcs[non_ground_mask]
    
    # Compute radial vectors n_i
    ranges = np.linalg.norm(filtered_xyz, axis=1)
    valid_range = ranges > 1e-4
    n_i = np.zeros_like(filtered_xyz)
    n_i[valid_range] = filtered_xyz[valid_range] / ranges[valid_range, np.newaxis]
    
    # -------------------------------------------------------------------
    # OPTIONAL: Auto-Compute Ego Velocity if IMU missing (Robust RANSAC / IRLS)
    # -------------------------------------------------------------------
    if auto_ego:
        print("Auto-estimating Ego Velocity using Robust Iterative Least Squares...")
        # (N^T * N)^-1 * N^T * D
        N_valid = n_i[valid_range]
        D_valid = filtered_doppler[valid_range]
        
        # 1st Pass: Naive Least Squares (bị nhiễm nhiễu bởi vật MBO)
        v_ls, _, _, _ = np.linalg.lstsq(N_valid, D_valid, rcond=None)
        
        # 2nd Pass: Loại bỏ Outliers (các điểm lệch > 0.4 m/s so với vận tốc LS để loại bỏ Dynamic bias)
        residuals_1st = np.abs(N_valid @ v_ls - D_valid)
        inlier_mask = residuals_1st < 0.4
        
        N_inliers = N_valid[inlier_mask]
        D_inliers = D_valid[inlier_mask]
        
        # Fit lần 2 trên tập Inliers sạch (nhập nền móng tĩnh)
        if len(D_inliers) > 10:
            v_robust, _, _, _ = np.linalg.lstsq(N_inliers, D_inliers, rcond=None)
            rmse = np.sqrt(np.mean(np.square(N_inliers @ v_robust - D_inliers)))
            v_mean = v_robust
            print(f"  -> Robust Ego Velocity [vx, vy, vz]: {v_mean}")
            print(f"  -> Inliers used for Ego Estimation: {len(D_inliers)} / {len(D_valid)}")
        else:
            rmse = np.sqrt(np.mean(np.square(N_valid @ v_ls - D_valid)))
            v_mean = v_ls
            print("  -> Warning: Not enough inliers. Fallback to naive LS.")

        v_cov = np.diag([rmse**2, rmse**2, rmse**2])
        
    print(f"Propagating IMU Mean: {v_mean}")
    
    # -------------------------------------------------------------------
    # Step 2: Continuous Velocity Preintegration Classification
    # -------------------------------------------------------------------
    sigma_hardware = 0.05  # Giảm mức sai số phần cứng lý thuyết xuống để nhạy hơn (Radar xịn thường rất chính xác <0.06m/s)
    sigma_hardware_squared = sigma_hardware**2 
    labels = np.zeros(len(filtered_xyz), dtype=int) # 0: Static, 1: Dynamic
    
    d_expect = np.sum(n_i * v_mean, axis=1)
    var_expect = np.sum((n_i @ v_cov) * n_i, axis=1) + sigma_hardware_squared
    sigma_expect = np.sqrt(var_expect)
    
    mahalanobis_err = np.abs(filtered_doppler - d_expect) / sigma_expect
    
    # Chi-Square Threshold (Giảm ngưỡng khắt khe từ 3.0 (99.7%) xuống 2.0 (95.4%) để bắt được nhiều Dynamic hơn)
    chi_square_threshold = 2.0
    dynamic_mask = mahalanobis_err > chi_square_threshold
    labels[dynamic_mask] = 1
    
    num_dyn = np.sum(dynamic_mask)
    num_stat = len(labels) - num_dyn
    print(f"  Detected Dynamic Points: {num_dyn}")
    print(f"  Detected Static Points:  {num_stat}")
    
    os.makedirs(os.path.dirname(output_txt), exist_ok=True)
    out_data = np.hstack([filtered_xyz, filtered_doppler[:, None], filtered_rcs[:, None], labels[:, None], mahalanobis_err[:, None]])
    np.savetxt(output_txt, out_data, fmt="%.6f", header="X Y Z Doppler RCS Label Mahalanobis_Error", comments="")
    
    # -------------------------------------------------------------------
    # Step 3: Visualization
    # -------------------------------------------------------------------
    if plot:
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')
        
        # Plot Static Background (Blue)
        stat_xyz = filtered_xyz[~dynamic_mask]
        if len(stat_xyz) > 0:
            ax.scatter(stat_xyz[:,0], stat_xyz[:,1], stat_xyz[:,2], c='blue', s=2, alpha=0.3, label=f'Static ({num_stat})')
            
        # Plot Dynamic Objects (Red)
        dyn_xyz = filtered_xyz[dynamic_mask]
        if len(dyn_xyz) > 0:
            ax.scatter(dyn_xyz[:,0], dyn_xyz[:,1], dyn_xyz[:,2], c='red', s=8, alpha=0.9, marker='x', label=f'Dynamic ({num_dyn})')
            
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        ax.set_title(f'Go-RIO Radar Feature Classification (Auto Ego Velocity)\nv_ego: {v_mean.round(2)}')
        ax.legend()
        
        plot_path = output_txt.replace('.txt', '.png')
        plt.savefig(plot_path, dpi=300, bbox_inches='tight')
        print(f"Saved visualization to {plot_path}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True, help="Input 5D txt file")
    parser.add_argument("--output", required=True, help="Output classified txt file")
    parser.add_argument("--vx", type=float, default=0.0)
    parser.add_argument("--vy", type=float, default=0.0)
    parser.add_argument("--vz", type=float, default=0.0)
    parser.add_argument("--auto_ego", action="store_true", help="Auto-calculate ego velocity using Least Squares if IMU vectors are unknown.")
    parser.add_argument("--plot", action="store_true", help="Save a matplotlib visualization PNG")
    
    args = parser.parse_args()
    v_mean = np.array([args.vx, args.vy, args.vz])
    v_cov = np.diag([0.05**2, 0.05**2, 0.05**2])
    
    classify_4d_radar(args.input, args.output, v_mean, v_cov, args.auto_ego, args.plot)
