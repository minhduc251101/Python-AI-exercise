import numpy as np

class RadarDataAugmentor:
    def __init__(self, doppler_std=0.1, rcs_scale_range=(0.8, 1.2)):
        """
        A data augmentor for 4D Radar Point Clouds.
        Assumes input is an (N, 5) numpy array with columns: [x, y, z, rcs, doppler]
        """
        self.doppler_std = doppler_std
        self.rcs_scale_range = rcs_scale_range

    def add_doppler_noise(self, points):
        """ 
        Add Gaussian noise to Doppler measurements to simulate sensor noise/inaccuracies.
        """
        aug_points = np.copy(points)
        noise = np.random.normal(0, self.doppler_std, size=aug_points.shape[0])
        aug_points[:, 4] += noise  # Assuming index 4 is Doppler
        return aug_points

    def scale_rcs(self, points):
        """ 
        Scale RCS to simulate Swerling fluctuations (target cross-section variations).
        RCS values naturally fluctuate wildly between radar scans.
        """
        aug_points = np.copy(points)
        scale = np.random.uniform(*self.rcs_scale_range)
        aug_points[:, 3] *= scale  # Assuming index 3 is RCS
        return aug_points

    def add_rcs_noise(self, points, rcs_std=2.0):
        """ Add Gaussian noise directly to RCS values. """
        aug_points = np.copy(points)
        noise = np.random.normal(0, rcs_std, size=aug_points.shape[0])
        aug_points[:, 3] += noise
        return aug_points

    def rcs_based_dropout(self, points, drop_ratio_high=0.1, drop_ratio_low=0.3, rcs_threshold=10.0):
        """ 
        Drop a percentage of points, penalizing points with lower RCS values more,
        as low-RCS points are more likely to be clutter, multi-path reflections, or thermal noise.
        """
        if points.shape[0] == 0:
            return points
            
        rcs = points[:, 3]
        
        # Apply a higher drop probability if RCS is below the threshold
        drop_probs = np.where(rcs < rcs_threshold, drop_ratio_low, drop_ratio_high)
        drop_probs = np.clip(drop_probs, 0.0, 1.0)
        
        # Random sample based on probability
        rand_vals = np.random.random(points.shape[0])
        keep_mask = rand_vals > drop_probs
        
        return points[keep_mask]
        
    def random_flip_y(self, points, flip_prob=0.5):
        """ 
        Mirror the point cloud along the Y-axis (left-right flip).
        Physics context:
        Doppler is scalar radial velocity (relative to the sensor at origin). 
        If we mirror the scene along the Y axis, the Y positions flip, and the Y velocities flip.
        Because radial velocity mathematically is the dot product of position unit vector and velocity vector,
        v_r = (x*Vx + y*Vy + z*Vz) / sqrt(x^2 + y^2 + z^2)
        If Y -> -Y and Vy -> -Vy, then y*Vy -> (-y)*(-Vy) = y*Vy.
        Therefore, the scalar Doppler radial velocity REMAINS UNCHANGED when flipping along Y!
        """
        aug_points = np.copy(points)
        
        if np.random.rand() < flip_prob:
            aug_points[:, 1] = -aug_points[:, 1] # Flip Y coordinate
            # Note: No change needed for Doppler (column 4) as explained above!
            
        return aug_points
        
    def augment_pipeline(self, points):
        """ 
        Apply a random combination of all augmentations, simulating a robust deep learning transform pipeline.
        """
        points = self.add_doppler_noise(points)
        points = self.scale_rcs(points)
        
        if np.random.rand() > 0.5:
            points = self.add_rcs_noise(points)
            
        points = self.rcs_based_dropout(points)
        points = self.random_flip_y(points)
        
        return points

# ----------------- Utility to Load Save PCD Files --------------
def load_radar_pcd_to_numpy(pcd_file_path, ran_rename_script=True):
    """
    Loads a .pcd file and extracts exactly [x, y, z, rcs, doppler] into an (N, 5) array.
    Requires: pip install pypcd4
    """
    try:
        from pypcd4 import PointCloud
    except ImportError:
        raise ImportError("Please install pypcd4 to read .pcd files directly: pip install pypcd4")

    print(f"Loading {pcd_file_path}...")
    pc = PointCloud.from_path(pcd_file_path)
    
    # If the user previously ran rename_pcd_fields.py, the fields are literally 'rcs' and 'doppler'
    # Otherwise, they are 'intensity' and 'curvature' (since it was saved from PointXYZINormal)
    rcs_field = 'rcs' if ran_rename_script else 'intensity'
    doppler_field = 'doppler' if ran_rename_script else 'curvature'
    
    try:
        x = pc.pc_data['x']
        y = pc.pc_data['y']
        z = pc.pc_data['z']
        rcs = pc.pc_data[rcs_field]
        doppler = pc.pc_data[doppler_field]
    except ValueError as e:
        print(f"Error extracting fields. Ensure ran_rename_script={ran_rename_script} is correct.")
        print(f"Fields available in this PCD: {pc.fields}")
        raise e

    # Stack them into the required (N, 5) numpy array
    return np.column_stack((x, y, z, rcs, doppler))

if __name__ == "__main__":
    import argparse
    import sys

    # Quick CLI Argument for testing a real PCD file
    if len(sys.argv) > 1 and sys.argv[1].endswith('.pcd'):
        pcd_path = sys.argv[1]
        
        # NOTE: Set ran_rename_script=False if you haven't run rename_pcd_fields.py on this file yet
        mock_points = load_radar_pcd_to_numpy(pcd_path, ran_rename_script=True)
        print(f"Loaded {mock_points.shape[0]} points from {pcd_path}.")
    else:
        print("No PCD file provided. Generating Mock Data for Demo...\n")
        # --- MOCK DEMO ---
        # Create mock data [N, 5]: x, y, z, rcs, doppler
        N = 2000
        mock_points = np.random.rand(N, 5) * 10
        mock_points[:, 3] *= 6.0   # Scale Mock RCS 
        mock_points[:, 4] = (mock_points[:, 4] - 5.0) * 3.0  # Scale Mock Doppler 
        
    augmentor = RadarDataAugmentor(doppler_std=0.25, rcs_scale_range=(0.7, 1.3))
    
    print("--- 4D Radar Augmentation Demo ---")
    print(f"Original properties -> Mean RCS: {np.mean(mock_points[:, 3]):.2f} | Mean Doppler: {np.mean(mock_points[:, 4]):.2f}")
    
    # Apply pipeline
    aug_points = augmentor.augment_pipeline(mock_points)
    
    print("\nApplying Augmentation Pipeline...")
    print(f"Augmented points shape: {aug_points.shape} (Notice points were dropped via RCS-Dropout)")
    print(f"Augmented properties -> Mean RCS: {np.mean(aug_points[:, 3]):.2f} | Mean Doppler: {np.mean(aug_points[:, 4]):.2f}")
