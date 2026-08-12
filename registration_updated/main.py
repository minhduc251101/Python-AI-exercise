"""
FPFH + Open3D RANSAC + final Arun evaluation against GT odom
"""

"""
python "/media/minhduc/TOSHIBA EXT1/C++ hoc/Python-AI-exercise/FPFH_RANSAC_SVD.py"   
--scan_root "/media/minhduc/TOSHIBA EXT1/4dradarslam/NTU4Dradlm/cp/cp/loop_true_fullpose"   
--idx1 27   --idx2 5032   --gt_poses "/media/minhduc/TOSHIBA EXT1/C++ hoc/Python-AI-exercise/gt_odom_cp.txt"   
--voxel_size 0.08   --distance_threshold 0.12   --max_iterations 100000   --sample_size 3   --edge_ratio 0.9   
--result_dir "/media/minhduc/TOSHIBA EXT1/C++ hoc/Python-AI-exercise/results_27_5032"   --save_pose   --plot_result   
--plot_save   --plot_show --num_trials 50 --max_translation_norm 5.0 --max_abs_roll_deg 10.0 --max_abs_pitch_deg 10.0 
--keep_ratio 0.2 --min_keep 12 --min_fitness 0.08

uv run FPFH_RANSAC_SVD.py --scan_root=/Users/mbp/Desktop/learning/teaching_duc/Python-AI-exercise/dataset --idx1=4940 --idx2=5232 --gt_poses=/Users/mbp/Desktop/learning/teaching_duc/Python-AI-exercise/gt_odom_cp.txt --result_dir results_20_06_2026   --save_pose   --plot_result  --plot_save   --plot_show --num_trials 50 --max_translation_norm 5.0 --max_abs_roll_deg 10.0 --max_abs_pitch_deg 10.0 --keep_ratio 0.2 --min_keep 12 --min_fitness 0.08
"""

import yaml
import os
from loguru import logger

logger.add(
    "/Users/mbp/Desktop/learning/teaching_duc/Python-AI-exercise/registration_updated/logs/registration.log",
    rotation="1 MB",
    level="INFO",
)


def load_config(config_path):
    with open(config_path, "r") as f:
        config = yaml.safe_load(f)
    return config


CONFIG_FILE_PATH = "/Users/mbp/Desktop/learning/teaching_duc/Python-AI-exercise/registration_updated/config.yaml"

config = load_config(CONFIG_FILE_PATH)


if __name__ == "__main__":
    logger.info("Running FPFH + Open3D RANSAC + final Arun evaluation against GT odom")
