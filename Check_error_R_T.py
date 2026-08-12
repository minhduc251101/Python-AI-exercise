import numpy as np


def get_angular_error(R_exp, R_est):
    """
    Calculate angular error
    """
    return abs(
        np.arccos(min(max(((np.matmul(R_exp.T, R_est)).trace() - 1) / 2, -1.0), 1.0))
    )


def pose_error(T_est, T_gt):
    T_err = T_gt @ np.linalg.inv(T_est)
    trans_error = np.linalg.norm(T_err[:3, 3])
    R_err = T_err[:3, :3]
    rot_error_rad = get_angular_error(T_gt[:3, :3], T_est[:3, :3])
    rot_error_deg = np.rad2deg(rot_error_rad)
    return trans_error, rot_error_deg


T_gt = np.loadtxt(
    "/media/minhduc/TOSHIBA EXT1/C++ hoc/Python-AI-exercise/results_27_5032/gt_transform.txt"
)
T_est = np.loadtxt(
    "/media/minhduc/TOSHIBA EXT1/C++ hoc/Python-AI-exercise/results_27_5032/estimated_transform_final.txt"
)
# T_est = np.loadtxt('/media/minhduc/TOSHIBA EXT1/C++ hoc/Python-AI-exercise/results_27_5032/go_icp_output.txt')

trans_error, rot_error_deg = pose_error(T_est, T_gt)
T_err = T_gt @ np.linalg.inv(T_est)
print("\n\nFPFH + RANSAC:\n\n")
# print("\n\nGO-ICP:\n\n")
print("T_gt =\n", T_gt)
print("T_est =\n", T_est)
print("translation error =", trans_error)
print("rotation error (deg) =", rot_error_deg)
<<<<<<< HEAD
print("\n Another method for rotation error (deg) =", np.degrees(abs(np.arccos(
        np.clip((np.trace(T_err[:3,:3]) - 1.0) / 2.0, -1.0, 1.0)))))
=======
>>>>>>> f7306eecd5ea669edebd8fb6d2ec260d9d067a16
