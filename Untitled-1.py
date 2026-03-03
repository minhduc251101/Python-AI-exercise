
#include <cmath>
#include <radar_graph_slam/loop_detector.hpp>

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/vertex_se3.h>

using namespace std;

Eigen::Vector3d R2ypr_(const Eigen::Matrix3d &R) {
  Eigen::Vector3d n = R.col(0);
  Eigen::Vector3d o = R.col(1);
  Eigen::Vector3d a = R.col(2);

  Eigen::Vector3d ypr(3);
  double y = std::atan2(
      n(1), n(0)); // tinh goc tu nhung duong point cloud gan nhau  -> map
  double p = std::atan2(-n(2), n(0) * std::cos(y) + n(1) * std::sin(y));
  double r = std::atan2(a(0) * std::sin(y) - a(1) * std::cos(y),
                        -o(0) * std::sin(y) + o(1) * std::cos(y));
  ypr(0) = y;
  ypr(1) = p;
  ypr(2) = r;

  return ypr;
}

double limit_value(int value, int min_, int max_) {
  if (value < min_)
    return min_;
  else if (value > max_)
    return max_;
  else
    return value;
}

Eigen::Vector3d monoToRainbow(int value) { // dung diem anh
  double k = 4.65454545454;                // tham so, goc khong vuot qua
  double blue = limit_value(-k * (value - 140), 0, 255);
  double green, red;
  if (value < 30) {
    green = 0;
    red = limit_value(-k * (value - 30), 0, 255);
  } else if (value < 140) {
    green = limit_value(k * (value - 30), 0, 255);
    red = 0;
  } else {
    green = limit_value(-k * (value - 250), 0, 255);
    red = limit_value(k * (value - 140), 0, 255);
  }
  return Eigen::Vector3d(blue, green, red);
}

namespace radar_graph_slam {

LoopDetector::LoopDetector(ros::NodeHandle &pnh) {
  enable_pf = pnh.param<bool>("enable_pf", true);
  enable_odom_check = pnh.param<bool>("enable_odom_check", true);

  distance_thresh = pnh.param<double>("distance_thresh", 5.0);
  accum_distance_thresh = pnh.param<double>("accum_distance_thresh", 8.0);
  min_loop_interval_dist = pnh.param<double>("min_loop_interval_dist", 5.0);

  fitness_score_max_range = pnh.param<double>(
      "fitness_score_max_range", std::numeric_limits<double>::max());
  fitness_score_thresh =
      pnh.param<double>("fitness_score_thresh", 0.7); // default 0.5

  odom_drift_xy = pnh.param<double>("odometry_drift_xy", 0.05);
  odom_drift_z = pnh.param<double>("odometry_drift_z", 0.20);
  drift_scale_xy = pnh.param<double>("odometry_drift_scale_xy", 2.0);
  drift_scale_z = pnh.param<double>("odometry_drift_scale_z", 2.0);

  max_baro_difference = pnh.param<double>("max_baro_difference", 3.0);
  max_yaw_difference = pnh.param<double>("max_yaw_difference", 45.0);

  double sc_dist_thresh = pnh.param<double>("sc_dist_thresh", 0.3);
  double sc_azimuth_range = pnh.param<double>("sc_azimuth_range", 0.3);

  registration = select_registration_method(pnh);

  // Loop Closure Parameters
  pnh.param<int>("lio_sam/historyKeyframeSearchNum", historyKeyframeSearchNum,
                 25);
  historyKeyframeFitnessScore =
      pnh.param<float>("historyKeyframeFitnessScore", 6);
  odom_check_trans_thresh = pnh.param<double>("odom_check_trans_thresh", 0.1);
  odom_check_rot_thresh = pnh.param<double>("odom_check_rot_thresh", 0.05);

  scManager.reset(new SCManager());
  inf_calclator.reset(new InformationMatrixCalculator(pnh));
  image_transport.reset(new image_transport::ImageTransport(pnh));
  pub_cur_sc = image_transport->advertise("/sc/cur", 1);
  pub_pre_sc = image_transport->advertise("/sc/pre", 1);

  scManager->setScDistThresh(sc_dist_thresh);
  scManager->setAzimuthRange(sc_azimuth_range);

  // Verification
  enable_loop_verification = pnh.param<bool>("enable_loop_verification", true);
  verify_graph_size = pnh.param<int>("verify_graph_size", 20);
  verify_chi2_thresh = pnh.param<double>("verify_chi2_thresh", 5000.0);
  verify_opt_iter = pnh.param<int>("verify_opt_iter", 5);

  structure_check_min_dist = pnh.param<double>("structure_check_min_dist", 1.0);
  structure_check_violation_ratio =
      pnh.param<double>("structure_check_violation_ratio", 0.05);
  max_allowed_drift_ratio = pnh.param<double>("max_allowed_drift_ratio", 0.1);
}

LoopDetector::~LoopDetector() {}

/**
 * @brief detect loops and add them to the pose graph
 * @param keyframes       keyframes
 * @param new_keyframes   newly registered keyframes
 * @param graph_slam      pose graph
 */
std::vector<Loop::Ptr>
LoopDetector::detect(const std::vector<KeyFrame::Ptr> &keyframes,
                     const std::deque<KeyFrame::Ptr> &new_keyframes,
                     radar_graph_slam::GraphSLAM &graph_slam) {
  std::vector<Loop::Ptr> detected_loops;
  for (const auto &new_keyframe : new_keyframes) {
    std::vector<KeyFrame::Ptr> candidates;
    Loop::Ptr loop = nullptr;
    if (enable_pf) {
      start_ms_pf = clock();
      candidates = find_candidates(
          keyframes, new_keyframe); // keyframe diem hien tai, new thi dung
                                    // thanh nhieu diem lan can
      end_ms_pf = clock();
      std::cout << "[LoopDetector] Keyframe " << new_keyframe->index
                << ": Found " << candidates.size() << " loop candidates."
                << std::endl;
    } else {
      for (auto kf : keyframes) {
        if (new_keyframe->accum_distance > kf->accum_distance + distance_thresh)
          ;
        candidates.push_back(kf);
      }
    }
    // 1. Try Scan Context First (Fast & Safe)
    loop = performScanContextLoopClosure(keyframes, candidates, new_keyframe);

    // 2. If Scan Context fails,
    //     Fallback to Geometric Matching(Brute Force) if (loop == nullptr) {
    //   std::cout << "\033[35m[LoopDetector] Scan Context loop closure failed.
    //                "
    //                "Attempting Geometric Matching...\033[0m"
    //             << std::endl;
    //   loop = matching(keyframes, candidates, new_keyframe);
    // }
    if (loop) {
      detected_loops.push_back(loop);
      double time_used = double(end_ms_pf - start_ms_pf) / CLOCKS_PER_SEC;
      pf_time.push_back(time_used);
      time_used = double(end_ms_sc - start_ms_sc) / CLOCKS_PER_SEC;
      sc_time.push_back(time_used);
      time_used = double(end_ms_oc - start_ms_oc) / CLOCKS_PER_SEC;
      oc_time.push_back(time_used);
    }
  }

  return detected_loops;
}

/**
 * @brief find loop candidates. A detected loop begins at one of #keyframes and
 * ends at #new_keyframe
 * @param keyframes      candidate keyframes of loop start
 * @param new_keyframe   loop end keyframe
 * @return loop candidates
 */
std::vector<KeyFrame::Ptr>
LoopDetector::find_candidates(const std::vector<KeyFrame::Ptr> &keyframes,
                              const KeyFrame::Ptr &new_keyframe) const {
  // 1. too close to the last registered loop edge
  double dist_btn_last_loop_edge =
      new_keyframe->accum_distance - last_loop_edge_accum_distance;
  if (dist_btn_last_loop_edge < min_loop_interval_dist) {
    return std::vector<KeyFrame::Ptr>();
  }
  // Pick loop candicates considering distance and barometer measurements
  std::vector<KeyFrame::Ptr> candidates;
  candidates.reserve(32);

  for (const auto &k : keyframes) {
    // 2. traveled distance between keyframes is too small
    double accum_distance = new_keyframe->accum_distance - k->accum_distance;
    if (accum_distance < accum_distance_thresh) {
      continue;
    }
    // 3. If keyframes_ have similar barometer measurements // value
    if (k->altitude.is_initialized()) {
      if (fabs(k->altitude.value()[0] - new_keyframe->altitude.value()[0]) >
          max_baro_difference)
        continue;
    }
    // 4. yaw angle difference
    Eigen::Matrix4d T = k->node->estimate().matrix().inverse() *
                        new_keyframe->node->estimate().matrix();
    Eigen::Vector3d ypr_diff = R2ypr_(T.block<3, 3>(0, 0));
    if (abs(ypr_diff(0) * 180 / M_PI) > max_yaw_difference)
      continue;
    // 5. estimated distance between new-keyframe is too Large
    Eigen::Vector3d pos_between = T.block<3, 1>(0, 3);
    double dist = pos_between.norm(); // measure x-y distance
    double x_diff = pos_between(0), y_diff = pos_between(1),
           z_diff = pos_between(2);
    double rad_xy_loop =
        3 + dist_btn_last_loop_edge * odom_drift_xy * drift_scale_xy;
    double rad_z_loop =
        3 + dist_btn_last_loop_edge * odom_drift_z * drift_scale_z;
    double aa_lle =
        pow((x_diff / rad_xy_loop), 2) + pow((y_diff / rad_xy_loop), 2);
    if (aa_lle > 1)
      continue;
    // cout << "last_loop_edge_index: " << last_loop_edge_index << "
    // new_keyframe: " << new_keyframe->index
    //         << " dist_btn_last_loop_edge = " << dist_btn_last_loop_edge <<
    //         endl;
    // cout << "rad_xy_loop " << rad_xy_loop << endl;
    // cout << "pos_btn: " << pos_between(0) << " " << pos_between(1) << " " <<
    // pos_between(2) << endl; cout << " aa_lle = " << aa_lle << endl << endl;

    double rad_xy = 10.0 + odom_drift_xy * accum_distance * drift_scale_xy;
    double rad_z = 10.0 + odom_drift_z * accum_distance * drift_scale_z;
    double aa = pow((x_diff / rad_xy), 2) +
                pow((y_diff / rad_xy), 2); //+pow((z_diff/rad_z),2);
    if (aa >
        1) // candidate keyframe pose is out of elipse  (dist > distance_thresh)
      continue;

    candidates.push_back(k);
  }

  return candidates;
}

Loop::Ptr LoopDetector::performScanContextLoopClosure(
    const std::vector<KeyFrame::Ptr> &keyframes,
    const std::vector<KeyFrame::Ptr> &candidate_keyframes,
    const KeyFrame::Ptr &new_keyframe) {
  start_ms_sc = clock();
  if (candidate_keyframes.empty() == true) {
    std::cout << "\033[33m[LoopDetector] No loop candidates found.\033[0m"
              << std::endl;
    return nullptr;
    // ROS_INFO("\033[33m Cannot find Loop Candidate \033[0m");
  }
  std::cout << "\033[36m" << " New keyframe index: " << new_keyframe->index
            << ". "
            << "Number of candidate frames: " << candidate_keyframes.size();
  // << "Candidate frames index: ";
  // for (auto& ckf : candidate_keyframes){
  //   std::cout << ckf->index << " ";
  // }
  std::cout << "\033[0m" << endl;

  // Detect keys, first: nn index, second: yaw diff
  auto detectResult =
      scManager->detectLoopClosureID(candidate_keyframes, new_keyframe);
  int loopKeyCur = new_keyframe->index;
  int loopKeyPre = detectResult.first;
  float yawDiffRad = detectResult.second; // not use for v1 (because pcl icp
                                          // withi initial somthing wrong...)
  if (loopKeyPre == -1) {
    std::cout << "\033[33m[LoopDetector] Scan Context failed to find a "
                 "matching candidates ID (score too low).\033[0m"
              << std::endl;
    return nullptr; /* No loop found */
  }

  // extract cloud
  pcl::PointCloud<PointT>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointT>());
  pcl::PointCloud<PointT>::Ptr prevKeyframeCloud(new pcl::PointCloud<PointT>());
  {
    *cureKeyframeCloud +=
        *new_keyframe->cloud; // Because new_keyframe is not in keyframes
    *prevKeyframeCloud += *keyframes[loopKeyPre]->cloud;
  }

  registration->setInputSource(cureKeyframeCloud);
  registration->setInputTarget(prevKeyframeCloud);
  pcl::PointCloud<PointT>::Ptr unused_result(new pcl::PointCloud<PointT>());
  registration->align(*unused_result);
  // TODO icp align with initial
  end_ms_sc = clock();

  if (registration->hasConverged() == false ||
      registration->getFitnessScore() > historyKeyframeFitnessScore) {
    bool converged = registration->hasConverged();
    std::cout
        << "\033[31m[LoopDetector] SC Loop Failed: ICP Fitness Score too high. "
        << "Score: " << registration->getFitnessScore()
        << " > Thresh: " << historyKeyframeFitnessScore << "\033[0m"
        << std::endl;
    return nullptr;
  }
  // Get pose transformation
  Eigen::Matrix4f correctionLidarFrame;
  correctionLidarFrame = registration->getFinalTransformation();

  Eigen::Isometry3d poseFrom(Eigen::Isometry3d::Identity());
  Eigen::Isometry3d poseTo(Eigen::Isometry3d::Identity());
  Eigen::Vector3d vec =
      Eigen::Vector3d(correctionLidarFrame(0, 3), correctionLidarFrame(1, 3),
                      correctionLidarFrame(2, 3));
  Eigen::Matrix3d mat = correctionLidarFrame.block(0, 0, 3, 3).cast<double>();
  poseFrom.prerotate(mat); // Set rotation
  poseFrom.pretranslate(vec);
  Eigen::Isometry3d T_lc_ij = poseFrom.inverse() * poseTo;

  /****** Incremental Consistent Measurement Set Maximization ******/
  // Odometry check (source_frame<->target_frame and
  // source_frame<->last_loop_edge)
  start_ms_oc = clock();
  if (enable_odom_check) {
    // source_frame <-> target_frame
    Eigen::Isometry3d T_odom_ji = new_keyframe->odom_scan2scan.inverse() *
                                  keyframes[loopKeyPre]->odom_scan2scan;
    Eigen::Isometry3d T_err_ij = T_lc_ij * T_odom_ji;
    int num_between =
        loopKeyCur - loopKeyPre; // the number of edges along the loop
    Eigen::AngleAxisd rotation_vector;
    rotation_vector.fromRotationMatrix(T_err_ij.rotation());
    double oc_err_trans = T_err_ij.translation().norm() / num_between;
    double oc_err_rot = rotation_vector.angle() / num_between;
    // double err_rot =
    // std::acos(Eigen::Quaterniond(T_err_ij.rotation()).w())/num_between; cout
    // << oc_err_rot << " " << err_rot << endl;
    if (oc_err_trans > odom_check_trans_thresh ||
        oc_err_rot > odom_check_rot_thresh) {
      cout << "\033[31m[LoopDetector] SC Loop Failed: Odometry Check invalid. "
           << "TransErr: " << oc_err_trans
           << " (Thresh: " << odom_check_trans_thresh << "), "
           << "RotErr: " << oc_err_rot << " (Thresh: " << odom_check_rot_thresh
           << ")\033[0m" << endl;
      return nullptr;
    }
    // source_frame <-> last_loop_edge
  }
  end_ms_oc = clock();

  std::cout
      << "\033[32m"
      << " Add this ScanContext loop. ICP fitness test passed ( Fitness Score "
      << registration->getFitnessScore() << " < " << historyKeyframeFitnessScore
      << "). " << "\033[0m" << std::endl;
  std::cout << "\033[32m" << ">>> SCAN CONTEXT LOOP FOUND! Keyframe "
            << new_keyframe->index << " <--> " << loopKeyPre << " \033[0m"
            << std::endl;

  // Publish context as image
  Eigen::MatrixXd &cur_sc = scManager->polarcontexts_.at(loopKeyCur);
  Eigen::MatrixXd &pre_sc = scManager->polarcontexts_.at(loopKeyPre);
  cv::Mat mat_cur_sc = makeSCImage(cur_sc);
  cv::Mat color_cur_sc = getColorImage(mat_cur_sc);
  cv::Mat mat_pre_sc = makeSCImage(pre_sc);
  cv::Mat color_pre_sc = getColorImage(mat_pre_sc);

  sensor_msgs::ImagePtr cur_img_msg =
      cv_bridge::CvImage(std_msgs::Header(), "bgr8", color_cur_sc).toImageMsg();
  pub_cur_sc.publish(cur_img_msg);
  sensor_msgs::ImagePtr pre_img_msg =
      cv_bridge::CvImage(std_msgs::Header(), "bgr8", color_pre_sc).toImageMsg();
  pub_pre_sc.publish(pre_img_msg);

  // robust kernel for a SC loop
  Eigen::MatrixXd information = inf_calclator->calc_information_matrix(
      cureKeyframeCloud, prevKeyframeCloud, T_lc_ij);

  if (new_keyframe->accum_distance > last_loop_edge_accum_distance) {
    last_loop_edge_accum_distance = new_keyframe->accum_distance;
    last_loop_edge_index = new_keyframe->index;
  }

  // Verify Loop (Structure + g2o)
  if (enable_loop_verification) {
    // 1. Structure Check
    if (!checkStructure(T_lc_ij, new_keyframe, keyframes[loopKeyPre])) {
      return nullptr;
    }

    // 2. g2o Verification
    if (!verifyLoop(T_lc_ij, loopKeyPre, new_keyframe, keyframes)) {
      return nullptr;
    }
  }

  // Add pose constraint
  mtx.lock();
  loopIndexQueue.push_back(make_pair(loopKeyCur, loopKeyPre));
  loopPoseQueue.push_back(T_lc_ij);
  loopInfoQueue.push_back(information);
  mtx.unlock();
  // Add loop constriant
  loopIndexContainer.insert(
      std::pair<int, int>(loopKeyCur, loopKeyPre)); // giseop for multimap

  return std::make_shared<Loop>(new_keyframe, keyframes[loopKeyPre],
                                T_lc_ij.matrix().cast<float>());
}

cv::Mat LoopDetector::makeSCImage(Eigen::MatrixXd src_mat) {
  // double max_ = src_mat.rowwise().maxCoeff().colwise().maxCoeff().value();
  // double min_ = src_mat.rowwise().minCoeff().colwise().minCoeff().value();
  double max_ = 35;
  double min_ = 0;
  cv::Mat ssc = cv::Mat::zeros(
      cv::Size(scManager->PC_NUM_SECTOR, scManager->PC_NUM_RING), CV_8U);
  for (int i = 0; i < scManager->PC_NUM_SECTOR; i++) { // 60
    for (int j = 0; j < scManager->PC_NUM_RING; j++) { // 20
      double &d = src_mat(j, i);
      int uc = round((d - min_) / (max_ - min_) * 255);
      ssc.at<uchar>(j, i) = uc;
    }
  }
  return ssc;
}

cv::Mat LoopDetector::getColorImage(cv::Mat &desc) {
  cv::Mat out = cv::Mat::zeros(desc.size(), CV_8UC3);
  for (int i = 0; i < desc.rows; ++i) {
    for (int j = 0; j < desc.cols; ++j) {
      int value_mono = (int)desc.at<uchar>(
          i, j); // std::get<2>(_argmax_to_rgb[(int)desc.at<uchar>(i, j)]);
      if (value_mono == 0) {
        out.at<cv::Vec3b>(i, j)[0] = 255;
        out.at<cv::Vec3b>(i, j)[1] = 255;
        out.at<cv::Vec3b>(i, j)[2] = 255;
      } else {
        Eigen::Vector3d bgr = monoToRainbow(value_mono);
        out.at<cv::Vec3b>(i, j)[0] = (int)bgr(0); // 255 - value_mono;
        out.at<cv::Vec3b>(i, j)[1] = (int)bgr(
            1); // 0;//std::min(std::max(128 - value_mono, 0), value_mono);
        out.at<cv::Vec3b>(i, j)[2] = (int)bgr(2); // value_mono;
      }
    }
  }
  return out;
}

double LoopDetector::get_distance_thresh() const { return distance_thresh; }

// #if 0
/**
 * @brief To validate a loop candidate this function applies a scan matching
 * between keyframes consisting the loop. If they are matched well, the loop is
 * added to the pose graph
 * @param candidate_keyframes  candidate keyframes of loop start
 * @param new_keyframe         loop end keyframe
 * @param graph_slam           graph slam
 */
Loop::Ptr
LoopDetector::matching(const std::vector<KeyFrame::Ptr> &keyframes,
                       const std::vector<KeyFrame::Ptr> &candidate_keyframes,
                       const KeyFrame::Ptr &new_keyframe) {
  if (candidate_keyframes.empty()) {
    return nullptr;
  }

  registration->setInputTarget(new_keyframe->cloud);

  double best_score = std::numeric_limits<double>::max();
  KeyFrame::Ptr best_matched;
  Eigen::Matrix4f relative_pose;

  std::cout << std::endl;
  std::cout << "--- loop detection ---" << std::endl;
  std::cout << "num_candidates: " << candidate_keyframes.size() << std::endl;
  std::cout << "matching" << std::flush;
  auto t1 = ros::Time::now();

  pcl::PointCloud<PointT>::Ptr aligned(new pcl::PointCloud<PointT>());
  for (const auto &candidate : candidate_keyframes) {
    registration->setInputSource(candidate->cloud);
    Eigen::Isometry3d new_keyframe_estimate = new_keyframe->node->estimate();
    new_keyframe_estimate.linear() =
        Eigen::Quaterniond(new_keyframe_estimate.linear())
            .normalized()
            .toRotationMatrix();
    Eigen::Isometry3d candidate_estimate = candidate->node->estimate();
    candidate_estimate.linear() =
        Eigen::Quaterniond(candidate_estimate.linear())
            .normalized()
            .toRotationMatrix();
    Eigen::Matrix4f guess =
        (new_keyframe_estimate.inverse() * candidate_estimate)
            .matrix()
            .cast<float>();
    guess(2, 3) = 0.0;
    registration->align(*aligned, guess);
    std::cout << "." << std::flush;

    double score = registration->getFitnessScore(fitness_score_max_range);
    if (!registration->hasConverged() || score > best_score) {
      continue;
    }

    best_score = score;
    best_matched = candidate;
    relative_pose = registration->getFinalTransformation();
  }

  auto t2 = ros::Time::now();
  std::cout << " done" << std::endl;
  std::cout << "best_score: " << boost::format("%.3f") % best_score
            << "    time: " << boost::format("%.3f") % (t2 - t1).toSec()
            << "[sec]" << std::endl;

  if (best_score > fitness_score_thresh) {
    std::cout
        << "\033[31m[LoopDetector] Geometric Matching Failed. Best Score: "
        << best_score << " > Thresh: " << fitness_score_thresh << "\033[0m"
        << std::endl;
    return nullptr;
  }

  std::cout << "loop found!!" << std::endl;
  std::cout << ">>> LOOP CLOSURE: Keyframe " << new_keyframe->index
            << " matched with Keyframe " << best_matched->index << " <<<"
            << std::endl;
  std::cout << "relative pose: " << relative_pose.block<3, 1>(0, 3) << " - "
            << Eigen::Quaternionf(relative_pose.block<3, 3>(0, 0))
                   .coeffs()
                   .transpose()
            << std::endl;

  Eigen::Isometry3d relative_pose_d = Eigen::Isometry3d::Identity();
  relative_pose_d.linear() = relative_pose.block<3, 3>(0, 0).cast<double>();
  relative_pose_d.translation() =
      relative_pose.block<3, 1>(0, 3).cast<double>();

  Eigen::MatrixXd information_matrix = inf_calclator->calc_information_matrix(
      new_keyframe->cloud, best_matched->cloud, relative_pose_d);
  // Verify Loop
  if (enable_loop_verification) {
    Eigen::Isometry3d T_lc(relative_pose.cast<double>());
    if (!verifyLoop(T_lc, best_matched->index, new_keyframe, keyframes)) {
      return nullptr;
    }
  }

  // update edge for loop closure connect keyframe 418 and keyframe 2; save
  // best_matched in LoopIndexQueue(vector<pair<int, int>>) +1 edge
  loopIndexQueue.push_back(
      std::make_pair(new_keyframe->index, best_matched->index));
  // loopIndexQueue.push_back({new_keyframe->index, best_matched->index});
  loopPoseQueue.push_back(relative_pose_d);
  loopInfoQueue.push_back(information_matrix);
  loopIndexContainer.insert(
      std::pair<int, int>(new_keyframe->index, best_matched->index));

  last_loop_edge_accum_distance = new_keyframe->accum_distance;

  return std::make_shared<Loop>(new_keyframe, best_matched, relative_pose);
}

bool LoopDetector::verifyLoop(const Eigen::Isometry3d &loop_transform,
                              int loop_kf_index,
                              const KeyFrame::Ptr &new_keyframe,
                              const std::vector<KeyFrame::Ptr> &keyframes) {
  if (!enable_loop_verification)
    return true;

  // 0. Drift Consistency Check (New)
  // Calculate path length of the chain
  double path_length = 0.0;
  Eigen::Isometry3d T_chain = Eigen::Isometry3d::Identity();

  // We need to trace from candidate (loop_kf) to new_keyframe.
  // Assuming 'keyframes' is time-ordered...
  // Find index range
  int start_idx = loop_kf_index;
  int end_idx = new_keyframe->index;

  // Verify order
  if (start_idx > end_idx)
    std::swap(start_idx, end_idx);

  // Accumulate odometry
  // Note: keyframes vector might not be contiguous by index if some were
  // dropped? But usually it is. Let's assume indices in 'keyframes' vector map
  // to ID. Actually, 'keyframes' is a std::vector created in detect(). It is
  // likely the full list.

  for (int i = 0; i < (int)keyframes.size() - 1; ++i) {
    if (keyframes[i]->index >= start_idx &&
        keyframes[i + 1]->index <= end_idx) {
      Eigen::Isometry3d d_odom = keyframes[i]->odom_scan2scan.inverse() *
                                 keyframes[i + 1]->odom_scan2scan;
      path_length += d_odom.translation().norm();
      T_chain = T_chain * d_odom;
    }
  }

  // If we couldn't calc path (e.g. indices mismatch), skip or warn
  if (path_length > 1.0) {
    // Loop Transform T_lc connects loop_kf -> new_keyframe
    // So T_loop should be approx T_chain
    // Error = T_chain.inverse * T_loop
    Eigen::Isometry3d T_diff = T_chain.inverse() * loop_transform;
    double drift_error = T_diff.translation().norm();
    double drift_ratio = drift_error / path_length;

    std::cout << "[LoopDetector] Drift Check: PathLen=" << path_length
              << "m, Error=" << drift_error << "m, Ratio=" << drift_ratio
              << std::endl;

    if (drift_ratio > max_allowed_drift_ratio) {
      std::cout << "\033[31m[LoopDetector] Drift Check REJECTED. Ratio "
                << drift_ratio << " > " << max_allowed_drift_ratio << "\033[0m"
                << std::endl;
      return false;
    }
  }

  // 1. Construct local graph
  g2o::SparseOptimizer optimizer;
  optimizer.setVerbose(false);
  std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver(
      new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>());
  std::unique_ptr<g2o::BlockSolver_6_3> solver_ptr(
      new g2o::BlockSolver_6_3(std::move(linearSolver)));
  g2o::OptimizationAlgorithmLevenberg *solver =
      new g2o::OptimizationAlgorithmLevenberg(std::move(solver_ptr));
  optimizer.setAlgorithm(solver);

  // Identify nodes to add
  std::map<int, g2o::VertexSE3 *> vertex_map;

  // A. Add Loop Candidate Keyframe (Fixed)
  auto *v_loop = new g2o::VertexSE3();
  v_loop->setId(loop_kf_index);

  // Get estimate for loop_kf
  // keyframes is the full list. We expect loop_kf_index to satisfy
  // keyframes[i]->index == loop_kf_index if indices are sequential. But safest
  // is to search or index if we trust structure. In this system keyframes is a
  // vector. Let's find it.
  KeyFrame::Ptr loop_kf = nullptr;
  // Fast check
  if (loop_kf_index < keyframes.size() &&
      keyframes[loop_kf_index]->index == loop_kf_index) {
    loop_kf = keyframes[loop_kf_index];
  } else {
    for (const auto &k : keyframes) {
      if (k->index == loop_kf_index) {
        loop_kf = k;
        break;
      }
    }
  }

  if (!loop_kf) {
    std::cout << "[LoopDetector] Verify: Could not find loop keyframe "
              << loop_kf_index << std::endl;
    return false;
  }

  v_loop->setEstimate(loop_kf->node->estimate());
  v_loop->setFixed(true);
  optimizer.addVertex(v_loop);
  vertex_map[loop_kf_index] = v_loop;

  // B. Add Recent Chain
  // We want the last 'verify_graph_size' keyframes from 'keyframes' +
  // new_keyframe.
  std::vector<KeyFrame::Ptr> chain;
  int count = 0;
  for (int i = (int)keyframes.size() - 1; i >= 0 && count < verify_graph_size;
       --i) {
    chain.push_back(keyframes[i]);
    count++;
  }
  std::reverse(chain.begin(), chain.end());
  chain.push_back(new_keyframe);

  for (size_t i = 0; i < chain.size(); ++i) {
    const auto &kf = chain[i];
    if (kf->index == loop_kf_index)
      continue; // Already added

    auto *v = new g2o::VertexSE3();
    v->setId(kf->index);
    /*For new_keyframe, we might not have a global estimate yet if it's not in
    the graph? Actually new_keyframe->node is created in
    radar_graph_slam_nodelet before detect? Let's check nodelet logic...
    detect() is called BEFORE add_se3_node usually? In nodelet:
       const auto &keyframe = keyframe_queue[i];
       new_keyframes.push_back(keyframe);
       keyframe->node = graph_slam->add_se3_node(odom);
       ...
       loop_detector->detect(...)
      So YES, new_keyframe->node exists and has an initialized estimate (odom
      based).*/
    v->setEstimate(kf->node->estimate());

    // Do NOT fix the start of the chain. Fixing it forces the loop to agree
    // with global drift. By floating it, we allow the chain to correct itself.
    v->setFixed(false);

    optimizer.addVertex(v);
    vertex_map[kf->index] = v;
  }

  // C. Add Odometry Edges
  // Between consecutive frames in the chain
  for (size_t i = 1; i < chain.size(); ++i) {
    auto prev = chain[i - 1];
    auto curr = chain[i];

    // Check if we have both vertices (we should)
    if (vertex_map.find(prev->index) == vertex_map.end() ||
        vertex_map.find(curr->index) == vertex_map.end())
      continue;

    Eigen::Isometry3d rel_odom =
        prev->odom_scan2scan.inverse() * curr->odom_scan2scan;
    auto *edge = new g2o::EdgeSE3();
    edge->setVertex(0, vertex_map[prev->index]);
    edge->setVertex(1, vertex_map[curr->index]);
    edge->setMeasurement(rel_odom);
    // Strong Odom Info
    Eigen::MatrixXd info = Eigen::MatrixXd::Identity(6, 6) * 10.0;
    edge->setInformation(info);
    optimizer.addEdge(edge);
  }

  // D. Add Candidate Loop Edge
  if (vertex_map.find(new_keyframe->index) != vertex_map.end()) {
    auto *edge = new g2o::EdgeSE3();
    edge->setVertex(0, vertex_map[loop_kf_index]);
    edge->setVertex(1, vertex_map[new_keyframe->index]);
    edge->setMeasurement(loop_transform);
    // Loop Info
    Eigen::MatrixXd info = Eigen::MatrixXd::Identity(6, 6) * 10.0;
    edge->setInformation(info);
    optimizer.addEdge(edge);

    // E. Optimize
    optimizer.initializeOptimization();
    optimizer.computeActiveErrors();
    double chi2_before = edge->chi2();
    optimizer.optimize(verify_opt_iter);

    double chi2 = edge->chi2();

    std::cout << "[LoopDetector] Verify Chi2 Before: " << chi2_before
              << " After: " << chi2 << std::endl;
    if (chi2 > verify_chi2_thresh) {
      std::cout << "\033[31m[LoopDetector] Verification REJECTED. Chi2: "
                << chi2 << " > " << verify_chi2_thresh << "\033[0m"
                << std::endl;
      return false;
    } else {
      std::cout << "\033[32m[LoopDetector] Verification PASSED. Chi2: " << chi2
                << "\033[0m" << std::endl;
      return true;
    }
  }

  return false;
}

bool LoopDetector::checkStructure(const Eigen::Isometry3d &loop_transform,
                                  const KeyFrame::Ptr &new_keyframe,
                                  const KeyFrame::Ptr &candidate_keyframe) {
  // 1. Calculate R_min for new_keyframe (nearest obstacle distance)
  double r_min = std::numeric_limits<double>::max();
  for (const auto &pt : new_keyframe->cloud->points) {
    double dist = pt.getVector3fMap().norm();
    if (dist > structure_check_min_dist && dist < r_min) {
      r_min = dist;
    }
  }

  // Safety check if point cloud is empty or all points too close
  if (r_min == std::numeric_limits<double>::max())
    return true;

  // 2. Check violation (New -> Candidate)
  // Transform candidate cloud to new_keyframe frame
  pcl::PointCloud<PointT>::Ptr transformed_candidate(
      new pcl::PointCloud<PointT>());
  pcl::transformPointCloud(*candidate_keyframe->cloud, *transformed_candidate,
                           loop_transform.cast<float>());

  int violation_count = 0;
  int total_points = 0;

  for (const auto &pt : transformed_candidate->points) {
    double dist = pt.getVector3fMap().norm();
    if (dist > structure_check_min_dist) { // Only check points in valid range
      total_points++;
      if (dist < r_min - 0.5) { // 0.5m margin
        violation_count++;
      }
    }
  }

  if (total_points == 0)
    return true;

  double ratio = (double)violation_count / total_points;

  std::cout << "[LoopDetector] Structure Check: R_min=" << r_min
            << " Violation Ratio=" << ratio << " (" << violation_count << "/"
            << total_points << ")" << std::endl;

  if (ratio > structure_check_violation_ratio) {
    std::cout
        << "\033[31m[LoopDetector] Structure Check (Forward) REJECTED.\033[0m"
        << std::endl;
    return false;
  }

  // 3. Bidirectional Check (Candidate -> New)
  // We already checked Candidate points in New Frame's free space.
  // Now check New Frame's points in Candidate's free space.

  // R_min for candidate
  double r_min_candidate = std::numeric_limits<double>::max();
  for (const auto &pt : candidate_keyframe->cloud->points) {
    double dist = pt.getVector3fMap().norm();
    if (dist > structure_check_min_dist && dist < r_min_candidate) {
      r_min_candidate = dist;
    }
  }

  if (r_min_candidate == std::numeric_limits<double>::max())
    return true;

  // Transform new_keyframe cloud to candidate frame
  // T_lc is candidate -> new. So new -> candidate is T_lc.inverse()
  pcl::PointCloud<PointT>::Ptr transformed_new(new pcl::PointCloud<PointT>());
  pcl::transformPointCloud(*new_keyframe->cloud, *transformed_new,
                           loop_transform.inverse().cast<float>());

  violation_count = 0;
  total_points = 0;

  for (const auto &pt : transformed_new->points) {
    double dist = pt.getVector3fMap().norm();
    if (dist > structure_check_min_dist) {
      total_points++;
      if (dist < r_min_candidate - 0.5) {
        violation_count++;
      }
    }
  }

  if (total_points == 0)
    return true;

  double ratio_rev = (double)violation_count / total_points;
  std::cout << "[LoopDetector] Structure Check (Reverse): R_min="
            << r_min_candidate << " Violation Ratio=" << ratio_rev << " ("
            << violation_count << "/" << total_points << ")" << std::endl;

  if (ratio_rev > structure_check_violation_ratio) {
    std::cout
        << "\033[31m[LoopDetector] Structure Check (Reverse) REJECTED.\033[0m"
        << std::endl;
    return false;
  }

  return true;
}
// #endif

} // namespace radar_graph_slam