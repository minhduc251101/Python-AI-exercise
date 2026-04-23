
# # # #include <cmath>
# # # #include <radar_graph_slam/loop_detector.hpp>

# # # #include <g2o/core/block_solver.h>
# # # #include <g2o/core/optimization_algorithm_factory.h>
# # # #include <g2o/core/optimization_algorithm_levenberg.h>
# # # #include <g2o/solvers/eigen/linear_solver_eigen.h>
# # # #include <g2o/types/slam3d/edge_se3.h>
# # # #include <g2o/types/slam3d/vertex_se3.h>

# # # using namespace std;

# # # Eigen::Vector3d R2ypr_(const Eigen::Matrix3d &R) {
# # #   Eigen::Vector3d n = R.col(0);
# # #   Eigen::Vector3d o = R.col(1);
# # #   Eigen::Vector3d a = R.col(2);

# # #   Eigen::Vector3d ypr(3);
# # #   double y = std::atan2(
# # #       n(1), n(0)); // tinh goc tu nhung duong point cloud gan nhau  -> map
# # #   double p = std::atan2(-n(2), n(0) * std::cos(y) + n(1) * std::sin(y));
# # #   double r = std::atan2(a(0) * std::sin(y) - a(1) * std::cos(y),
# # #                         -o(0) * std::sin(y) + o(1) * std::cos(y));
# # #   ypr(0) = y;
# # #   ypr(1) = p;
# # #   ypr(2) = r;

# # #   return ypr;
# # # }

# # # double limit_value(int value, int min_, int max_) {
# # #   if (value < min_)
# # #     return min_;
# # #   else if (value > max_)
# # #     return max_;
# # #   else
# # #     return value;
# # # }

# # # Eigen::Vector3d monoToRainbow(int value) { // dung diem anh
# # #   double k = 4.65454545454;                // tham so, goc khong vuot qua
# # #   double blue = limit_value(-k * (value - 140), 0, 255);
# # #   double green, red;
# # #   if (value < 30) {
# # #     green = 0;
# # #     red = limit_value(-k * (value - 30), 0, 255);
# # #   } else if (value < 140) {
# # #     green = limit_value(k * (value - 30), 0, 255);
# # #     red = 0;
# # #   } else {
# # #     green = limit_value(-k * (value - 250), 0, 255);
# # #     red = limit_value(k * (value - 140), 0, 255);
# # #   }
# # #   return Eigen::Vector3d(blue, green, red);
# # # }

# # # namespace radar_graph_slam {

# # # LoopDetector::LoopDetector(ros::NodeHandle &pnh) {
# # #   enable_pf = pnh.param<bool>("enable_pf", true);
# # #   enable_odom_check = pnh.param<bool>("enable_odom_check", true);

# # #   distance_thresh = pnh.param<double>("distance_thresh", 5.0);
# # #   accum_distance_thresh = pnh.param<double>("accum_distance_thresh", 8.0);
# # #   min_loop_interval_dist = pnh.param<double>("min_loop_interval_dist", 5.0);

# # #   fitness_score_max_range = pnh.param<double>(
# # #       "fitness_score_max_range", std::numeric_limits<double>::max());
# # #   fitness_score_thresh =
# # #       pnh.param<double>("fitness_score_thresh", 0.7); // default 0.5

# # #   odom_drift_xy = pnh.param<double>("odometry_drift_xy", 0.05);
# # #   odom_drift_z = pnh.param<double>("odometry_drift_z", 0.20);
# # #   drift_scale_xy = pnh.param<double>("odometry_drift_scale_xy", 2.0);
# # #   drift_scale_z = pnh.param<double>("odometry_drift_scale_z", 2.0);

# # #   max_baro_difference = pnh.param<double>("max_baro_difference", 3.0);
# # #   max_yaw_difference = pnh.param<double>("max_yaw_difference", 45.0);

# # #   double sc_dist_thresh = pnh.param<double>("sc_dist_thresh", 0.3);
# # #   double sc_azimuth_range = pnh.param<double>("sc_azimuth_range", 0.3);

# # #   registration = select_registration_method(pnh);

# # #   // Loop Closure Parameters
# # #   pnh.param<int>("lio_sam/historyKeyframeSearchNum", historyKeyframeSearchNum,
# # #                  25);
# # #   historyKeyframeFitnessScore =
# # #       pnh.param<float>("historyKeyframeFitnessScore", 6);
# # #   odom_check_trans_thresh = pnh.param<double>("odom_check_trans_thresh", 0.1);
# # #   odom_check_rot_thresh = pnh.param<double>("odom_check_rot_thresh", 0.05);

# # #   scManager.reset(new SCManager());
# # #   inf_calclator.reset(new InformationMatrixCalculator(pnh));
# # #   image_transport.reset(new image_transport::ImageTransport(pnh));
# # #   pub_cur_sc = image_transport->advertise("/sc/cur", 1);
# # #   pub_pre_sc = image_transport->advertise("/sc/pre", 1);

# # #   scManager->setScDistThresh(sc_dist_thresh);
# # #   scManager->setAzimuthRange(sc_azimuth_range);

# # #   // Verification
# # #   enable_loop_verification = pnh.param<bool>("enable_loop_verification", true);
# # #   verify_graph_size = pnh.param<int>("verify_graph_size", 20);
# # #   verify_chi2_thresh = pnh.param<double>("verify_chi2_thresh", 5000.0);
# # #   verify_opt_iter = pnh.param<int>("verify_opt_iter", 5);

# # #   structure_check_min_dist = pnh.param<double>("structure_check_min_dist", 1.0);
# # #   structure_check_violation_ratio =
# # #       pnh.param<double>("structure_check_violation_ratio", 0.05);
# # #   max_allowed_drift_ratio = pnh.param<double>("max_allowed_drift_ratio", 0.1);
# # # }

# # # LoopDetector::~LoopDetector() {}

# # # /**
# # #  * @brief detect loops and add them to the pose graph
# # #  * @param keyframes       keyframes
# # #  * @param new_keyframes   newly registered keyframes
# # #  * @param graph_slam      pose graph
# # #  */
# # # std::vector<Loop::Ptr>
# # # LoopDetector::detect(const std::vector<KeyFrame::Ptr> &keyframes,
# # #                      const std::deque<KeyFrame::Ptr> &new_keyframes,
# # #                      radar_graph_slam::GraphSLAM &graph_slam) {
# # #   std::vector<Loop::Ptr> detected_loops;
# # #   for (const auto &new_keyframe : new_keyframes) {
# # #     std::vector<KeyFrame::Ptr> candidates;
# # #     Loop::Ptr loop = nullptr;
# # #     if (enable_pf) {
# # #       start_ms_pf = clock();
# # #       candidates = find_candidates(
# # #           keyframes, new_keyframe); // keyframe diem hien tai, new thi dung
# # #                                     // thanh nhieu diem lan can
# # #       end_ms_pf = clock();
# # #       std::cout << "[LoopDetector] Keyframe " << new_keyframe->index
# # #                 << ": Found " << candidates.size() << " loop candidates."
# # #                 << std::endl;
# # #     } else {
# # #       for (auto kf : keyframes) {
# # #         if (new_keyframe->accum_distance > kf->accum_distance + distance_thresh)
# # #           ;
# # #         candidates.push_back(kf);
# # #       }
# # #     }
# # #     // 1. Try Scan Context First (Fast & Safe)
# # #     loop = performScanContextLoopClosure(keyframes, candidates, new_keyframe);

# # #     // 2. If Scan Context fails,
# # #     //     Fallback to Geometric Matching(Brute Force) if (loop == nullptr) {
# # #     //   std::cout << "\033[35m[LoopDetector] Scan Context loop closure failed.
# # #     //                "
# # #     //                "Attempting Geometric Matching...\033[0m"
# # #     //             << std::endl;
# # #     //   loop = matching(keyframes, candidates, new_keyframe);
# # #     // }
# # #     if (loop) {
# # #       detected_loops.push_back(loop);
# # #       double time_used = double(end_ms_pf - start_ms_pf) / CLOCKS_PER_SEC;
# # #       pf_time.push_back(time_used);
# # #       time_used = double(end_ms_sc - start_ms_sc) / CLOCKS_PER_SEC;
# # #       sc_time.push_back(time_used);
# # #       time_used = double(end_ms_oc - start_ms_oc) / CLOCKS_PER_SEC;
# # #       oc_time.push_back(time_used);
# # #     }
# # #   }

# # #   return detected_loops;
# # # }

# # # /**
# # #  * @brief find loop candidates. A detected loop begins at one of #keyframes and
# # #  * ends at #new_keyframe
# # #  * @param keyframes      candidate keyframes of loop start
# # #  * @param new_keyframe   loop end keyframe
# # #  * @return loop candidates
# # #  */
# # # std::vector<KeyFrame::Ptr>
# # # LoopDetector::find_candidates(const std::vector<KeyFrame::Ptr> &keyframes,
# # #                               const KeyFrame::Ptr &new_keyframe) const {
# # #   // 1. too close to the last registered loop edge
# # #   double dist_btn_last_loop_edge =
# # #       new_keyframe->accum_distance - last_loop_edge_accum_distance;
# # #   if (dist_btn_last_loop_edge < min_loop_interval_dist) {
# # #     return std::vector<KeyFrame::Ptr>();
# # #   }
# # #   // Pick loop candicates considering distance and barometer measurements
# # #   std::vector<KeyFrame::Ptr> candidates;
# # #   candidates.reserve(32);

# # #   for (const auto &k : keyframes) {
# # #     // 2. traveled distance between keyframes is too small
# # #     double accum_distance = new_keyframe->accum_distance - k->accum_distance;
# # #     if (accum_distance < accum_distance_thresh) {
# # #       continue;
# # #     }
# # #     // 3. If keyframes_ have similar barometer measurements // value
# # #     if (k->altitude.is_initialized()) {
# # #       if (fabs(k->altitude.value()[0] - new_keyframe->altitude.value()[0]) >
# # #           max_baro_difference)
# # #         continue;
# # #     }
# # #     // 4. yaw angle difference
# # #     Eigen::Matrix4d T = k->node->estimate().matrix().inverse() *
# # #                         new_keyframe->node->estimate().matrix();
# # #     Eigen::Vector3d ypr_diff = R2ypr_(T.block<3, 3>(0, 0));
# # #     if (abs(ypr_diff(0) * 180 / M_PI) > max_yaw_difference)
# # #       continue;
# # #     // 5. estimated distance between new-keyframe is too Large
# # #     Eigen::Vector3d pos_between = T.block<3, 1>(0, 3);
# # #     double dist = pos_between.norm(); // measure x-y distance
# # #     double x_diff = pos_between(0), y_diff = pos_between(1),
# # #            z_diff = pos_between(2);
# # #     double rad_xy_loop =
# # #         3 + dist_btn_last_loop_edge * odom_drift_xy * drift_scale_xy;
# # #     double rad_z_loop =
# # #         3 + dist_btn_last_loop_edge * odom_drift_z * drift_scale_z;
# # #     double aa_lle =
# # #         pow((x_diff / rad_xy_loop), 2) + pow((y_diff / rad_xy_loop), 2);
# # #     if (aa_lle > 1)
# # #       continue;
# # #     // cout << "last_loop_edge_index: " << last_loop_edge_index << "
# # #     // new_keyframe: " << new_keyframe->index
# # #     //         << " dist_btn_last_loop_edge = " << dist_btn_last_loop_edge <<
# # #     //         endl;
# # #     // cout << "rad_xy_loop " << rad_xy_loop << endl;
# # #     // cout << "pos_btn: " << pos_between(0) << " " << pos_between(1) << " " <<
# # #     // pos_between(2) << endl; cout << " aa_lle = " << aa_lle << endl << endl;

# # #     double rad_xy = 10.0 + odom_drift_xy * accum_distance * drift_scale_xy;
# # #     double rad_z = 10.0 + odom_drift_z * accum_distance * drift_scale_z;
# # #     double aa = pow((x_diff / rad_xy), 2) +
# # #                 pow((y_diff / rad_xy), 2); //+pow((z_diff/rad_z),2);
# # #     if (aa >
# # #         1) // candidate keyframe pose is out of elipse  (dist > distance_thresh)
# # #       continue;

# # #     candidates.push_back(k);
# # #   }

# # #   return candidates;
# # # }

# # # Loop::Ptr LoopDetector::performScanContextLoopClosure(
# # #     const std::vector<KeyFrame::Ptr> &keyframes,
# # #     const std::vector<KeyFrame::Ptr> &candidate_keyframes,
# # #     const KeyFrame::Ptr &new_keyframe) {
# # #   start_ms_sc = clock();
# # #   if (candidate_keyframes.empty() == true) {
# # #     std::cout << "\033[33m[LoopDetector] No loop candidates found.\033[0m"
# # #               << std::endl;
# # #     return nullptr;
# # #     // ROS_INFO("\033[33m Cannot find Loop Candidate \033[0m");
# # #   }
# # #   std::cout << "\033[36m" << " New keyframe index: " << new_keyframe->index
# # #             << ". "
# # #             << "Number of candidate frames: " << candidate_keyframes.size();
# # #   // << "Candidate frames index: ";
# # #   // for (auto& ckf : candidate_keyframes){
# # #   //   std::cout << ckf->index << " ";
# # #   // }
# # #   std::cout << "\033[0m" << endl;

# # #   // Detect keys, first: nn index, second: yaw diff
# # #   auto detectResult =
# # #       scManager->detectLoopClosureID(candidate_keyframes, new_keyframe);
# # #   int loopKeyCur = new_keyframe->index;
# # #   int loopKeyPre = detectResult.first;
# # #   float yawDiffRad = detectResult.second; // not use for v1 (because pcl icp
# # #                                           // withi initial somthing wrong...)
# # #   if (loopKeyPre == -1) {
# # #     std::cout << "\033[33m[LoopDetector] Scan Context failed to find a "
# # #                  "matching candidates ID (score too low).\033[0m"
# # #               << std::endl;
# # #     return nullptr; /* No loop found */
# # #   }

# # #   // extract cloud
# # #   pcl::PointCloud<PointT>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointT>());
# # #   pcl::PointCloud<PointT>::Ptr prevKeyframeCloud(new pcl::PointCloud<PointT>());
# # #   {
# # #     *cureKeyframeCloud +=
# # #         *new_keyframe->cloud; // Because new_keyframe is not in keyframes
# # #     *prevKeyframeCloud += *keyframes[loopKeyPre]->cloud;
# # #   }

# # #   registration->setInputSource(cureKeyframeCloud);
# # #   registration->setInputTarget(prevKeyframeCloud);
# # #   pcl::PointCloud<PointT>::Ptr unused_result(new pcl::PointCloud<PointT>());
# # #   registration->align(*unused_result);
# # #   // TODO icp align with initial
# # #   end_ms_sc = clock();

# # #   if (registration->hasConverged() == false ||
# # #       registration->getFitnessScore() > historyKeyframeFitnessScore) {
# # #     bool converged = registration->hasConverged();
# # #     std::cout
# # #         << "\033[31m[LoopDetector] SC Loop Failed: ICP Fitness Score too high. "
# # #         << "Score: " << registration->getFitnessScore()
# # #         << " > Thresh: " << historyKeyframeFitnessScore << "\033[0m"
# # #         << std::endl;
# # #     return nullptr;
# # #   }
# # #   // Get pose transformation
# # #   Eigen::Matrix4f correctionLidarFrame;
# # #   correctionLidarFrame = registration->getFinalTransformation();

# # #   Eigen::Isometry3d poseFrom(Eigen::Isometry3d::Identity());
# # #   Eigen::Isometry3d poseTo(Eigen::Isometry3d::Identity());
# # #   Eigen::Vector3d vec =
# # #       Eigen::Vector3d(correctionLidarFrame(0, 3), correctionLidarFrame(1, 3),
# # #                       correctionLidarFrame(2, 3));
# # #   Eigen::Matrix3d mat = correctionLidarFrame.block(0, 0, 3, 3).cast<double>();
# # #   poseFrom.prerotate(mat); // Set rotation
# # #   poseFrom.pretranslate(vec);
# # #   Eigen::Isometry3d T_lc_ij = poseFrom.inverse() * poseTo;

# # #   /****** Incremental Consistent Measurement Set Maximization ******/
# # #   // Odometry check (source_frame<->target_frame and
# # #   // source_frame<->last_loop_edge)
# # #   start_ms_oc = clock();
# # #   if (enable_odom_check) {
# # #     // source_frame <-> target_frame
# # #     Eigen::Isometry3d T_odom_ji = new_keyframe->odom_scan2scan.inverse() *
# # #                                   keyframes[loopKeyPre]->odom_scan2scan;
# # #     Eigen::Isometry3d T_err_ij = T_lc_ij * T_odom_ji;
# # #     int num_between =
# # #         loopKeyCur - loopKeyPre; // the number of edges along the loop
# # #     Eigen::AngleAxisd rotation_vector;
# # #     rotation_vector.fromRotationMatrix(T_err_ij.rotation());
# # #     double oc_err_trans = T_err_ij.translation().norm() / num_between;
# # #     double oc_err_rot = rotation_vector.angle() / num_between;
# # #     // double err_rot =
# # #     // std::acos(Eigen::Quaterniond(T_err_ij.rotation()).w())/num_between; cout
# # #     // << oc_err_rot << " " << err_rot << endl;
# # #     if (oc_err_trans > odom_check_trans_thresh ||
# # #         oc_err_rot > odom_check_rot_thresh) {
# # #       cout << "\033[31m[LoopDetector] SC Loop Failed: Odometry Check invalid. "
# # #            << "TransErr: " << oc_err_trans
# # #            << " (Thresh: " << odom_check_trans_thresh << "), "
# # #            << "RotErr: " << oc_err_rot << " (Thresh: " << odom_check_rot_thresh
# # #            << ")\033[0m" << endl;
# # #       return nullptr;
# # #     }
# # #     // source_frame <-> last_loop_edge
# # #   }
# # #   end_ms_oc = clock();

# # #   std::cout
# # #       << "\033[32m"
# # #       << " Add this ScanContext loop. ICP fitness test passed ( Fitness Score "
# # #       << registration->getFitnessScore() << " < " << historyKeyframeFitnessScore
# # #       << "). " << "\033[0m" << std::endl;
# # #   std::cout << "\033[32m" << ">>> SCAN CONTEXT LOOP FOUND! Keyframe "
# # #             << new_keyframe->index << " <--> " << loopKeyPre << " \033[0m"
# # #             << std::endl;

# # #   // Publish context as image
# # #   Eigen::MatrixXd &cur_sc = scManager->polarcontexts_.at(loopKeyCur);
# # #   Eigen::MatrixXd &pre_sc = scManager->polarcontexts_.at(loopKeyPre);
# # #   cv::Mat mat_cur_sc = makeSCImage(cur_sc);
# # #   cv::Mat color_cur_sc = getColorImage(mat_cur_sc);
# # #   cv::Mat mat_pre_sc = makeSCImage(pre_sc);
# # #   cv::Mat color_pre_sc = getColorImage(mat_pre_sc);

# # #   sensor_msgs::ImagePtr cur_img_msg =
# # #       cv_bridge::CvImage(std_msgs::Header(), "bgr8", color_cur_sc).toImageMsg();
# # #   pub_cur_sc.publish(cur_img_msg);
# # #   sensor_msgs::ImagePtr pre_img_msg =
# # #       cv_bridge::CvImage(std_msgs::Header(), "bgr8", color_pre_sc).toImageMsg();
# # #   pub_pre_sc.publish(pre_img_msg);

# # #   // robust kernel for a SC loop
# # #   Eigen::MatrixXd information = inf_calclator->calc_information_matrix(
# # #       cureKeyframeCloud, prevKeyframeCloud, T_lc_ij);

# # #   if (new_keyframe->accum_distance > last_loop_edge_accum_distance) {
# # #     last_loop_edge_accum_distance = new_keyframe->accum_distance;
# # #     last_loop_edge_index = new_keyframe->index;
# # #   }

# # #   // Verify Loop (Structure + g2o)
# # #   if (enable_loop_verification) {
# # #     // 1. Structure Check
# # #     if (!checkStructure(T_lc_ij, new_keyframe, keyframes[loopKeyPre])) {
# # #       return nullptr;
# # #     }

# # #     // 2. g2o Verification
# # #     if (!verifyLoop(T_lc_ij, loopKeyPre, new_keyframe, keyframes)) {
# # #       return nullptr;
# # #     }
# # #   }

# # #   // Add pose constraint
# # #   mtx.lock();
# # #   loopIndexQueue.push_back(make_pair(loopKeyCur, loopKeyPre));
# # #   loopPoseQueue.push_back(T_lc_ij);
# # #   loopInfoQueue.push_back(information);
# # #   mtx.unlock();
# # #   // Add loop constriant
# # #   loopIndexContainer.insert(
# # #       std::pair<int, int>(loopKeyCur, loopKeyPre)); // giseop for multimap

# # #   return std::make_shared<Loop>(new_keyframe, keyframes[loopKeyPre],
# # #                                 T_lc_ij.matrix().cast<float>());
# # # }

# # # cv::Mat LoopDetector::makeSCImage(Eigen::MatrixXd src_mat) {
# # #   // double max_ = src_mat.rowwise().maxCoeff().colwise().maxCoeff().value();
# # #   // double min_ = src_mat.rowwise().minCoeff().colwise().minCoeff().value();
# # #   double max_ = 35;
# # #   double min_ = 0;
# # #   cv::Mat ssc = cv::Mat::zeros(
# # #       cv::Size(scManager->PC_NUM_SECTOR, scManager->PC_NUM_RING), CV_8U);
# # #   for (int i = 0; i < scManager->PC_NUM_SECTOR; i++) { // 60
# # #     for (int j = 0; j < scManager->PC_NUM_RING; j++) { // 20
# # #       double &d = src_mat(j, i);
# # #       int uc = round((d - min_) / (max_ - min_) * 255);
# # #       ssc.at<uchar>(j, i) = uc;
# # #     }
# # #   }
# # #   return ssc;
# # # }

# # # cv::Mat LoopDetector::getColorImage(cv::Mat &desc) {
# # #   cv::Mat out = cv::Mat::zeros(desc.size(), CV_8UC3);
# # #   for (int i = 0; i < desc.rows; ++i) {
# # #     for (int j = 0; j < desc.cols; ++j) {
# # #       int value_mono = (int)desc.at<uchar>(
# # #           i, j); // std::get<2>(_argmax_to_rgb[(int)desc.at<uchar>(i, j)]);
# # #       if (value_mono == 0) {
# # #         out.at<cv::Vec3b>(i, j)[0] = 255;
# # #         out.at<cv::Vec3b>(i, j)[1] = 255;
# # #         out.at<cv::Vec3b>(i, j)[2] = 255;
# # #       } else {
# # #         Eigen::Vector3d bgr = monoToRainbow(value_mono);
# # #         out.at<cv::Vec3b>(i, j)[0] = (int)bgr(0); // 255 - value_mono;
# # #         out.at<cv::Vec3b>(i, j)[1] = (int)bgr(
# # #             1); // 0;//std::min(std::max(128 - value_mono, 0), value_mono);
# # #         out.at<cv::Vec3b>(i, j)[2] = (int)bgr(2); // value_mono;
# # #       }
# # #     }
# # #   }
# # #   return out;
# # # }

# # # double LoopDetector::get_distance_thresh() const { return distance_thresh; }

# # # // #if 0
# # # /**
# # #  * @brief To validate a loop candidate this function applies a scan matching
# # #  * between keyframes consisting the loop. If they are matched well, the loop is
# # #  * added to the pose graph
# # #  * @param candidate_keyframes  candidate keyframes of loop start
# # #  * @param new_keyframe         loop end keyframe
# # #  * @param graph_slam           graph slam
# # #  */
# # # Loop::Ptr
# # # LoopDetector::matching(const std::vector<KeyFrame::Ptr> &keyframes,
# # #                        const std::vector<KeyFrame::Ptr> &candidate_keyframes,
# # #                        const KeyFrame::Ptr &new_keyframe) {
# # #   if (candidate_keyframes.empty()) {
# # #     return nullptr;
# # #   }

# # #   registration->setInputTarget(new_keyframe->cloud);

# # #   double best_score = std::numeric_limits<double>::max();
# # #   KeyFrame::Ptr best_matched;
# # #   Eigen::Matrix4f relative_pose;

# # #   std::cout << std::endl;
# # #   std::cout << "--- loop detection ---" << std::endl;
# # #   std::cout << "num_candidates: " << candidate_keyframes.size() << std::endl;
# # #   std::cout << "matching" << std::flush;
# # #   auto t1 = ros::Time::now();

# # #   pcl::PointCloud<PointT>::Ptr aligned(new pcl::PointCloud<PointT>());
# # #   for (const auto &candidate : candidate_keyframes) {
# # #     registration->setInputSource(candidate->cloud);
# # #     Eigen::Isometry3d new_keyframe_estimate = new_keyframe->node->estimate();
# # #     new_keyframe_estimate.linear() =
# # #         Eigen::Quaterniond(new_keyframe_estimate.linear())
# # #             .normalized()
# # #             .toRotationMatrix();
# # #     Eigen::Isometry3d candidate_estimate = candidate->node->estimate();
# # #     candidate_estimate.linear() =
# # #         Eigen::Quaterniond(candidate_estimate.linear())
# # #             .normalized()
# # #             .toRotationMatrix();
# # #     Eigen::Matrix4f guess =
# # #         (new_keyframe_estimate.inverse() * candidate_estimate)
# # #             .matrix()
# # #             .cast<float>();
# # #     guess(2, 3) = 0.0;
# # #     registration->align(*aligned, guess);
# # #     std::cout << "." << std::flush;

# # #     double score = registration->getFitnessScore(fitness_score_max_range);
# # #     if (!registration->hasConverged() || score > best_score) {
# # #       continue;
# # #     }

# # #     best_score = score;
# # #     best_matched = candidate;
# # #     relative_pose = registration->getFinalTransformation();
# # #   }

# # #   auto t2 = ros::Time::now();
# # #   std::cout << " done" << std::endl;
# # #   std::cout << "best_score: " << boost::format("%.3f") % best_score
# # #             << "    time: " << boost::format("%.3f") % (t2 - t1).toSec()
# # #             << "[sec]" << std::endl;

# # #   if (best_score > fitness_score_thresh) {
# # #     std::cout
# # #         << "\033[31m[LoopDetector] Geometric Matching Failed. Best Score: "
# # #         << best_score << " > Thresh: " << fitness_score_thresh << "\033[0m"
# # #         << std::endl;
# # #     return nullptr;
# # #   }

# # #   std::cout << "loop found!!" << std::endl;
# # #   std::cout << ">>> LOOP CLOSURE: Keyframe " << new_keyframe->index
# # #             << " matched with Keyframe " << best_matched->index << " <<<"
# # #             << std::endl;
# # #   std::cout << "relative pose: " << relative_pose.block<3, 1>(0, 3) << " - "
# # #             << Eigen::Quaternionf(relative_pose.block<3, 3>(0, 0))
# # #                    .coeffs()
# # #                    .transpose()
# # #             << std::endl;

# # #   Eigen::Isometry3d relative_pose_d = Eigen::Isometry3d::Identity();
# # #   relative_pose_d.linear() = relative_pose.block<3, 3>(0, 0).cast<double>();
# # #   relative_pose_d.translation() =
# # #       relative_pose.block<3, 1>(0, 3).cast<double>();

# # #   Eigen::MatrixXd information_matrix = inf_calclator->calc_information_matrix(
# # #       new_keyframe->cloud, best_matched->cloud, relative_pose_d);
# # #   // Verify Loop
# # #   if (enable_loop_verification) {
# # #     Eigen::Isometry3d T_lc(relative_pose.cast<double>());
# # #     if (!verifyLoop(T_lc, best_matched->index, new_keyframe, keyframes)) {
# # #       return nullptr;
# # #     }
# # #   }

# # #   // update edge for loop closure connect keyframe 418 and keyframe 2; save
# # #   // best_matched in LoopIndexQueue(vector<pair<int, int>>) +1 edge
# # #   loopIndexQueue.push_back(
# # #       std::make_pair(new_keyframe->index, best_matched->index));
# # #   // loopIndexQueue.push_back({new_keyframe->index, best_matched->index});
# # #   loopPoseQueue.push_back(relative_pose_d);
# # #   loopInfoQueue.push_back(information_matrix);
# # #   loopIndexContainer.insert(
# # #       std::pair<int, int>(new_keyframe->index, best_matched->index));

# # #   last_loop_edge_accum_distance = new_keyframe->accum_distance;

# # #   return std::make_shared<Loop>(new_keyframe, best_matched, relative_pose);
# # # }

# # # bool LoopDetector::verifyLoop(const Eigen::Isometry3d &loop_transform,
# # #                               int loop_kf_index,
# # #                               const KeyFrame::Ptr &new_keyframe,
# # #                               const std::vector<KeyFrame::Ptr> &keyframes) {
# # #   if (!enable_loop_verification)
# # #     return true;

# # #   // 0. Drift Consistency Check (New)
# # #   // Calculate path length of the chain
# # #   double path_length = 0.0;
# # #   Eigen::Isometry3d T_chain = Eigen::Isometry3d::Identity();

# # #   // We need to trace from candidate (loop_kf) to new_keyframe.
# # #   // Assuming 'keyframes' is time-ordered...
# # #   // Find index range
# # #   int start_idx = loop_kf_index;
# # #   int end_idx = new_keyframe->index;

# # #   // Verify order
# # #   if (start_idx > end_idx)
# # #     std::swap(start_idx, end_idx);

# # #   // Accumulate odometry
# # #   // Note: keyframes vector might not be contiguous by index if some were
# # #   // dropped? But usually it is. Let's assume indices in 'keyframes' vector map
# # #   // to ID. Actually, 'keyframes' is a std::vector created in detect(). It is
# # #   // likely the full list.

# # #   for (int i = 0; i < (int)keyframes.size() - 1; ++i) {
# # #     if (keyframes[i]->index >= start_idx &&
# # #         keyframes[i + 1]->index <= end_idx) {
# # #       Eigen::Isometry3d d_odom = keyframes[i]->odom_scan2scan.inverse() *
# # #                                  keyframes[i + 1]->odom_scan2scan;
# # #       path_length += d_odom.translation().norm();
# # #       T_chain = T_chain * d_odom;
# # #     }
# # #   }

# # #   // If we couldn't calc path (e.g. indices mismatch), skip or warn
# # #   if (path_length > 1.0) {
# # #     // Loop Transform T_lc connects loop_kf -> new_keyframe
# # #     // So T_loop should be approx T_chain
# # #     // Error = T_chain.inverse * T_loop
# # #     Eigen::Isometry3d T_diff = T_chain.inverse() * loop_transform;
# # #     double drift_error = T_diff.translation().norm();
# # #     double drift_ratio = drift_error / path_length;

# # #     std::cout << "[LoopDetector] Drift Check: PathLen=" << path_length
# # #               << "m, Error=" << drift_error << "m, Ratio=" << drift_ratio
# # #               << std::endl;

# # #     if (drift_ratio > max_allowed_drift_ratio) {
# # #       std::cout << "\033[31m[LoopDetector] Drift Check REJECTED. Ratio "
# # #                 << drift_ratio << " > " << max_allowed_drift_ratio << "\033[0m"
# # #                 << std::endl;
# # #       return false;
# # #     }
# # #   }

# # #   // 1. Construct local graph
# # #   g2o::SparseOptimizer optimizer;
# # #   optimizer.setVerbose(false);
# # #   std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver(
# # #       new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>());
# # #   std::unique_ptr<g2o::BlockSolver_6_3> solver_ptr(
# # #       new g2o::BlockSolver_6_3(std::move(linearSolver)));
# # #   g2o::OptimizationAlgorithmLevenberg *solver =
# # #       new g2o::OptimizationAlgorithmLevenberg(std::move(solver_ptr));
# # #   optimizer.setAlgorithm(solver);

# # #   // Identify nodes to add
# # #   std::map<int, g2o::VertexSE3 *> vertex_map;

# # #   // A. Add Loop Candidate Keyframe (Fixed)
# # #   auto *v_loop = new g2o::VertexSE3();
# # #   v_loop->setId(loop_kf_index);

# # #   // Get estimate for loop_kf
# # #   // keyframes is the full list. We expect loop_kf_index to satisfy
# # #   // keyframes[i]->index == loop_kf_index if indices are sequential. But safest
# # #   // is to search or index if we trust structure. In this system keyframes is a
# # #   // vector. Let's find it.
# # #   KeyFrame::Ptr loop_kf = nullptr;
# # #   // Fast check
# # #   if (loop_kf_index < keyframes.size() &&
# # #       keyframes[loop_kf_index]->index == loop_kf_index) {
# # #     loop_kf = keyframes[loop_kf_index];
# # #   } else {
# # #     for (const auto &k : keyframes) {
# # #       if (k->index == loop_kf_index) {
# # #         loop_kf = k;
# # #         break;
# # #       }
# # #     }
# # #   }

# # #   if (!loop_kf) {
# # #     std::cout << "[LoopDetector] Verify: Could not find loop keyframe "
# # #               << loop_kf_index << std::endl;
# # #     return false;
# # #   }

# # #   v_loop->setEstimate(loop_kf->node->estimate());
# # #   v_loop->setFixed(true);
# # #   optimizer.addVertex(v_loop);
# # #   vertex_map[loop_kf_index] = v_loop;

# # #   // B. Add Recent Chain
# # #   // We want the last 'verify_graph_size' keyframes from 'keyframes' +
# # #   // new_keyframe.
# # #   std::vector<KeyFrame::Ptr> chain;
# # #   int count = 0;
# # #   for (int i = (int)keyframes.size() - 1; i >= 0 && count < verify_graph_size;
# # #        --i) {
# # #     chain.push_back(keyframes[i]);
# # #     count++;
# # #   }
# # #   std::reverse(chain.begin(), chain.end());
# # #   chain.push_back(new_keyframe);

# # #   for (size_t i = 0; i < chain.size(); ++i) {
# # #     const auto &kf = chain[i];
# # #     if (kf->index == loop_kf_index)
# # #       continue; // Already added

# # #     auto *v = new g2o::VertexSE3();
# # #     v->setId(kf->index);
# # #     /*For new_keyframe, we might not have a global estimate yet if it's not in
# # #     the graph? Actually new_keyframe->node is created in
# # #     radar_graph_slam_nodelet before detect? Let's check nodelet logic...
# # #     detect() is called BEFORE add_se3_node usually? In nodelet:
# # #        const auto &keyframe = keyframe_queue[i];
# # #        new_keyframes.push_back(keyframe);
# # #        keyframe->node = graph_slam->add_se3_node(odom);
# # #        ...
# # #        loop_detector->detect(...)
# # #       So YES, new_keyframe->node exists and has an initialized estimate (odom
# # #       based).*/
# # #     v->setEstimate(kf->node->estimate());

# # #     // Do NOT fix the start of the chain. Fixing it forces the loop to agree
# # #     // with global drift. By floating it, we allow the chain to correct itself.
# # #     v->setFixed(false);

# # #     optimizer.addVertex(v);
# # #     vertex_map[kf->index] = v;
# # #   }

# # #   // C. Add Odometry Edges
# # #   // Between consecutive frames in the chain
# # #   for (size_t i = 1; i < chain.size(); ++i) {
# # #     auto prev = chain[i - 1];
# # #     auto curr = chain[i];

# # #     // Check if we have both vertices (we should)
# # #     if (vertex_map.find(prev->index) == vertex_map.end() ||
# # #         vertex_map.find(curr->index) == vertex_map.end())
# # #       continue;

# # #     Eigen::Isometry3d rel_odom =
# # #         prev->odom_scan2scan.inverse() * curr->odom_scan2scan;
# # #     auto *edge = new g2o::EdgeSE3();
# # #     edge->setVertex(0, vertex_map[prev->index]);
# # #     edge->setVertex(1, vertex_map[curr->index]);
# # #     edge->setMeasurement(rel_odom);
# # #     // Strong Odom Info
# # #     Eigen::MatrixXd info = Eigen::MatrixXd::Identity(6, 6) * 10.0;
# # #     edge->setInformation(info);
# # #     optimizer.addEdge(edge);
# # #   }

# # #   // D. Add Candidate Loop Edge
# # #   if (vertex_map.find(new_keyframe->index) != vertex_map.end()) {
# # #     auto *edge = new g2o::EdgeSE3();
# # #     edge->setVertex(0, vertex_map[loop_kf_index]);
# # #     edge->setVertex(1, vertex_map[new_keyframe->index]);
# # #     edge->setMeasurement(loop_transform);
# # #     // Loop Info
# # #     Eigen::MatrixXd info = Eigen::MatrixXd::Identity(6, 6) * 10.0;
# # #     edge->setInformation(info);
# # #     optimizer.addEdge(edge);

# # #     // E. Optimize
# # #     optimizer.initializeOptimization();
# # #     optimizer.computeActiveErrors();
# # #     double chi2_before = edge->chi2();
# # #     optimizer.optimize(verify_opt_iter);

# # #     double chi2 = edge->chi2();

# # #     std::cout << "[LoopDetector] Verify Chi2 Before: " << chi2_before
# # #               << " After: " << chi2 << std::endl;
# # #     if (chi2 > verify_chi2_thresh) {
# # #       std::cout << "\033[31m[LoopDetector] Verification REJECTED. Chi2: "
# # #                 << chi2 << " > " << verify_chi2_thresh << "\033[0m"
# # #                 << std::endl;
# # #       return false;
# # #     } else {
# # #       std::cout << "\033[32m[LoopDetector] Verification PASSED. Chi2: " << chi2
# # #                 << "\033[0m" << std::endl;
# # #       return true;
# # #     }
# # #   }

# # #   return false;
# # # }

# # # bool LoopDetector::checkStructure(const Eigen::Isometry3d &loop_transform,
# # #                                   const KeyFrame::Ptr &new_keyframe,
# # #                                   const KeyFrame::Ptr &candidate_keyframe) {
# # #   // 1. Calculate R_min for new_keyframe (nearest obstacle distance)
# # #   double r_min = std::numeric_limits<double>::max();
# # #   for (const auto &pt : new_keyframe->cloud->points) {
# # #     double dist = pt.getVector3fMap().norm();
# # #     if (dist > structure_check_min_dist && dist < r_min) {
# # #       r_min = dist;
# # #     }
# # #   }

# # #   // Safety check if point cloud is empty or all points too close
# # #   if (r_min == std::numeric_limits<double>::max())
# # #     return true;

# # #   // 2. Check violation (New -> Candidate)
# # #   // Transform candidate cloud to new_keyframe frame
# # #   pcl::PointCloud<PointT>::Ptr transformed_candidate(
# # #       new pcl::PointCloud<PointT>());
# # #   pcl::transformPointCloud(*candidate_keyframe->cloud, *transformed_candidate,
# # #                            loop_transform.cast<float>());

# # #   int violation_count = 0;
# # #   int total_points = 0;

# # #   for (const auto &pt : transformed_candidate->points) {
# # #     double dist = pt.getVector3fMap().norm();
# # #     if (dist > structure_check_min_dist) { // Only check points in valid range
# # #       total_points++;
# # #       if (dist < r_min - 0.5) { // 0.5m margin
# # #         violation_count++;
# # #       }
# # #     }
# # #   }

# # #   if (total_points == 0)
# # #     return true;

# # #   double ratio = (double)violation_count / total_points;

# # #   std::cout << "[LoopDetector] Structure Check: R_min=" << r_min
# # #             << " Violation Ratio=" << ratio << " (" << violation_count << "/"
# # #             << total_points << ")" << std::endl;

# # #   if (ratio > structure_check_violation_ratio) {
# # #     std::cout
# # #         << "\033[31m[LoopDetector] Structure Check (Forward) REJECTED.\033[0m"
# # #         << std::endl;
# # #     return false;
# # #   }

# # #   // 3. Bidirectional Check (Candidate -> New)
# # #   // We already checked Candidate points in New Frame's free space.
# # #   // Now check New Frame's points in Candidate's free space.

# # #   // R_min for candidate
# # #   double r_min_candidate = std::numeric_limits<double>::max();
# # #   for (const auto &pt : candidate_keyframe->cloud->points) {
# # #     double dist = pt.getVector3fMap().norm();
# # #     if (dist > structure_check_min_dist && dist < r_min_candidate) {
# # #       r_min_candidate = dist;
# # #     }
# # #   }

# # #   if (r_min_candidate == std::numeric_limits<double>::max())
# # #     return true;

# # #   // Transform new_keyframe cloud to candidate frame
# # #   // T_lc is candidate -> new. So new -> candidate is T_lc.inverse()
# # #   pcl::PointCloud<PointT>::Ptr transformed_new(new pcl::PointCloud<PointT>());
# # #   pcl::transformPointCloud(*new_keyframe->cloud, *transformed_new,
# # #                            loop_transform.inverse().cast<float>());

# # #   violation_count = 0;
# # #   total_points = 0;

# # #   for (const auto &pt : transformed_new->points) {
# # #     double dist = pt.getVector3fMap().norm();
# # #     if (dist > structure_check_min_dist) {
# # #       total_points++;
# # #       if (dist < r_min_candidate - 0.5) {
# # #         violation_count++;
# # #       }
# # #     }
# # #   }

# # #   if (total_points == 0)
# # #     return true;

# # #   double ratio_rev = (double)violation_count / total_points;
# # #   std::cout << "[LoopDetector] Structure Check (Reverse): R_min="
# # #             << r_min_candidate << " Violation Ratio=" << ratio_rev << " ("
# # #             << violation_count << "/" << total_points << ")" << std::endl;

# # #   if (ratio_rev > structure_check_violation_ratio) {
# # #     std::cout
# # #         << "\033[31m[LoopDetector] Structure Check (Reverse) REJECTED.\033[0m"
# # #         << std::endl;
# # #     return false;
# # #   }

# # #   return true;
# # # }
# # # // #endif

# # # } // namespace radar_graph_slam



import copy
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt
import pyvista as pv
#Read and plot pcd and ply file
voxel_size = 0.001 # Giu nhieu pcd hon 0.1

source_pv = pv.read("bun000.ply")
target_pv = pv.read("bun_zipper.ply")
# pcd = o3d.io.read_point_cloud("../../test_data/fragment.ply")
source_o3d = o3d.geometry.PointCloud()
source_o3d.points = o3d.utility.Vector3dVector(np.asarray(source_pv.points))

target_o3d = o3d.geometry.PointCloud()
target_o3d.points = o3d.utility.Vector3dVector(np.asarray(target_pv.points))

source_down = source_o3d.voxel_down_sample(voxel_size)
target_down = target_o3d.voxel_down_sample(voxel_size)

print("source plot\n")
source_pv.plot(eye_dome_lighting=True)

print("target plot\n")
target_pv.plot(eye_dome_lighting=True)

# print("Number of source",source_o3d)
# print("Number of target", target_o3d)
# print("Number of source down", source_down)
# print("Number of target down",target_down)
print("Number of source points      :", np.asarray(source_o3d.points))
print("Number of target points      :", np.asarray(target_o3d.points).shape[0])
print("Number of source down points :", np.asarray(source_down.points).shape[0])
print("Number of target down points :", np.asarray(target_down.points).shape[0])
o3d.visualization.draw_geometries([source_o3d], window_name="Source")
o3d.visualization.draw_geometries([target_o3d], window_name="Target")
o3d.visualization.draw_geometries([source_down], window_name="Source Down")
o3d.visualization.draw_geometries([target_down], window_name="Target Down")


# # import numpy as np
# # from scipy.spatial.transform import Rotation

# # roll, pitch, yaw = 30, 45, 60  # độ

# # # Extrinsic xyz với [roll, pitch, yaw]
# # R_ext = Rotation.from_euler('xyz', [roll, pitch, yaw], degrees=True).as_matrix()

# # # Intrinsic ZYX với [yaw, pitch, roll]
# # R_int = Rotation.from_euler('ZYX', [yaw, pitch, roll], degrees=True).as_matrix()
# # print("\nR ext\n", R_ext)
# # print("\nR int\n", R_int)
# # print("Sai số giữa hai ma trận:", np.max(np.abs(R_ext - R_int)))
# # # Kết quả: 0.0 (hoặc cỡ 1e-16 do làm tròn số)

# # print("Sai số giữa hai ma trận theo matlab:\n", R_int @ np.linalg.inv(R_ext) - np.eye(3))

# #!/usr/bin/env python3
# """
# Real Radar Scan Matching Evaluation
# ===================================

# Supports two input modes:

# A) Nested frame folders (recommended for your dataset):
#    scan_root/
#       000000/
#          cloud.pcd
#          data
#       ...

# B) Two explicit scan files:
#     python real_radar_scan_matching_eval.py \
#         --scan1 frame_0001.pcd \
#         --scan2 frame_0010.pcd \
#         --gt_poses ground_truth.txt

# Important:
# - For ICP, source = scan2, target = scan1.
# - Therefore GT alignment maps scan2 -> scan1.
# """

# import argparse
# import copy
# import glob
# import os
# import re
# import sys

# import matplotlib.pyplot as plt
# import numpy as np
# import open3d as o3d
# from scipy.spatial.transform import Rotation


# # ============================================================
# # 1. BASIC HELPERS
# # ============================================================

# def natural_sort_key(path):
#     name = os.path.basename(path)
#     return [int(text) if text.isdigit() else text.lower()
#             for text in re.split(r'(\d+)', name)]

# def is_unix_timestamp(x):
#     return x > 1e9

# def load_pcd(filepath):
#     if not os.path.exists(filepath):
#         print(f"[ERROR] File not found: {filepath}")
#         sys.exit(1)
#     pcd = o3d.io.read_point_cloud(filepath)
#     print(f"  Loaded {filepath}: {len(pcd.points)} points")
#     return pcd

# def parse_numeric_tokens(line):
#     tokens = re.split(r'[,\s]+', line.strip())
#     vals = []
#     for tok in tokens:
#         if tok == "":
#             continue
#         try:
#             vals.append(float(tok))
#         except ValueError:
#             continue
#     return vals


# # ============================================================
# # 2. DATASET LAYOUT
# # ============================================================

# def get_frame_entries(scan_root, scan_name="cloud.pcd", meta_name="data"):
#     if not os.path.isdir(scan_root):
#         print(f"[ERROR] Scan root not found: {scan_root}")
#         sys.exit(1)

#     folders = sorted(
#         [d for d in glob.glob(os.path.join(scan_root, "*")) if os.path.isdir(d)],
#         key=natural_sort_key
#     )

#     entries = []
#     for folder in folders:
#         scan_path = os.path.join(folder, scan_name)
#         meta_path = os.path.join(folder, meta_name)

#         if os.path.exists(scan_path):
#             entries.append({
#                 "folder": folder,
#                 "frame_name": os.path.basename(folder),
#                 "scan_path": scan_path,
#                 "meta_path": meta_path if os.path.exists(meta_path) else None
#             })

#     if len(entries) == 0:
#         print(f"[ERROR] No frame folders containing {scan_name} found in: {scan_root}")
#         sys.exit(1)

#     return entries


# def load_scan_from_entries(entries, idx):
#     if idx < 0 or idx >= len(entries):
#         print(f"[ERROR] Frame index out of range. Available: 0-{len(entries)-1}")
#         sys.exit(1)

#     entry = entries[idx]
#     return load_pcd(entry["scan_path"]), entry


# # ============================================================
# # 3. TIMESTAMP PARSING
# # ============================================================

# def read_text_file(filepath):
#     if filepath is None or not os.path.exists(filepath):
#         return ""
#     with open(filepath, "r", encoding="utf-8", errors="ignore") as f:
#         return f.read()

# def extract_timestamp_from_text(text):
#     if not text:
#         return None

#     m = re.search(r'(?:timestamp|stamp)\D+(\d{10})\D+(\d{1,9})', text, re.IGNORECASE)
#     if m:
#         return int(m.group(1)) + int(m.group(2)) * 1e-9

#     m = re.search(r'(?:timestamp|stamp)\D+(\d{10}\.\d+)', text, re.IGNORECASE)
#     if m:
#         return float(m.group(1))

#     m = re.search(r'(\d{10}\.\d+)', text)
#     if m:
#         return float(m.group(1))

#     m = re.search(r'(\d{10})\s+(\d{1,9})', text)
#     if m:
#         return int(m.group(1)) + int(m.group(2)) * 1e-9

#     nums = parse_numeric_tokens(text)
#     for v in nums:
#         if is_unix_timestamp(v):
#             return float(v)
#     return None

# def read_frame_timestamp(meta_path):
#     text = read_text_file(meta_path)
#     ts = extract_timestamp_from_text(text)
#     if ts is None:
#         raise ValueError(f"Could not parse frame timestamp from: {meta_path}")
#     return ts


# # ============================================================
# # 4. POSE PARSING
# # ============================================================

# def pose_xyz_quat_to_T(x, y, z, qx, qy, qz, qw):
#     T = np.eye(4)
#     T[:3, 3] = [x, y, z]
#     T[:3, :3] = Rotation.from_quat([qx, qy, qz, qw]).as_matrix()
#     return T

# def pose_xyz_rpy_to_T(x, y, z, roll, pitch, yaw, degrees=False):
#     T = np.eye(4)
#     T[:3, 3] = [x, y, z]
#     T[:3, :3] = Rotation.from_euler('xyz', [roll, pitch, yaw], degrees=degrees).as_matrix()
#     return T

# def pose_xy_yaw_to_T(x, y, yaw, z=0.0, degrees=False):
#     T = np.eye(4)
#     T[:3, 3] = [x, y, z]
#     T[:3, :3] = Rotation.from_euler('z', yaw, degrees=degrees).as_matrix()
#     return T

# def parse_gt_row(values):
#     n = len(values)
#     if n == 8:
#         return values[0], pose_xyz_quat_to_T(*values[1:8])
#     if n == 7:
#         if is_unix_timestamp(values[0]):
#             return values[0], pose_xyz_rpy_to_T(*values[1:7], degrees=False)
#         else:
#             return None, pose_xyz_quat_to_T(*values[:7])
#     if n == 6:
#         return None, pose_xyz_rpy_to_T(*values[:6], degrees=False)
#     if n == 5 and is_unix_timestamp(values[0]):
#         return values[0], pose_xy_yaw_to_T(values[1], values[2], values[4], z=values[3], degrees=False)
#     if n == 4:
#         if is_unix_timestamp(values[0]):
#             return values[0], pose_xy_yaw_to_T(values[1], values[2], values[3], z=0.0, degrees=False)
#         else:
#             return None, pose_xy_yaw_to_T(values[0], values[1], values[3], z=values[2], degrees=False)
#     if n == 3:
#         return None, pose_xy_yaw_to_T(values[0], values[1], values[2], z=0.0, degrees=False)
#     raise ValueError(f"Unsupported pose row with {n} numeric values: {values}")

# def load_ground_truth(filepath):
#     if not os.path.exists(filepath):
#         print(f"[ERROR] Ground truth file not found: {filepath}")
#         sys.exit(1)

#     rows = []
#     with open(filepath, "r", encoding="utf-8", errors="ignore") as f:
#         for line in f:
#             line = line.strip()
#             if line == "" or line.startswith("#"):
#                 continue
#             vals = parse_numeric_tokens(line)
#             if len(vals) == 0:
#                 continue
#             try:
#                 ts, T = parse_gt_row(vals)
#                 rows.append({"timestamp": ts, "T": T, "raw": vals})
#             except Exception:
#                 continue

#     if len(rows) == 0:
#         print(f"[ERROR] Could not parse any valid GT poses from: {filepath}")
#         sys.exit(1)
#     n_with_ts = sum(r["timestamp"] is not None for r in rows)
#     print(f"  Loaded {len(rows)} ground truth poses from {filepath} ({n_with_ts} rows have timestamps)")
#     return rows

# def nearest_gt_index(frame_ts, gt_rows):
#     gt_timestamps = np.array([
#         np.nan if r["timestamp"] is None else float(r["timestamp"])
#         for r in gt_rows
#     ])
#     valid = np.where(~np.isnan(gt_timestamps))[0]
#     if len(valid) == 0:
#         raise ValueError("GT file has no timestamps; cannot do timestamp-based matching.")
#     return int(valid[np.argmin(np.abs(gt_timestamps[valid] - frame_ts))])


# # ============================================================
# # 5. TRANSFORMS / ERRORS
# # ============================================================

# def transform_from_A_to_B(T_A_world, T_B_world):
#     return np.linalg.inv(T_A_world) @ T_B_world

# def print_transform(T, label=""):
#     t = T[:3, 3]
#     r = Rotation.from_matrix(T[:3, :3]).as_euler('xyz', degrees=True)
#     dist = np.linalg.norm(t)
#     print(f"  {label}")
#     print(f"    Translation: [{t[0]:.4f}, {t[1]:.4f}, {t[2]:.4f}] (dist: {dist:.4f} m)")
#     print(f"    Rotation:    [{r[0]:.2f}, {r[1]:.2f}, {r[2]:.2f}] deg (roll, pitch, yaw)")

# def pose_error(T_est, T_gt):
#     trans_error = np.linalg.norm(T_est[:3, 3] - T_gt[:3, 3])
#     R_err = T_est[:3, :3].T @ T_gt[:3, :3]
#     val = np.clip((np.trace(R_err) - 1.0) / 2.0, -1.0, 1.0)
#     rot_error_deg = np.degrees(np.arccos(val))
#     return trans_error, rot_error_deg


# # ============================================================
# # 6. PREPROCESSING & OVERLAP
# # ============================================================

# def preprocess(pcd, voxel_size=0.1):
#     pcd_down = pcd.voxel_down_sample(voxel_size)
#     if len(pcd_down.points) == 0:
#         return pcd_down
#     _, ind = pcd_down.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
#     pcd_clean = pcd_down.select_by_index(ind)
#     print(f"    {len(pcd.points)} -> {len(pcd_clean.points)} points")
#     return pcd_clean

# def check_overlap(target_scan, source_scan, T_source_to_target, overlap_radius=1.0):
#     src = copy.deepcopy(source_scan)
#     src.transform(T_source_to_target)
#     if len(target_scan.points) == 0 or len(src.points) == 0:
#         return 0.0
#     tree = o3d.geometry.KDTreeFlann(target_scan)
#     count = 0
#     for p in src.points:
#         k, _, _ = tree.search_radius_vector_3d(p, overlap_radius)
#         if k > 0: count += 1
#     return count / len(src.points)

# def overlap_label(overlap, threshold=0.30):
#     return "OVERLAPPING" if overlap >= threshold else "NON-OVERLAPPING / WEAK OVERLAP"


# # ============================================================
# # 7. FPFH UTILITIES (EXACT SAC-IA MAPPING)
# # ============================================================

# def preprocess_for_fpfh(pcd, voxel_size=0.1, normal_radius=None, feature_radius=None,
#                         normal_max_nn=30, feature_max_nn=100):
#     """
#     Prepare a cloud for FPFH. SAC-IA methodology step 1.
#     """
#     pcd_feat = copy.deepcopy(pcd)
#     if len(pcd_feat.points) == 0:
#         return pcd_feat, None

#     if normal_radius is None: normal_radius = voxel_size * 2.0
#     if feature_radius is None: feature_radius = voxel_size * 5.0

#     pcd_feat.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=normal_radius, max_nn=normal_max_nn))
#     try:
#         pcd_feat.orient_normals_consistent_tangent_plane(10)
#     except Exception:
#         pass

#     fpfh = o3d.pipelines.registration.compute_fpfh_feature(
#         pcd_feat,
#         o3d.geometry.KDTreeSearchParamHybrid(radius=feature_radius, max_nn=feature_max_nn)
#     )
#     return pcd_feat, fpfh


# def plot_fpfh_bins_exact(pcd, voxel_size=0.1, point_idx=0, prefix="scan",
#                          normal_radius=None, feature_radius=None,
#                          normal_max_nn=30, feature_max_nn=100,
#                          save=True, show=True):
#     """
#     Plot the EXACT FPFH descriptor of one selected point mapping correctly to Rusu (2009).
#     """
#     pcd_feat, fpfh = preprocess_for_fpfh(
#         pcd, voxel_size=voxel_size, normal_radius=normal_radius,
#         feature_radius=feature_radius, normal_max_nn=normal_max_nn, feature_max_nn=feature_max_nn
#     )

#     if fpfh is None:
#         print(f"[WARNING] No FPFH descriptors available for {prefix}.")
#         return

#     fpfh_array = np.asarray(fpfh.data)
#     if fpfh_array.ndim != 2 or fpfh_array.shape[0] != 33:
#         print(f"[WARNING] Unexpected FPFH shape for {prefix}: {fpfh_array.shape}")
#         return

#     n_points = fpfh_array.shape[1]
#     if n_points == 0:
#         return
#     if point_idx < 0 or point_idx >= n_points:
#         print(f"[ERROR] point_idx={point_idx} out of range for {prefix}.")
#         return

#     desc = fpfh_array[:, point_idx]
#     desc_norm = np.linalg.norm(desc)

#     alpha_bins = desc[0:11]
#     phi_bins   = desc[11:22]
#     theta_bins = desc[22:33]

#     print(f"  {prefix} FPFH shape: {fpfh_array.shape}")
#     print(f"  Plotting EXACT FPFH for {prefix}, point index {point_idx}")
    
#     fig, axes = plt.subplots(1, 3, figsize=(15, 4))

#     # CORRECTED LABELS MATCHING THE FPFH PAPER
#     axes[0].bar(range(11), alpha_bins, color='steelblue', edgecolor='black', linewidth=0.5)
#     axes[0].set_title(r'$\alpha$ (normal vs normal)')
#     axes[0].set_xlabel('Bin')
#     axes[0].set_ylabel('Value')
#     axes[0].grid(True, alpha=0.3)

#     axes[1].bar(range(11), phi_bins, color='coral', edgecolor='black', linewidth=0.5)
#     axes[1].set_title(r'$\phi$ (normal vs connection vector)')
#     axes[1].set_xlabel('Bin')
#     axes[1].grid(True, alpha=0.3)

#     axes[2].bar(range(11), theta_bins, color='seagreen', edgecolor='black', linewidth=0.5)
#     axes[2].set_title(r'$\theta$ (rotation around axis)')
#     axes[2].set_xlabel('Bin')
#     axes[2].grid(True, alpha=0.3)

#     fig.suptitle(f'{prefix} | point {point_idx} | norm={desc_norm:.3f}', fontsize=13, fontweight='bold')
#     plt.subplots_adjust(top=0.82, wspace=0.28)

#     if save:
#         out1 = f"{prefix}_point_{point_idx}_split.png"
#         fig.savefig(out1, dpi=150, bbox_inches='tight')
#         print(f"  Saved: {out1}")

#     fig2, ax = plt.subplots(figsize=(16, 5))
#     colors = ['steelblue'] * 11 + ['coral'] * 11 + ['seagreen'] * 11
#     ax.bar(range(33), desc, color=colors, edgecolor='black', linewidth=0.5)
#     ax.axvline(x=10.5, color='black', linestyle='--', linewidth=1.0)
#     ax.axvline(x=21.5, color='black', linestyle='--', linewidth=1.0)
#     ax.set_xlabel('Bin Index')
#     ax.set_ylabel('Value')
#     ax.set_title(f'Full FPFH Descriptor for {prefix}, point {point_idx}')
#     ax.grid(True, alpha=0.3)

#     ymax = max(np.max(desc), 1.0)
#     ax.text(5,  ymax * 0.92, r'$\alpha$', ha='center', fontsize=16)
#     ax.text(16, ymax * 0.92, r'$\phi$',   ha='center', fontsize=16)
#     ax.text(27, ymax * 0.92, r'$\theta$', ha='center', fontsize=16)
#     plt.subplots_adjust(top=0.88, bottom=0.15)

#     if save:
#         out2 = f"{prefix}_point_{point_idx}_full.png"
#         fig2.savefig(out2, dpi=150, bbox_inches='tight')
#         print(f"  Saved: {out2}")
#     if show: plt.show()
#     else:
#         plt.close(fig)
#         plt.close(fig2)


# def run_fpfh_ransac_icp(source, target, voxel_size=0.1, max_dist=2.0,
#                         normal_radius=None, feature_radius=None,
#                         normal_max_nn=30, feature_max_nn=100,
#                         ransac_distance_threshold=None,
#                         ransac_n=4,
#                         ransac_max_iteration=100000,
#                         ransac_confidence=500):
#     """
#     SAC-IA Pipeline: FPFH computing -> Global RANSAC alignment -> Refinement
#     """
#     if normal_radius is None: normal_radius = voxel_size * 2.0
#     if feature_radius is None: feature_radius = voxel_size * 5.0
#     if ransac_distance_threshold is None: ransac_distance_threshold = voxel_size * 3.0

#     source_feat_cloud, source_fpfh = preprocess_for_fpfh(
#         source, voxel_size=voxel_size, normal_radius=normal_radius, feature_radius=feature_radius,
#         normal_max_nn=normal_max_nn, feature_max_nn=feature_max_nn
#     )
#     target_feat_cloud, target_fpfh = preprocess_for_fpfh(
#         target, voxel_size=voxel_size, normal_radius=normal_radius, feature_radius=feature_radius,
#         normal_max_nn=normal_max_nn, feature_max_nn=feature_max_nn
#     )

#     if source_fpfh is None or target_fpfh is None:
#         empty_result = o3d.pipelines.registration.RegistrationResult()
#         empty_result.transformation = np.eye(4)
#         return empty_result, empty_result

#     # Sample Consensus Initial Alignment (SAC-IA)
#     result_ransac = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
#         source_feat_cloud, target_feat_cloud, source_fpfh, target_fpfh,
#         mutual_filter=True,
#         max_correspondence_distance=ransac_distance_threshold,
#         estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
#         ransac_n=ransac_n,
#         checkers=[
#             o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
#             o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(ransac_distance_threshold)
#         ],
#         criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(ransac_max_iteration, ransac_confidence)
#     )

#     src_icp = copy.deepcopy(source_feat_cloud)
#     tgt_icp = copy.deepcopy(target_feat_cloud)

#     result_icp = o3d.pipelines.registration.registration_icp(
#         src_icp, tgt_icp, max_dist, result_ransac.transformation,
#         o3d.pipelines.registration.TransformationEstimationPointToPlane(),
#         o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=500, relative_fitness=1e-6, relative_rmse=1e-6)
#     )
#     return result_icp, result_ransac


# # ============================================================
# # 8. ICP + SCAN MATCHING (Point-to-Point / Point-to-Plane)
# # ============================================================

# def ensure_normals(pcd):
#     if not pcd.has_normals():
#         pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=1.0, max_nn=30))

# def run_icp_point_to_point(source, target, init_T=np.eye(4), max_dist=2.0):
#     return o3d.pipelines.registration.registration_icp(
#         source, target, max_dist, init_T,
#         o3d.pipelines.registration.TransformationEstimationPointToPoint(),
#         o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=500, relative_fitness=1e-6, relative_rmse=1e-6)
#     )

# def run_icp_point_to_plane(source, target, init_T=np.eye(4), max_dist=2.0):
#     ensure_normals(source)
#     ensure_normals(target)
#     return o3d.pipelines.registration.registration_icp(
#         source, target, max_dist, init_T,
#         o3d.pipelines.registration.TransformationEstimationPointToPlane(),
#         o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=500, relative_fitness=1e-6, relative_rmse=1e-6)
#     )

# def run_generalized_icp(source, target, init_T=np.eye(4), max_dist=2.0):
#     ensure_normals(source)
#     ensure_normals(target)
#     return o3d.pipelines.registration.registration_generalized_icp(
#         source, target, max_dist, init_T,
#         o3d.pipelines.registration.TransformationEstimationForGeneralizedICP(),
#         o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=500, relative_fitness=1e-6, relative_rmse=1e-6)
#     )


# # ============================================================
# # 9. VISUALIZATION
# # ============================================================

# def visualize_scans_3d(scan1, scan2, T_est, T_gt):
#     s1 = copy.deepcopy(scan1).paint_uniform_color([1, 0, 0])
#     s2_est = copy.deepcopy(scan2).transform(T_est).paint_uniform_color([0, 1, 0])
#     s2_gt = copy.deepcopy(scan2).transform(T_gt).paint_uniform_color([0, 0, 1])

#     print("\n  3D Visualization colors:")
#     print("    Red   = Reference scan (scan1)")
#     print("    Green = Scan2 aligned by ESTIMATED pose")
#     print("    Blue  = Scan2 aligned by GROUND TRUTH pose")

#     o3d.visualization.draw_geometries([s1, s2_est, s2_gt], window_name="Scan Matching", width=1200, height=800)

# def visualize_scans_2d(scan1, scan2, T_est, T_gt, save=False):
#     pts1 = np.asarray(scan1.points)
#     pts2 = np.asarray(scan2.points)
#     pts2_est = np.asarray(copy.deepcopy(scan2).transform(T_est).points)
#     pts2_gt = np.asarray(copy.deepcopy(scan2).transform(T_gt).points)

#     fig1, ax1 = plt.subplots(figsize=(10, 8))
#     ax1.scatter(pts1[:, 0], pts1[:, 1], s=1, c='tab:blue', label='Reference scan')
#     ax1.scatter(pts2[:, 0], pts2[:, 1], s=1, c='tab:orange', label='Current scan')
#     ax1.set_title('Before alignment'), ax1.set_aspect('equal'), ax1.grid(True, alpha=0.3), ax1.legend(loc='upper left', markerscale=5)

#     fig2, ax2 = plt.subplots(figsize=(10, 8))
#     ax2.scatter(pts1[:, 0], pts1[:, 1], s=1, c='tab:blue', label='Reference scan')
#     ax2.scatter(pts2_est[:, 0], pts2_est[:, 1], s=1, c='tab:orange', label='Scan2 aligned by estimated pose')
#     ax2.set_title('Estimated alignment'), ax2.set_aspect('equal'), ax2.grid(True, alpha=0.3), ax2.legend(loc='upper left', markerscale=5)

#     fig3, ax3 = plt.subplots(figsize=(10, 8))
#     ax3.scatter(pts1[:, 0], pts1[:, 1], s=1, c='tab:blue', label='Reference scan')
#     ax3.scatter(pts2_gt[:, 0], pts2_gt[:, 1], s=1, c='tab:green', label='Scan2 aligned by GT pose')
#     ax3.set_title('Ground-truth alignment'), ax3.set_aspect('equal'), ax3.grid(True, alpha=0.3), ax3.legend(loc='upper left', markerscale=5)

#     if save:
#         fig1.savefig("plot_before_alignment.png", dpi=150, bbox_inches='tight')
#         fig2.savefig("plot_icp_aligned.png", dpi=150, bbox_inches='tight')
#         fig3.savefig("plot_gt_aligned.png", dpi=150, bbox_inches='tight')
#     plt.show()


# # ============================================================
# # 10. ARGUMENT VALIDATION & MAIN
# # ============================================================

# def validate_inputs(args):
#     using_root = args.scan_root is not None
#     using_pair = (args.scan1 is not None) or (args.scan2 is not None)
#     if using_root and using_pair:
#         print("[ERROR] Use either --scan_root OR (--scan1 and --scan2), not both."); sys.exit(1)
#     if not using_root and not using_pair:
#         print("[ERROR] Provide either --scan_root OR (--scan1 and --scan2)."); sys.exit(1)
#     if using_pair and (args.scan1 is None or args.scan2 is None):
#         print("[ERROR] If using explicit file mode, both --scan1 and --scan2 are required."); sys.exit(1)
#     if (args.pose1 is None) ^ (args.pose2 is None):
#         print("[ERROR] Provide both --pose1 and --pose2 together."); sys.exit(1)
#     if (args.stamp1 is None) ^ (args.stamp2 is None):
#         print("[ERROR] Provide both --stamp1 and --stamp2 together."); sys.exit(1)
#     if (args.data1 is None) ^ (args.data2 is None):
#         print("[ERROR] Provide both --data1 and --data2 together."); sys.exit(1)

# def main():
#     parser = argparse.ArgumentParser(description="Evaluate scan matching between two real radar point clouds")
#     parser.add_argument("--scan_root", "--scan_dir", dest="scan_root", default=None)
#     parser.add_argument("--scan_name", default="cloud.pcd")
#     parser.add_argument("--meta_name", default="data")

#     parser.add_argument("--scan1", default=None)
#     parser.add_argument("--scan2", default=None)
#     parser.add_argument("--data1", default=None)
#     parser.add_argument("--data2", default=None)
#     parser.add_argument("--stamp1", type=float, default=None)
#     parser.add_argument("--stamp2", type=float, default=None)

#     parser.add_argument("--idx1", type=int, default=0)
#     parser.add_argument("--idx2", type=int, default=1)

#     parser.add_argument("--gt_poses", default=None)
#     parser.add_argument("--pose1", default=None)
#     parser.add_argument("--pose2", default=None)
#     parser.add_argument("--use_gt_init", action="store_true")

#     parser.add_argument("--voxel_size", type=float, default=0.1)
#     parser.add_argument("--icp_max_dist", type=float, default=2.0)
#     parser.add_argument("--overlap_radius", type=float, default=1.0)
#     parser.add_argument("--overlap_threshold", type=float, default=0.30)
#     parser.add_argument("--method", choices=["point2point", "point2plane", "gicp", "fpfh", "all"], default="all")
    
#     parser.add_argument("--plot_fpfh", action="store_true")
#     parser.add_argument("--fpfh_point_idx1", type=int, default=0)
#     parser.add_argument("--fpfh_point_idx2", type=int, default=100)

#     parser.add_argument("--no_vis", action="store_true")
#     parser.add_argument("--save_plots", action="store_true")
#     parser.add_argument("--save_pose", action="store_true")
#     parser.add_argument("--save_pose_name", default="estimated_relative_pose.txt")

#     args = parser.parse_args()
#     validate_inputs(args)

#     print("=" * 78)
#     print("  REAL RADAR SCAN MATCHING EVALUATION (TIMESTAMP-AWARE)")
#     print("=" * 78)

#     # Loads scans
#     print("\n[1] Loading scans...")
#     frame_ts1 = frame_ts2 = gt_match_mode = None
#     if args.scan_root is not None:
#         entries = get_frame_entries(args.scan_root, args.scan_name, args.meta_name)
#         scan1, entry1 = load_scan_from_entries(entries, args.idx1)
#         scan2, entry2 = load_scan_from_entries(entries, args.idx2)
#         scan1_path, scan2_path = entry1["scan_path"], entry2["scan_path"]
#         if entry1["meta_path"]: frame_ts1 = read_frame_timestamp(entry1["meta_path"])
#         if entry2["meta_path"]: frame_ts2 = read_frame_timestamp(entry2["meta_path"])
#     else:
#         scan1, scan2 = load_pcd(args.scan1), load_pcd(args.scan2)
#         scan1_path, scan2_path = args.scan1, args.scan2
#         if args.stamp1 and args.stamp2: frame_ts1, frame_ts2 = args.stamp1, args.stamp2
#         elif args.data1 and args.data2: frame_ts1, frame_ts2 = read_frame_timestamp(args.data1), read_frame_timestamp(args.data2)

#     if len(scan1.points) == 0 or len(scan2.points) == 0:
#         print("[ERROR] One or both scans are empty."); sys.exit(1)

#     print("\n[2] Resolving poses...")
#     has_gt = False
#     T1_world = T2_world = T_gt_source_to_target = matched_gt_idx1 = matched_gt_idx2 = None

#     if args.pose1 and args.pose2:
#         vals1, vals2 = [float(v) for v in args.pose1.split(',')], [float(v) for v in args.pose2.split(',')]
#         vals1[3:], vals2[3:] = np.radians(vals1[3:]).tolist(), np.radians(vals2[3:]).tolist()
#         T1_world, T2_world = pose_xyz_rpy_to_T(*vals1, degrees=False), pose_xyz_rpy_to_T(*vals2, degrees=False)
#         has_gt, gt_match_mode = True, "manual poses"
#     elif args.gt_poses:
#         gt_rows = load_ground_truth(args.gt_poses)
#         if frame_ts1 is not None and frame_ts2 is not None:
#             try:
#                 matched_gt_idx1, matched_gt_idx2 = nearest_gt_index(frame_ts1, gt_rows), nearest_gt_index(frame_ts2, gt_rows)
#                 T1_world, T2_world = gt_rows[matched_gt_idx1]["T"], gt_rows[matched_gt_idx2]["T"]
#                 has_gt, gt_match_mode = True, "timestamp"
#             except Exception:
#                 pass
#         if not has_gt:
#             matched_gt_idx1, matched_gt_idx2 = args.idx1, args.idx2
#             T1_world, T2_world = gt_rows[matched_gt_idx1]["T"], gt_rows[matched_gt_idx2]["T"]
#             has_gt, gt_match_mode = True, "index"

#     if has_gt:
#         T_gt_source_to_target = transform_from_A_to_B(T1_world, T2_world)

#     print(f"\n[3] Preprocessing (voxel_size={args.voxel_size:.3f} m)...")
#     scan1_proc, scan2_proc = preprocess(scan1, args.voxel_size), preprocess(scan2, args.voxel_size)

#     if args.plot_fpfh:
#         print("\n[3.5] Plotting EXACT FPFH descriptor bins...")
#         plot_fpfh_bins_exact(scan1_proc, voxel_size=args.voxel_size, point_idx=args.fpfh_point_idx1, prefix=f"scan1_poseidx_{args.idx1}", save=args.save_plots)
#         plot_fpfh_bins_exact(scan2_proc, voxel_size=args.voxel_size, point_idx=args.fpfh_point_idx2, prefix=f"scan2_poseidx_{args.idx2}", save=args.save_plots)

#     print(f"\n[4] Checking overlap...")
#     if has_gt:
#         overlap_gt = check_overlap(scan1_proc, scan2_proc, T_gt_source_to_target, overlap_radius=args.overlap_radius)
#         T_init = T_gt_source_to_target if args.use_gt_init else np.eye(4)
#         overlap_init = check_overlap(scan1_proc, scan2_proc, T_init, overlap_radius=args.overlap_radius)
#     else:
#         overlap_gt = None
#         T_init = np.eye(4)
#         overlap_init = check_overlap(scan1_proc, scan2_proc, T_init, overlap_radius=args.overlap_radius)

#     print(f"\n[5] Running scan matching...")
#     methods = {
#         "point2point": ("Point-to-Point ICP", run_icp_point_to_point),
#         "point2plane": ("Point-to-Plane ICP", run_icp_point_to_plane),
#         "gicp":        ("Generalized ICP",    run_generalized_icp),
#         "fpfh":        ("FPFH + RANSAC + ICP", run_fpfh_ransac_icp),
#     }
#     to_run = methods if args.method == "all" else {args.method: methods[args.method]}

#     results = {}
#     best_key, best_score = None, -1e18

#     for key, (name, func) in to_run.items():
#         print(f"\n  --- {name} ---")
#         src, tgt = copy.deepcopy(scan2_proc), copy.deepcopy(scan1_proc)

#         if key == "fpfh":
#             result, result_ransac = func(src, tgt, voxel_size=args.voxel_size, max_dist=args.icp_max_dist)
#         else:
#             result = func(src, tgt, init_T=T_init, max_dist=args.icp_max_dist)
#             result_ransac = None

#         T_est = result.transformation
#         eval_reg = o3d.pipelines.registration.evaluate_registration(src, tgt, args.icp_max_dist, T_est)
#         fitness, rmse = eval_reg.fitness, eval_reg.inlier_rmse
#         overlap_est = check_overlap(scan1_proc, scan2_proc, T_est, overlap_radius=args.overlap_radius)
#         valid = (fitness > 1e-6) and (overlap_est >= args.overlap_threshold)

#         entry = {"name": name, "transform": T_est, "fitness": fitness, "rmse": rmse, "overlap_est": overlap_est, "valid": valid}

#         if has_gt:
#             t_err, r_err = pose_error(T_est, T_gt_source_to_target)
#             entry["trans_error"], entry["rot_error"] = t_err, r_err
#             score = -(t_err + 0.02 * r_err) if valid else -1e18
#         else:
#             score = (2.0 * fitness + overlap_est - 0.1 * rmse) if valid else -1e18

#         entry["score"] = score
#         results[key] = entry
#         if score > best_score:
#             best_score, best_key = score, key

#     print("\n" + "=" * 78)
#     print("RESULTS SUMMARY")
#     print("=" * 78)

#     if has_gt:
#         print(f"\n  {'Method':<25} {'Valid':<8} {'Trans Err(m)':<14} {'Rot Err(deg)':<14} {'Fitness':<10} {'RMSE':<10} {'Overlap':<10}")
#         print("  " + "-" * 108)
#         for key, r in results.items():
#             tag = " * best" if key == best_key else ""
#             print(f"  {r['name']:<25} {str(r['valid']):<8} {r.get('trans_error', float('nan')):<14.4f} "
#                   f"{r.get('rot_error', float('nan')):<14.4f} {r['fitness']:<10.4f} {r['rmse']:<10.4f} {r['overlap_est']:<10.4f}{tag}")
#     else:
#         print(f"\n  {'Method':<25} {'Valid':<8} {'Fitness':<10} {'RMSE':<10} {'Overlap':<10}")
#         print("  " + "-" * 78)
#         for key, r in results.items():
#             tag = " * best" if key == best_key else ""
#             print(f"  {r['name']:<25} {str(r['valid']):<8} {r['fitness']:<10.4f} {r['rmse']:<10.4f} {r['overlap_est']:<10.4f}{tag}")

#     if best_key is None or results[best_key]["score"] <= -1e17:
#         print("\n  [ERROR] No valid registration result passed.")
#     else:
#         best_T = results[best_key]["transform"]
#         if args.save_pose:
#             np.savetxt(args.save_pose_name, best_T, fmt="%.6f")

#         if not args.no_vis:
#             visualize_scans_2d(scan1_proc, scan2_proc, best_T, T_gt_source_to_target if has_gt else best_T, save=args.save_plots)
#             visualize_scans_3d(scan1_proc, scan2_proc, best_T, T_gt_source_to_target if has_gt else best_T)

# if __name__ == "__main__":
#     main()
# import open3d as o3d
# import numpy as np
# voxel_sizes = o3d.utility.DoubleVector([0.1, 0.05, 0.025])
# v =  np.asarray(voxel_sizes)
# #    None nghĩa là chèn thêm một trục mới kích thước 1.
# column = v[1:, None]
# print ("voxel_sizes transpose \n", column)
# print ("voxel_sizes shape", column.shape)