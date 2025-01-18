
// This file is licensed under the Apache License, Version 2.0,
// with additional restrictions under the Commons Clause.
// See the LICENSE file for more details.

#include "yaml-cpp/yaml.h"
#include <eigen3/Eigen/Dense>
#include <chrono>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <math.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <string>
#include <vector>
#include <queue>
#include "kalman.hpp"
#include "parameters.h"

// Global variables
double TTC, contrast_threshold, first_ts_flag;
int n_target, n_state, image_count, image_id, start_event_ts;
double t1;
double t_next_publish = 0;
int EventId = 0;
double ts_kf_last = 0;
double gyro_ts = 0;
double gyro_ts_last = 0;
double next_save_traj_ts = 0;
double dt_save_traj = 0.0001;
double alpha_dist = 1.4;
bool first_gyro_flag = true;
bool use_gyro_flag;

std::queue<std::pair<int, std::pair<int, int>>> same_ts_e_buffer;
std::vector<double> dist_threshold;
std::vector<double> ts_last;
cv::Mat log_intensity_state, ts_array;
Eigen::MatrixXd F, C, R, P, Q, A, P_x, x0, x_hat;
std::ofstream output_txt_;
std::string input_event_path, output_image_path;
cv::Point selectedPoint(-1, -1);
std::vector<cv::Point> selectedPoints;
std::vector<cv::Point> selectedPointsSpeed;
cv::VideoWriter writer_ref, writer;

void onMouse(int event, int x, int y, int flags, void *userdata)
{
  if (event == cv::EVENT_LBUTTONDOWN)
  {
    selectedPoint = cv::Point(x, y);
  }
}
void high_pass(double ts, int x, int y, int p, int &alpha);
void high_pass_global(double ts, int &alpha);
void init(Parameters &params);
void read_events(std::stringstream &ss, const int data_format, double &ts,
                 int &c, int &r, int &p);
void display(const DisplayParams &dispParams, Parameters &params,
             const double &ts, std::vector<cv::Mat> &output_video,
             std::vector<cv::Mat> &output_video_ref,
             std::vector<double> &image_ref_ts, int p);
void display_for_select();
void load_gryo(std::ifstream &my_gyro, KalmanFilter &kf);

int main(int argc, char *argv[])
{
  std::string data_name = argv[2];
  Parameters params;
  DisplayParams dispParams;
  try
  {
    std::string file_config_path = "../configs/" + data_name + ".yaml";
    params = loadParametersFromYAML(file_config_path);
    dispParams = loadDispParametersFromYAML(file_config_path);
  }
  catch (const std::exception &ex)
  {
    std::cerr << "Error: " << ex.what() << std::endl;
    return 1;
  }
  log_intensity_state = cv::Mat::zeros(params.height, params.width, CV_64FC1);
  ts_array = cv::Mat::zeros(params.height, params.width, CV_64FC1);
  use_gyro_flag = params.use_gyro_flag;

  contrast_threshold = params.contrast_threshold;
  n_target = params.n_target;
  n_state = params.n_state;
  input_event_path = params.input_folder_path + params.input_data_name;
  dist_threshold.resize(n_target);
  std::fill(dist_threshold.begin(), dist_threshold.end(),
            params.dist_threshold);
  std::ofstream track_output_txt_, TTC_txt_;
  double distance;
  int id;
  std::string input_file_name = input_event_path + ".csv";

  // Read image timestamp
  std::vector<double> image_ref_ts;
  if (params.ref_image_ts_flag)
  {
    std::ifstream ImageTimefile(params.input_folder_path + "image_ts.txt");
    if (ImageTimefile)
    {
      std::string line;
      while (std::getline(ImageTimefile, line))
      {
        std::stringstream ss(line);
        std::string value;
        if (std::getline(ss, value, ','))
        {
          double timestamp = std::stod(value);
          image_ref_ts.push_back(timestamp);
        }
      }
      ImageTimefile.close();
    }
    else
    {
      std::cout << "Failed to open the image timestamp file." << std::endl;
      return 1;
    }
  }

  // Save video with tracks
  cv::namedWindow("Video");
  cv::setMouseCallback("Video", onMouse);
  std::vector<cv::Mat> output_video;
  std::vector<cv::Mat> output_video_ref;
  std::string output_ref_video_path = input_event_path + "_video_ref.avi";
  std::string output_video_path = input_event_path + "_video.avi";
  if (params.save_video_flag)
  {
    writer.open(output_video_path,
                cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 40.0,
                cv::Size(params.width, params.height));

    if (params.ref_image_ts_flag)
    {
      writer_ref.open(output_ref_video_path,
                      cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 40.0,
                      cv::Size(params.width - 1, params.height - 1));
    }
  }

  // Display reference image
  if (params.ref_image_ts_flag)
  {
    cv::namedWindow("Video_ref", cv::WINDOW_NORMAL);
  }

  if (params.save_image_flag)
  {
    std::filesystem::path path(input_event_path + "/output_img");
    std::filesystem::create_directories(path);
    output_image_path = path.string();
  }

  if (params.save_track_flag)
  {
    std::string output_info_path = input_event_path + "_traj.csv";
    track_output_txt_.open(output_info_path);
  }

  if (params.compute_TTC_flag)
  {
    std::string output_TTC_path = input_event_path + "_ttc.txt";
    TTC_txt_.open(output_TTC_path);
  }

  // Load gyro
  std::string input_gyro_name = params.input_folder_path + "gyro.csv";
  std::ifstream my_gyro(input_gyro_name);
  if (use_gyro_flag)
  {
    if (!my_gyro.is_open())
    {
      std::cout << input_gyro_name << std::endl;
      throw std::runtime_error("Could not open file");
    }

    std::string gyro_line;
    std::getline(my_gyro, gyro_line); // skip header line
  }

  // Load raw events
  std::ifstream myFileFilter(input_file_name);
  if (!myFileFilter.is_open())
  {
    std::cout << input_file_name << std::endl;
    throw std::runtime_error("Could not open file");
  }

  std::string line;
  int c, r, p;
  double ts;
  Eigen::Vector3d e(3); // Event measurement

  // Skip the header
  std::getline(myFileFilter, line);

  if (params.event_num_start > 0)
  {
    // start from certain event data
    while (std::getline(myFileFilter, line))
    {
      EventId++;
      if (EventId >= params.event_num_start)
      {
        std::stringstream ss(line);
        read_events(ss, params.data_format, ts, c, r, p);
        break;
      }
    }
  }
  else
  {
    while (std::getline(myFileFilter, line))
    {
      std::stringstream ss(line);
      read_events(ss, params.data_format, ts, c, r, p);
      if (ts > params.process_ts_start)
      {
        break;
      }
    }
  }

  // Filter events
  if (params.select_target_flag == 1)
  {
    while (std::getline(myFileFilter, line))
    {
      EventId++;
      std::stringstream ss(line);
      read_events(ss, params.data_format, ts, c, r, p);

      e << c, r, ts;
      high_pass(ts, c, r, p, params.alpha);

      if (params.publish_framerate > 0 && ts > t_next_publish)
      {
        high_pass_global(ts, params.alpha);
        t_next_publish = ts + (1.0 / params.publish_framerate);
        image_count += 1;
      }
      if (image_count >= 4)
      {
        std::cout << ts << std::endl;
        display_for_select();
        std::cout << "Number of target: " << n_target << "\n";
        image_count += 1;
        break;
      }
    }
  }

  // Construct the filter
  init(params);
  std::cout << "Selected points are: \n";
  for (int j = 0; j < selectedPoints.size(); j++)
  {
    const cv::Point &point = selectedPoints[j];
    std::cout << "(" << point.x << ", " << point.y << ")\n";
    x0(j, 0) = point.x;
    x0(j, 1) = point.y;
  }
  KalmanFilter kf(params.dt, F, C, Q, R, P, A, x0, n_state, n_target,
                  params.ring_buffer_len);
  kf.init(0, input_event_path); // Starts from time 0

  std::cout << "Finish filter construction" << std::endl;
  x_hat = kf.state();
  P_x = kf.P_x();

  // Display video after selecting points
  display(dispParams, params, ts, output_video, output_video_ref,
          image_ref_ts, p);

  while (std::getline(myFileFilter, line))
  {
    EventId++;
    std::stringstream ss(line);
    read_events(ss, params.data_format, ts, c, r, p);

    if (params.event_num_end > 0)
    {
      if (EventId > params.event_num_end)
      {
        std::cout << "End Event Id: " << params.event_num_end << std::endl;
        break;
      }
    }
    else if (params.process_ts_end > 0)
    {
      if (ts > params.process_ts_end)
      {
        std::cout << "End time: " << params.process_ts_end << std::endl;
        break;
      }
    }
    e << c, r, ts;
    high_pass(ts, c, r, p, params.alpha);

    // Event data association - find the closet target id
    double dist_min = 1e6; // Any large number
    for (int i = 0; i < n_target; i++)
    {
      distance = sqrt(pow((c - x_hat(i, 0)), 2) + pow((r - x_hat(i, 1)), 2));
      if (distance < dist_min)
      {
        dist_min = distance;
        id = i;
      }
    }

    distance = sqrt(pow((c - x_hat(id, 0)), 2) + pow((r - x_hat(id, 1)), 2));
    if ((P_x(id * n_state, 0) < 3) && (P_x(id * n_state + 1, 1) < 3))
    {
      double gamma = std::exp(-alpha_dist * (ts - ts_last[id]));
      if (x_hat(id, 4) > x_hat(id, 5))
      {
        dist_threshold[id] =
            gamma * dist_threshold[id] + (1 - gamma) * 2.5 * x_hat(id, 4);
      }
      else
      {
        dist_threshold[id] =
            gamma * dist_threshold[id] + (1 - gamma) * 2.5 * x_hat(id, 5);
      }

      if (dist_threshold[id] < params.dist_threshold)
      {
        dist_threshold[id] = params.dist_threshold;
      }
    }

    if (distance < dist_threshold[id])
    {
      if ((ts - ts_kf_last) == 0)
      {
        std::pair<int, std::pair<int, int>> new_ts_buffer;
        new_ts_buffer.first = id;
        new_ts_buffer.second = std::make_pair(c, r);
        same_ts_e_buffer.push(new_ts_buffer);
        continue;
      }

      if (same_ts_e_buffer.empty())
      {
        kf.update(e, id, p);
        x_hat = kf.state();

        if (params.compute_TTC_flag)
        {
          double relativePositionMagnitude, relativeVelocityMagnitude;
          Eigen::Vector2d relativePosition(2), relativeVelocity(2);
          relativePosition << x_hat(0, 0) - x_hat(1, 0),
              x_hat(0, 1) - x_hat(1, 1);
          relativeVelocity << x_hat(0, 2) - x_hat(1, 2),
              x_hat(0, 3) - x_hat(1, 3);
          relativePositionMagnitude = relativePosition.norm();
          TTC = (relativeVelocity.dot(relativePosition)) /
                (relativePositionMagnitude * relativePositionMagnitude);
          TTC_txt_ << std::fixed << std::setprecision(7) << ts << " " << TTC << std::endl;
        }

        // Save tracks
        if (params.save_track_flag)
        {
          // Output low rate trajectory for plotting
          if (ts > next_save_traj_ts)
          {
            track_output_txt_ << std::fixed << std::setprecision(7) << ts;
            for (int i = 0; i < x_hat.rows(); ++i)
            {
              for (int j = 0; j < x_hat.cols(); ++j)
              {
                track_output_txt_ << "," << x_hat(i, j);
              }
            }
            track_output_txt_ << std::endl;
            next_save_traj_ts = next_save_traj_ts + dt_save_traj;
          }
        }
      }
      else
      {
        double ts_each = (ts - ts_kf_last) / same_ts_e_buffer.size();
        ts = ts_kf_last;
        while (!same_ts_e_buffer.empty())
        {
          ts += ts_each;

          if (use_gyro_flag)
          {
            if (ts > gyro_ts)
            {
              load_gryo(my_gyro, kf);
            }
          }

          e << same_ts_e_buffer.front().second.first,
              same_ts_e_buffer.front().second.second, ts;

          kf.update(e, same_ts_e_buffer.front().first, p);

          same_ts_e_buffer.pop();
          x_hat = kf.state();
          if (params.compute_TTC_flag)
          {
            double relativePositionMagnitude, relativeVelocityMagnitude;
            Eigen::Vector2d relativePosition(2), relativeVelocity(2);
            relativePosition << x_hat(0, 0) - x_hat(1, 0),
                x_hat(0, 1) - x_hat(1, 1);
            relativeVelocity << x_hat(0, 2) - x_hat(1, 2),
                x_hat(0, 3) - x_hat(1, 3);
            relativePositionMagnitude = relativePosition.norm();
            TTC = (relativeVelocity.dot(relativePosition)) /
                  (relativePositionMagnitude * relativePositionMagnitude);
            TTC_txt_ << std::fixed << std::setprecision(7) << ts << " " << TTC << std::endl;
          }

          // Save tracks
          if (params.save_track_flag)
          {
            // Output low rate trajectory for plotting
            if (ts > next_save_traj_ts)
            {
              // Note: ttc only support one object (two targets on the same object)
              track_output_txt_ << std::fixed << std::setprecision(7) << ts;
              for (int i = 0; i < x_hat.rows(); ++i)
              {
                for (int j = 0; j < x_hat.cols(); ++j)
                {
                  track_output_txt_ << "," << x_hat(i, j);
                }
              }

              track_output_txt_ << std::endl;
              next_save_traj_ts = next_save_traj_ts + dt_save_traj;
            }
          }
        }
      }
    }

    // Global update high pass filter and display
    if (params.publish_framerate > 0 && ts > t_next_publish)
    {
      if (params.compute_TTC_flag & params.print_TTC_flag)
      {
        std::cout << std::fixed << std::setprecision(7) << ts << " " << TTC << std::endl;
      }
      x_hat = kf.state();
      P_x = kf.P_x();
      high_pass_global(ts, params.alpha);
      display(dispParams, params, ts, output_video, output_video_ref,
              image_ref_ts, p);
      kf.update_start_point(x_hat);
      t_next_publish = ts + (1.0 / params.publish_framerate);
    }
    ts_last[id] = ts;
    ts_kf_last = ts;
  }

  // Save to video
  if (params.save_video_flag)
  {
    if (params.ref_image_ts_flag)
    {
      for (int i = 0; i < output_video_ref.size(); i++)
      {
        writer.write(output_video[i]);
        writer_ref.write(output_video_ref[i]);
      }
    }
    else
    {
      for (int i = 0; i < output_video.size(); i++)
      {
        writer.write(output_video[i]);
      }
    }
  }
  writer.release();
  return 0;
}

// High-pass Filter
void high_pass(double ts, int x, int y, int p, int &alpha)
{
  log_intensity_state.at<double>(y, x) =
      exp(-alpha * (ts - ts_array.at<double>(y, x))) *
      log_intensity_state.at<double>(y, x);
  log_intensity_state.at<double>(y, x) +=
      (p > 0) ? -contrast_threshold : contrast_threshold;
  ts_array.at<double>(y, x) = ts;
};

void high_pass_global(double ts, int &alpha)
{
  cv::Mat beta;
  cv::exp(-alpha * (ts - ts_array), beta);
  log_intensity_state = log_intensity_state.mul(beta);
  ts_array.setTo(ts);
};

// View image
void display(const DisplayParams &dispParams, Parameters &params,
             const double &ts, std::vector<cv::Mat> &output_video,
             std::vector<cv::Mat> &output_video_ref,
             std::vector<double> &image_ref_ts, int p)
{
  cv::Mat image, ref_image;
  cv::exp(log_intensity_state, image);
  double minVal = 1.4;
  double maxVal = 0.4;
  image = (image - minVal) / (maxVal - minVal);
  cv::Mat img(image.rows, image.cols, CV_64FC1, (char *)image.data);
  img.convertTo(img, CV_8U, 255.0 / 1.0);
  cv::Mat cimg;
  cv::cvtColor(img, cimg, cv::COLOR_GRAY2RGB);

  // IMU colour data plot
  std::vector<cv::Scalar> colors = {
      cv::Scalar(0, 0, 255), cv::Scalar(0, 255, 0), cv::Scalar(255, 0, 0),
      cv::Scalar(255, 255, 0), cv::Scalar(255, 0, 255), cv::Scalar(0, 255, 255),
      cv::Scalar(0, 0, 128), cv::Scalar(0, 128, 0), cv::Scalar(128, 0, 0),
      cv::Scalar(128, 128, 0), cv::Scalar(128, 0, 128), cv::Scalar(0, 128, 128)};
  std::vector<cv::Scalar> colors_dark = {
      cv::Scalar(0, 0, 128), cv::Scalar(0, 128, 0), cv::Scalar(128, 0, 0),
      cv::Scalar(128, 128, 0), cv::Scalar(128, 0, 128), cv::Scalar(0, 128, 128)};
  for (int i = 0; i < n_target; i++)
  {
    // (180.0 / 3.14) =
    double rotationAngle = x_hat(i, 6) * 57.295;

    rotationAngle = std::fmod(rotationAngle, 360.0);
    if (rotationAngle < 0.0)
    {
      rotationAngle += 360.0;
    }

    cv::ellipse(cimg, cv::Point(x_hat(i, 0), x_hat(i, 1)),
                cv::Size(x_hat(i, 4), x_hat(i, 5)),
                rotationAngle, 0, 360, colors[i % colors.size()], 2, cv::LINE_AA);

    if (dispParams.disp_covariance_flag)
    {
      cv::ellipse(
          cimg, cv::Point(x_hat(i, 0), x_hat(i, 1)),
          cv::Size(50 * P_x(i * n_state, 0), 50 * P_x(i * n_state + 1, 1)),
          x_hat(i, 6) * (180.0 / M_PI), 0, 360, cv::Scalar(255, 0, 0), 1,
          cv::LINE_AA);
    }
  }

  // Plot image
  cv::imshow("Video", cimg);
  if (params.ref_image_ts_flag)
  {
    double event_ts = ts * 1e6 + t1;
    auto it = std::lower_bound(image_ref_ts.begin(), image_ref_ts.end(), event_ts);
    if (it == image_ref_ts.begin())
    {
      image_id = 1;
    }
    else if (it == image_ref_ts.end())
    {
      image_id = image_ref_ts.size() - 1;
    }
    else
    {
      image_id = std::distance(image_ref_ts.begin(), it);
    }

    image_id = image_id + 1; // please sync image id here
    std::string ref_image_path = params.input_folder_path + "/reproject_rgb/" +
                                 std::to_string(image_id) + ".png";

    ref_image = cv::imread(ref_image_path);
    // ref_image = ref_image * 2.5; // Option: make image brighter
    if (!ref_image.empty())
    {
      cv::imshow("Video_ref", ref_image);
    }
    else
    {
      std::cerr << "Error loading reference image: " << ref_image_path
                << std::endl;
    }
  }

  // Key input
  int keyPressed = cv::waitKey(30);
  if (keyPressed == 27)
  {
    // Select init points by press "Esc"
    for (int i = 0; i < n_target; i++)
    {
      std::cout << "Press ESC to start selecting points, and press Enter when "
                   "finished.\n";
      std::cout << "Waiting to click to select a point ... \n";
      selectedPoint = cv::Point(-1, -1); // Reset selected point
      while ((selectedPoint.x == -1 && selectedPoint.y == -1) &&
             keyPressed != 13)
      {
        keyPressed = cv::waitKey(1);
      }
      if (keyPressed == 13)
      {
        // 13 is the ASCII code for "Enter" key
        std::cout << "Selection completed.\n";
        std::cout << "Event Id: " << EventId << "\n";

        std::cout << x_hat << std::endl;
        params.var_x = 2;
        params.var_y = 2;
        init(params);
        dist_threshold.resize(n_target);
        std::fill(dist_threshold.begin(), dist_threshold.end(),
                  params.dist_threshold);
        break;
      }
      selectedPoints.push_back(
          selectedPoint); // Add selected point to the vector
      std::cout << "Selected point: (" << selectedPoint.x << ", "
                << selectedPoint.y << ")\n";
      std::cout << "Event Id: " << EventId << "\n";
      std::cout << "Time: " << ts << "\n";

      x_hat(i, 0) = selectedPoint.x; // Store selected point x-coordinate
      x_hat(i, 1) = selectedPoint.y; // Store selected point y-coordinate
      x_hat(i, 2) = 0;
      x_hat(i, 3) = 0;
      x_hat(i, 4) = 2;
      x_hat(i, 5) = 2;
    }
  }
  else if (keyPressed == 116 ||
           keyPressed ==
               84)
  { // Pressed 'T' key for display current time and location
    std::cout << "ts: " << ts << " (second)\n";
    std::cout << "Event Id: " << EventId << "\n";
    std::cout << "Current location x: (" << x_hat(0, 0) << ", " << x_hat(0, 1)
              << ")\n";
    std::cout << "Current lambda: (" << x_hat(0, 4) << ")\n";
    std::cout << "Current P_x: (" << P_x(0, 0) << ", " << P_x(1, 1) << ")\n";
  }
  cv::waitKey(1);

  // Save video
  if (dispParams.save_video_flag)
  {
    output_video.push_back(cimg);
    if (params.ref_image_ts_flag)
    {
      output_video_ref.push_back(ref_image);
    }
  }
  // Save image
  if (dispParams.save_image_flag)
  {
    std::string output_image_name =
        output_image_path + "/image" + std::to_string(image_count) + ".png";
    cv::imwrite(output_image_name, cimg);

    if (params.ref_image_ts_flag)
    {
      std::string output_image_ref_name =
          output_image_path + "/image" + std::to_string(image_count) + "_ref.png";
      cv::imwrite(output_image_ref_name, ref_image);
    }
  }
  image_count += 1;
}

void display_for_select()
{
  cv::Mat image;
  cv::exp(log_intensity_state, image);
  double minVal = 1.4;
  double maxVal = 0.4;
  image = (image - minVal) / (maxVal - minVal);
  cv::Mat img(image.rows, image.cols, CV_64FC1, (char *)image.data);
  img.convertTo(img, CV_8U, 255.0 / 1.0);
  cv::Mat cimg;
  cv::cvtColor(img, cimg, cv::COLOR_GRAY2RGB);

  // Plot image
  cv::imshow("Video", cimg);
  int keyPressed = cv::waitKey(30);
  for (int i = 0; i < n_target; i++)
  {
    std::cout << "Press ESC to start selecting points, and press Enter when "
                 "finished.\n";
    std::cout << "Waiting to click to select a point ... \n";
    selectedPoint = cv::Point(-1, -1); // Reset selected point

    while ((selectedPoint.x == -1 && selectedPoint.y == -1) &&
           keyPressed != 13)
    {
      keyPressed = cv::waitKey(1);
    }
    if (keyPressed == 13)
    {
      // 13 is the ASCII code for "Enter" key
      std::cout << "Event Id: " << EventId << "\n";
      std::cout << "Selection completed.\n";
      n_target = i;
      break;
    }
    selectedPoints.push_back(selectedPoint); // Add selected point to the vector
    std::cout << "Selected point: (" << selectedPoint.x << ", "
              << selectedPoint.y << ")\n";
    if (i == (n_target - 1))
    {
      std::cout << "Reach to upper limit: " << n_target << " points"
                << std::endl;
      std::cout << "Selection completed.\n";
      std::cout << "Event Id: " << EventId << "\n";
    }
  }
  cv::waitKey(1);
}

void init(Parameters &params)
{
  F = Eigen::MatrixXd::Zero(n_target * n_state, n_state);
  C = Eigen::MatrixXd::Zero(3, n_state);
  R = Eigen::MatrixXd::Identity(3, 3); // Measurement noise covariance
  P = Eigen::MatrixXd::Zero(n_target * n_state,
                            n_state);          // Estimate error covariance
  Q = Eigen::MatrixXd::Zero(n_state, n_state); // Process noise covariance
  A = Eigen::MatrixXd::Zero(2, 2);             // Sigma^(1/2)
  x0 = Eigen::MatrixXd::Zero(n_target, n_state);
  for (int j = 0; j < n_target; j++)
  {
    for (int i = 0; i < n_state; i++)
    {
      F(j * n_state + i, i) = 1;
    }
    F(j * n_state, 2) = params.dt;
    F(j * n_state + 1, 3) = params.dt;
    F(j * n_state + 6, 7) = params.dt;
  }

  if ((params.position_x_init[0] > 0) && (params.position_y_init[0] > 0))
  {
    for (int i = 0; i < n_target; i++)
    {
      x0(i, 0) = params.position_x_init[i];
      x0(i, 1) = params.position_y_init[i];
    }
  }
  Eigen::MatrixXd P_block = Eigen::MatrixXd::Zero(n_state, n_state);
  x0.block(0, 4, n_target, 1).setConstant(params.lambda_init);
  x0.block(0, 8, n_target, 2).setConstant(0);
  x0.block(0, 5, n_target, 1).setConstant(params.lambda_init);
  P_block.diagonal() << params.var_x, params.var_y, params.var_vx,
      params.var_vy, params.var_lambda_1, params.var_lambda_2, params.var_theta,
      params.var_q, 50, 50;
  Q.diagonal() << params.q_x, params.q_y, params.q_vx, params.q_vy,
      params.q_lambda_1, params.q_lambda_2, params.q_theta, params.q_q, 5, 5;
  for (int row = 0; row < n_target; row++)
  {
    P.block(row * n_state, 0, n_state, n_state) << P_block;
  }
  ts_last.resize(n_target);
  std::fill(ts_last.begin(), ts_last.end(), 0);
}

void read_events(std::stringstream &ss, const int data_format, double &ts,
                 int &c, int &r, int &p)
{
  // 0: ts,c,r,p; 1: c,r,p,ts
  if (data_format == 0)
  {
    ss >> ts;
    ss.get();
    ss >> c;
    ss.get();
    ss >> r;
    ss.get();
    ss >> p;
  }
  else
  {
    ss >> c;
    ss.get();
    ss >> r;
    ss.get();
    ss >> p;
    ss.get();
    ss >> ts;
  }

  if (first_ts_flag == 0)
  {
    // t1 = ts;
    t1 = 0; // don't minus first ts
    first_ts_flag = 1;
    std::cout << "ts: " << ts << std::endl;
  }
  ts = ts - t1;
  ts = ts * 1e-6; // second
  c = c + 1;
  r = r + 1;
}

void load_gryo(std::ifstream &my_gyro, KalmanFilter &kf)
{
  std::string gyro_line;
  std::getline(my_gyro, gyro_line);
  std::stringstream ss_gyro(gyro_line);
  double gyroX, gyroY, gyroZ;

  ss_gyro >> gyro_ts;
  ss_gyro.get();
  ss_gyro >> gyroX;
  ss_gyro.get();
  ss_gyro >> gyroY;
  ss_gyro.get();
  ss_gyro >> gyroZ;
  gyro_ts = (gyro_ts - t1) * 1e-6;

  if (first_gyro_flag)
  {
    gyro_ts_last = gyro_ts;
    first_gyro_flag = false;
  }

  // Note: update your IMU parameters here
  // bias gyro - our davis 240c data
  // remove bias, radian/s to degree/s
  gyroX = (gyroX + 1.7) * 0.017453;
  gyroY = (gyroY + 0.9) * 0.017453;
  gyroZ = (gyroZ - 0.2) * 0.017453;
  // Update v dot in tracker
  double f_x = 186.9;
  double f_y = 185.9;

  Eigen::MatrixXd term1 = x_hat.block(0, 0, n_target, 1).array() * x_hat.block(0, 1, n_target, 1).array() / f_x;
  Eigen::MatrixXd term2 = f_x + (x_hat.block(0, 0, n_target, 1).array() * x_hat.block(0, 0, n_target, 1).array() / f_x);
  Eigen::MatrixXd vx_gyro = term1.array() * gyroX - term2.array() * gyroY + x_hat.block(0, 1, n_target, 1).array() * gyroZ;
  Eigen::MatrixXd term3 = f_y + (x_hat.block(0, 1, n_target, 1).array() * x_hat.block(0, 1, n_target, 1).array() / f_y);
  Eigen::MatrixXd term4 = x_hat.block(0, 0, n_target, 1).array() * x_hat.block(0, 1, n_target, 1).array() / f_y;
  Eigen::MatrixXd vy_gyro = term3.array() * gyroX - term4.array() * gyroY - x_hat.block(0, 0, n_target, 1).array() * gyroZ;

  double gyro_dt = gyro_ts - gyro_ts_last;
  kf.update_gyro(vx_gyro.array() * gyro_dt, vy_gyro.array() * gyro_dt);

  gyro_ts_last = gyro_ts;
}
