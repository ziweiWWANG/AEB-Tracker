#ifndef PARAMETERS_H
#define PARAMETERS_H
#include "yaml-cpp/yaml.h"
#include <string>

struct Parameters
{
  int width;
  int height;
  int data_format;
  double dist_threshold;
  int n_target;
  int n_state;
  int m;
  int publish_framerate;
  double dt;
  double contrast_threshold;
  int alpha;
  int use_gyro_flag;
  int save_video_flag;
  int save_image_flag;
  int disp_covariance_flag;
  int ref_image_ts_flag;
  double process_ts_start;
  double process_ts_end;
  int event_num_start;
  double event_num_end;
  int print_TTC_flag;
  int compute_TTC_flag;
  int save_track_flag;
  int ring_buffer_len;
  std::string input_data_name;
  std::string input_folder_path;
  double var_x;
  double var_y;
  double var_vx;
  double var_vy;
  double var_lambda_1;
  double var_lambda_2;
  double var_theta;
  double var_q;
  double q_x;
  double q_y;
  double q_vx;
  double q_vy;
  double q_lambda_1;
  double q_lambda_2;
  double q_theta;
  double q_q;
  std::vector<double> position_x_init;
  std::vector<double> position_y_init;
  double lambda_init;
  int select_target_flag;
};

struct DisplayParams
{
  int save_video_flag;
  int save_image_flag;
  int disp_covariance_flag;
};

Parameters loadParametersFromYAML(const std::string &yaml_file_path)
{
  const YAML::Node config = YAML::LoadFile(yaml_file_path);

  Parameters params;
  params.width = config["width"].as<int>();
  params.height = config["height"].as<int>();
  params.data_format = config["data_format"].as<int>();
  params.dist_threshold = config["dist_threshold"].as<double>();
  params.n_target = config["n_target"].as<int>();
  params.n_state = config["n_state"].as<int>();
  params.m = config["m"].as<int>();
  params.publish_framerate = config["publish_framerate"].as<int>();
  params.dt = config["dt"].as<double>();
  params.contrast_threshold = config["contrast_threshold"].as<double>();
  params.alpha = config["alpha"].as<int>();
  params.use_gyro_flag = config["use_gyro_flag"].as<int>();
  params.save_video_flag = config["save_video_flag"].as<int>();
  params.save_image_flag = config["save_image_flag"].as<int>();
  params.ref_image_ts_flag = config["ref_image_ts_flag"].as<int>();
  params.disp_covariance_flag = config["disp_covariance_flag"].as<int>();
  params.process_ts_start = config["process_ts_start"].as<double>();
  params.process_ts_end = config["process_ts_end"].as<double>();
  params.event_num_start = config["event_num_start"].as<int>();
  params.event_num_end = config["event_num_end"].as<double>();
  params.ring_buffer_len = config["ring_buffer_len"].as<int>();

  params.input_data_name = config["input_data_name"].as<std::string>();
  params.print_TTC_flag = config["print_TTC_flag"].as<int>();
  params.compute_TTC_flag = config["compute_TTC_flag"].as<int>();
  params.save_track_flag = config["save_track_flag"].as<int>();
  params.select_target_flag = config["select_target_flag"].as<int>();
  params.input_folder_path = config["input_folder_path"].as<std::string>();

  params.var_x = config["var_x"].as<double>();
  params.var_y = config["var_y"].as<double>();
  params.var_vx = config["var_vx"].as<double>();
  params.var_vy = config["var_vy"].as<double>();
  params.var_lambda_1 = config["var_lambda_1"].as<double>();
  params.var_lambda_2 = config["var_lambda_2"].as<double>();
  params.var_theta = config["var_theta"].as<double>();
  params.var_q = config["var_q"].as<double>();
  params.q_x = config["q_x"].as<double>();
  params.position_x_init = config["position_x_init"].as<std::vector<double>>();
  params.position_y_init = config["position_y_init"].as<std::vector<double>>();
  params.lambda_init = config["lambda_init"].as<double>();

  params.q_y = config["q_y"].as<double>();
  params.q_vx = config["q_vx"].as<double>();
  params.q_vy = config["q_vy"].as<double>();
  params.q_lambda_1 = config["q_lambda_1"].as<double>();
  params.q_lambda_2 = config["q_lambda_2"].as<double>();
  params.q_theta = config["q_theta"].as<double>();
  params.q_q = config["q_q"].as<double>();

  return params;
}

DisplayParams loadDispParametersFromYAML(const std::string &yaml_file_path)
{
  const YAML::Node config = YAML::LoadFile(yaml_file_path);
  DisplayParams dispParams;
  dispParams.save_video_flag = config["save_video_flag"].as<int>();
  dispParams.save_image_flag = config["save_image_flag"].as<int>();
  dispParams.disp_covariance_flag = config["disp_covariance_flag"].as<int>();

  return dispParams;
}
#endif // PARAMETERS_H
