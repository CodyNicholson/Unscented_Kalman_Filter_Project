#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <stdlib.h>
#include "Eigen/Dense"
#include "ukf.h"
#include "ground_truth_package.h"
#include "measurement_package.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

void validate_args(int arg_count, char* arg_values[])
{
  string usage_instructions = "Usage instructions: ";
  usage_instructions += arg_values[0];
  usage_instructions += " path/to/input.txt output.txt";

  bool valid_args_check = false;

  // Make sure the user has provided input and output files
  if (arg_count == 1)
  {
    // cerr - Object of class ostream that represents the standard error stream oriented to narrow characters (of type char)
    // It corresponds to the C stream stderr
    cerr << usage_instructions << endl;
  } else if (arg_count == 2) {
    cerr << "Include output file\n" << usage_instructions << endl;
  } else if (arg_count == 3) {
    valid_args_check = true;
  } else if (arg_count > 3) {
    cerr << "Too many arguments\n" << usage_instructions << endl;
  }

  if (!valid_args_check)
  {
    exit(EXIT_FAILURE);
  }
}

void validate_files(ifstream& in_file, string& in_name, ofstream& out_file, string& out_name)
{
  if (!in_file.is_open())
  {
    cerr << "Error with input file: " << in_name << endl;
    exit(EXIT_FAILURE);
  }

  if (!out_file.is_open())
  {
    cerr << "Error with output file: " << out_name << endl;
    exit(EXIT_FAILURE);
  }
}

int main(int arg_count, char* arg_values[])
{
  validate_args(arg_count, arg_values);

  string in_file_name_ = arg_values[1];
  ifstream in_file_(in_file_name_.c_str(), ifstream::in);

  string out_file_name_ = arg_values[2];
  ofstream out_file_(out_file_name_.c_str(), ofstream::out);

  validate_files(in_file_, in_file_name_, out_file_, out_file_name_);

  // ** Set Measurements ** //
  vector<MeasurementPackage> measurement_list;
  vector<GroundTruthPackage> ground_truth_list;

  string line;

  // Get measurement packages
  while (getline(in_file_, line))
  {
    string sensor_type;
    MeasurementPackage measurement_package;
    GroundTruthPackage ground_truth_package;
    istringstream iss(line);
    long long timestamp;

    // Read first element from current line
    iss >> sensor_type;

    // Laser measurement
    if (sensor_type.compare("L") == 0)
    {
      // Read measurements at timestamp
      measurement_package.sensor_type_ = MeasurementPackage::LASER;
      measurement_package.raw_measurements_ = VectorXd(2);
      float px;
      float py;
      iss >> px;
      iss >> py;
      measurement_package.raw_measurements_ << px, py;
      iss >> timestamp;
      measurement_package.timestamp_ = timestamp;
      measurement_list.push_back(measurement_package);
    }
    // radar measurement
    else if (sensor_type.compare("R") == 0) {
      // read measurements at this timestamp
      measurement_package.sensor_type_ = MeasurementPackage::RADAR;
      measurement_package.raw_measurements_ = VectorXd(3);
      float ro;
      float phi;
      float ro_dot;
      iss >> ro;
      iss >> phi;
      iss >> ro_dot;
      measurement_package.raw_measurements_ << ro, phi, ro_dot;
      iss >> timestamp;
      measurement_package.timestamp_ = timestamp;
      measurement_list.push_back(measurement_package);
    }

      // Read ground truth data
      float x_gt;
      float y_gt;
      float vx_gt;
      float vy_gt;
      iss >> x_gt;
      iss >> y_gt;
      iss >> vx_gt;
      iss >> vy_gt;
      ground_truth_package.gt_values_ = VectorXd(4);
      ground_truth_package.gt_values_ << x_gt, y_gt, vx_gt, vy_gt;
      ground_truth_list.push_back(ground_truth_package);
  }

  // Create a UKF instance
  UKF ukf;

  // Used to compute the RMSE later
  vector<VectorXd> estimations;
  vector<VectorXd> ground_truths;

  // Start filtering from the second frame since the first frame alone tells us nothing
  size_t number_of_measurements = measurement_list.size();

  // Column names for output file
  out_file_ << "px" << "\t";
  out_file_ << "py" << "\t";
  out_file_ << "v" << "\t";
  out_file_ << "yaw_angle" << "\t";
  out_file_ << "yaw_rate" << "\t";
  out_file_ << "px_measured" << "\t";
  out_file_ << "py_measured" << "\t";
  out_file_ << "px_true" << "\t";
  out_file_ << "py_true" << "\t";
  out_file_ << "vx_true" << "\t";
  out_file_ << "vy_true" << "\t";
  out_file_ << "NIS" << "\n";

  for (size_t k = 0; k < number_of_measurements; ++k)
  {
    // Call the UKF-based fusion
    ukf.ProcessMeasurement(measurement_list[k]);

    // Output the estimation
    out_file_ << ukf.x_(0) << "\t"; // pos1 - est
    out_file_ << ukf.x_(1) << "\t"; // pos2 - est
    out_file_ << ukf.x_(2) << "\t"; // vel_abs -est
    out_file_ << ukf.x_(3) << "\t"; // yaw_angle -est
    out_file_ << ukf.x_(4) << "\t"; // yaw_rate -est

    // Output the measurements
    if (measurement_list[k].sensor_type_ == MeasurementPackage::LASER)
    {
      // p1_meas
      out_file_ << measurement_list[k].raw_measurements_(0) << "\t";
      // p2_meas
      out_file_ << measurement_list[k].raw_measurements_(1) << "\t";
    }
    else if (measurement_list[k].sensor_type_ == MeasurementPackage::RADAR)
    {
      // Output the estimation in the Cartesian coordinates
      float ro = measurement_list[k].raw_measurements_(0);
      float phi = measurement_list[k].raw_measurements_(1);
      out_file_ << ro * cos(phi) << "\t"; // p1_meas
      out_file_ << ro * sin(phi) << "\t"; // p2_meas
    }

    // Output the ground truth packages
    out_file_ << ground_truth_list[k].gt_values_(0) << "\t";
    out_file_ << ground_truth_list[k].gt_values_(1) << "\t";
    out_file_ << ground_truth_list[k].gt_values_(2) << "\t";
    out_file_ << ground_truth_list[k].gt_values_(3) << "\t";

    // Output the NIS values
    if (measurement_list[k].sensor_type_ == MeasurementPackage::LASER)
    {
      out_file_ << ukf.NIS_laser_ << "\n";
    }
    else if (measurement_list[k].sensor_type_ == MeasurementPackage::RADAR)
    {
      out_file_ << ukf.NIS_radar_ << "\n";
    }


    // Convert ukf x vector to Cartesian to compare to ground truth
    VectorXd ukf_x_cartesian_ = VectorXd(4);

    float x_estimate_ = ukf.x_(0);
    float y_estimate_ = ukf.x_(1);
    float vx_estimate_ = ukf.x_(2) * cos(ukf.x_(3));
    float vy_estimate_ = ukf.x_(2) * sin(ukf.x_(3));

    ukf_x_cartesian_ << x_estimate_, y_estimate_, vx_estimate_, vy_estimate_;

    estimations.push_back(ukf_x_cartesian_);
    ground_truths.push_back(ground_truth_list[k].gt_values_);
  }

  // Compute the accuracy RMSE
  Tools tools;
  cout << "Accuracy of RMSE:" << endl << tools.CalculateRMSE(estimations, ground_truths) << endl;

  // Close files
  if (out_file_.is_open())
  {
    out_file_.close();
  }

  if (in_file_.is_open())
  {
    in_file_.close();
  }

  cout << "Done!" << endl;
  return 0;
}
