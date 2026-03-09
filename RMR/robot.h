#ifndef ROBOT_H
#define ROBOT_H
#include "librobot/librobot.h"
#include <QObject>
#include <QWidget>

#ifndef DISABLE_OPENCV
#include "opencv2/core/utility.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

Q_DECLARE_METATYPE(cv::Mat)
#endif
#ifndef DISABLE_SKELETON
Q_DECLARE_METATYPE(skeleton)
#endif
Q_DECLARE_METATYPE(std::vector<LaserData>)
class robot : public QObject {
  Q_OBJECT
public:
  explicit robot(QObject *parent = nullptr);

  void initAndStartRobot(std::string ipaddress);

  // tato funkcia len nastavuje hodnoty.. posielaju sa v callbacku(dobre, kvoli
  // asynchronnosti a zabezpeceniu,ze sa poslu len raz pri viacero prepisoch
  // vramci callu)
  void setSpeedVal(double forw, double rots);
  // tato funkcia fyzicky posiela hodnoty do robota
  void setSpeed(double forw, double rots);
  void updateArcTrajectory(int current_target_index);

signals:
  void publishPosition(double x, double y, double z);
  void publishLidar(const std::vector<LaserData> &lidata);
#ifndef DISABLE_OPENCV
  void publishCamera(const cv::Mat &camframe);
#endif
#ifndef DISABLE_SKELETON
  void publishSkeleton(const skeleton &skeledata);
#endif

private:
  /// toto su vase premenne na vasu odometriu
  double x;
  double y;
  double fi;
  ///-----------------------------
  /// toto su rychlosti ktore sa nastavuju setSpeedVal a posielaju v
  /// processThisRobot
  double forwardspeed;  // mm/s
  double rotationspeed; // omega/s


// // nase privat premenne ˇˇˇˇˇˇˇˇˇˇˇˇˇˇ
  double enc_left = 0;  /// rozsah IRC 0~65535 (2 byte) - TREBA OSETRIT PRETECIENIE
  double enc_right = 0;
  double enc_left_prev = 0;  // Previous encoder values for delta calculation
  double enc_right_prev = 0;
  double enc_left_distance = 0;
  double enc_right_distance = 0;
  double delta_left_distance = 0;  // Distance traveled since last call
  double delta_right_distance = 0;
  double distance_traveled = 0;    // Total distance traveled since last call (average of both wheels)
  double gyro_angle = 0;  /// rozsah gyra -18000~18000 -> -180°~180° - TREBA OSETRIT PRETECIENIE
  double gyro_angle_prev = 0;
  bool first_reading_flag = true;
  double gyro_correction = 0;
  double sample_period = 0.025; // 25 ms,  1/40 Hz
  double wheel_base_distance = 0.23; // vzdialenost medzi kolesami v metroch
  double x_position = 0;
  double y_position = 0;
  double x_target = 0;
  double y_target = 0;
  // float x_target_position[2] = {0.0, 1.0};
  // float y_target_position[2] = {3.0, 2.9};
  float x_target_position[6] = {1.0, 1.0, 0.0, 0.0, 0.0, 3.0};
  float y_target_position[6] = {0.1, 0.0, 0.1, 0.0, 3.5, 4.0};
  int curve_steps = 1;
  bool last_target_reached = false;

  // S-curve velocity ramping parameters
  double scurve_progress = 0.0;     // Current progress through S-curve (0 to 1)
  double scurve_current_pct = 0.0;  // Current velocity as percentage (0 to 1)
  double scurve_target_pct = 0.0;   // Target velocity as percentage (0 to 1)
  double scurve_start_pct = 0.0;    // Starting velocity percentage when ramp began
  int scurve_total_steps = 20;      // Total steps to complete S-curve ramp
  int scurve_current_step = 0;      // Current step in the ramp
  bool scurve_active = false;       // Is S-curve ramping active
  
  // Actual speeds being sent to robot (smoothed by S-curve)
  double actual_forwardspeed = 0.0;
  double actual_rotationspeed = 0.0;
  double speed_change_threshold = 0.15; // 15% threshold for using S-curve
  
  // S-curve ramping for forward speed
  double fwd_scurve_start = 0.0;
  double fwd_scurve_target = 0.0;
  int fwd_scurve_step = 0;
  int fwd_scurve_total_steps = 20;
  bool fwd_scurve_active = false;
  
  // S-curve ramping for rotation speed
  double rot_scurve_start = 0.0;
  double rot_scurve_target = 0.0;
  int rot_scurve_step = 0;
  int rot_scurve_total_steps = 20;
  bool rot_scurve_active = false;

  // PI regulator parameters
  double Kp = 0.01;           // Proportional gain
  double Ki = 0.001;          // Integral gain
  double integral_error = 0; // Accumulated integral error
  double max_integral = 10; // Anti-windup limit
  double max_rotation_speed = 30; // max rotation speed deg/s
  double min_forward_speed = 25;   // minimum forward speed mm/s
  double max_forward_speed = 100;  // maximum forward speed mm/s
  double target_tolerance = 0.04;  // target reach tolerance in meters
  
  // Arc trajectory variables
  double arc_radius = 0;     // Current arc radius
  double angle_to_target = 0; // Angle from robot to target
  double heading_error = 0;   // Error between current heading and target angle
  int current_target_index = 0; // Index in target array

  // Helper functions for PI regulation
  double calculateAngleToTarget(double x_curr, double y_curr, double x_tgt, double y_tgt);
  double normalizeAngle(double angle);
  double piRegulator(double error);
  void updateArcTrajectory();
  
  // Odometry function - calculates position and distance traveled since last call
  void updateOdometry(const TKobukiData &robotdata);
  
  // S-curve velocity ramping functions
  double sCurveRamp(double current_pct, double target_pct);
  void startSCurveRamp(double target_pct, int steps = 20);
  double smootherstep(double t);
  double applySpeedRamp(double current, double target, double max_speed, 
                        double& ramp_start, double& ramp_target, 
                        int& ramp_step, int& ramp_total_steps, bool& ramp_active);

  enum CURVE_STATE
  {
    CURVE_CHANGING,
    CURVE_FINAL
  };
  CURVE_STATE curve_state = CURVE_FINAL;
// nase privat premenne ^^^^^^^^^^^^^^^^^
// nase pomocne funkcie

// nase pomocne funkcie ^^^^^^^^^^^^^^^^^
  double curve_modulation(double low, double high);
  double regulator(double error);

  /// toto su callbacky co sa sa volaju s novymi datami
  int processThisLidar(const std::vector<LaserData> &laserData);
  int processThisRobot(const TKobukiData &robotdata);
#ifndef DISABLE_OPENCV
  int processThisCamera(cv::Mat cameraData);
#endif

  /// pomocne strukutry aby ste si trosku nerobili race conditions
  std::vector<LaserData> copyOfLaserData;
#ifndef DISABLE_OPENCV
  cv::Mat frame[3];
#endif
  /// classa ktora riesi komunikaciu s robotom
  libRobot robotCom;

  /// pomocne premenne... moc nerieste naco su
  int datacounter;
#ifndef DISABLE_OPENCV
  bool useCamera1;
  int actIndex;
#endif

#ifndef DISABLE_SKELETON
  int processThisSkeleton(skeleton skeledata);
  int updateSkeletonPicture;
  skeleton skeleJoints;
#endif
  int useDirectCommands;
};

#endif // ROBOT_H
