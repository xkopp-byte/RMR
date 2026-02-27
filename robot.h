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

//   // JAKUB: toto som zatial zakomentoval, nech to nestratime, tym ze idem "prepisovat" main branch
// // nase privat premenne ˇˇˇˇˇˇˇˇˇˇˇˇˇˇ
//   float distance_whole_meter = 0;
//   float left_wheel[10] = {0};
//   float right_wheel[10] = {0};  
//   float phi = 0;
//   uint8_t increment = 0;
//   float x_robot_last_position = 0;
//   float y_robot_last_position = 0;    
//   float wheel_base_distance = 0.23; // vzdialenost medzi kolesami v metroch
//   float robot_radius = 0.11;
//   float gyro_actual[10] = {0};
//   float gyro_rads_prev = 0;
// // nase privat premenne ^^^^^^^^^^^^^^^^^
  double enc_left = 0;  /// rozsah IRC 0~65535 (2 byte) - TREBA OSETRIT PRETECIENIE
  double enc_right = 0;
  double enc_left_distance = 0;
  double enc_right_distance = 0;
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
  int curve_steps = 1
 

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
