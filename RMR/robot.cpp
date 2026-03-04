#include "robot.h"

robot::robot(QObject *parent) : QObject(parent)
{
    qRegisterMetaType<LaserMeasurement>("LaserMeasurement");
    #ifndef DISABLE_OPENCV
    qRegisterMetaType<cv::Mat>("cv::Mat");
#endif
#ifndef DISABLE_SKELETON
qRegisterMetaType<skeleton>("skeleton");
#endif
}

void robot::initAndStartRobot(std::string ipaddress)
{

    forwardspeed=0;
    rotationspeed=0;
    ///setovanie veci na komunikaciu s robotom/lidarom/kamerou.. su tam adresa porty a callback.. laser ma ze sa da dat callback aj ako lambda.
    /// lambdy su super, setria miesto a ak su rozumnej dlzky,tak aj prehladnost... ak ste o nich nic nepoculi poradte sa s vasim doktorom alebo lekarnikom...
    robotCom.setLaserParameters([this](const std::vector<LaserData>& dat)->int{return processThisLidar(dat);},ipaddress);
    robotCom.setRobotParameters([this](const TKobukiData& dat)->int{return processThisRobot(dat);},ipaddress);
  #ifndef DISABLE_OPENCV
    robotCom.setCameraParameters(std::bind(&robot::processThisCamera,this,std::placeholders::_1),"http://"+ipaddress+":8000/stream.mjpg");
#endif
   #ifndef DISABLE_SKELETON
      robotCom.setSkeletonParameters(std::bind(&robot::processThisSkeleton,this,std::placeholders::_1));
#endif
    ///ked je vsetko nasetovane tak to tento prikaz spusti (ak nieco nieje setnute,tak to normalne nenastavi.cize ak napr nechcete kameru,vklude vsetky info o nej vymazte)
    robotCom.robotStart();


}

void robot::setSpeedVal(double forw, double rots)
{
    forwardspeed=forw;
    rotationspeed=rots;
    useDirectCommands=0;
}

double robot::curve_modulation(double low, double high) // TODO, ale nepredbiehajme ...
{
    double actual_speed = low;
    if (curve_state == CURVE_FINAL)
    {
        return high;
    }
    else if (curve_state == CURVE_CHANGING)
    {
        actual_speed = low + (high - low) * (1.0001 - 1 /static_cast<double>(curve_steps));
        curve_steps++;
        if (curve_steps > 10)
        {
            curve_state = CURVE_FINAL;
            curve_steps = 0;
        }
    }

    return actual_speed;
}

double robot::regulator(double error)
{
    forwardspeed = 0;
    rotationspeed = 0;
    curve_state = CURVE_CHANGING;
    return 0; 
}

// Calculates angle from current position to target position
double robot::calculateAngleToTarget(double x_curr, double y_curr, double x_tgt, double y_tgt)
{
    double dx = x_tgt - x_curr;
    double dy = y_tgt - y_curr;
    return atan2(dy, dx); // Returns angle in radians
}

// Normalizes angle to range [-PI, PI]
double robot::normalizeAngle(double angle)
{
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

// PI regulator for heading control
double robot::piRegulator(double error)
{
    // Proportional term
    double p_term = Kp * error;
    
    // Integral term with anti-windup
    integral_error += error * sample_period;
    if (integral_error > max_integral) integral_error = max_integral;
    if (integral_error < -max_integral) integral_error = -max_integral;
    double i_term = Ki * integral_error;
    
    // Combined PI output
    double output = p_term + i_term;
    
    // Limit rotation speed
    if (output > max_rotation_speed) output = max_rotation_speed;
    if (output < -max_rotation_speed) output = -max_rotation_speed;
    
    return output;
}

// Odometry function - calculates position and distance traveled since last call
void robot::updateOdometry(const TKobukiData &robotdata)
{
    // Initialize on first reading
    if(first_reading_flag == true)
    {
        gyro_correction = robotdata.GyroAngle;
        gyro_angle_prev = 0;
        enc_left_prev = robotdata.EncoderLeft;
        enc_right_prev = robotdata.EncoderRight;
        first_reading_flag = false;
        return; // Skip first iteration, no delta to calculate
    }

    // Store previous gyro angle for arc calculation
    gyro_angle_prev = gyro_angle;

    // Read current encoder and gyro values
    enc_left = robotdata.EncoderLeft;
    enc_right = robotdata.EncoderRight;
    gyro_angle = robotdata.GyroAngle - gyro_correction;

    // Calculate encoder deltas (distance traveled since last call)
    // Handle encoder overflow (16-bit encoder: 0~65535)
    double delta_enc_left = enc_left - enc_left_prev;
    double delta_enc_right = enc_right - enc_right_prev;
    
    // Handle overflow for left encoder
    if (delta_enc_left > 32767) delta_enc_left -= 65536;
    if (delta_enc_left < -32767) delta_enc_left += 65536;
    
    // Handle overflow for right encoder
    if (delta_enc_right > 32767) delta_enc_right -= 65536;
    if (delta_enc_right < -32767) delta_enc_right += 65536;

    // Convert encoder ticks to meters
    delta_left_distance = delta_enc_left * robotCom.getTickToMeter();
    delta_right_distance = delta_enc_right * robotCom.getTickToMeter();
    
    // Distance traveled since last call (average of both wheels)
    distance_traveled = (delta_left_distance + delta_right_distance) / 2.0;

    // Convert gyro angles from centidegrees to radians for trigonometry
    double gyro_rad = gyro_angle * M_PI / 18000.0;
    double gyro_rad_prev = gyro_angle_prev * M_PI / 18000.0;

    // Calculate new position X and Y using gyro for heading
    
        // Straight line motion
        x_position += distance_traveled * cos(gyro_rad);
        y_position += distance_traveled * sin(gyro_rad);
    
    
    /*
    {
        // Arc motion - using differential drive kinematics with gyro heading
        x_position += (wheel_base_distance * (delta_left_distance + delta_right_distance)) 
                        / (2.0 * (delta_right_distance - delta_left_distance)) 
                        * (sin(gyro_rad) - sin(gyro_rad_prev));
        y_position -= (wheel_base_distance * (delta_left_distance + delta_right_distance))
                        / (2.0 * (delta_right_distance - delta_left_distance)) 
                        * (cos(gyro_rad) - cos(gyro_rad_prev));
    }
    */
    // Update previous encoder values for next iteration
    enc_left_prev = enc_left;
    enc_right_prev = enc_right;
}

// Updates arc trajectory to navigate towards target
void robot::updateArcTrajectory()
{
    // Get current target from arrays
    double x_tgt = x_target_position[current_target_index];
    double y_tgt = y_target_position[current_target_index];
    
    // Calculate distance to target
    double dx = x_tgt - x_position;
    double dy = y_tgt - y_position;
    double distance_to_target = sqrt(dx * dx + dy * dy);
    
    // Check if target reached - stop
    if (distance_to_target < target_tolerance)
    {
        current_target_index++;
        if(current_target_index >= 3)
        {
            current_target_index = 0;
        }
        x_tgt = x_target_position[current_target_index];
        y_tgt = y_target_position[current_target_index];
        dx = x_tgt - x_position;
        dy = y_tgt - y_position;
        distance_to_target = sqrt(dx * dx + dy * dy);
    }
    
    // Calculate angle to target
    angle_to_target = calculateAngleToTarget(x_position, y_position, x_tgt, y_tgt);
    
    // Calculate heading error (difference between robot heading and target angle)
    // gyro_angle is in centidegrees (-18000~18000 = -180°~180°), convert to radians
    double current_heading_rad = gyro_angle * M_PI / 18000.0;
    heading_error = normalizeAngle(angle_to_target - current_heading_rad);
    
    // Apply PI regulator to get rotation speed (convert error to degrees)
    double error_degrees = heading_error * 180.0 / M_PI;
    rotationspeed = piRegulator(error_degrees);
    
    // Calculate forward speed
    // If target is behind robot (heading error > 90°), turn in place first
    if (fabs(heading_error) > M_PI / 2.0)
    {
        // Target is behind - turn in place
        forwardspeed = 0;
    }
    else
    {
        // Target is in front - move and turn
        // Reduce speed when turning sharply
        double heading_factor = cos(heading_error); // 1 when aligned, 0 when perpendicular
        if (heading_factor < 0.1) heading_factor = 0.1; // Minimum factor to keep moving
        
        forwardspeed = min_forward_speed + (max_forward_speed - min_forward_speed) * heading_factor;
    }
    
    setSpeed(forwardspeed, rotationspeed);
    // Calculate arc radius for trajectory (positive = turn left, negative = turn right)
    if (fabs(rotationspeed) > 0.01)
    {
        // R = v / omega (convert rotationspeed from deg/s to rad/s)
        double omega_rad = rotationspeed * M_PI / 180.0;
        arc_radius = forwardspeed / omega_rad; // Arc radius in mm
    }
    else
    {
        arc_radius = 0; // Straight line (infinite radius)
    }
    cout<<"Position: ("<<x_position<<", "<<y_position<<"), Target: ("<<x_tgt<<", "<<y_tgt<<"), Distance to Target: "<<distance_to_target<<" m, Heading Error: "<<error_degrees<<" deg, Forward Speed: "<<forwardspeed<<" mm/s, Rotation Speed: "<<rotationspeed<<" deg/s\n";
          
}


void robot::setSpeed(double forw, double rots)
{
    if(forw==0 && rots!=0)
        robotCom.setRotationSpeed(rots);
    else if(forw!=0 && rots==0)
        robotCom.setTranslationSpeed(forw);
    else if((forw!=0 && rots!=0))
        robotCom.setArcSpeed(forw,forw/rots);
    else
        robotCom.setTranslationSpeed(0);
    useDirectCommands=1;
}



///toto je calback na data z robota, ktory ste podhodili robotu vo funkcii initAndStartRobot
/// vola sa vzdy ked dojdu nove data z robota. nemusite nic riesit, proste sa to stane
int robot::processThisRobot(const TKobukiData &robotdata)
{
// JAKUB: toto som zatial zakomentoval, nech to nestratime, tym ze idem "prepisovat" main branch
//     //rad by som spravil zmeny rychlosti kazdych 0.25 sekundy ked uz zacne robit jednu zmenu. aby sa ta s krivka aj prejavila 
//     //robot sa realne zacne hybat okolo desiny rychlosti
//     //od rychlosti 30 sa zacne robot spravat stabilnejsie, skok z pokoja na 30 moze byr rychlejsi a robot nebude presmykovat
// 
//     left_wheel[increment]=robotdata.EncoderLeft* robot.gettickToMeter;
//     right_wheel[increment]=robotdata.EncoderRight* robot.gettickToMeter;
//     gyro_actual[increment] = robotdata.GyroAngle;
//     float gyro_rads = robotdata.GyroAngle
//     float gyro_rads_prev = gyro_actual[(increment + 9) % 10];
// 
//     
//     x_robot_last_position = x_robot_last_position  + ((wheel_base_distance * (left_wheel[increment] + right_wheel[increment]))
//                                                    / (                   2 * (left_wheel[increment] + right_wheel[increment])))
//                                                    * (sin(gyro_rads) - sin(gyro_rads_prev));
//     y_robot_last_position = y_robot_last_position  - ((wheel_base_distance * (left_wheel[increment] + right_wheel[increment]))
//                                                    / (                   2 * (left_wheel[increment] + right_wheel[increment])))
//                                                    * (cos(gyro_rads) - cos(gyro_rads_prev));
//     phi = gyro_rads;
//     
//     if (increment==9)
//         increment=0;
//         distance_whole_meter+=(sum(left_wheel, 10) + sum(right_wheel, 10)) / 2;
//         x_robot_last_position += (left_wheel_speed + right_wheel_speed) / 2 * cos(phi) * 0.025;
//         y_robot_last_position += (left_wheel_speed + right_wheel_speed) / 2 * sin(phi) * 0.025;
//     else
//         increment++;
// 
//     float x_actual_positin = x_robot_last_position + (left_wheel_speed + right_wheel_speed) / 2 * cos(phi) * 0.025; 
//     float y_actual_position = y_robot_last_position + (left_wheel_speed + right_wheel_speed) / 2 * sin(phi) * 0.025;

    ///tu mozete robit s datami z robota
    
    // Update odometry - calculates position and distance traveled since last call
    updateOdometry(robotdata);

    // PI regulation for arc trajectory towards target
    updateArcTrajectory();
    //synctimestamp = robotdata.Timestamp; na zadanie 3 
///TU PISTE KOD... TOTO JE TO MIESTO KED NEVIETE KDE ZACAT,TAK JE TO NAOZAJ TU. AK AJ TAK NEVIETE, SPYTAJTE SA CVICIACEHO MA TU NATO STRING KTORY DA DO HLADANIA XXX

    ///kazdy piaty krat, aby to ui moc nepreblikavalo..
    if(datacounter%5==0)
    {

        ///ak nastavite hodnoty priamo do prvkov okna,ako je to na tychto zakomentovanych riadkoch tak sa moze stat ze vam program padne
        // ui->lineEdit_2->setText(QString::number(robotdata.EncoderRight));
        //ui->lineEdit_3->setText(QString::number(robotdata.EncoderLeft));
        //ui->lineEdit_4->setText(QString::number(robotdata.GyroAngle));
        /// lepsi pristup je nastavit len nejaku premennu, a poslat signal oknu na prekreslenie
        /// okno pocuva vo svojom slote a vasu premennu nastavi tak ako chcete. prikaz emit to presne takto spravi
        /// viac o signal slotoch tu: https://doc.qt.io/qt-5/signalsandslots.html
        ///posielame sem nezmysli.. pohrajte sa nech sem idu zmysluplne veci
        emit publishPosition(robotdata.EncoderLeft,y,fi);
        ///toto neodporucam na nejake komplikovane struktury.signal slot robi kopiu dat. radsej vtedy posielajte
        /// prazdny signal a slot bude vykreslovat strukturu (vtedy ju musite mat samozrejme ako member premmennu v mainwindow.ak u niekoho najdem globalnu premennu,tak bude cistit bludisko zubnou kefkou.. kefku dodam)
        /// vtedy ale odporucam pouzit mutex, aby sa vam nestalo ze budete pocas vypisovania prepisovat niekde inde

    }
    ///---tu sa posielaju rychlosti do robota... vklude zakomentujte ak si chcete spravit svoje
    if(useDirectCommands==0)
    {
        if(forwardspeed==0 && rotationspeed!=0)
            robotCom.setRotationSpeed(rotationspeed);
        else if(forwardspeed!=0 && rotationspeed==0)
            robotCom.setTranslationSpeed(forwardspeed);
        else if((forwardspeed!=0 && rotationspeed!=0))
            robotCom.setArcSpeed(forwardspeed,forwardspeed/rotationspeed);
        else
            robotCom.setTranslationSpeed(0);
    }
    datacounter++;

    return 0;

}

///toto je calback na data z lidaru, ktory ste podhodili robotu vo funkcii initAndStartRobot
/// vola sa ked dojdu nove data z lidaru
int robot::processThisLidar(const std::vector<LaserData>& laserData)
{

    copyOfLaserData=laserData;

    //tu mozete robit s datami z lidaru.. napriklad najst prekazky, zapisat do mapy. naplanovat ako sa prekazke vyhnut.
    // ale nic vypoctovo narocne - to iste vlakno ktore cita data z lidaru
   // updateLaserPicture=1;
    emit publishLidar(copyOfLaserData);
   // update();//tento prikaz prinuti prekreslit obrazovku.. zavola sa paintEvent funkcia


    return 0;

}

  #ifndef DISABLE_OPENCV
///toto je calback na data z kamery, ktory ste podhodili robotu vo funkcii initAndStartRobot
/// vola sa ked dojdu nove data z kamery
int robot::processThisCamera(cv::Mat cameraData)
{

    cameraData.copyTo(frame[(actIndex+1)%3]);//kopirujem do nasej strukury
    actIndex=(actIndex+1)%3;//aktualizujem kde je nova fotka

    emit publishCamera(frame[actIndex]);
    return 0;
}
#endif

  #ifndef DISABLE_SKELETON
/// vola sa ked dojdu nove data z trackera
int robot::processThisSkeleton(skeleton skeledata)
{

    memcpy(&skeleJoints,&skeledata,sizeof(skeleton));

    emit publishSkeleton(skeleJoints);
    return 0;
}
#endif
