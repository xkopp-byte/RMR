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

double robot::curve_modulation(double low, double high) 
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

    cout << "Odometry Update - X: " << x_position << " m, Y: " << y_position << " m, Distance Traveled: " << distance_traveled << " m\n";
}

// Smootherstep function (Ken Perlin's improved smoothstep)
// Returns S-curve value for t in [0, 1]
// Pattern: slow start -> fast middle -> slow end
double robot::smootherstep(double t)
{
    if (t <= 0.0) return 0.0;
    if (t >= 1.0) return 1.0;
    // 6t^5 - 15t^4 + 10t^3
    return t * t * t * (t * (t * 6.0 - 15.0) + 10.0);
}

// Start a new S-curve ramp to target percentage
// target_pct: target velocity as percentage (0 to 1)
// steps: number of steps to complete the ramp (default 20)
void robot::startSCurveRamp(double target_pct, int steps)
{
    scurve_target_pct = target_pct;
    scurve_total_steps = steps;
    scurve_current_step = 0;
    scurve_active = true;
    // scurve_current_pct remains at current value for smooth transition
}

// S-curve velocity ramping function
// Returns percentage (0 to 1) that should be applied to velocities
// Maintains ratio between forward and rotation speeds for arc trajectories
double robot::sCurveRamp(double current_pct, double target_pct)
{
    // If target changed, start new ramp
    if (fabs(scurve_target_pct - target_pct) > 0.001)
    {
        startSCurveRamp(target_pct);
        scurve_start_pct = current_pct; // Store starting percentage
    }
    
    // If not ramping or already at target, return target
    if (!scurve_active || fabs(current_pct - target_pct) < 0.001)
    {
        scurve_active = false;
        return target_pct;
    }
    
    // Increment step
    scurve_current_step++;
    
    // Calculate progress through S-curve (0 to 1)
    scurve_progress = static_cast<double>(scurve_current_step) / static_cast<double>(scurve_total_steps);
    if (scurve_progress > 1.0) scurve_progress = 1.0;
    
    // Apply smootherstep to get S-curve interpolation factor
    double s_factor = smootherstep(scurve_progress);
    
    // Interpolate: start + (target - start) * s_factor
    double new_pct = scurve_start_pct + (target_pct - scurve_start_pct) * s_factor;
    
    // Check if ramp is complete
    if (scurve_current_step >= scurve_total_steps)
    {
        scurve_active = false;
        scurve_current_step = 0;
        new_pct = target_pct;
    }
    
    return new_pct;
}

// Apply S-curve ramping to a speed value
// For changes > 15% of max, use S-curve; otherwise apply directly
double robot::applySpeedRamp(double current, double target, double max_speed,
                              double& ramp_start, double& ramp_target,
                              int& ramp_step, int& ramp_total_steps, bool& ramp_active)
{
    double change = fabs(target - current);
    double threshold = speed_change_threshold * max_speed;
    
    // Small change - apply directly
    if (change <= threshold && !ramp_active)
    {
        return target;
    }
    
    // Large change or ramp in progress - use S-curve
    // Start new ramp if target changed significantly
    if (fabs(ramp_target - target) > 0.001 && !ramp_active)
    {
        ramp_start = current;
        ramp_target = target;
        ramp_step = 0;
        ramp_total_steps = 20;
        ramp_active = true;
    }
    
    // If not ramping, return target
    if (!ramp_active)
    {
        return target;
    }
    
    // Increment step
    ramp_step++;
    
    // Calculate progress through S-curve (0 to 1)
    double progress = static_cast<double>(ramp_step) / static_cast<double>(ramp_total_steps);
    if (progress > 1.0) progress = 1.0;
    
    // Apply smootherstep to get S-curve interpolation factor
    double s_factor = smootherstep(progress);
    
    // Interpolate: start + (target - start) * s_factor
    double new_speed = ramp_start + (ramp_target - ramp_start) * s_factor;
    
    // Check if ramp is complete
    if (ramp_step >= ramp_total_steps)
    {
        ramp_active = false;
        ramp_step = 0;
        new_speed = ramp_target;
    }
    
    return new_speed;
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
    if ((distance_to_target < target_tolerance) && last_target_reached == false)
    {
        current_target_index++;
        cout << "-----------------\nTarget " << current_target_index << " reached. \n-----------------\n\n";
        // if(current_target_index >= sizeof(x_target_position)/sizeof(x_target_position[0]))
        // {
        //     current_target_index = 0;
        // }
        x_tgt = x_target_position[current_target_index];
        y_tgt = y_target_position[current_target_index];
        dx = x_tgt - x_position;
        dy = y_tgt - y_position;
        distance_to_target = sqrt(dx * dx + dy * dy);
        
        if(current_target_index >= sizeof(x_target_position)/sizeof(x_target_position[0]))
        {
            last_target_reached = true;
            current_target_index = 0;
            forwardspeed = 0;
            rotationspeed = 0;
            setSpeed(forwardspeed, rotationspeed);
            return;
        }
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
    
    // Calculate desired forward speed
    double desired_forwardspeed = 0;
    double desired_rotationspeed = rotationspeed;
    
    // If target is behind robot (heading error > 90°), turn in place first
    if (fabs(heading_error) > M_PI / 2.0)
    {
        // Target is behind - turn in place
        desired_forwardspeed = 0;
    }
    else
    {
        // Target is in front - move and turn
        // Reduce speed when turning sharply
        double heading_factor = cos(heading_error); // 1 when aligned, 0 when perpendicular
        if (heading_factor < 0.1) heading_factor = 0.1; // Minimum factor to keep moving
        
        desired_forwardspeed = min_forward_speed + (max_forward_speed - min_forward_speed) * heading_factor;
    }
    
    // Apply S-curve ramping for large changes (>15% of max), direct for small changes
    actual_forwardspeed = applySpeedRamp(actual_forwardspeed, desired_forwardspeed, max_forward_speed,
                                          fwd_scurve_start, fwd_scurve_target,
                                          fwd_scurve_step, fwd_scurve_total_steps, fwd_scurve_active);
    
    actual_rotationspeed = applySpeedRamp(actual_rotationspeed, desired_rotationspeed, max_rotation_speed,
                                           rot_scurve_start, rot_scurve_target,
                                           rot_scurve_step, rot_scurve_total_steps, rot_scurve_active);
    
    forwardspeed = actual_forwardspeed;
    rotationspeed = actual_rotationspeed;
    
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
    ///tu mozete robit s datami z robota
    
    // Update odometry - calculates position and distance traveled since last call
    updateOdometry(robotdata);

    // PI regulation for arc trajectory towards target
    if(last_target_reached == false)
    {
        updateArcTrajectory();
    }
    else
    {
        forwardspeed = 0;
        rotationspeed = 0;
        setSpeed(forwardspeed, rotationspeed);
    }
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
