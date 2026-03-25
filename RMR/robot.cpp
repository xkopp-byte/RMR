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


double robot::calculateAngleToTarget(double x_curr, double y_curr, double x_target, double y_target)
{
    double x_distance_to = x_target - x_curr;
    double y_distance_to = y_target - y_curr;
    return atan2(y_distance_to, x_distance_to); 
    //return uhol k cielu
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
    double p_term = Kp * error;
    
    integral_error += error * sample_period;
    if (integral_error > max_integral) integral_error = max_integral;
    if (integral_error < -max_integral) integral_error = -max_integral;
    double i_term = Ki * integral_error;
    
    double output = p_term + i_term;
    if (output > max_rotation_speed) output = max_rotation_speed;
    if (output < -max_rotation_speed) output = -max_rotation_speed;
    cout<<"PI Regulator - Error: "<<error<<" deg, P: "<<p_term<<" I: "<<i_term<<" Output: "<<output<<" deg/s\n";
    return output;
}

// Odometry function - calculates position and distance traveled since last call
void robot::updateOdometry(const TKobukiData &robotdata)
{
    if(first_reading_flag == true)
    {
        gyro_correction = robotdata.GyroAngle;
        gyro_angle_prev = 0;
        enc_left_prev = robotdata.EncoderLeft;
        enc_right_prev = robotdata.EncoderRight;
        first_reading_flag = false;
        return;
    }
    gyro_angle_prev = gyro_angle;
    enc_left = robotdata.EncoderLeft;
    enc_right = robotdata.EncoderRight;
    gyro_angle = robotdata.GyroAngle - gyro_correction;

    //GYROOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO
    // Keep lidar segment indexing aligned with robot heading (segment 0 = forward).
    // 36000 gyro units = 360 degrees, so one of 80 segments is 450 gyro units.
    double delta_gyro_units = gyro_angle - gyro_angle_prev;
    if (delta_gyro_units > 18000.0) delta_gyro_units -= 36000.0;
    if (delta_gyro_units < -18000.0) delta_gyro_units += 36000.0;
    const double segment_gyro_units = 36000.0 / 80.0;
    // Negate so right turn shifts segments to the right.
    lidar_rotation_remainder += -delta_gyro_units;
    const int segment_steps = static_cast<int>(lidar_rotation_remainder / segment_gyro_units);
    rotateLidarSegmentsBySteps(segment_steps);
    lidar_rotation_remainder -= static_cast<double>(segment_steps) * segment_gyro_units;
    cout << "\ngyro_delta=" << delta_gyro_units
            << " steps=" << segment_steps;
        for (int i = 0; i < 80; i++)
    {
        cout << lidar_segments[i];
    }
    //GYROOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO
    double delta_enc_left = enc_left - enc_left_prev;
    double delta_enc_right = enc_right - enc_right_prev;
    if (delta_enc_left > 32767) delta_enc_left -= 65536;
    if (delta_enc_left < -32767) delta_enc_left += 65536;
    if (delta_enc_right > 32767) delta_enc_right -= 65536;
    if (delta_enc_right < -32767) delta_enc_right += 65536;
    delta_left_distance = delta_enc_left * robotCom.getTickToMeter();
    delta_right_distance = delta_enc_right * robotCom.getTickToMeter();
    distance_traveled = (delta_left_distance + delta_right_distance) / 2.0;
    //gyro do radianov
    double gyro_rad = gyro_angle * M_PI / 18000.0;
    double gyro_rad_prev = gyro_angle_prev * M_PI / 18000.0;

    x_position += distance_traveled * cos(gyro_rad);
    y_position += distance_traveled * sin(gyro_rad);
    /*
    {
        // maybe we need to use this
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
    cout << "\nOdometry Update - X: " << x_position << " m, Y: " << y_position << " m, Distance Traveled: " << distance_traveled << " m\n";
}


double robot::applySpeedRamp(double current, double target, double max_speed,
                              double& ramp_start, double& ramp_target,
                              int& ramp_step, int& ramp_total_steps, bool& ramp_active)
{  

    if (fabs(ramp_target - target) > 0.05 && !ramp_active) //0.05 je threshold zmeny pri ktorej sa to vykona inicializacia a vypocet krokov
    {
        ramp_start = current;
        ramp_target = target;
        ramp_step = 0;
        // this is smart. vypocitava to pocet krokov na zmenu rychlosti prpoporcne k velkosti zmeny a max rychlosti. takze pri velkych zmenach bude ramp trvat dlhsie, pri malych zmenach bude ramp rychlejsia
        ramp_total_steps = static_cast<int>(50.0 * fabs(target - current) / max_speed);
        if (ramp_total_steps < 1) ramp_total_steps = 1; // handler na 1 step pri malych zmenach

        ramp_active = true;
    }
    if (!ramp_active)
    {
        return target;
    }
    ramp_step++;
    
    double t = static_cast<double>(ramp_step) / static_cast<double>(ramp_total_steps);
    if (t > 1.0) t = 1.0;
    if (t <= 0.0) t = 0.0;
    
    // 6t^5 - 15t^4 + 10t^3
    double s_factor = t * t * t * (t * (t * 6.0 - 15.0) + 10.0);
    double new_speed = ramp_start + (ramp_target - ramp_start) * s_factor;
    cout<<"Speed Ramp - Step: "<<ramp_step<<"/"<<ramp_total_steps<<", Target: "<<target<<" Current: "<<current<<" New Speed: "<<new_speed<<"\n"<<"S factor: "<<s_factor<<"\n";
    if (ramp_step >= ramp_total_steps)
    {
        ramp_active = false;
        ramp_step = 0;
        new_speed = ramp_target;
    }

    return new_speed;
}


void robot::updateArcTrajectory()
{
    //tato vs chcena pozicia a checking ked sa tam dostane, ak sa tam dostane pojde na dalsiu poziciu, ak sa dostane na vsetky pozicie, tak zastavi a nic viac nebude robit
    double x_target = x_target_position[current_target_index];
    double y_target = y_target_position[current_target_index];
    double x_distance_to = x_target - x_position; //x a y_position su definovane v robot.h ako startovacie parametre 0,0
    double y_distance_to = y_target - y_position;
    double distance_to_target = sqrt(x_distance_to * x_distance_to + y_distance_to * y_distance_to);
    double angle_to_target = atan2(y_distance_to, x_distance_to); 
    //check ci je robot v bode, ak je v bode, tak sa posunie na dalsi bod, ak je vsetky body dosiahnute, tak zastavi
    if ((distance_to_target < target_tolerance) && last_target_reached == false)
    {
        cout << "-----------------\nTarget " << current_target_index << " reached. \n-----------------\n\n";
        current_target_index++;
        cout << "Moving to Target " << current_target_index << ": (" << x_target_position[current_target_index] << ", " << y_target_position[current_target_index] << ")\n";
        x_target = x_target_position[current_target_index];
        y_target = y_target_position[current_target_index];
        x_distance_to = x_target - x_position;
        y_distance_to = y_target - y_position;
        distance_to_target = sqrt(x_distance_to * x_distance_to + y_distance_to * y_distance_to);
        is_in_vicinity_of_target = false; // reset proximity flag for new target
        flag = false; 
        
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
    if(obstacleDetector(distance_to_target, angle_to_target))
    {
        obstacle_detected = true; // neskor vyriesit globalna vs returnovana
        cout << "\nObstacle detected: " << obstacle_detected << "\n";
        double targetDirection = lidar_segments(candidateDirection());
        
        // rotacia na smer ku g
        rotationspeed = piRegulator(targetDirection*4.5);
        if(lidar_segments[0] == 2) // prekazka pred robotom
        {
            forwardspeed = 0;
        }
        else if (lidar_segments[0] == 1) // hystereza
        {
            forwardspeed = min_forward_speed;
        }
    } 
    else
    {
        cout << "\nObstacle detected: " << obstacle_detected << "\n";
        double error_degrees = 0.0;
        double desired_forwardspeed = 0.0;

        if (!obstacle_detected)
        {
            if (0.4 >= distance_to_target)
            {
                is_in_vicinity_of_target = true;
                if (flag == true)
                {
                    moment_forwardspeed = forwardspeed;
                    flag = false;
                }
            }
            else
            {
                flag = true;
                is_in_vicinity_of_target = false;
            }

            double current_heading_rad = gyro_angle * M_PI / 18000.0;
            heading_error = normalizeAngle(angle_to_target - current_heading_rad);
            error_degrees = heading_error * 180.0 / M_PI;
            rotationspeed = piRegulator(error_degrees);

            if (fabs(heading_error) >= M_PI / 2.0)
            {
                desired_forwardspeed = 0;
            }
            else
            {
                if(cos(heading_error) >= 0.995)
                    integral_error = 0;
                desired_forwardspeed = max_forward_speed * cos(heading_error);
                if (desired_forwardspeed < min_forward_speed && distance_to_target > target_tolerance)
                    desired_forwardspeed = min_forward_speed;
            }

            if (is_in_vicinity_of_target)
            {
                desired_forwardspeed = moment_forwardspeed * distance_to_target * 2;
                if (desired_forwardspeed < 0)
                    desired_forwardspeed = 0;
                if (desired_forwardspeed > max_forward_speed)
                    desired_forwardspeed = max_forward_speed;
            }

            actual_forwardspeed = applySpeedRamp(actual_forwardspeed, desired_forwardspeed, max_forward_speed,
                                                fwd_scurve_start, fwd_scurve_target,
                                                fwd_scurve_step, fwd_scurve_total_steps, fwd_scurve_active);
            if (actual_forwardspeed > max_forward_speed)
                actual_forwardspeed = max_forward_speed;
            if (actual_forwardspeed < 0)
                actual_forwardspeed = 0;

            forwardspeed = actual_forwardspeed;
        }
        else
        {
            forwardspeed = 0;
            desired_forwardspeed = 0;
        }

        setSpeed(forwardspeed, rotationspeed);

        cout<<"Position: ("<<x_position<<", "<<y_position<<"), Target: ("<<x_target<<", "<<y_target<<"), Distance to Target: "<<distance_to_target<<" m, Heading Error: "<<error_degrees<<" deg, Forward Speed: "<<forwardspeed<<" mm/s, Rotation Speed: "<<rotationspeed<<" deg/s, Obstacle: "<<obstacle_detected<<"\n";
        cout<<"\n ----- Distance to target: " <<distance_to_target<<" Actual speed: "<<actual_forwardspeed<<" mm/s, Desired forward speed: "<<desired_forwardspeed<<" mm/s, In Vicinity: "<<is_in_vicinity_of_target<<"\n\n";

    }
}

double robot::candidateDirection() // toto sa bude mergovat
{
    double g = 0.0;





    return g;
}



///toto je calback na data z robota, ktory ste podhodili robotu vo funkcii initAndStartRobot
/// vola sa vzdy ked dojdu nove data z robota. nemusite nic riesit, proste sa to stane
int robot::processThisRobot(const TKobukiData &robotdata)
{   
    updateOdometry(robotdata);
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
    cout<<"\n";
    //synctimestamp = robotdata.Timestamp; na zadanie 3 
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
    updateLidarSegments();
    //tu mozete robit s datami z lidaru.. napriklad najst prekazky, zapisat do mapy. naplanovat ako sa prekazke vyhnut.
    // ale nic vypoctovo narocne - to iste vlakno ktore cita data z lidaru
   // updateLaserPicture=1;
    emit publishLidar(copyOfLaserData);
   // update();//tento prikaz prinuti prekreslit obrazovku.. zavola sa paintEvent funkcia


    return 0;

}

void robot::rotateLidarSegmentsBySteps(int steps)
{
    const int segment_count = static_cast<int>(sizeof(lidar_segments) / sizeof(lidar_segments[0]));
    if (segment_count <= 1)
        return;

    int normalized_steps = steps % segment_count;
    if (normalized_steps < 0)
        normalized_steps += segment_count;
    if (normalized_steps == 0)
        return;

    int rotated[80] = {0};
    for (int i = 0; i < segment_count; i++)
    {
        int new_index = (i + normalized_steps) % segment_count;
        rotated[new_index] = lidar_segments[i];
    }

    for (int i = 0; i < segment_count; i++)
        lidar_segments[i] = rotated[i];
}

void robot::updateLidarSegments()
{
    const int segment_count = static_cast<int>(sizeof(lidar_segments) / sizeof(lidar_segments[0]));
    const size_t total_points = copyOfLaserData.size();
    const double threshold_mm = 1500.0;
    const double hysteresis_mm = 750.0;

    if (total_points == 0)
    {
        for (int i = 0; i < segment_count; i++)
            lidar_segments[i] = 0;
        return;
    }

    std::vector<int> next_segment_state(segment_count, -1);
    bool has_valid_segment = false;

    for (int seg = 0; seg < segment_count; seg++)
    {
        const size_t start = static_cast<size_t>((static_cast<unsigned long long>(seg) * total_points) / segment_count);
        const size_t end = static_cast<size_t>((static_cast<unsigned long long>(seg + 1) * total_points) / segment_count);

        if (end <= start)
            continue;

        double sum = 0.0;
        size_t valid_count = 0;
        for (size_t i = start; i < end; i++)
        {
            const double scan_mm = copyOfLaserData[i].scanDistance;
            if (scan_mm <= 0.0)
                continue;
            sum += scan_mm;
            valid_count++;
        }

        if (valid_count == 0)
            continue;
        if(lidar_segments[seg] == 3)
            continue;
        has_valid_segment = true;
        const double segment_average_mm = sum / static_cast<double>(valid_count);
        if (segment_average_mm > threshold_mm)
            next_segment_state[seg] = 0;
        else if (segment_average_mm > hysteresis_mm)
            next_segment_state[seg] = 1;
        else
            next_segment_state[seg] = 2;
    }

    if (!has_valid_segment)
    {
        for (int i = 0; i < segment_count; i++)
            lidar_segments[i] = 0;
        return;
    }

    for (int seg = 0; seg < segment_count; seg++)
    {
        if (next_segment_state[seg] != -1)
            continue;

        for (int offset = 1; offset < segment_count; offset++)
        {
            const int left = (seg - offset + segment_count) % segment_count;
            if (next_segment_state[left] != -1)
            {
                next_segment_state[seg] = next_segment_state[left];
                break;
            }

            const int right = (seg + offset) % segment_count;
            if (next_segment_state[right] != -1)
            {
                next_segment_state[seg] = next_segment_state[right];
                break;
            }
        }

        if (next_segment_state[seg] == -1)
            next_segment_state[seg] = 0;
    }

    for (int seg = 0; seg < segment_count; seg++)
    {
        lidar_segments[seg] = next_segment_state[seg];
    }
}

bool robot::obstacleDetector(double distance_to_target, double angle_to_target)
{
    const int segment_count = static_cast<int>(sizeof(lidar_segments) / sizeof(lidar_segments[0]));
    const size_t total_points = copyOfLaserData.size();
    const double mm_to_m = 0.001;

    if (total_points == 0 || segment_count <= 0)
    {
        obstacle_detected = false;
        return obstacle_detected;
    }

    // Convert target direction into robot frame where segment 0 is forward.
    double current_heading_rad = gyro_angle * M_PI / 18000.0;
    double relative_angle = normalizeAngle(angle_to_target - current_heading_rad);
    if (relative_angle < 0)
        relative_angle += 2.0 * M_PI;

    const double segment_width = (2.0 * M_PI) / static_cast<double>(segment_count);
    int target_segment = static_cast<int>(relative_angle / segment_width);
    if (target_segment >= segment_count)
        target_segment = 0;

    const size_t start = static_cast<size_t>((static_cast<unsigned long long>(target_segment) * total_points) / segment_count);
    const size_t end = static_cast<size_t>((static_cast<unsigned long long>(target_segment + 1) * total_points) / segment_count);

    if (end <= start)
    {
        obstacle_detected = false;
        return obstacle_detected;
    }

    double sum = 0.0;
    size_t valid_count = 0;
    for (size_t i = start; i < end; i++)
    {
        const double scan_mm = copyOfLaserData[i].scanDistance;
        if (scan_mm <= 0.0)
            continue;
        sum += scan_mm;
        valid_count++;
    }

    if (valid_count == 0)
    {
        obstacle_detected = false;
        return obstacle_detected;
    }
    lidar_segments[target_segment] = 3;
    const double segment_average_mm = sum / static_cast<double>(valid_count);
    const double distance_to_target_mm = distance_to_target * 1000.0;
    cout << "\nDistance to target [mm]: " << distance_to_target_mm << ", Segment average [mm]: " << segment_average_mm << "\n";
    obstacle_detected = (distance_to_target_mm > segment_average_mm);
    return obstacle_detected;
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
