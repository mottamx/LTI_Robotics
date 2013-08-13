// -------------------------------------------------------------------------
// Odometria para ARDrone
// Lectura de datos y calculo de posicion
// Carlos Alberto Motta Avila
// for LTI Cinvestav - Robotics
// 2013
// -------------------------------------------------------------------------


#include "odometry.h"

using namespace std;

// --------------------------------------------------------------------------
// Odometry::Init()
// Description  : Initialize R_{n} matrices
// Return value : None
// --------------------------------------------------------------------------
void Odometry::Init(){
    P= cv::Mat::zeros(3, 1, CV_64FC1);
    RX= cv::Mat::zeros(3, 3, CV_64FC1);
    RY= cv::Mat::zeros(3, 3, CV_64FC1);
    RZ= cv::Mat::zeros(3, 3, CV_64FC1);
    mapa= cv::Mat::zeros(500, 500,  CV_8UC3);
}

// --------------------------------------------------------------------------
// Odometry::resetP()
// Description  : Restore Position matrix to 0s
// Return value : None
// --------------------------------------------------------------------------
void Odometry::resetP(const double& altura){
    P= Mat::zeros(3, 1, CV_64FC1);
    P.at<double>(2,0)=altura;
}

// --------------------------------------------------------------------------
// Odometry::getVelFromDrone()
// Description  : Read velocities from drone sensors
// Return value : None
// --------------------------------------------------------------------------
void Odometry::getVelFromDrone(){
    velocity = thisArdrone.getVelocity(&vx, &vy, &vz);
}

// --------------------------------------------------------------------------
// Odometry::getAnglesFromDrone()
// Description  : Read angles pitch, roll & yaw
// Return value : None
// --------------------------------------------------------------------------
void Odometry::getAnglesFromDrone(){
      roll  = thisArdrone.getRoll();
      pitch = thisArdrone.getPitch();
      yaw   = thisArdrone.getYaw();
}

// --------------------------------------------------------------------------
// Odometry::deadReckon()
// Description  : Calculate position based upon known speeds over elapsed time, and course.
// Return value : None
// --------------------------------------------------------------------------
void Odometry::deadReckon(){

    getVelFromDrone();
    getAnglesFromDrone();
    // Time
    double dt = (cv::getTickCount() - last) / cv::getTickFrequency();
    last= cv::getTickCount();
    //double altitude = thisArdrone.getAltitude(); //porque vz es 0

    // Rotation matrices
        double _RX[] = {        1.0,       0.0,        0.0,
                                0.0, cos(roll), -sin(roll),
                                0.0, sin(roll),  cos(roll)};
        double _RY[] = { cos(pitch),       0.0,  sin(pitch),
                                0.0,       1.0,        0.0,
                        -sin(pitch),       0.0,  cos(pitch)};
        double _RZ[] = {   cos(yaw), -sin(yaw),        0.0,
                           sin(yaw),  cos(yaw),        0.0,
                                0.0,       0.0,        1.0};
    Mat RX(3, 3, CV_64FC1, _RX);
    Mat RY(3, 3, CV_64FC1, _RY);
    Mat RZ(3, 3, CV_64FC1, _RZ);


    double _M[] = {vx * dt, vy * dt, vz * dt};
    Mat M(3, 1, CV_64FC1, _M);
    // Local movements (vx, vy, z)
    P = P + RZ * RY * RX * M;
}


// --------------------------------------------------------------------------
// Odometry::getPosition()
// Description  : Get estimated position
// Return value : None
// --------------------------------------------------------------------------
Mat Odometry::getPosition(){
    return P;
}

// --------------------------------------------------------------------------
// Odometry::getVx()
// Description  : Get vx
// Return value : None
// --------------------------------------------------------------------------
double Odometry::getVx(){
    return vx;
}
// --------------------------------------------------------------------------
// Odometry::getVx()
// Description  : Get vy
// Return value : None
// --------------------------------------------------------------------------
double Odometry::getVy(){
    return vy;
}
// --------------------------------------------------------------------------
// Odometry::getVx()
// Description  : Get vz
// Return value : None
// --------------------------------------------------------------------------
double Odometry::getVz(){
    return vz;
}
// --------------------------------------------------------------------------
// Odometry::getBattery()
// Description  : Debug. Check if drone is properly referenced
// Return value : None
// --------------------------------------------------------------------------
void Odometry::getBattery(){
    int battery = thisArdrone.getBatteryPercentage();
    printf("ardrone.battery = %d [%%]\n", battery);
}
// --------------------------------------------------------------------------
// Odometry::updateMap()
// Description  : Include new estimated position into map
// Return value : None
// --------------------------------------------------------------------------
void Odometry::updateMap(){
    double pos[2] = {P.at<double>(0,0), P.at<double>(1,0)};
    pos[1]=thisArdrone.getAltitude();
    circle(mapa, Point (-pos[1]*(double)30.0 + mapa.rows/2, -pos[0]*(double)30.0 + mapa.cols/2), 5, CV_RGB(255,0,0),1);
}
// --------------------------------------------------------------------------
// Odometry::getMap()
// Description  : Get current map
// Return value : None
// --------------------------------------------------------------------------
Mat Odometry::getMap(){
    return mapa;
}

// --------------------------------------------------------------------------
// Odometry::showPosition()
// Description  : Print on console current position
// Return value : None
// --------------------------------------------------------------------------
void Odometry::showPosition(){
    // Position (x, y, z)
    double pos[2] = {P.at<double>(0,0), P.at<double>(1,0)};
    printf("x = %3.2f, y = %3.2f\n", pos[0], pos[1]);
}
