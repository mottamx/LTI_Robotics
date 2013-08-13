// -------------------------------------------------------------------------
// Odometria para ARDrone
// Empleando matrices de rotacion y velocidades
// Carlos Alberto Motta Avila
// for LTI Cinvestav - Robotics
// 2013
// -------------------------------------------------------------------------


#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "ardrone.h"
#include <opencv2/opencv.hpp>

using namespace cv;

#define PI 3.14159265

class Odometry{

	//Para leer sensores del VANT
	ARDrone& thisArdrone;
	//Angulos de navegacion
	double pitch; 	//Cabeceo
	double roll; 	//Alabeo
	double yaw;		//Guinada
	//Velocidades
	double vx, vy, vz; //vz es 0
	double velocity;
	//Matrices de rotacion
	Mat RX; //(3,3,CV_64FC1)
	Mat RY; //(3,3,CV_64FC1)
	Mat RZ; //(3,3,CV_64FC1)
	//Matriz posición
	Mat P; //(3,1,CV_64FC1)

	//Mapa
	Mat mapa;

public:

	Odometry();
	Odometry(ARDrone& ardrone):thisArdrone(ardrone){};

	static int64 last; //Tiempo
	void getVelFromDrone();		//Velocidades
	void getAnglesFromDrone(); 	//Angulos de navegacion
	void deadReckon(); 			//Calcula coordenadas
	void Init(); 	//Llenar de zeros matrices
	void resetP(const double& altura); //Reiniciar origen

	//Getters
	void showPosition();
	Mat getPosition();
	double getVx();
	double getVy();
	double getVz();
	void getBattery();

	//Funciones revisadas
	//int initState(MarkerWrap& detMark, camCalib& undis, double altInit,float distance);
	//int initStateV2(MarkerWrap& detMark, camCalib& undis, double altInit,float distance);
	void updateMap();
	Mat getMap();

};
#endif //ODOMETRY_H
