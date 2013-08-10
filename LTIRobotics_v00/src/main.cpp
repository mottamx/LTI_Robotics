#include "ardrone/ardrone.h"
#include <iostream>
#include <iomanip>
#include <time.h>


using namespace cv;
using namespace std;

void Sleep(unsigned long ms) {
    while (ms--) usleep(1000);
}

int main(){
	cout<<"Hol<"<<endl;

	//Mat img=imread("/home/mottamx/Pictures/batman.jpg");
	//namedWindow("batman");
	//imshow("batman", img);
	//waitKey(1);
	cout<<"Hol<<"<<endl;
	ARDrone ardrone;
	if (!ardrone.open()){
		cout<<"error de comunicación"<<endl;
		return -1;
	}
	cout<<"Bat: "<<ardrone.getBatteryPercentage()<<endl;
	ardrone.takeoff();
	sleep(2);
	time_t start, end;
	time (&start);
	cout.flush();
	double elapsed=0;
	namedWindow("dron");
	namedWindow("dron2");
	while(elapsed<15){
		//sleep(2);
		IplImage *im=ardrone.getImage();
		Mat img = Mat(im);
		Mat img2;
		resize(img, img2, Size(), 2,2, INTER_AREA);
		resize(img, img, Size(), 2,2, INTER_LANCZOS4);
		imshow("dron", img);
		waitKey(1);
		imshow("dron2", img2);
		waitKey(1);
		time(&end);
		elapsed=difftime(end,start);
	}
	cout<<"Tiempo: "<<setprecision(3)<<elapsed<<"segundos"<<endl;
	cout<<"<dios"<<endl;

	ardrone.landing();
	ardrone.close();
return 0;
}
