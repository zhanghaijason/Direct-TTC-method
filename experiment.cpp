/**********************************************************
 Software developed by AVA ( Ava Group of the University of Cordoba, ava  at uco dot es)
 Main author Rafael Munoz Salinas (rmsalinas at uco dot es)
 This software is released under BSD license as expressed below
-------------------------------------------------------------------
Copyright (c) 2013, AVA ( Ava Group University of Cordoba, ava  at uco dot es)
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:
1. Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.
3. All advertising materials mentioning features or use of this software
   must display the following acknowledgement:

   This product includes software developed by the Ava group of the University of Cordoba.

4. Neither the name of the University nor the names of its contributors
   may be used to endorse or promote products derived from this software
   without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AVA ''AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL AVA BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
****************************************************************/


#include <iostream>
#include <ctime>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include "raspicam_cv.h"
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <opencv/highgui.h>
//#include "opencv2/imgproc.hpp"
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;
bool doTestSpeedOnly=false;	
//parse command line
//returns the index of a command line param in argv. If not found, return -1
int findParam ( string param,int argc,char **argv ) {
    int idx=-1;
    for ( int i=0; i<argc && idx==-1; i++ )
        if ( string ( argv[i] ) ==param ) idx=i;
    return idx;

}
//parse command line
//returns the value of a command line param. If not found, defvalue is returned
float getParamVal ( string param,int argc,char **argv,float defvalue=-1 ) {
    int idx=-1;
    for ( int i=0; i<argc && idx==-1; i++ )
        if ( string ( argv[i] ) ==param ) idx=i;
    if ( idx==-1 ) return defvalue;
    else return atof ( argv[  idx+1] );
}

void processCommandLine ( int argc,char **argv,raspicam::RaspiCam_Cv &Camera ) {
    Camera.set ( CV_CAP_PROP_FRAME_WIDTH,  getParamVal ( "-w",argc,argv,192 ) );
    Camera.set ( CV_CAP_PROP_FRAME_HEIGHT, getParamVal ( "-h",argc,argv,144 ) );
    Camera.set ( CV_CAP_PROP_BRIGHTNESS,getParamVal ( "-br",argc,argv,50 ) );
    Camera.set ( CV_CAP_PROP_CONTRAST ,getParamVal ( "-co",argc,argv,50 ) );
    Camera.set ( CV_CAP_PROP_SATURATION, getParamVal ( "-sa",argc,argv,50 ) );
    Camera.set ( CV_CAP_PROP_GAIN, getParamVal ( "-g",argc,argv ,50 ) );
    Camera.set ( CV_CAP_PROP_EXPOSURE, getParamVal ( "-ss",argc,argv,50)  );
    if ( findParam ( "-gr",argc,argv ) !=-1 )
        Camera.set ( CV_CAP_PROP_FORMAT, CV_8UC1 );
    if ( findParam ( "-test_speed",argc,argv ) !=-1 )
        doTestSpeedOnly=true;
//    if ( findParam ( "-ss",argc,argv ) !=-1 )
//        Camera.set ( CV_CAP_PROP_EXPOSURE, getParamVal ( "-ss",argc,argv )  );
    if ( findParam ( "-wb_r",argc,argv ) !=-1 )
        Camera.set ( CV_CAP_PROP_WHITE_BALANCE_RED_V,getParamVal ( "-wb_r",argc,argv )     );
    if ( findParam ( "-wb_b",argc,argv ) !=-1 )
        Camera.set ( CV_CAP_PROP_WHITE_BALANCE_BLUE_U,getParamVal ( "-wb_b",argc,argv )     );
}

void showUsage() {
    cout<<"Usage: "<<endl;
    cout<<"[-gr set gray color capture]\n";
    cout<<"[-test_speed use for test speed and no images will be saved]\n";
    cout<<"[-w width] [-h height] \n[-br brightness_val(0,100)]\n";
    cout<<"[-co contrast_val (0 to 100)]\n[-sa saturation_val (0 to 100)]";
    cout<<"[-g gain_val  (0 to 100)]\n";
    cout<<"[-ss shutter_speed (0 to 100) 0 auto]\n";
    cout<<"[-wb_r val  (0 to 100),0 auto: white balance red component]\n";
    cout<<"[-wb_b val  (0 to 100),0 auto: white balance blue component]\n";

    cout<<endl;
}

double compute_ttc(Mat Im1, Mat Im2){
	Mat dst1;
	Mat dst2;
	Mat sIm1;
	Mat sIm2;
	//Gaussian Filter
	for (int i = 1; i < 3; i = i + 2)
	{
		GaussianBlur(Im1, dst1, Size(i, i), 0, 0);
	}
	for (int j = 1; j < 3; j = j + 2)
	{
		GaussianBlur(Im2, dst2, Size(j, j), 0, 0);
	}
	//crop the image
	//if (Num < 44){
	sIm1 = cv::Mat(dst1, cv::Rect(29, 0, 133, 70));//(29,0,133,70)(19,0,60,35)
	sIm2 = cv::Mat(dst2, cv::Rect(29, 0, 133, 70));
	//}
	int Height = sIm1.rows;
	int Width = sIm2.cols;
	Mat smallIm1;
	Mat smallIm2;
	sIm1.convertTo(smallIm1, CV_64F);
	sIm2.convertTo(smallIm2, CV_64F);
        //defined all needed matrix
	Mat Ix(Height - 2, Width - 2, DataType<double>::type);
	Mat Iy(Height - 2, Width - 2, DataType<double>::type);
	Mat It(Height - 2, Width - 2, DataType<double>::type);
	Mat G(Height - 2, Width - 2, DataType<double>::type);
	Mat A(3, 3, DataType<double>::type);
	Mat B(3, 1, DataType<double>::type);
	Mat C(3, 1, DataType<double>::type);
	Mat sumA00; Mat sumA01; Mat sumA02; Mat sumA10; Mat sumA11; Mat sumA12; Mat sumA20; Mat sumA21; Mat sumA22;
	Mat sumB00; Mat sumB10; Mat sumB20;
	int m = Height;
	int n = Width;
	Ix = (smallIm1(Range(2, m), Range(1, n - 1)) - smallIm1(Range(1, m - 1), Range(1, n - 1)) + smallIm1(Range(2, m), Range(2, n)) - smallIm1(Range(1, m - 1), Range(2, n)) + smallIm2(Range(2, m), Range(1, n - 1)) - smallIm2(Range(1, m - 1), Range(1, n - 1)) + smallIm2(Range(2, m), Range(2, n)) - smallIm2(Range(1, m - 1), Range(2, n))) / 4;
	Iy = (smallIm1(Range(1, m - 1), Range(2, n)) - smallIm1(Range(1, m - 1), Range(1, n - 1)) + smallIm1(Range(2, m), Range(2, n)) - smallIm1(Range(2, m), Range(1, n - 1)) + smallIm2(Range(1, m - 1), Range(2, n)) - smallIm2(Range(1, m - 1), Range(1, n - 1)) + smallIm2(Range(2, m), Range(2, n)) - smallIm2(Range(2, m), Range(1, n - 1))) / 4;
	It = (smallIm2(Range(1, m - 1), Range(1, n - 1)) - smallIm1(Range(1, m - 1), Range(1, n - 1)) + smallIm2(Range(2, m), Range(1, n - 1)) - smallIm1(Range(2, m), Range(1, n - 1)) + smallIm2(Range(1, m - 1), Range(2, n)) - smallIm1(Range(1, m - 1), Range(2, n)) + smallIm2(Range(2, m), Range(2, n)) - smallIm1(Range(2, m), Range(2, n))) / 4;
	for (int a = 0; a < Height - 2; a++){
		for (int b = 0; b < Width - 2; b++) {
			G.at<double>(a, b) = (b + 1)*Iy.at<double>(a, b) + (a + 1)*Ix.at<double>(a, b);
		}
	}
	sumA00 = Ix.mul(Ix); sumA01 = Iy.mul(Ix); sumA02 = Ix.mul(G); sumA10 = Ix.mul(Iy); sumA11 = Iy.mul(Iy); sumA12 = G.mul(Iy); sumA20 = Ix.mul(G); sumA21 = Iy.mul(G); sumA22 = G.mul(G);
	sumB00 = Ix.mul(It); sumB10 = Iy.mul(It); sumB20 = G.mul(It);
	A.at<double>(0, 0) = sum(sumA00)[0]; A.at<double>(0, 1) = sum(sumA01)[0]; A.at<double>(0, 2) = sum(sumA02)[0];
	A.at<double>(1, 0) = sum(sumA10)[0]; A.at<double>(1, 1) = sum(sumA11)[0]; A.at<double>(1, 2) = sum(sumA12)[0];
	A.at<double>(2, 0) = sum(sumA20)[0]; A.at<double>(2, 1) = sum(sumA21)[0]; A.at<double>(2, 2) = sum(sumA22)[0];
	B.at<double>(0, 0) = sum(sumB00)[0];
	B.at<double>(1, 0) = sum(sumB10)[0];
	B.at<double>(2, 0) = sum(sumB20)[0];
	C = A.inv()*B;	  //Since we have xA = b, so (xA)^T = b^T <=> A^T * x^T = b^T, so we can use cv::solve(A.t(), b.t(), x), and x.t() is an result:
	return (-1 / C.at<double>(2, 0));
}
// Compute velocity command
int compute_velocity(double t, int v){
	double timetocontact=65;
	int new_speed;
//	float error;
        float kp;
//        float kp1=0.3;
//        float kp2=0.4;
//	error = float(timetocontact-t);
//       if(error>0){
	//sum_error = sum_error + error;
//	new_speed = -int(Kp1*error) +v;}
//        else{ new_speed = -int(Kp2*error) +v;}
        //Kd(error - lasterror) + Ki*sum_error);
	//last_error = error;
        // gain scheduling
        if (t>=70&&t<80) {kp=0.2;}
        else if (t>=80&&t<100) {kp=0.3;}
        else if (t>=100&&t<110){kp=0.4;}
        else if (t>=110)       {kp=0.5;}
        else if (t>=60&&t<70)  {kp=0.1;}
        else if (t>=50&&t<60)  {kp=0.2;}
        else if (t>=40&&t<50)  {kp=0.3;}
        else {kp=0.4;}
        new_speed=v+kp*(t-timetocontact);
        if (new_speed>180){new_speed=180;}
        if (new_speed<=0){new_speed=0;}
	return new_speed;
}



int main ( int argc,char **argv ) {
	int fd;
	char begin;
        char speed_act;
        char stop=0+'0';
        double ttc_last;
	/*float error;
	float sum_error=0;
	float lasterror=0;*/
	// Below for wiringPi and Serial
	if ((fd = serialOpen("/dev/ttyAMA0", 9600)) < 0)
	{
		fprintf(stderr, "Unable to open serial device: %s\n", strerror(errno));
		return 1;
	}

	if (wiringPiSetup() == -1)
	{
		fprintf(stdout, "Unable to start wiringPi: %s\n", strerror(errno));
		return 1;
	}
    if ( argc==1 ) {
        cerr<<"Usage (-help for help)"<<endl;
    }
    if ( findParam ( "-help",argc,argv ) !=-1 ) {
        showUsage();
        return -1;
    }
	begin = 60 + '0';//begin arduino
	fflush(stdout);
	serialPutchar(fd, begin);
    raspicam::RaspiCam_Cv Camera;
    Camera.set (CV_CAP_PROP_FPS, 90);
    processCommandLine ( argc,argv,Camera );
    cout<<"Connecting to camera"<<endl;
    if ( !Camera.open() ) {
        cerr<<"Error opening camera"<<endl;
        return -1;
    }
    cout<<"Connected to camera ="<<Camera.getId() <<endl;
    int m=0;
    FILE *fp;
    FILE *fp2;
    Mat image;
    vector<Mat> Concat;
    Mat gray_image1; //modified
    Mat gray_image2; //modified
    double ttc; 
    int velocity=60;
    double time1=cv::getTickCount();
    double time2;
    double time_interval;
    double time3;
    time3=time1;
//    Camera.release();
    cout<<"Capturing"<<endl;
    fp=fopen("TTC_data.txt","w");
    fp2=fopen("Speed.txt","w");
    if (fp==NULL)
      exit(-1);    
    for ( int i=0;i<200 ; i++ ) {
	for (;;){
		time2 = cv::getTickCount();
		time_interval = (time2 - time1) / double(cv::getTickFrequency());
			//wait for robot move steadyly
		if (i == 0){
			delay(0.1);
			time1 = time2;
			break;
			}
                else break;
			// do computation and image taking every 30ms
		/*if (time_interval > 0.09){
			time1 = time2;
			break;
			}*/
		}
        //if (velocity<10){break;}
        Camera.grab();
		if (i == 0){
                        Camera.grab();
			Camera.retrieve(image);
                        Concat.push_back(image.clone());
			cvtColor(image, gray_image1, CV_RGB2GRAY);
		}
		if (i == 1){
			Camera.retrieve(image);
                        Concat.push_back(image.clone());
			cvtColor(image, gray_image2, CV_RGB2GRAY);
			ttc=compute_ttc(gray_image1, gray_image2);
                        if((ttc>0)&&(ttc<300)){
			velocity = compute_velocity(ttc, velocity);
                        ttc_last=ttc;}
		}
		if (i >= 2){
			Camera.retrieve(image);
                        Concat.push_back(image.clone());
			gray_image1 = gray_image2.clone();
			cvtColor(image, gray_image2, CV_RGB2GRAY);
			ttc=compute_ttc(gray_image1, gray_image2);
                        if((ttc>0)&&(ttc<500)&&(abs(ttc-ttc_last)<60)){
			velocity=compute_velocity(ttc,velocity);}
                        ttc_last=ttc;
		}
                printf("Number %d image\n ttc:%f\n speed:%d\n",m,ttc,velocity);
                fprintf(fp,"%f\n",ttc);
                fprintf(fp2,"%d\n",velocity);
                //ttc_last=ttc;
                m++;
                speed_act=velocity+'0';
//                speed_act=120+'0';
		fflush(stdout);
		serialPutchar(fd, speed_act);
                if(velocity<=35){break;}
    }   
        fflush(stdout);
        serialPutchar(fd, stop);
        fclose(fp);
        fclose(fp2);
        cout<<"\n Total Time:%f"<<((getTickCount()-time3)/double(getTickFrequency()))<<endl;

        cout<<"Print Images\n"<<endl;
        for (int j=0;j<m+1;j++){
         imwrite("image"+std::to_string(j)+".jpg",Concat[j]);
   }    
        cout<<"\n Total Time:%f"<<((getTickCount()-time3)/double(getTickFrequency()))<<endl;
        fflush(stdout);
        serialPutchar(fd, stop);
        Camera.release(); 
	return 0;
}
