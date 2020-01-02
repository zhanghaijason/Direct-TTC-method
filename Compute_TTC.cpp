#include <opencv/highgui.h>
//#include "opencv2/imgproc.hpp"
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <iostream>
#include <stdlib.h>
using namespace cv;
using namespace std;
//FILE *fp1;
//FILE *fp2;
double compute_ttc_new_v(Mat Im1, Mat Im2){
	Mat dst1;
	Mat dst2;
        Mat sIm1;
        Mat sIm2; 
//Used to check image pixel----OK 
	 //Gaussian Filter
/*	for (int i = 1; i < 5; i = i + 2)
	{
		GaussianBlur(Im1, dst1, Size(i, i), 0, 0);
		//if (display_dst(DELAY_BLUR) != 0) { return 0; }
	}
	for (int j = 1; j < 5; j = j + 2)
	{
		GaussianBlur(Im2, dst2, Size(j, j), 0, 0);
		//if (display_dst(DELAY_BLUR) != 0) { return 0; }
	}*/
	sIm1 = cv::Mat(Im1, cv::Rect(29, 0, 133, 70));
	sIm2 = cv::Mat(Im2, cv::Rect(29, 0, 133, 70));
        int Height = sIm1.rows;
        int Width = sIm1.cols;
        Mat smallIm1;
        Mat smallIm2;
        sIm1.convertTo(smallIm1,CV_64F);
        sIm2.convertTo(smallIm2,CV_64F);
	Mat Ix(Height-2, Width-2, DataType<double>::type);
	Mat Iy(Height - 2, Width - 2, DataType<double>::type);
	Mat It(Height - 2, Width - 2, DataType<double>::type);
	Mat G(Height - 2, Width - 2, DataType<double>::type);
	Mat A(3, 3, DataType<double>::type);
	Mat B(3, 1, DataType<double>::type);
	Mat C(3, 1, DataType<double>::type);
    Mat sumA00;Mat sumA01;Mat sumA02;Mat sumA10; Mat sumA11; Mat sumA12; Mat sumA20; Mat sumA21; Mat sumA22;
	Mat sumB00;Mat sumB10;Mat sumB20;
    int m=Height;
    int n=Width;
    Ix=(smallIm1(Range(2,m),Range(1,n-1))-smallIm1(Range(1,m-1),Range(1,n-1))+smallIm1(Range(2,m),Range(2,n))-smallIm1(Range(1,m-1),Range(2,n))+smallIm2(Range(2,m),Range(1,n-1))-smallIm2(Range(1,m-1),Range(1,n-1))+smallIm2(Range(2,m),Range(2,n))-smallIm2(Range(1,m-1),Range(2,n)))/4;
    Iy=(smallIm1(Range(1,m-1),Range(2,n))-smallIm1(Range(1,m-1),Range(1,n-1))+smallIm1(Range(2,m),Range(2,n))-smallIm1(Range(2,m),Range(1,n-1))+smallIm2(Range(1,m-1),Range(2,n))-smallIm2(Range(1,m-1),Range(1,n-1))+smallIm2(Range(2,m),Range(2,n))-smallIm2(Range(2,m),Range(1,n-1)))/4;
    It=(smallIm2(Range(1,m-1),Range(1,n-1))-smallIm1(Range(1,m-1),Range(1,n-1))+smallIm2(Range(2,m),Range(1,n-1))-smallIm1(Range(2,m),Range(1,n-1))+smallIm2(Range(1,m-1),Range(2,n))-smallIm1(Range(1,m-1),Range(2,n))+smallIm2(Range(2,m),Range(2,n))-smallIm1(Range(2,m),Range(2,n)))/4;
for (int a = 0; a < Height - 2; a++){
  for (int b = 0; b < Width - 2; b++) {
     G.at<double>(a, b)  = (b + 1)*Iy.at<double>(a, b) + (a + 1)*Ix.at<double>(a, b);
    }
}
	sumA00 = Ix.mul(Ix); sumA01= Iy.mul(Ix);sumA02= Ix.mul(G);sumA10= Ix.mul(Iy); sumA11= Iy.mul(Iy); sumA12= G.mul(Iy); sumA20= Ix.mul(G); sumA21 = Iy.mul(G); sumA22 = G.mul(G);
        sumB00 = Ix.mul(It); sumB10= Iy.mul(It); sumB20= G.mul(It);
        A.at<double>(0,0)=sum(sumA00)[0];A.at<double>(0,1)=sum(sumA01)[0];A.at<double>(0,2)=sum(sumA02)[0];
        A.at<double>(1,0)=sum(sumA10)[0];A.at<double>(1,1)=sum(sumA11)[0];A.at<double>(1,2)=sum(sumA12)[0];
        A.at<double>(2,0)=sum(sumA20)[0];A.at<double>(2,1)=sum(sumA21)[0];A.at<double>(2,2)=sum(sumA22)[0];
        B.at<double>(0,0)=sum(sumB00)[0];
        B.at<double>(1,0)=sum(sumB10)[0];
        B.at<double>(2,0)=sum(sumB20)[0];
        C = A.inv()*B;	  //Since we have xA = b, so (xA)^T = b^T <=> A^T * x^T = b^T, so we can use cv::solve(A.t(), b.t(), x), and x.t() is an result:
	return (-1/C.at<double>(2, 0));
}
int main(int argc, char** argv)
{       
  //      vector<Mat> image;
	/*string images[100];
	Mat img_raw1;
        Mat img_raw2;
	Mat TTC(99, 1, DataType<double>::type);
	for (int fileNumber = 1; fileNumber < 100; fileNumber++) //You should also check you have not more than MAX_NUM_OF_IMG
	{
	   stringstream ss;
           ss<<"\image"<<fileNumber<<"\.jpg"<<endl;
           string fullfileName;
           ss>>fullfileName;
           images[fileNumber]=fullfileName;}
         for (int file=1;file<100;file++){
          cout<<"Loading"<<images[file]<<endl;
          img_raw1=imread(images[file],0);
          img_raw2=imread(images[file+1],0);*/
               // double time1=getTickCount();
  //        printf("0 OK"); 
    //      for (int i=1;i<argc;i++)
//{   //      image[i-1]=imread(argv[i],0);
     //     image[i]  =imread(argv[i+1],0);
     //     printf("\n1 ok\n");
               
		Mat img1 = imread("image3.jpg");
		Mat img2 = imread("image4.jpg");
		Mat gray_image1;
		Mat gray_image2;
		cvtColor(img1, gray_image1, CV_BGR2GRAY);
        	cvtColor(img2, gray_image2, CV_BGR2GRAY);
		//imshow("tu", gray_image1);
		//imshow("t2", gray_image2);
		//waitKey(0);
	//	TTC.at<double>(file-1, 0) = compute_ttc_new_v(img_raw1, img_raw2);
//}  //                imshow("1",img1);
//                waitKey(0);
/*               int H=img1.rows;
                int W=img1.cols;
             fp1=fopen("matrix.txt","w");
            fp2=fopen("matrix2.txt","w");
            for(int m=0;m<H;m++){
               for(int n=0;n<W;n++){
                 printf("%d ",img1.at<uchar>(m,n));
                // fprintf(fp2,"%d ",gray_image2.at<uchar>(m,n));
}               printf("\n");
                 fprintf(fp1,";");
                 fprintf(fp2,";"); 
 }    
                printf("\n%d,%d\n",H,W);
               cout<<"Place -2 OK"<<endl;*/
               double time1=getTickCount(); 
               double ttc=compute_ttc_new_v(gray_image1,gray_image2);
//	}
/*	for (int i = 0; i < 99; i++){
		printf("%f ", TTC.at<double>(i, 0));
	}*/
        double time2=getTickCount();
        double consumed=(time2-time1)/double(getTickFrequency());
        printf("%f, time consumed:%f",ttc,consumed);
        return 0;
}						  
