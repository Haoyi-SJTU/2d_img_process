#include<opencv2/opencv.hpp>
#include<iostream>
#include<unistd.h>
#include<fstream>

#define N 20
//激光笔停留多久给予记录
using namespace std;
using namespace cv;

int CH(int a[],int n);
int ave(int a[],int n);
void file_write(char *stuid, char *name, float score);

int main(int argc,char **argv)
{
	VideoCapture capture(0);
	namedWindow("origin",CV_WINDOW_AUTOSIZE);
	ofstream OutFile("data.txt");
	while(1)
	{	
		int counter = 0;
		int pointx [N];
		int pointy [N];
		//每20个小循环
		//while(counter < N)
		//{	
			sleep(0.2);
			Mat imag;
			//原图
			Mat gray;
			//灰度图
			Mat binary;
			//二值图
			capture>>imag;                  //For test!!
			//imshow("hhh",imag);
			//imag = imread("/home/shy/catkin_ws/src/opencvtry/testphoto/123.jpg");//For test!!
			//需要设置为绝对路径
			cvtColor(imag,gray,CV_BGR2GRAY);
			medianBlur(gray, gray, 3);
			//中值滤波
			threshold(gray, binary, 250, 900, CV_THRESH_TOZERO);
			//二值化
			vector<Vec3f> cir;
			HoughCircles(binary, cir, CV_HOUGH_GRADIENT, 1, 50, 100, 10, 8, 50);
		
			//同心圆界限100像素，半径界限25-90
			double sum_x = 0, sum_y = 0, sum_r = 0;
			double x = 0, y = 0, r = 0;
			
			for (size_t i = 0; i < cir.size(); i++) 
			{
				if(cir[i][2] > 0)
				{
					//circle(imag, Point(cir[i][0], cir[i][1]), cir[i][2], Scalar(255, 0, 0), 2, 8);
					//std::cout << "position(" <<i<<cir[i][0]<<", "<<cir[i][1]<<") r="<<cir[i][2]<<endl;
				}
				sum_x = sum_x+cir[i][0];
				sum_y = sum_y+cir[i][1];
				sum_r = sum_r+cir[i][2];
			}
			x = sum_x / cir.size();
			y = sum_y / cir.size();
			r = sum_r / cir.size();
			if(r>0)
			{
				circle(imag, Point(x, y), r, Scalar(255, 0, 0), 2, 8);
				//求算数平均，对单激光点可用
				std::cout << "SUMposition(" <<x<<", "<<y<<") r="<<r<<endl;
				pointx[counter] = x;
				pointy[counter] = y;
				counter++;
			}
			else{std::cout << "cannot find laserpoint!!!"<<endl;}
			imshow("origin", imag);
			namedWindow("result", CV_WINDOW_NORMAL); 
			imshow("result", binary);
		//}
		//if(CH(pointx, N) < 100 && CH(pointy, N) < 100)
		//std::cout << "SUMposition(" <<ave(pointx, N)<<", "<<ave(pointy, N) << ")" <<endl;
		//OutFile <<ave(pointx, N)<<" "<<ave(pointy, N)<<endl;;
		//counter = 0;
		waitKey(30);
	}
	return 0;
}

//求差值
int CH(int a[],int n)
{
	int i;
	int max = a[0];
	int min = a[0];
	int c;
	for (i = 0; i < n; i++)
		{if(max < a[i])
			max = a[i];
		if(min > a[i])
			min = a[i];
		}
	return (max - min);
}
//求平均
int ave(int a[],int n)  
{  
	int sum=0;  
	for (int i=0;i<n;i++)  
		sum+=a[i];  
	return sum/n;  
}

void file_write(char *stuid,char *name,float score)
{

	int i;
	FILE *outfile;
	outfile = fopen("scores.txt","w");
	if(outfile==NULL)
	{
		printf("Can't open the file!\n");
	}
	fprintf(outfile,"学号\t姓名\t入学成绩\n");
	scanf("%s%s%f",stuid,name,&score);
	fprintf(outfile,"%s\t%s\t%f\n",stuid,name,score);
	fclose(outfile);
} 
