
#include "spline.h"
#include <iostream>
#include <fstream>
using namespace std;

int main() {

	const int ROW = 61, VOL = 4;
	double d[ROW][VOL];
	ifstream in("Test.txt");//打开文件
							//读数据。。
	for (int i = 0; i < ROW; ++i) {
		for (int j = 0; j < VOL; ++j) {
			in >> d[i][j];
		}
	}
	in.close();//关闭文件
			   //输出结果
	//for (int i = 0; i < ROW; ++i) {
	//	for (int j = 0; j < VOL; ++j) {
	//		cout << d[i][j] << " ";
	//	}
	//	cout << endl;
	//}
	double deltaT = (0.18 - d[60][3])*0.5;
	std::vector<double> Acc(61), Vel(61), Pos(61), Time(61);
	for (int i = 0; i < ROW; ++i) {
		Pos[i] = d[i][2];
		if (i==0)
			Time[i] = d[i][3];
		else if (i==60)
			Time[i] = d[i][3] + 2*deltaT;
		else
			Time[i] = d[i][3]+ deltaT;
	}

	
	//double Acc[61], Vel[61], Pos[61], Time[61];
	

	
	//double a1_start = - 132, a1_end = -118, a2_start = -79, a2_end = -35;



	tk::spline s;
	s.set_boundary(s.first_deriv, 0.0, s.first_deriv, 0.0, false);
	s.set_points(Time,Pos);    // currently it is required that X is already sorted

	double xx = 0;
	int data_size = 61;
	for (int i = 0; i < data_size; i++) {
		xx = i*1.0 / (data_size-1) * Time[60];
		cout << xx << "  " << s(xx) << "  " << s.deriv(1,xx) << "  " << s.deriv(2, xx) << endl;

	}

	std::ofstream outFile;
	//打开文件  
	outFile.open("../../CubicSpline.txt");//ios::app 
	for (int i = 0; i < data_size; i++)
	{
		//写入数据
		xx = i*1.0 / (data_size - 1) * Time[60];
	   outFile << xx << "  " << s(xx) << "  " << s.deriv(1, xx) << "  " << s.deriv(2, xx) << endl;

	}
	//关闭文件
	outFile.close();
	//printf("spline at %f is %f\n", x, s(x));
	return 0;
}