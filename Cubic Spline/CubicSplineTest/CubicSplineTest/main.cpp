
#include "spline.h"
#include <iostream>
#include <fstream>
#include <math.h>
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
	double tinyT = (0.18 - d[60][3])*0.5; //首尾时间加长
	tinyT = 0;

	int sz = 61;
	std::vector<double> Acc(sz), Vel(sz), Pos(sz), Time(sz);
	for (int i = 0; i < ROW; ++i) {
		Pos[i] = d[i][2];
		if (i==0)
			Time[i] = d[i][3];
		else if (i==sz-1)
			Time[i] = d[i][3] + 2*tinyT;
		else
			Time[i] = d[i][3]+ tinyT;
	}
	
	
	
	//double a1_start = - 132, a1_end = -118, a2_start = -79, a2_end = -35;



	tk::spline s;
	s.set_boundary(s.first_deriv, 0.0, s.first_deriv, 0.0, false);
	s.set_points(Time,Pos);    // currently it is required that X is already sorted


	
	const int order =8;
	double xx = 0;
	int data_size = ceil(Time[sz-1]*1000);
	std::vector<double> re_Vel(data_size + order,0),
		re_Pos(data_size + order, 0);
	
	std::vector<double> filter_Vel(data_size + order, 0), 
		filter_Pos(data_size + order, 0),
		filter_Pos2(data_size + order, 0),  
		filter_Acc(data_size + order, 0);
	double sum_vel = 0.0, coeff = 1.0 / (order+1);
	double total_T = Time[60], deltaT = total_T / (data_size - 1);


	for (int i = 0; i < data_size; i++) {
		xx = i*1.0 / (data_size-1) * Time[60];
		re_Vel[i] = s.deriv(1, xx);//every 1ms cycle vel
		if (i == 0)
			re_Pos[i] = 0;
		else
			//re_Pos[i] = re_Pos[i - 1] + (re_Vel[i]+ re_Vel[i-1]) / 2 * deltaT;
			re_Pos[i] = re_Pos[i - 1] + re_Vel[i]* deltaT;
		//再计算平均速度
		//if (i > 0)
		//	re_Vel[i] = (re_Vel[i]+ re_Vel[i-1]) / 2;
		//cout << xx << "  " << s(xx) << "  " << s.deriv(1,xx) << "  " << s.deriv(2, xx) << endl;

	}	

	
	for (int i = 0; i < data_size + order; i++)  //计算滤波速度
	{
		if (i < order) {
			for (int j = 0; j <= i; j++) {
				sum_vel += re_Vel[j];
			}

		}
		else
			for (int j = i; j >= i - order; j--) {
				sum_vel += re_Vel[j];
			}

		filter_Vel[i] = coeff*sum_vel; //滤波速度
		sum_vel = 0.0;
	}
	
	for (int i = 0; i < data_size + order; i++) { //计算滤波后的位置、加速度
		if (i == 0)
			filter_Pos[i] = 0;
		else {
			filter_Pos[i] = filter_Pos[i - 1] + (filter_Vel[i - 1] + filter_Vel[i]) / 2 * deltaT;
			filter_Pos2[i] = filter_Pos2[i - 1] + filter_Vel[i] * deltaT;
		}

		if (i < data_size + order - 1)
			filter_Acc[i] = (filter_Vel[i + 1] - filter_Vel[i])/deltaT;
		else
			filter_Acc[i] = filter_Acc[i-1];
		cout << i << "  " << filter_Pos[i] << "  " << filter_Pos2[i] << "  " << filter_Vel[i] << "  " << filter_Acc[i] << endl;
	}

	std::ofstream outFile;
	//打开文件  
	outFile.open("../../CubicSpline.txt");//ios::app 
	for (int i = 0; i < data_size+order; i++)
	{
		//写入数据
		//xx = i*1.0 / (data_size - 1) * Time[60];
	   //outFile << xx << "  " << s(xx) << "  " << s.deriv(1, xx) << "  " << s.deriv(2, xx) << endl;
	   outFile<< i << "  " << filter_Pos[i] << "  " << filter_Vel[i] << "  " << filter_Acc[i] << endl;
	}
	//关闭文件
	outFile.close();
	//printf("spline at %f is %f\n", x, s(x));
	return 0;
}