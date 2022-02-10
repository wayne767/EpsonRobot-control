#include "stdafx.h"
#include <math.h>
#include <string>
#include "stdafx.h"
#include <stdlib.h>
#include "mymain.h"
#include <windows.h>
#include <sstream>
#include <iostream>
#include <fstream>
#include <iomanip> 
#include <vector>
#include <unisted.h>

using namespace std;
using namespace robot;
using namespace SpelNetLib;

double crossproduct(double in1[3], double in2[3], double out[3])
{
	out[0] = (in1[1] * in2[2]) - (in1[2] * in2[1]);
	out[1] = (in1[2] * in2[0]) - (in1[0] * in2[2]);
	out[2] = (in1[0] * in2[1]) - (in1[1] * in2[0]);
	return(0);
}

void point_offset(double v1[3], double v2[3], double ray_dir[3], double ray_point[3])
{
	double temp[3];
	temp[0] = 0; temp[1] = 0; temp[2] = 0;
	for (int l = 0; l<3; l++)
	{
		ray_point[l] = v1[l];
		ray_dir[l] = v2[l];
	}
	if (ray_dir[0] != temp[0])
		ray_point[0] = ray_point[0] + ray_dir[0] * 50;
	if (ray_dir[1] != temp[1])
		ray_point[1] = ray_point[1] + ray_dir[1] * 50;
	if (ray_dir[2] != temp[2])
		ray_point[2] = ray_point[2] + ray_dir[2] * 50;
}

void MarshalString(System::String^  s, std::string& os) {

	using namespace Runtime::InteropServices;

	char* chars =
		(char*)(Marshal::StringToHGlobalAnsi(s)).ToPointer();

	os = chars;
}

System::Void mymain::mymain_Load(System::Object^  sender, System::EventArgs^  e)
{
	m_spel = gcnew SpelNetLib::Spel();
	try
	{
		m_spel->Initialize();
	}
	catch (SpelException ^ex)
	{
		MessageBox::Show(ex->Message);
	}
	m_spel->Project = "c:\\EPSONRC50\\projects\\test\\test.sprj";
	m_spel->Homeset(0, 0, 0, 0, 0, 0);
	m_spel->EventReceived += gcnew SpelNetLib::Spel::EventReceivedEventHandler(this, &mymain::m_spel_EventReceived);
	
}

System::Void mymain::get_detect_project(float angle, float dis_x, float dis_y)
{
}

System::Void mymain::MyGo(float x, float y, float z, float u, float v, float w)
{
	SpelPoint^ mypoint = gcnew SpelPoint();
	mypoint->X = x;
	mypoint->Y = y;
	mypoint->Z = z;
	mypoint->U = u;
	mypoint->V = v;
	mypoint->W = w;
	mymain::m_spel->Go(mypoint);
}

//------------------------------------------------------------------------------------------------------------------------
System::Void mymain::mymain_FormClosed(System::Object^  sender, System::Windows::Forms::FormClosedEventArgs^  e)
{
	delete m_spel;
}

System::Void mymain::btnStart_Click(System::Object^  sender, System::EventArgs^  e)
{
	m_spel->MotorsOn = true;
	m_spel->PowerHigh = true;
}

System::Void mymain::btnStop_Click(System::Object^  sender, System::EventArgs^  e)
{
	try
	{
		m_spel->Stop();
	}
	catch (SpelException ^ex)
	{
		System::String^ error_text = gcnew System::String(ex->Message);
		ERROR_lbl->Text = error_text;
	}
}

System::Void mymain::btnPause_Click(System::Object^  sender, System::EventArgs^  e)
{
	try
	{
		m_spel->Pause();
	}
	catch (SpelException ^ex)
	{
		System::String^ error_text = gcnew System::String(ex->Message);
		ERROR_lbl->Text = error_text;
	}
}

System::Void mymain::btnCotinue_Click(System::Object^  sender, System::EventArgs^  e)
{
	try
	{
		m_spel->Continue();
	}
	catch (SpelException ^ex)
	{
		System::String^ error_text = gcnew System::String(ex->Message);
		ERROR_lbl->Text = error_text;
	}
}

System::Void mymain::btnRun_Click(System::Object^  sender, System::EventArgs^  e)
{
	ifstream filein3;
	int o;
	filein3.open("E:\\賴以衛\\研究用\\CODE\\pointnet.pytorch-master\\utils\\File_number.txt", ios::in);//零件數量
	while (!filein3.eof())
	{
		filein3 >> o;
	}
	for (int c = 0; c < o; c++)
	{
#define SIZE 62500
#define PI 3.14159265359
	int time = 1, position1, position2, i = 0, j = 0, matrix_sz, position_min, position3, swap_tmp, ditrction_matrix_sz, p = 0, check_number_count = 0;
	double X[3] = { 1,0,0 }, Y[3] = { 0,1,0 };
	double  angle_2_pnt, min_dist, angle, u, cod_dist_tmp, w, v, temp_check_number, check_direction_angle;
	double pnt_info[SIZE], vector_temp[3], vector_temp2[3], vector_two_pnt[3], chosen_mid[3], final_pnt[3], ditrction_info[SIZE], vector_temp3[3];
	double COD[5];
	double Pye[3], Pxtemp[3], Pxe[3], Pz[3], Wy[3] = { 0,1,0 }, Py[3], final_mid[3], check_normal[3];
	double stamdard_y_position, grap_y_position;
	SpelPoint^ localpoint = gcnew SpelPoint();		//設定大地座標
	time_t begin, end;
	begin = clock();
	localpoint->X = 0;
	localpoint->Y = 0;
	localpoint->Z = 0;
	localpoint->U = 180;
	localpoint->V = 0;
	localpoint->W = 90;
	m_spel->TLSet(3, 0, 0, 130, 0, 0, 0);
	m_spel->Tool(3);//使用tool1
	m_spel->Local(3, localpoint);//使用local1

	SpelPoint^ temp_point = gcnew SpelPoint();		//設定temp_point
	

		//--------------------------夾取點TXT資料匯入-----------------------------------------------	
		ifstream filein;
		filein.open("E:\\賴以衛\\研究用\\CODE\\pointnet.pytorch-master\\utils\\trans_data_" + std::to_string(c) + ".txt", ios::in);
		while (!filein.eof())
		{
			filein >> pnt_info[i];
			i++;
		}
		matrix_sz = i;
		//--------------------------檢查方向TXT資料匯入-----------------------------------------------	
		ifstream filein1;
		filein1.open("E:\\賴以衛\\研究用\\CODE\\pointnet.pytorch-master\\utils\\trans_diretion_" + std::to_string(c) + ".txt", ios::in);
		while (!filein1.eof())
		{
			filein1 >> ditrction_info[p];
			p++;
		}
		ditrction_matrix_sz = p;
		//---------------------------------重心點座標匯入-------------------------------------------
		ifstream filein2;
		filein2.open("E:\\賴以衛\\研究用\\CODE\\pointnet.pytorch-master\\utils\\trans_cod_" + std::to_string(c) + ".txt", ios::in);
		while (!filein2.eof())
		{
			filein2 >> COD[j];
			j++;
		}

		//---------------------------將匯入檔案放進vector中-----------------------------------------
		std::vector<double>pntinfoX;
		std::vector<double>pntinfoY;
		std::vector<double>pntinfoZ;
		std::vector<double>check_number;
		std::vector<double>planeToplane;
		std::vector<double>planeTocamber;
		std::vector<double>camberTocamber;
		std::vector<double>position;
		std::vector<double>position_number;
		std::vector<double>finalposition;
		std::vector<double>checkvector_X;
		std::vector<double>checkvector_Y;
		std::vector<double>checkvector_Z;
		std::vector<double>direction_check_number;
		std::vector<double>direction_angle;
		std::vector<double>tenp_direction_angle;
		for (int n = 0; n < matrix_sz - 1; n = n + 4)//夾取點
		{
			pntinfoX.push_back(pnt_info[n]);
			pntinfoY.push_back(pnt_info[n + 1]);
			pntinfoZ.push_back(pnt_info[n + 2]);
			check_number.push_back(pnt_info[n + 3]);
		}
		for (int n = 0; n < ditrction_matrix_sz - 1; n = n + 4) //檢查方向
		{
			checkvector_X.push_back(ditrction_info[n]);
			checkvector_Y.push_back(ditrction_info[n + 1]);
			checkvector_Z.push_back(ditrction_info[n + 2]);
			direction_check_number.push_back(ditrction_info[n + 3]);
		}

		//-----------------find where to commence work devidedly------------------------
		//---------------------------計算與指尖距離及與Z軸角度，值丟入VECTOR--------------------------



		for (int n = 2; n < pntinfoY.size(); n = n + 3)
		{
			double aaa = check_number[n - 1];
			if (check_number[n - 1] != 0)
			{
				double x1, x2, y1, y2, z1, z2;
				x1 = pntinfoX[n - 1];
				x2 = pntinfoX[n - 2];
				y1 = pntinfoY[n - 1];
				y2 = pntinfoY[n - 2];
				z1 = pntinfoZ[n - 1];
				z2 = pntinfoZ[n - 2];
				vector_temp2[0] = pntinfoX[n - 1] - pntinfoX[n - 2]; //求得夾取點與Y軸夾角
				vector_temp2[1] = pntinfoY[n - 1] - pntinfoY[n - 2];
				vector_temp2[2] = pntinfoZ[n - 1] - pntinfoZ[n - 2];
				//normalize the vector_temp2 單位向量
				vector_two_pnt[0] = vector_temp2[0] / sqrt(pow(vector_temp2[0], 2) + pow(vector_temp2[1], 2) + pow(vector_temp2[2], 2));
				vector_two_pnt[1] = vector_temp2[1] / sqrt(pow(vector_temp2[0], 2) + pow(vector_temp2[1], 2) + pow(vector_temp2[2], 2));
				vector_two_pnt[2] = vector_temp2[2] / sqrt(pow(vector_temp2[0], 2) + pow(vector_temp2[1], 2) + pow(vector_temp2[2], 2));
				angle_2_pnt = (180 / 3.14159)*acos(vector_two_pnt[1]); //下爪方向與Y軸夾角
				temp_check_number = check_number[n - 1];//紀錄暫時的編號
				check_number_count = 0;
				for (int nn = 0; nn < direction_check_number.size(); nn++)
				{
					double hh = direction_check_number[nn];
					if (direction_check_number[nn] == temp_check_number)
					{
						check_normal[0] = checkvector_X[nn];  //檢測方向的向量
						check_normal[1] = checkvector_Y[nn];
						check_normal[2] = checkvector_Z[nn];
						check_direction_angle = (180 / 3.14159)*acos(check_normal[1]); //求得檢測向量與Y軸夾角
						if (angle_2_pnt<(check_direction_angle + 15) && angle_2_pnt>(check_direction_angle - 15))
						{
							check_number_count = 0;
							break;
						}
						else
						{
							check_number_count++; //紀錄是否有通過檢查
						}
					}
				}
				if (check_number_count > 0)
				{
					if (pntinfoY[n - 1] > pntinfoY[n - 2])
					{
						grap_y_position = pntinfoY[n - 2];
					}
					else
					{
						grap_y_position = pntinfoY[n - 1];
					}
					if (grap_y_position > (stamdard_y_position - 0.5) && grap_y_position < (stamdard_y_position + 0.5))  //檢查夾取點Y值是否與桌面貼齊
					{

					}
					else
					{
						if (angle_2_pnt <= 95 && angle_2_pnt > 85) //allow angle btwn 80-90 degree 
						{
							position.push_back(n);
						}
					}
				}
			}
			else
			{
				double x1, x2, y1, y2, z1, z2;
				x1 = pntinfoX[n - 1];
				x2 = pntinfoX[n - 2];
				y1 = pntinfoY[n - 1];
				y2 = pntinfoY[n - 2];
				z1 = pntinfoZ[n - 1];
				z2 = pntinfoZ[n - 2];
				//vector of two gripping points
				vector_temp2[0] = pntinfoX[n - 1] - pntinfoX[n - 2];
				vector_temp2[1] = pntinfoY[n - 1] - pntinfoY[n - 2];
				vector_temp2[2] = pntinfoZ[n - 1] - pntinfoZ[n - 2];
				//normalize the vector_temp2
				vector_two_pnt[0] = vector_temp2[0] / sqrt(pow(vector_temp2[0], 2) + pow(vector_temp2[1], 2) + pow(vector_temp2[2], 2));
				vector_two_pnt[1] = vector_temp2[1] / sqrt(pow(vector_temp2[0], 2) + pow(vector_temp2[1], 2) + pow(vector_temp2[2], 2));
				vector_two_pnt[2] = vector_temp2[2] / sqrt(pow(vector_temp2[0], 2) + pow(vector_temp2[1], 2) + pow(vector_temp2[2], 2));
				angle_2_pnt = (180 / 3.14159)*acos(vector_two_pnt[1]);//angle btwn Z-axis and vector_two_pnt
				if (pntinfoY[n - 1] > pntinfoY[n - 2])
				{
					grap_y_position = pntinfoY[n - 2];
				}
				else
				{
					grap_y_position = pntinfoY[n - 1];
				}
				if (grap_y_position > (stamdard_y_position - 0.5) && grap_y_position < (stamdard_y_position + 0.5))  //檢查夾取點Y值是否與桌面貼齊
				{

				}
				else
				{
					if (angle_2_pnt <= 95 && angle_2_pnt > 85) //allow angle btwn 80-90 degree 
					{
						position.push_back(n);
					}
				}
			}
		}

		//--------------------------------------------found the proper answer------------------------------------------------
		if (position.size() > 0)
		{
			//sorting out the sequence from small to large
			for (int i = 0; i < position.size(); i++)
			{
				for (int j = 0; j < position.size() - i - 1; j++)
				{
					if (pntinfoY[position[j]] < pntinfoY[position[j + 1]])
					{
						swap_tmp = position[j];
						position[j] = position[j + 1];
						position[j + 1] = swap_tmp;
					}
				}
			}

			//take out 12
			if (position.size() >= 5)
			{
				for (int i = 0; i < 5; i++)
				{
					finalposition.push_back(position[i]);
				}
			}
			else
			{
				for (int i = 0; i < position.size(); i++)
				{
					finalposition.push_back(position[i]);
				}
			}

			//find the closest one to COD
			min_dist = sqrt(pow(pntinfoX[finalposition[0]] - COD[0], 2) + pow(pntinfoY[finalposition[0]] - COD[1], 2) + pow(pntinfoZ[finalposition[0]] - COD[2], 2));
			for (int i = 0; i < finalposition.size(); i++)
			{
				cod_dist_tmp = sqrt(pow(pntinfoX[finalposition[i]] - COD[0], 2) + pow(pntinfoY[finalposition[i]] - COD[1], 2) + pow(pntinfoZ[finalposition[i]] - COD[2], 2));
				if (cod_dist_tmp <= min_dist)
				{
					min_dist = cod_dist_tmp;
					position3 = i;
				}
			}
			int nnn = finalposition[position3];

			position_min = finalposition[position3];

		}
		cout << position_min << endl;
		chosen_mid[0] = pntinfoX[position_min];
		chosen_mid[1] = pntinfoY[position_min];
		chosen_mid[2] = pntinfoZ[position_min];
		point_offset(chosen_mid, Y, Y, final_mid);
		end = clock();
		double Times = double(end - begin) / CLOCKS_PER_SEC; //將clock()函數的結果轉化爲以秒爲單位的量
		FILE *pFile;
		int arrSize;
		pFile = fopen("time_robot.txt", "a");
		fprintf(pFile, "%f \n", Times);
		fclose(pFile);

		//----------------------------------find the 6th & 5th axis turning angle---------------------------------------------------------
		if (pntinfoX[position_min - 2] < pntinfoX[position_min - 1])
		{
			Py[0] = pntinfoX[position_min - 2] - pntinfoX[position_min - 1];
			Py[1] = pntinfoY[position_min - 2] - pntinfoY[position_min - 1];
			Py[2] = pntinfoZ[position_min - 2] - pntinfoZ[position_min - 1];
		}
		else if (pntinfoX[position_min - 2] > pntinfoX[position_min - 1])
		{
			Py[0] = pntinfoX[position_min - 1] - pntinfoX[position_min - 2];
			Py[1] = pntinfoY[position_min - 1] - pntinfoY[position_min - 2];
			Py[2] = pntinfoZ[position_min - 1] - pntinfoZ[position_min - 2];
		}
		double a = pntinfoZ[position_min - 1];
		double b = pntinfoZ[position_min - 2];
		double aa = pntinfoY[position_min - 1];
		double bb = pntinfoY[position_min - 2];
		double aaa = pntinfoX[position_min - 1];
		double bbb = pntinfoX[position_min - 2];
		Py[0] = (-416.571247 + 433.644715);
		Py[1] = (-13.461739 + 13.014872);
		Py[2] = (-34.845667 + 24.429504);
		//normalize the Py
		Pye[0] = Py[0] / sqrt(pow(Py[0], 2) + pow(Py[1], 2) + pow(Py[2], 2));
		Pye[1] = Py[1] / sqrt(pow(Py[0], 2) + pow(Py[1], 2) + pow(Py[2], 2));
		Pye[2] = Py[2] / sqrt(pow(Py[0], 2) + pow(Py[1], 2) + pow(Py[2], 2));
		//Wz cross Pye
		Pxtemp[0] = Wy[1] * Pye[2] - Wy[2] * Pye[1];
		Pxtemp[1] = Wy[2] * Pye[0] - Wy[0] * Pye[2];
		Pxtemp[2] = Wy[0] * Pye[1] - Wy[1] * Pye[0];
		//normalize the Pxtemp
		Pxe[0] = Pxtemp[0] / sqrt(pow(Pxtemp[0], 2) + pow(Pxtemp[1], 2) + pow(Pxtemp[2], 2));
		Pxe[1] = Pxtemp[1] / sqrt(pow(Pxtemp[0], 2) + pow(Pxtemp[1], 2) + pow(Pxtemp[2], 2));
		Pxe[2] = Pxtemp[2] / sqrt(pow(Pxtemp[0], 2) + pow(Pxtemp[1], 2) + pow(Pxtemp[2], 2));
		//Pxe cross Pye
		Pz[0] = Pxe[1] * Pye[2] - Pxe[2] * Pye[1];
		Pz[1] = Pxe[2] * Pye[0] - Pxe[0] * Pye[2];
		Pz[2] = Pxe[0] * Pye[1] - Pxe[1] * Pye[0];
		//angle conditions
		double vtemp = -asin(Pxe[2]);
		u = (180 / PI)*atan2((Pxe[1] / cos(vtemp)), (Pxe[0] / cos(vtemp)));
		v = -(180 / PI)*asin(Pxe[2]);
		w = (180 / PI)*atan2((Pye[2] / cos(vtemp)), (Pz[2] / cos(vtemp)));
		/* excel公式
		u = (180 / PI)*atan2((Pxe[0] / cos(vtemp)), (Pxe[1] / cos(vtemp)));
		w = (180 / PI)*atan2((Pz[2] / cos(vtemp)), (Pye[2] / cos(vtemp)));
		*/
		temp_point = m_spel->GetPoint("P0");
		m_spel->Accel(15, 15);
		m_spel->Speed(15, 15, 15);
		m_spel->AccelR(15, 15);
		m_spel->SpeedR(15);

		temp_point->X = 0;
		temp_point->Y = 300;
		temp_point->Z = 500;
		temp_point->U = 0;
		temp_point->V = 90;
		temp_point->W = 90;
		m_spel->Go(temp_point);

		temp_point->X = final_mid[1];
		temp_point->Y = final_mid[0] + 50;
		temp_point->Z = final_mid[2];
		temp_point->U = 0;
		temp_point->V = 90;
		temp_point->W = 90;
		m_spel->Go(temp_point);

		temp_point->X = final_mid[1];
		temp_point->Y = final_mid[0] + 50;
		temp_point->Z = final_mid[2];
		temp_point->U = u;
		temp_point->V = v;
		temp_point->W = w;
		m_spel->Go(temp_point);
		m_spel->Delay(200);
		m_spel->Off(8);
		temp_point->X = final_mid[1];
		temp_point->Y = final_mid[0] - 55;
		temp_point->Z = final_mid[2];
		temp_point->U = u;
		temp_point->V = v;
		temp_point->W = w;
		m_spel->Go(temp_point);
		m_spel->On(8);
		temp_point->X = final_mid[1];
		temp_point->Y = 150;
		temp_point->Z = final_mid[2];
		temp_point->U = u;
		temp_point->V = v;
		temp_point->W = w;
		m_spel->Go(temp_point);
		temp_point->X = 0;
		temp_point->Y = 300;
		temp_point->Z = 500;
		temp_point->U = 0;
		temp_point->V = 90;
		temp_point->W = 90;
		m_spel->Go(temp_point);
		temp_point->X = 0;
		temp_point->Y = 20; //mouse-15 
		temp_point->Z = 500;
		temp_point->U = 0;
		temp_point->V = 90;
		temp_point->W = 90;
		m_spel->Go(temp_point);
		m_spel->Off(8);
		temp_point->X = 0;
		temp_point->Y = 300;
		temp_point->Z = 500;
		temp_point->U = 0;
		temp_point->V = 90;
		temp_point->W = 90;
		m_spel->Go(temp_point);
		//MyGo(chosen_mid[0], chosen_mid[1], chosen_mid[2], u, v, w);
		//MyGo(chosen_mid[0], chosen_mid[1], chosen_mid[2] - 5, u, v, w);
		//////clip the object 
		//m_spel->Delay(500);
		//m_spel->On(8);
		//m_spel->Delay(200);
		//MyGo(chosen_mid[0], 150, chosen_mid[2], u, v, w); //rise from the position to height y=150
		//MyGo(0, 100, 500, 0, 90, 90);
		//MyGo(0, 20, 500, 0, 90, 90);
		//m_spel->Delay(200);
		//m_spel->Off(8);
		//MyGo(-250, 200, 0, 0, 90, 90);//back to datum point

	}
}

System::Void mymain::btnIOMonitor_Click(System::Object^  sender, System::EventArgs^  e)
{
	try
	{
		m_spel->ShowWindow(SpelWindows::IOMonitor, this);
	}
	catch (SpelException ^ex)
	{
		System::String^ error_text = gcnew System::String(ex->Message);
		ERROR_lbl->Text = error_text;
	}
}

System::Void mymain::btnSimulate_Click(System::Object^  sender, System::EventArgs^  e)
{
	m_spel->ShowWindow(SpelWindows::Simulator, this);
}

System::Void mymain::btnHome_Click(System::Object^  sender, System::EventArgs^  e)
{
	try
	{
		m_spel->Home();
	}
	catch (SpelException ^ex)
	{
		System::String^ error_text = gcnew System::String(ex->Message);
		ERROR_lbl->Text = error_text;
	}

}

System::Void mymain::btnReset_Click(System::Object^  sender, System::EventArgs^  e)
{
	try
	{
		m_spel->Reset();
	}
	catch (SpelException ^ex)
	{
		System::String^ error_text = gcnew System::String(ex->Message);
		ERROR_lbl->Text = error_text;
	}
}

System::Void mymain::btnExit_Click(System::Object^  sender, System::EventArgs^  e)
{
	Close();
}

System::Void mymain::m_spel_EventReceived(System::Object^  sender, SpelNetLib::SpelEventArgs^ e)
{
	System::String^ error_text = gcnew System::String(e->Message);
	ERROR_lbl->Text = error_text;
}

System::Void mymain::Datum_Pnt_Click(System::Object^  sender, System::EventArgs^  e)
{
	MyGo(-250, 470, 120, 90, 0, 180);
	MyGo(-635, -64, 144.39, 90, 0, 180);
}