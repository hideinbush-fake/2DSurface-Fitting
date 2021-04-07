// surfacefit.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include<fstream>
#include<iostream>
#include<iomanip>
#include<dense>
#include<math.h>
#include "G:\1f version\NxtSCAN_CTI\3rdparty\ACSMot\ACSC.h"
using namespace std;
const int N = 40;
using namespace Eigen;
#pragma comment(lib, "G:\\1f version\\NxtSCAN_CTI\\3rdparty\\ACSMot\\ACSCL_x86.lib")
HANDLE hComm;
BOOL Init(void)
{

	BOOL bRet = FALSE;
	char* strAddr = "10.0.0.100";
	int nConMode = ACSC_SOCKET_STREAM_PORT;
	hComm = acsc_OpenCommEthernet(strAddr, nConMode);

	if (hComm == ACSC_INVALID)
	{
		::MessageBox(GetActiveWindow(),
			_T("ACS运动控制卡连接失败!"),
			_T("Mot.dll"),
			MB_OK);
		bRet = FALSE;
	}
	else
	{
		acsc_OpenHistoryBuffer(hComm, 10000);
		bRet = TRUE;
	}


	//InitializeCriticalSection(&m_cs);

	return bRet;
}
UINT InitBuffer(int BASE0, int INCREMENT0, int BASE1, int INCREMENT1, int x_num, int y_num){
	if (!acsc_ClearBuffer(hComm,
		9,
		1, ACSC_MAX_LINE,
		NULL
		))
	{
		printf("transaction error: %d\n", acsc_GetLastError());
		return 0;
	}

	char buf[256];
	//strcpy(buf, "!This is a test ACSPL+ program\n" );
	sprintf(buf, "!This is a test ACSPL+ program\n"
		" GLOBAL REAL BASE0, BASE1, INCREMENT0, INCREMENT1, CORRECTION_MAP0(%d)(%d), CORRECTION_MAP1(%d)(%d)\n"
		"BASE0 = %d\n"
		"INCREMENT0 = %d\n"
		"BASE1 = %d\n"
		"INCREMENT1 = %d\n"
		, x_num, y_num, x_num, y_num, BASE0, INCREMENT0, BASE1, INCREMENT1);
	if (!acsc_AppendBuffer(hComm, // communication handle
		9, // ACSPL+ program buffer number
		buf, // buffer contained ACSPL+ program(s)
		strlen(buf), // size of this buffer
		NULL // waiting call
		))
	{
		printf("transaction error: %d\n", acsc_GetLastError());
		return 0;
	}

	return 1;
}

UINT WriteBuffer(float Offset, int col, int line){
	char buf[256];
	sprintf(buf, "CORRECTION_MAP0(%d)(%d) = %f \n", col, line, Offset);
	if (!acsc_AppendBuffer(hComm, // communication handle
		9, // ACSPL+ program buffer number
		buf, // buffer contained ACSPL+ program(s)
		strlen(buf), // size of this buffer
		NULL // waiting call
		))
	{
		printf("transaction error: %d\n", acsc_GetLastError());
		return 0;
	}

	return 1;
}
UINT WriteBuffer_1(float Offset, int col, int line){
	char buf[256];
	sprintf(buf, "CORRECTION_MAP1(%d)(%d) = %f \n", col, line, Offset);
	if (!acsc_AppendBuffer(hComm, // communication handle
		9, // ACSPL+ program buffer number
		buf, // buffer contained ACSPL+ program(s)
		strlen(buf), // size of this buffer
		NULL // waiting call
		))
	{
		printf("transaction error: %d\n", acsc_GetLastError());
		return 0;
	}

	return 1;
}
UINT WriteBufferEnd(){
	char buf[512];
	sprintf(buf, "MFLAGS(0).#DEFCON = 0\n"
		"CONNECT RPOS(0) = APOS(0) + MAP2(APOS(0), APOS(1), CORRECTION_MAP0, BASE0, INCREMENT0, BASE1, INCREMENT1)\n"
		"DEPENDS 0,(0,1)\n"
		"MFLAGS(1).#DEFCON = 0\n"
		"CONNECT RPOS(1) = APOS(1) + MAP2(APOS(0), APOS(1), CORRECTION_MAP1, BASE0, INCREMENT0, BASE1, INCREMENT1)\n"
		"DEPENDS 1,(0,1)\n"
		"STOP"
		);
	int len = strlen(buf);

	if (!acsc_AppendBuffer(hComm, // communication handle
		9, // ACSPL+ program buffer number
		buf, // buffer contained ACSPL+ program(s)
		strlen(buf), // size of this buffer
		NULL // waiting call
		))
	{
		printf("transaction error: %d\n", acsc_GetLastError());
		return 0;
	}

	return 1;
}

UINT LoadBuffer(){
	if (!acsc_CompileBuffer(hComm, // communication handle
		9, // ACSPL+ program buffer number
		NULL // waiting call
		))
	{
		printf("compilation error: %d\n", acsc_GetLastError());
		return 0;
	}

	if (!acsc_RunBuffer(hComm,// communication handle
		9, // ACSPL+ program buffer number
		NULL, // from the beginning of this buffer
		NULL // waiting call
		))
	{
		printf("transaction error: %d\n", acsc_GetLastError());
		return 0;
	}
	return 1;
}

UINT SaveBufferToFalsh(){
	int Buffers[] = { ACSC_BUFFER_ALL, -1 };
	if (!acsc_ControllerSaveToFlash(
		hComm, // communication handle
		NULL, // Array of axis constants
		Buffers, // Array of buffer constants
		NULL, // Array of SP constants
		NULL // User Arrays list
		))
	{
		printf("acsc_ControllerSaveToFlash(): Error Occurred - %d\n", acsc_GetLastError());
		return 0;
	}
	return 1;
}

VectorXf GetEfficient(MatrixXf mat, VectorXf input)
{
	return (mat.inverse())*input;
}
float e[6];
float f[6];
float getex(float xm, float ym)
{
	//return -0.000001*xm*ym - 0.000020*xm + 0.000106*ym + 0.017272;
	return (e[0] + e[1] * xm + e[2] * ym + e[3] * xm*xm + e[4] *xm* ym + e[5]*ym*ym);
}
float getey(float xm, float ym)
{
	//return -0.000001*xm - 0.000129*ym - 0.029155;
	return (f[0] + f[1] * xm + f[2] * ym + f[3] * xm*xm + f[4] * xm* ym + f[5] * ym*ym);
}
int _tmain(int argc, _TCHAR* argv[])
{
	float x[N][N];
	float y[N][N];
	float ex[N][N];
	float ey[N][N];
	float matele[6][6];
	float vecele_ex[6];
	float vecele_ey[6];
	memset(matele, 0, sizeof(matele));
	memset(vecele_ex, 0, sizeof(vecele_ex));
	memset(vecele_ey, 0, sizeof(vecele_ey));
	string sEQU;
	const char * fn = "D:\\数据.txt";
	std::ifstream is(fn);
	if (!is)
	{
		cout << "fail to open the file" << endl;
		getchar();
		getchar();

		return -1;//或者抛出异常。
	}
	//is.open(fn, std::ios::in);
	for (int i = 0; i < N; i++)
	{
		for (int j = 0; j < N; j++)
		{
			is >> x[i][j] >> y[i][j] >> ex[i][j] >> ey[i][j];
		}
	}
	/*cout << "--------------------\n";
	for (int i = 0; i < N; i++)
	{
	for (int j = 0; j < N; j++)
	{
	cout<< setiosflags(ios::fixed)<<x[i][j] <<'\t'<< y[i][j] <<'\t'<< ex[i][j] <<'\t'<< ey[i][j]<<endl;
	}
	}*/

	is.close();
	for (int i = 0; i < N; i++)
	{
		for (int j = 0; j < N; j++)
		{
			x[i][j] = -x[i][j] -52.5;
			y[i][j] = y[i][j] - 219.0;
		}
	}
	
	matele[0][0] = N*N;
	for (int i = 0; i < N; i++)
	{
		for (int j = 0; j < N; j++)
		{
			matele[0][1] += x[i][j];
			matele[0][2] += y[i][j];
			matele[0][3] += powf(x[i][j], 2);
			matele[0][4] += x[i][j] * y[i][j];
			matele[0][5] += powf(y[i][j], 2);

			matele[1][0] += x[i][j];
			matele[1][1] += powf(x[i][j], 2);
			matele[1][2] += x[i][j] * y[i][j];
			matele[1][3] += powf(x[i][j], 3);
			matele[1][4] += powf(x[i][j], 2)*y[i][j];
			matele[1][5] += powf(y[i][j], 2)*x[i][j];

			matele[2][0] += y[i][j];
			matele[2][1] += x[i][j] * y[i][j];
			matele[2][2] += powf(y[i][j], 2);
			matele[2][3] += powf(x[i][j], 2)*y[i][j];
			matele[2][4] += powf(y[i][j], 2)*x[i][j];
			matele[2][5] += powf(y[i][j], 3);

			matele[3][0] += powf(x[i][j], 2);
			matele[3][1] += powf(x[i][j], 3);
			matele[3][2] += powf(x[i][j], 2)*y[i][j];
			matele[3][3] += powf(x[i][j], 4);
			matele[3][4] += powf(x[i][j], 3)*y[i][j];
			matele[3][5] += powf(x[i][j], 2)*powf(y[i][j], 2);

			matele[4][0] += x[i][j] * y[i][j];
			matele[4][1] += powf(x[i][j], 2)*y[i][j];
			matele[4][2] += powf(y[i][j], 2)*x[i][j];
			matele[4][3] += powf(x[i][j], 3)*y[i][j];
			matele[4][4] += powf(x[i][j], 2)*powf(y[i][j], 2);
			matele[4][5] += powf(y[i][j], 3)*x[i][j];

			matele[5][0] += powf(y[i][j], 2);
			matele[5][1] += powf(y[i][j], 2)*x[i][j];
			matele[5][2] += powf(y[i][j], 3);
			matele[5][3] += powf(x[i][j], 2)*powf(y[i][j], 2);
			matele[5][4] += powf(y[i][j], 3)*x[i][j];
			matele[5][5] += powf(y[i][j], 4);

			vecele_ex[0] += ex[i][j];
			vecele_ex[1] += ex[i][j] * x[i][j];
			vecele_ex[2] += ex[i][j] * y[i][j];
			vecele_ex[3] += ex[i][j] * powf(x[i][j], 2);
			vecele_ex[4] += ex[i][j] * x[i][j] * y[i][j];
			vecele_ex[5] += ex[i][j] * powf(y[i][j], 2);

			vecele_ey[0] += ey[i][j];
			vecele_ey[1] += ey[i][j] * x[i][j];
			vecele_ey[2] += ey[i][j] * y[i][j];
			vecele_ey[3] += ey[i][j] * powf(x[i][j], 2);
			vecele_ey[4] += ey[i][j] * x[i][j] * y[i][j];
			vecele_ey[5] += ey[i][j] * powf(y[i][j], 2);

		}


	}

	MatrixXf Matrix_XY(6, 6);
	//cout << Matrix_XY.rows() << '\t' << Matrix_XY.cols() << endl;   //6   6
	VectorXf Vector_Ex(6);
	VectorXf Vector_Ey(6);
	Matrix_XY.setZero();
	Vector_Ex.setZero();
	Vector_Ey.setZero();
	//cout << Vector_Ex.rows() << '\t' << Vector_Ex.cols() << endl;   //6   1
	for (int i = 0; i < 6; i++)
	{
		for (int j = 0; j < 6; j++)
		{
			Matrix_XY(i, j) = matele[i][j];
		}
		Vector_Ex(i) = vecele_ex[i];
		Vector_Ey(i) = vecele_ey[i];
	}
	VectorXf Effi_Ex(6);
	VectorXf Effi_Ey(6);
	Effi_Ex.setZero();
	Effi_Ey.setZero();
	Effi_Ex = GetEfficient(Matrix_XY,Vector_Ex);
	Effi_Ey = GetEfficient(Matrix_XY, Vector_Ey);
	cout << "第一组系数如下 :\n" << Effi_Ex << endl << "第二组系数如下 :\n" << Effi_Ey << endl;
	for (int i = 0; i < 6; i++)

	{
		e[i] = Effi_Ex(i);
		f[i] = Effi_Ey(i);
	}
	std::cout << "验算:\n";
	float errorex = 0.0f;
	float errorey = 0.0f;
	for (int i = 0; i < N; i++)
	{
		for (int j = 0; j < N; j++)
		{
			float t = fabs(ex[i][j]- getex(x[i][j], y[i][j]));
			float tt = fabs(ey[i][j] - getey(x[i][j], y[i][j]));
			errorex += t;
			errorey += tt;
			std::cout << "delta_ex: " << t << '\t' << "delta_ey: " << tt << std::endl;
		}
	}
	std::cout << "拟合曲面对于此"<<N*N<<"组数据的平均误差为(mm)： \t DELTA_Ex: " << errorex /(N*N)<< "\t DELTA_Ey: " << errorey /(N*N) << std::endl;
	getchar();
	getchar();
	std::cout << "现在开始写Buffer...\n";
	int signx;
	int signy;
	std::cout << "请依次输入Ex,Ey的符号，输入-1或者1：\n";
	while (1)
	{
		std::cin >> signx >> signy;
		if (signx == -1 && signy == -1)
		{
			std::cout << "Ex取反！\tEy取反！\n";
			break;

		}
		else if (signx == 1 && signy == -1)
		{
			std::cout << "Ex保持原样！\tEy取反！\n";
			break;
		}
		else if (signx == -1 && signy == 1)
		{
			std::cout << "Ex取反！\tEy保持原样!\n";
			break;
		}
		else if (signx == 1 && signy == 1)
		{
			std::cout << "Ex保持原样！\tEy保持原样！\n";
			break;
		}
		else
		{
			std::cout << "输入有误！请重新输入！\n";
			continue;
		}
	}
	if (!Init())
	{
		std::cout << "初始化失败\n";
		return -1;
	}
	InitBuffer(-615, 10, -240, 10, 53, 64);
	for (int i = 0; i < 53; i++)
	{
		for (int j = 0; j < 64; j++)
			WriteBuffer(signx*getex(-615 + 10 * j, -240 + 10 * i), i, j);
	}
	for (int i = 0; i < 64; i++)
	{
		for (int j = 0; j < 53; j++)
			WriteBuffer_1(signy*getey(-615 + 10 * i, -240 + 10 * j), j, i);
	}
	WriteBufferEnd();
	LoadBuffer();
	SaveBufferToFalsh();
	getchar();
	getchar();
	return 0;
}

