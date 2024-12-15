#include <unistd.h>
#include <iostream>
#include <time.h>
#include "opencv2/opencv.hpp"
#include <wiringPi.h>
#include <math.h>
#include <X11/Xlib.h>
#include <ctime>
#include <string>
#include <cstring>
#include <algorithm>
#include <dirent.h>
#include <vector>
#include <filesystem>

#define WEB_CAM			0
#define INPUT_PORT		22	//INPUT5
#define INPUT_DATA_RESET	27	//INPUT4

using namespace std;
using namespace cv;
namespace fs = std::filesystem;

unsigned int	Accuracy_Percent[7];
unsigned int	Accuracy_Percent_Temp[7];
int		flag = -1;
bool		Setting_Mode_Flag = false;
unsigned int	Compare_Total, Compare_Good, Compare_Bad;
bool		ROI_flag[7] = { 0, 0, 0, 0, 0, 0, 0 };
bool		NP_flag[7] = { 1, 1, 1, 1, 1, 1, 1 };
bool		ROI_flag_Temp[7] = { 0, 0, 0, 0, 0, 0, 0 };
bool		NP_flag_Temp[7] = { 1, 1, 1, 1, 1, 1, 1 };
bool		Set_Flag = false;
int		Set_Click = -1;
int		Viewer_key = 0;

VideoCapture	cap;
Mat		frame, Show_Frame, Inspect_ROI;
Mat		Set_Frame, Set_Frame_Clone;
vector<Mat>	Standard_ROI, Standard_ROI_Temp;
Point		temp, temp2;
Point		glo_Viewer_center;

int		Inspect_ROI_Point[28], Inspect_ROI_Point_Temp_List[28];
vector<Point>	State_Message_Point;
vector<String>	State_Message_S;
Point		ROI_Sel_Pt = Point(5, 86);
vector<Scalar>	List_Scalar;
vector<Scalar>	NP_List_Scalar;
vector<String>  NP_char;

Point		Title_Point = Point(5, 70);
Point		Rectangle_Point_x = Point(135, 60);
Point		Rectangle_Point_y = Point(165, 60);
Point		Resolution_Pt = Point(100, 60);
Point		Set_Button_Pt = Point(230, 60);
Size		Resolution_Size = Size(40, 15);
Size		Rectangle_Size = Size(20, 15);
Point		Reset_Button_Point = Point(135, 230);
Size		Reset_Button_Size = Point(55, 23);
int		button_dis = 30;
double		Message_Font_Size = 0.4;
Point		Resolution_Point = Point(90, 200);
unsigned int	width, height, ch_width_temp, ch_height_temp;
int		State_Message_Flag = 0;

char		img_data_addr[40] = "/home/cellon/CELLON/";
char		resol_data_addr[40] = "/home/cellon/CELLON/resol_data.txt";
char		Point_data_addr[40] = "/home/cellon/CELLON/Point_data.txt";
char		Acc_addr[40] = "/home/cellon/CELLON/Acc_data.txt";
char		ROI_flag_addr[40] = "/home/cellon/CELLON/ROI_flag_data.txt";
char            NP_flag_addr[40] = "/home/cellon/CELLON/NP_flag_data.txt";
char            total_data_addr[40] = "/home/cellon/CELLON/total_data.txt";
char            good_data_addr[40] = "/home/cellon/CELLON/good_data.txt";
char            bad_data_addr[40] = "/home/cellon/CELLON/bad_data.txt";
char		Live_mode_addr[40] = "/home/cellon/CELLON/Live_data.txt";
char		result_image_addr[40] = "/home/cellon/CELLON/Result/";
char            Fput_STR[100];

const int	Led[] = { 18, 23, 24 };		//Led[0] = Ready, Led[1] = Bad, Led[2] = Good
bool		start_signal = false;
bool		Live_flag = true;
int		Viewer_Flag;
vector<Scalar>	Live_Scalar_List;

///////////////////////Count Segment/////////////////////////
void State_Message() {
	if (State_Message_S[6] != State_Message_S[7]) {
		State_Message_S[0] = State_Message_S[1];
		State_Message_S[1] = State_Message_S[2];
		State_Message_S[2] = State_Message_S[3];
		State_Message_S[3] = State_Message_S[4];
		State_Message_S[4] = State_Message_S[5];
		State_Message_S[5] = State_Message_S[6];
		State_Message_S[6] = State_Message_S[7];
	}

	for (int i = 6; i >= 0; i--) {
		putText(Show_Frame, State_Message_S[i], State_Message_Point[i], FONT_HERSHEY_SIMPLEX, Message_Font_Size, Scalar(0, 0, 0), 2);
		putText(Show_Frame, State_Message_S[i], State_Message_Point[i], FONT_HERSHEY_SIMPLEX, Message_Font_Size, Scalar(220, 220, 220), 1);
	}
}

void Compare_State(Scalar state_scalar) {
	rectangle(Show_Frame, Rect(width - 135, height - 30, 95, 25), Scalar(0, 0, 0), -1);
	rectangle(Show_Frame, Rect(width - 130, height - 25, 85, 15), state_scalar, -1);
}

void Compare_Result() {
	char Compare_Total_S[15], Compare_Good_S[15], Compare_Bad_S[15];

	sprintf(Compare_Total_S, "TOTAL  : %d", Compare_Total);
	sprintf(Compare_Good_S, "GOOD  : %d", Compare_Good);
	sprintf(Compare_Bad_S, "BAD    : %d", Compare_Bad);

	putText(Show_Frame, Compare_Total_S, Point(width - 135, height - 70), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 3);
	putText(Show_Frame, Compare_Total_S, Point(width - 135, height - 70), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 1);

	putText(Show_Frame, Compare_Good_S, Point(width - 135, height - 55), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 3);
	putText(Show_Frame, Compare_Good_S, Point(width - 135, height - 55), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 1);

	putText(Show_Frame, Compare_Bad_S, Point(width - 135, height - 40), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 3);
	putText(Show_Frame, Compare_Bad_S, Point(width - 135, height - 40), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 1);

}

void ALAM_Marking() {
	Point ALAM_Location = Point(8, 25);
	putText(Show_Frame, "=", ALAM_Location, FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 0), 5);
	putText(Show_Frame, "CELLON", ALAM_Location + Point(30, -2), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 0), 4);
	putText(Show_Frame, "=", ALAM_Location, FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 127, 255), 2);
	putText(Show_Frame, "CELLON", ALAM_Location + Point(30, -2), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 127, 255), 1);
}

void Center_Marking(double width, double height) {
	line(Show_Frame, Point(width / 2, height / 2 + 15), Point(width / 2, height / 2 - 15), Scalar(0, 0, 255), 1);
	line(Show_Frame, Point(width / 2 + 15, height / 2), Point(width / 2 - 15, height / 2), Scalar(0, 0, 255), 1);
}

void Setting_Mode() {
	char	Accuracy_Percent_S[4];
	Point	Value_Point = Point(109, 70);

	/////////////////////			Reset Result / Button, Text	///////////////////////
	rectangle(Show_Frame, Point(width - 30, height - 30), Point(width - 5, height - 7), Scalar(0, 0, 0), -1);
	rectangle(Show_Frame, Point(width - 27, height - 27), Point(width - 8, height - 10), Scalar(0, 0, 255), -1);

	putText(Show_Frame, "R", Point(width - 23, height - 13), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 3);
	putText(Show_Frame, "R", Point(width - 23, height - 13), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 1);


	/*///////////////////           	Menu Box                	/////////////////////*/
	rectangle(Show_Frame, Rect(Title_Point - Point(2, 30) - Point(3, 3), Title_Point + Point(325, button_dis * 9) + Point(3, 3)), Scalar(0, 0, 0), -1);
	rectangle(Show_Frame, Rect(Title_Point - Point(2, 30), Title_Point + Point(325, button_dis * 9)), Scalar(125, 125, 125), -1);

	/*///////////////////			ROI_? Text, Button				/////////////////////*/
	rectangle(Show_Frame, Rect(ROI_Sel_Pt + Point(-3, -3), Reset_Button_Size + Size(-8, -4) + Size(6, 6)), Scalar(0, 0, 0), -1);
	rectangle(Show_Frame, Rect(ROI_Sel_Pt, Reset_Button_Size + Size(-8, -4)), List_Scalar[0], -1);
	rectangle(Show_Frame, Rect(ROI_Sel_Pt + Point(0, button_dis) + Point(-3, -3), Reset_Button_Size + Size(-8, -4) + Size(6, 6)), Scalar(0, 0, 0), -1);
	rectangle(Show_Frame, Rect(ROI_Sel_Pt + Point(0, button_dis), Reset_Button_Size + Size(-8, -4)), List_Scalar[1], -1);
	rectangle(Show_Frame, Rect(ROI_Sel_Pt + Point(0, button_dis) * 2 + Point(-3, -3), Reset_Button_Size + Size(-8, -4) + Size(6, 6)), Scalar(0, 0, 0), -1);
	rectangle(Show_Frame, Rect(ROI_Sel_Pt + Point(0, button_dis) * 2, Reset_Button_Size + Size(-8, -4)), List_Scalar[2], -1);
	rectangle(Show_Frame, Rect(ROI_Sel_Pt + Point(0, button_dis) * 3 + Point(-3, -3), Reset_Button_Size + Size(-8, -4) + Size(6, 6)), Scalar(0, 0, 0), -1);
	rectangle(Show_Frame, Rect(ROI_Sel_Pt + Point(0, button_dis) * 3, Reset_Button_Size + Size(-8, -4)), List_Scalar[3], -1);
	rectangle(Show_Frame, Rect(ROI_Sel_Pt + Point(0, button_dis) * 4 + Point(-3, -3), Reset_Button_Size + Size(-8, -4) + Size(6, 6)), Scalar(0, 0, 0), -1);
	rectangle(Show_Frame, Rect(ROI_Sel_Pt + Point(0, button_dis) * 4, Reset_Button_Size + Size(-8, -4)), List_Scalar[4], -1);
	rectangle(Show_Frame, Rect(ROI_Sel_Pt + Point(0, button_dis) * 5 + Point(-3, -3), Reset_Button_Size + Size(-8, -4) + Size(6, 6)), Scalar(0, 0, 0), -1);
	rectangle(Show_Frame, Rect(ROI_Sel_Pt + Point(0, button_dis) * 5, Reset_Button_Size + Size(-8, -4)), List_Scalar[5], -1);
	rectangle(Show_Frame, Rect(ROI_Sel_Pt + Point(0, button_dis) * 6 + Point(-3, -3), Reset_Button_Size + Size(-8, -4) + Size(6, 6)), Scalar(0, 0, 0), -1);
	rectangle(Show_Frame, Rect(ROI_Sel_Pt + Point(0, button_dis) * 6, Reset_Button_Size + Size(-8, -4)), List_Scalar[6], -1);

	putText(Show_Frame, "Resolution", Title_Point, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 3);
	putText(Show_Frame, "Resolution", Title_Point, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 1);
	putText(Show_Frame, "ROI_1", Title_Point + Point(0, button_dis), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 3);
	putText(Show_Frame, "ROI_1", Title_Point + Point(0, button_dis), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 1);
	putText(Show_Frame, "ROI_2", Title_Point + Point(0, button_dis) * 2, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 3);
	putText(Show_Frame, "ROI_2", Title_Point + Point(0, button_dis) * 2, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 1);
	putText(Show_Frame, "ROI_3", Title_Point + Point(0, button_dis) * 3, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 3);
	putText(Show_Frame, "ROI_3", Title_Point + Point(0, button_dis) * 3, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 1);
	putText(Show_Frame, "ROI_4", Title_Point + Point(0, button_dis) * 4, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 3);
	putText(Show_Frame, "ROI_4", Title_Point + Point(0, button_dis) * 4, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 1);
	putText(Show_Frame, "ROI_5", Title_Point + Point(0, button_dis) * 5, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 3);
	putText(Show_Frame, "ROI_5", Title_Point + Point(0, button_dis) * 5, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 1);
	putText(Show_Frame, "ROI_6", Title_Point + Point(0, button_dis) * 6, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 3);
	putText(Show_Frame, "ROI_6", Title_Point + Point(0, button_dis) * 6, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 1);
	putText(Show_Frame, "ROI_7", Title_Point + Point(0, button_dis) * 7, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 3);
	putText(Show_Frame, "ROI_7", Title_Point + Point(0, button_dis) * 7, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 1);

	/*////////////////////          	Resolution Button	             ////////////////////*/
	int mar = Resolution_Size.width;
	rectangle(Show_Frame, Rect(Resolution_Pt + Point(-3, -3), Resolution_Size + Size(6, 6)), Scalar(0, 0, 0), -1);
	rectangle(Show_Frame, Rect(Resolution_Pt, Resolution_Size), Scalar(170, 170, 170), -1);
	rectangle(Show_Frame, Rect(Resolution_Pt + Point(-3, -3) + Point(mar + 10, 0), Resolution_Size + Size(6, 6)), Scalar(0, 0, 0), -1);
	rectangle(Show_Frame, Rect(Resolution_Pt + Point(mar + 10, 0), Resolution_Size), Scalar(170, 170, 170), -1);
	rectangle(Show_Frame, Rect(Resolution_Pt + Point(-3, -3) + Point(mar * 2 + 20, 0), Resolution_Size + Size(6, 6)), Scalar(0, 0, 0), -1);
	rectangle(Show_Frame, Rect(Resolution_Pt + Point(mar * 2 + 20, 0), Resolution_Size), Scalar(170, 170, 170), -1);

	/*///////////////////			USB_Backup Button, Text		    //////////////////////*/
	putText(Show_Frame, "USB :", Resolution_Pt + Point(3, 12) + Point(mar * 2 + 20, -1) + Point(44, 0), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0, 0, 0), 3);
	putText(Show_Frame, "USB :", Resolution_Pt + Point(3, 12) + Point(mar * 2 + 20, -1) + Point(44, 0), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(255, 255, 255), 1);

	rectangle(Show_Frame, Rect(Resolution_Pt + Point(-3, -3) + Point(mar * 2 + 20, 0) + Point(92, 0), Rectangle_Size + Size(6, 6)), Scalar(0, 0, 0), -1);
	rectangle(Show_Frame, Rect(Resolution_Pt + Point(mar * 2 + 20, 0) + Point(92, 0), Rectangle_Size), Scalar(170, 170, 170), -1);

	putText(Show_Frame, "BP", Resolution_Pt + Point(3, 12) + Point(mar * 2 + 20, -1) + Point(90, 0), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0, 0, 0), 3);
	putText(Show_Frame, "BP", Resolution_Pt + Point(3, 12) + Point(mar * 2 + 20, -1) + Point(90, 0), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(255, 255, 255), 1);

	//Button Text
	putText(Show_Frame, "640", Resolution_Pt + Point(3, 12) + Point(4, -1), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0, 0, 0), 3);
	putText(Show_Frame, "640", Resolution_Pt + Point(3, 12) + Point(4, -1), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(255, 255, 255), 1);
	putText(Show_Frame, "1024", Resolution_Pt + Point(3, 12) + Point(mar + 12, -1), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0, 0, 0), 3);
	putText(Show_Frame, "1024", Resolution_Pt + Point(3, 12) + Point(mar + 12, -1), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(255, 255, 255), 1);
	putText(Show_Frame, "1280", Resolution_Pt + Point(3, 12) + Point(mar * 2 + 20, -1), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0, 0, 0), 3);
	putText(Show_Frame, "1280", Resolution_Pt + Point(3, 12) + Point(mar * 2 + 20, -1), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(255, 255, 255), 1);

	/*////////////////////		+, - Button		////////////////////*/
	//	-	//
	rectangle(Show_Frame, Rect(Rectangle_Point_x + Point(0, button_dis) + Point(-3, -3), Rectangle_Size + Size(6, 6)), Scalar(0, 0, 0), -1);
	rectangle(Show_Frame, Rect(Rectangle_Point_x + Point(0, button_dis), Rectangle_Size), Scalar(170, 170, 170), -1);
	rectangle(Show_Frame, Rect(Rectangle_Point_x + Point(0, button_dis) * 2 + Point(-3, -3), Rectangle_Size + Size(6, 6)), Scalar(0, 0, 0), -1);
	rectangle(Show_Frame, Rect(Rectangle_Point_x + Point(0, button_dis) * 2, Rectangle_Size), Scalar(170, 170, 170), -1);
	rectangle(Show_Frame, Rect(Rectangle_Point_x + Point(0, button_dis) * 3 + Point(-3, -3), Rectangle_Size + Size(6, 6)), Scalar(0, 0, 0), -1);
	rectangle(Show_Frame, Rect(Rectangle_Point_x + Point(0, button_dis) * 3, Rectangle_Size), Scalar(170, 170, 170), -1);
	rectangle(Show_Frame, Rect(Rectangle_Point_x + Point(0, button_dis) * 4 + Point(-3, -3), Rectangle_Size + Size(6, 6)), Scalar(0, 0, 0), -1);
	rectangle(Show_Frame, Rect(Rectangle_Point_x + Point(0, button_dis) * 4, Rectangle_Size), Scalar(170, 170, 170), -1);
	rectangle(Show_Frame, Rect(Rectangle_Point_x + Point(0, button_dis) * 5 + Point(-3, -3), Rectangle_Size + Size(6, 6)), Scalar(0, 0, 0), -1);
	rectangle(Show_Frame, Rect(Rectangle_Point_x + Point(0, button_dis) * 5, Rectangle_Size), Scalar(170, 170, 170), -1);
	rectangle(Show_Frame, Rect(Rectangle_Point_x + Point(0, button_dis) * 6 + Point(-3, -3), Rectangle_Size + Size(6, 6)), Scalar(0, 0, 0), -1);
	rectangle(Show_Frame, Rect(Rectangle_Point_x + Point(0, button_dis) * 6, Rectangle_Size), Scalar(170, 170, 170), -1);
	rectangle(Show_Frame, Rect(Rectangle_Point_x + Point(0, button_dis) * 7 + Point(-3, -3), Rectangle_Size + Size(6, 6)), Scalar(0, 0, 0), -1);
	rectangle(Show_Frame, Rect(Rectangle_Point_x + Point(0, button_dis) * 7, Rectangle_Size), Scalar(170, 170, 170), -1);

	//	+	//
	rectangle(Show_Frame, Rect(Rectangle_Point_y + Point(0, button_dis) + Point(-3, -3), Rectangle_Size + Size(6, 6)), Scalar(0, 0, 0), -1);
	rectangle(Show_Frame, Rect(Rectangle_Point_y + Point(0, button_dis), Rectangle_Size), Scalar(170, 170, 170), -1);
	rectangle(Show_Frame, Rect(Rectangle_Point_y + Point(0, button_dis) * 2 + Point(-3, -3), Rectangle_Size + Size(6, 6)), Scalar(0, 0, 0), -1);
	rectangle(Show_Frame, Rect(Rectangle_Point_y + Point(0, button_dis) * 2, Rectangle_Size), Scalar(170, 170, 170), -1);
	rectangle(Show_Frame, Rect(Rectangle_Point_y + Point(0, button_dis) * 3 + Point(-3, -3), Rectangle_Size + Size(6, 6)), Scalar(0, 0, 0), -1);
	rectangle(Show_Frame, Rect(Rectangle_Point_y + Point(0, button_dis) * 3, Rectangle_Size), Scalar(170, 170, 170), -1);
	rectangle(Show_Frame, Rect(Rectangle_Point_y + Point(0, button_dis) * 4 + Point(-3, -3), Rectangle_Size + Size(6, 6)), Scalar(0, 0, 0), -1);
	rectangle(Show_Frame, Rect(Rectangle_Point_y + Point(0, button_dis) * 4, Rectangle_Size), Scalar(170, 170, 170), -1);
	rectangle(Show_Frame, Rect(Rectangle_Point_y + Point(0, button_dis) * 5 + Point(-3, -3), Rectangle_Size + Size(6, 6)), Scalar(0, 0, 0), -1);
	rectangle(Show_Frame, Rect(Rectangle_Point_y + Point(0, button_dis) * 5, Rectangle_Size), Scalar(170, 170, 170), -1);
	rectangle(Show_Frame, Rect(Rectangle_Point_y + Point(0, button_dis) * 6 + Point(-3, -3), Rectangle_Size + Size(6, 6)), Scalar(0, 0, 0), -1);
	rectangle(Show_Frame, Rect(Rectangle_Point_y + Point(0, button_dis) * 6, Rectangle_Size), Scalar(170, 170, 170), -1);
	rectangle(Show_Frame, Rect(Rectangle_Point_y + Point(0, button_dis) * 7 + Point(-3, -3), Rectangle_Size + Size(6, 6)), Scalar(0, 0, 0), -1);
	rectangle(Show_Frame, Rect(Rectangle_Point_y + Point(0, button_dis) * 7, Rectangle_Size), Scalar(170, 170, 170), -1);

	/*////////////////////		+, - Button Text		////////////////////*/
	putText(Show_Frame, "-", Rectangle_Point_x + Point(0, button_dis) + Point(3, 12), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1);
	putText(Show_Frame, "-", Rectangle_Point_x + Point(0, button_dis) * 2 + Point(3, 12), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1);
	putText(Show_Frame, "-", Rectangle_Point_x + Point(0, button_dis) * 3 + Point(3, 12), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1);
	putText(Show_Frame, "-", Rectangle_Point_x + Point(0, button_dis) * 4 + Point(3, 12), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1);
	putText(Show_Frame, "-", Rectangle_Point_x + Point(0, button_dis) * 5 + Point(3, 12), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1);
	putText(Show_Frame, "-", Rectangle_Point_x + Point(0, button_dis) * 6 + Point(3, 12), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1);
	putText(Show_Frame, "-", Rectangle_Point_x + Point(0, button_dis) * 7 + Point(3, 12), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1);

	putText(Show_Frame, "+", Rectangle_Point_y + Point(0, button_dis) + Point(3, 12), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1);
	putText(Show_Frame, "+", Rectangle_Point_y + Point(0, button_dis) * 2 + Point(3, 12), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1);
	putText(Show_Frame, "+", Rectangle_Point_y + Point(0, button_dis) * 3 + Point(3, 12), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1);
	putText(Show_Frame, "+", Rectangle_Point_y + Point(0, button_dis) * 4 + Point(3, 12), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1);
	putText(Show_Frame, "+", Rectangle_Point_y + Point(0, button_dis) * 5 + Point(3, 12), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1);
	putText(Show_Frame, "+", Rectangle_Point_y + Point(0, button_dis) * 6 + Point(3, 12), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1);
	putText(Show_Frame, "+", Rectangle_Point_y + Point(0, button_dis) * 7 + Point(3, 12), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1);

	/*///////////////////           Negative Button		        ////////////////////*/
	rectangle(Show_Frame, Rect(Set_Button_Pt + Point(60, button_dis) - Point(3, 3), Rectangle_Size + Size(16, 6)), Scalar(0, 0, 0), -1);
	rectangle(Show_Frame, Rect(Set_Button_Pt + Point(60, button_dis), Rectangle_Size + Size(10, 0)), NP_List_Scalar[0], -1);
	rectangle(Show_Frame, Rect(Set_Button_Pt + Point(60, button_dis * 2) - Point(3, 3), Rectangle_Size + Size(16, 6)), Scalar(0, 0, 0), -1);
	rectangle(Show_Frame, Rect(Set_Button_Pt + Point(60, button_dis * 2), Rectangle_Size + Size(10, 0)), NP_List_Scalar[1], -1);
	rectangle(Show_Frame, Rect(Set_Button_Pt + Point(60, button_dis * 3) - Point(3, 3), Rectangle_Size + Size(16, 6)), Scalar(0, 0, 0), -1);
	rectangle(Show_Frame, Rect(Set_Button_Pt + Point(60, button_dis * 3), Rectangle_Size + Size(10, 0)), NP_List_Scalar[2], -1);
	rectangle(Show_Frame, Rect(Set_Button_Pt + Point(60, button_dis * 4) - Point(3, 3), Rectangle_Size + Size(16, 6)), Scalar(0, 0, 0), -1);
	rectangle(Show_Frame, Rect(Set_Button_Pt + Point(60, button_dis * 4), Rectangle_Size + Size(10, 0)), NP_List_Scalar[3], -1);
	rectangle(Show_Frame, Rect(Set_Button_Pt + Point(60, button_dis * 5) - Point(3, 3), Rectangle_Size + Size(16, 6)), Scalar(0, 0, 0), -1);
	rectangle(Show_Frame, Rect(Set_Button_Pt + Point(60, button_dis * 5), Rectangle_Size + Size(10, 0)), NP_List_Scalar[4], -1);
	rectangle(Show_Frame, Rect(Set_Button_Pt + Point(60, button_dis * 6) - Point(3, 3), Rectangle_Size + Size(16, 6)), Scalar(0, 0, 0), -1);
	rectangle(Show_Frame, Rect(Set_Button_Pt + Point(60, button_dis * 6), Rectangle_Size + Size(10, 0)), NP_List_Scalar[5], -1);
	rectangle(Show_Frame, Rect(Set_Button_Pt + Point(60, button_dis * 7) - Point(3, 3), Rectangle_Size + Size(16, 6)), Scalar(0, 0, 0), -1);
	rectangle(Show_Frame, Rect(Set_Button_Pt + Point(60, button_dis * 7), Rectangle_Size + Size(10, 0)), NP_List_Scalar[6], -1);

	/*///////////////////           Negative Text	                ////////////////////*/
	putText(Show_Frame, NP_char[0], Set_Button_Pt + Point(69, button_dis + 12), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 3);
	putText(Show_Frame, NP_char[0], Set_Button_Pt + Point(69, button_dis + 12), FONT_HERSHEY_SIMPLEX, 0.5, NP_List_Scalar[0], 1);
	putText(Show_Frame, NP_char[1], Set_Button_Pt + Point(69, button_dis * 2 + 12), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 3);
	putText(Show_Frame, NP_char[1], Set_Button_Pt + Point(69, button_dis * 2 + 12), FONT_HERSHEY_SIMPLEX, 0.5, NP_List_Scalar[1], 1);
	putText(Show_Frame, NP_char[2], Set_Button_Pt + Point(69, button_dis * 3 + 12), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 3);
	putText(Show_Frame, NP_char[2], Set_Button_Pt + Point(69, button_dis * 3 + 12), FONT_HERSHEY_SIMPLEX, 0.5, NP_List_Scalar[2], 1);
	putText(Show_Frame, NP_char[3], Set_Button_Pt + Point(69, button_dis * 4 + 12), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 3);
	putText(Show_Frame, NP_char[3], Set_Button_Pt + Point(69, button_dis * 4 + 12), FONT_HERSHEY_SIMPLEX, 0.5, NP_List_Scalar[3], 1);
	putText(Show_Frame, NP_char[4], Set_Button_Pt + Point(69, button_dis * 5 + 12), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 3);
	putText(Show_Frame, NP_char[4], Set_Button_Pt + Point(69, button_dis * 5 + 12), FONT_HERSHEY_SIMPLEX, 0.5, NP_List_Scalar[4], 1);
	putText(Show_Frame, NP_char[5], Set_Button_Pt + Point(69, button_dis * 6 + 12), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 3);
	putText(Show_Frame, NP_char[5], Set_Button_Pt + Point(69, button_dis * 6 + 12), FONT_HERSHEY_SIMPLEX, 0.5, NP_List_Scalar[5], 1);
	putText(Show_Frame, NP_char[6], Set_Button_Pt + Point(69, button_dis * 7 + 12), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 3);
	putText(Show_Frame, NP_char[6], Set_Button_Pt + Point(69, button_dis * 7 + 12), FONT_HERSHEY_SIMPLEX, 0.5, NP_List_Scalar[6], 1);

	/*/////////////////// SET BUTTON Rect, Text ////////////////////*/
	rectangle(Show_Frame, Rect(Set_Button_Pt + Point(0, button_dis) - Point(3, 3), Rectangle_Size + Size(26, 6)), Scalar(0, 0, 0), -1);
	rectangle(Show_Frame, Rect(Set_Button_Pt + Point(0, button_dis), Rectangle_Size + Size(20, 0)), Scalar(170, 170, 170), -1);
	rectangle(Show_Frame, Rect(Set_Button_Pt + Point(0, button_dis * 2) - Point(3, 3), Rectangle_Size + Size(26, 6)), Scalar(0, 0, 0), -1);
	rectangle(Show_Frame, Rect(Set_Button_Pt + Point(0, button_dis * 2), Rectangle_Size + Size(20, 0)), Scalar(170, 170, 170), -1);
	rectangle(Show_Frame, Rect(Set_Button_Pt + Point(0, button_dis * 3) - Point(3, 3), Rectangle_Size + Size(26, 6)), Scalar(0, 0, 0), -1);
	rectangle(Show_Frame, Rect(Set_Button_Pt + Point(0, button_dis * 3), Rectangle_Size + Size(20, 0)), Scalar(170, 170, 170), -1);
	rectangle(Show_Frame, Rect(Set_Button_Pt + Point(0, button_dis * 4) - Point(3, 3), Rectangle_Size + Size(26, 6)), Scalar(0, 0, 0), -1);
	rectangle(Show_Frame, Rect(Set_Button_Pt + Point(0, button_dis * 4), Rectangle_Size + Size(20, 0)), Scalar(170, 170, 170), -1);
	rectangle(Show_Frame, Rect(Set_Button_Pt + Point(0, button_dis * 5) - Point(3, 3), Rectangle_Size + Size(26, 6)), Scalar(0, 0, 0), -1);
	rectangle(Show_Frame, Rect(Set_Button_Pt + Point(0, button_dis * 5), Rectangle_Size + Size(20, 0)), Scalar(170, 170, 170), -1);
	rectangle(Show_Frame, Rect(Set_Button_Pt + Point(0, button_dis * 6) - Point(3, 3), Rectangle_Size + Size(26, 6)), Scalar(0, 0, 0), -1);
	rectangle(Show_Frame, Rect(Set_Button_Pt + Point(0, button_dis * 6), Rectangle_Size + Size(20, 0)), Scalar(170, 170, 170), -1);
	rectangle(Show_Frame, Rect(Set_Button_Pt + Point(0, button_dis * 7) - Point(3, 3), Rectangle_Size + Size(26, 6)), Scalar(0, 0, 0), -1);
	rectangle(Show_Frame, Rect(Set_Button_Pt + Point(0, button_dis * 7), Rectangle_Size + Size(20, 0)), Scalar(170, 170, 170), -1);

	putText(Show_Frame, "SET", Set_Button_Pt + Point(8, button_dis + 11), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0, 0, 0), 3);
	putText(Show_Frame, "SET", Set_Button_Pt + Point(8, button_dis + 11), FONT_HERSHEY_SIMPLEX, 0.4, Live_Scalar_List[Live_flag], 1);
	putText(Show_Frame, "SET", Set_Button_Pt + Point(8, button_dis * 2 + 11), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0, 0, 0), 3);
	putText(Show_Frame, "SET", Set_Button_Pt + Point(8, button_dis * 2 + 11), FONT_HERSHEY_SIMPLEX, 0.4, Live_Scalar_List[Live_flag], 1);
	putText(Show_Frame, "SET", Set_Button_Pt + Point(8, button_dis * 3 + 11), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0, 0, 0), 3);
	putText(Show_Frame, "SET", Set_Button_Pt + Point(8, button_dis * 3 + 11), FONT_HERSHEY_SIMPLEX, 0.4, Live_Scalar_List[Live_flag], 1);
	putText(Show_Frame, "SET", Set_Button_Pt + Point(8, button_dis * 4 + 11), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0, 0, 0), 3);
	putText(Show_Frame, "SET", Set_Button_Pt + Point(8, button_dis * 4 + 11), FONT_HERSHEY_SIMPLEX, 0.4, Live_Scalar_List[Live_flag], 1);
	putText(Show_Frame, "SET", Set_Button_Pt + Point(8, button_dis * 5 + 11), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0, 0, 0), 3);
	putText(Show_Frame, "SET", Set_Button_Pt + Point(8, button_dis * 5 + 11), FONT_HERSHEY_SIMPLEX, 0.4, Live_Scalar_List[Live_flag], 1);
	putText(Show_Frame, "SET", Set_Button_Pt + Point(8, button_dis * 6 + 11), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0, 0, 0), 3);
	putText(Show_Frame, "SET", Set_Button_Pt + Point(8, button_dis * 6 + 11), FONT_HERSHEY_SIMPLEX, 0.4, Live_Scalar_List[Live_flag], 1);
	putText(Show_Frame, "SET", Set_Button_Pt + Point(8, button_dis * 7 + 11), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0, 0, 0), 3);
	putText(Show_Frame, "SET", Set_Button_Pt + Point(8, button_dis * 7 + 11), FONT_HERSHEY_SIMPLEX, 0.4, Live_Scalar_List[Live_flag], 1);

	//-> Print
	putText(Show_Frame, "->", Set_Button_Pt + Point(-30, button_dis + 11), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0, 0, 0), 3);
	putText(Show_Frame, "->", Set_Button_Pt + Point(-30, button_dis + 11), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(255, 255, 255), 1);
	putText(Show_Frame, "->", Set_Button_Pt + Point(-30, button_dis * 2 + 11), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0, 0, 0), 3);
	putText(Show_Frame, "->", Set_Button_Pt + Point(-30, button_dis * 2 + 11), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(255, 255, 255), 1);
	putText(Show_Frame, "->", Set_Button_Pt + Point(-30, button_dis * 3 + 11), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0, 0, 0), 3);
	putText(Show_Frame, "->", Set_Button_Pt + Point(-30, button_dis * 3 + 11), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(255, 255, 255), 1);
	putText(Show_Frame, "->", Set_Button_Pt + Point(-30, button_dis * 4 + 11), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0, 0, 0), 3);
	putText(Show_Frame, "->", Set_Button_Pt + Point(-30, button_dis * 4 + 11), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(255, 255, 255), 1);
	putText(Show_Frame, "->", Set_Button_Pt + Point(-30, button_dis * 5 + 11), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0, 0, 0), 3);
	putText(Show_Frame, "->", Set_Button_Pt + Point(-30, button_dis * 5 + 11), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(255, 255, 255), 1);
	putText(Show_Frame, "->", Set_Button_Pt + Point(-30, button_dis * 6 + 11), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0, 0, 0), 3);
	putText(Show_Frame, "->", Set_Button_Pt + Point(-30, button_dis * 6 + 11), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(255, 255, 255), 1);
	putText(Show_Frame, "->", Set_Button_Pt + Point(-30, button_dis * 7 + 11), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0, 0, 0), 3);
	putText(Show_Frame, "->", Set_Button_Pt + Point(-30, button_dis * 7 + 11), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(255, 255, 255), 1);

	//OK Button, Text
	rectangle(Show_Frame, Rect(Title_Point + Point(260, button_dis * 8) + Point(-3, -3), Reset_Button_Size + Size(6, 6)), Scalar(0, 0, 0), -1);
	rectangle(Show_Frame, Rect(Title_Point + Point(260, button_dis * 8), Reset_Button_Size), Scalar(170, 170, 170), -1);
	putText(Show_Frame, "OK", Title_Point + Point(274, button_dis * 8 + 14), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1);

	//Cancel Button, Text
	rectangle(Show_Frame, Rect(Title_Point + Point(190, button_dis * 8) + Point(-3, -3), Reset_Button_Size + Size(6, 6)), Scalar(0, 0, 0), -1);
	rectangle(Show_Frame, Rect(Title_Point + Point(190, button_dis * 8), Reset_Button_Size), Scalar(170, 170, 170), -1);
	putText(Show_Frame, "Cancel", Title_Point + Point(191, button_dis * 8 + 14), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1);

	//Live Mode Button, Text
	rectangle(Show_Frame, Rect(Title_Point + Point(0, button_dis * 8) + Point(-3, -3), Reset_Button_Size + Size(6, 6)), Scalar(0, 0, 0), -1);
	rectangle(Show_Frame, Rect(Title_Point + Point(0, button_dis * 8), Reset_Button_Size), Live_Scalar_List[Live_flag], -1);
	putText(Show_Frame, "Live", Title_Point + Point(12, button_dis * 8 + 14), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 3);
	putText(Show_Frame, "Live", Title_Point + Point(12, button_dis * 8 + 14), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 1);

	//Image Viewer Button, Text
	rectangle(Show_Frame, Rect(Title_Point + Point(70, button_dis * 8) + Point(-3, -3), Reset_Button_Size + Size(6, 6)), Scalar(0, 0, 0), -1);
	rectangle(Show_Frame, Rect(Title_Point + Point(70, button_dis * 8), Reset_Button_Size), Scalar(170, 170, 170), -1);
	putText(Show_Frame, "View", Title_Point + Point(82, button_dis * 8 + 14), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 3);
	putText(Show_Frame, "View", Title_Point + Point(82, button_dis * 8 + 14), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 1);

	//ACQ Text
	putText(Show_Frame, "ACQ :", Value_Point + Point(-50, button_dis), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 3);
	putText(Show_Frame, "ACQ :", Value_Point + Point(-50, button_dis), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 1);
	putText(Show_Frame, "ACQ :", Value_Point + Point(-50, button_dis * 2), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 3);
	putText(Show_Frame, "ACQ :", Value_Point + Point(-50, button_dis * 2), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 1);
	putText(Show_Frame, "ACQ :", Value_Point + Point(-50, button_dis * 3), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 3);
	putText(Show_Frame, "ACQ :", Value_Point + Point(-50, button_dis * 3), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 1);
	putText(Show_Frame, "ACQ :", Value_Point + Point(-50, button_dis * 4), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 3);
	putText(Show_Frame, "ACQ :", Value_Point + Point(-50, button_dis * 4), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 1);
	putText(Show_Frame, "ACQ :", Value_Point + Point(-50, button_dis * 5), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 3);
	putText(Show_Frame, "ACQ :", Value_Point + Point(-50, button_dis * 5), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 1);
	putText(Show_Frame, "ACQ :", Value_Point + Point(-50, button_dis * 6), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 3);
	putText(Show_Frame, "ACQ :", Value_Point + Point(-50, button_dis * 6), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 1);
	putText(Show_Frame, "ACQ :", Value_Point + Point(-50, button_dis * 7), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 3);
	putText(Show_Frame, "ACQ :", Value_Point + Point(-50, button_dis * 7), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 1);

	//ACQ Value Text
	sprintf(Accuracy_Percent_S, "%d", Accuracy_Percent[0]);
	putText(Show_Frame, Accuracy_Percent_S, Value_Point + Point(0, button_dis), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 3);
	putText(Show_Frame, Accuracy_Percent_S, Value_Point + Point(0, button_dis), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 1);

	sprintf(Accuracy_Percent_S, "%d", Accuracy_Percent[1]);
	putText(Show_Frame, Accuracy_Percent_S, Value_Point + Point(0, button_dis) * 2, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 3);
	putText(Show_Frame, Accuracy_Percent_S, Value_Point + Point(0, button_dis) * 2, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 1);

	sprintf(Accuracy_Percent_S, "%d", Accuracy_Percent[2]);
	putText(Show_Frame, Accuracy_Percent_S, Value_Point + Point(0, button_dis) * 3, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 3);
	putText(Show_Frame, Accuracy_Percent_S, Value_Point + Point(0, button_dis) * 3, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 1);

	sprintf(Accuracy_Percent_S, "%d", Accuracy_Percent[3]);
	putText(Show_Frame, Accuracy_Percent_S, Value_Point + Point(0, button_dis) * 4, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 3);
	putText(Show_Frame, Accuracy_Percent_S, Value_Point + Point(0, button_dis) * 4, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 1);

	sprintf(Accuracy_Percent_S, "%d", Accuracy_Percent[4]);
	putText(Show_Frame, Accuracy_Percent_S, Value_Point + Point(0, button_dis) * 5, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 3);
	putText(Show_Frame, Accuracy_Percent_S, Value_Point + Point(0, button_dis) * 5, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 1);

	sprintf(Accuracy_Percent_S, "%d", Accuracy_Percent[5]);
	putText(Show_Frame, Accuracy_Percent_S, Value_Point + Point(0, button_dis) * 6, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 3);
	putText(Show_Frame, Accuracy_Percent_S, Value_Point + Point(0, button_dis) * 6, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 1);

	sprintf(Accuracy_Percent_S, "%d", Accuracy_Percent[6]);
	putText(Show_Frame, Accuracy_Percent_S, Value_Point + Point(0, button_dis) * 7, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 3);
	putText(Show_Frame, Accuracy_Percent_S, Value_Point + Point(0, button_dis) * 7, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 1);
}

void Wait_for_Cap(int ch_width) {
	double Wait_Font_Size;
	Point text_pt;

	if (width == 1280) {
		Wait_Font_Size = 1.5;
		text_pt = Point(width / 6, height / 3 * 2);
	}
	else if (width == 640) {
		Wait_Font_Size = 0.8;
		text_pt = Point(640 / 6, 480 / 3 * 2);
	}
	else {
		Wait_Font_Size = 1;
		text_pt = Point(1024 / 6, 576 / 3 * 2);
	}

	putText(Show_Frame, "Wait for Changing Resolution...", text_pt, FONT_HERSHEY_SIMPLEX, Wait_Font_Size, Scalar(0, 0, 0), 3);
	putText(Show_Frame, "Wait for Changing Resolution...", text_pt, FONT_HERSHEY_SIMPLEX, Wait_Font_Size, Scalar(255, 255, 255), 1);
	imshow("Show_Frame", Show_Frame);
	waitKey(1);
}

void Resolution_Event(int ch_width, int ch_height, char Message[]) {
	Wait_for_Cap(ch_width);

	ch_width_temp = ch_width;
	ch_height_temp = ch_height;

	for (int i = 0; i < 28; i++) {
		Inspect_ROI_Point[i] = 0;
		Inspect_ROI_Point_Temp_List[i] = 0;
	}

	cap.release();
	cap = VideoCapture(WEB_CAM);
	cap.set(3, ch_width);
	cap.set(4, ch_height);
}

void ROI_Click(int Rflag) {
	ROI_flag[Rflag] = !ROI_flag[Rflag];

	List_Scalar.clear();
	for (int i = 0; i < 7; i++) {
		if (ROI_flag[i] == 0)
			List_Scalar.push_back(Scalar(0, 0, 255, 0));
		else
			List_Scalar.push_back(Scalar(50, 255, 50, 0));
	}
}

void NP_Click(int NPflag) {
	NP_flag[NPflag] = !NP_flag[NPflag];

	NP_List_Scalar.clear();
	NP_char.clear();
	for (int i = 0; i < 7; i++) {
		if (NP_flag[i] == 0) {
			NP_List_Scalar.push_back(Scalar(0, 0, 255, 0));
			NP_char.push_back("N");
		}
		else {
			NP_List_Scalar.push_back(Scalar(50, 255, 50, 0));
			NP_char.push_back("P");
		}
	}
}

void NP_ROI_Reset() {
	NP_List_Scalar.clear();
	NP_char.clear();
	List_Scalar.clear();

	for (int i = 0; i < 7; i++) {
		if (ROI_flag[i] == 0)
			List_Scalar.push_back(Scalar(0, 0, 255, 0));
		else
			List_Scalar.push_back(Scalar(50, 255, 50, 0));
	}

	for (int i = 0; i < 7; i++) {
		if (NP_flag[i] == 0) {
			NP_List_Scalar.push_back(Scalar(0, 0, 255, 0));
			NP_char.push_back("N");
		}
		else {
			NP_List_Scalar.push_back(Scalar(50, 255, 50, 0));
			NP_char.push_back("P");
		}
	}
}

void Plu_Min_Button_Event(bool sign, int Acc_flag) {
	if (sign == 1)	Accuracy_Percent[Acc_flag]++;
	else Accuracy_Percent[Acc_flag]--;
}

void Set_ROI_Mode() {
	cap >> Set_Frame;
	flip(Set_Frame, Set_Frame, 1);
	Set_Frame_Clone = Set_Frame.clone();
	imshow("Show_Frame", Set_Frame);
}

void Live_Click(){
	Live_flag = !Live_flag;
}

vector<string> getPNGFilesInDirectory(const string& directoryPath) {
	vector<string> pngFiles;
	DIR* directory;
	struct dirent* entry;
	if ((directory = opendir(directoryPath.c_str())) != nullptr) {
		while ((entry = readdir(directory)) != nullptr) {
			string fileName = entry->d_name;
			if (entry->d_type == DT_REG && fileName.length() >= 4 && fileName.substr(fileName.length() - 4) == ".PNG") {
				pngFiles.push_back(fileName);
			}
		}
		closedir(directory);
	}

	sort(pngFiles.begin(), pngFiles.end());
	return pngFiles;
}

void Viewer_Button(Mat& View_image){
	Point bt_dis = Point(20, 0);
	Point Viewer_center = Point(View_image.cols / 2 - 15, View_image.rows / 2);
	rectangle(View_image, Rect(Viewer_center + Point(-2, -2), Size(30, 12) + Size(4, 4)), Scalar(0, 0, 0), -1);
	rectangle(View_image, Rect(Viewer_center, Size(30, 12)), Scalar(170, 170, 170), -1);
        rectangle(View_image, Rect(Viewer_center + Point(-2, -2) + bt_dis + Point(20, 0), Size(12, 12) + Size(4, 4)), Scalar(0, 0, 0), -1);
        rectangle(View_image, Rect(Viewer_center + bt_dis + Point(20, 0), Size(12, 12)), Scalar(170, 170, 170), -1);
        rectangle(View_image, Rect(Viewer_center + Point(-2, -2) + bt_dis * 2 + Point(20, 0), Size(12, 12) + Size(4, 4)), Scalar(0, 0, 0), -1);
        rectangle(View_image, Rect(Viewer_center + bt_dis * 2 + Point(20, 0), Size(12, 12)), Scalar(170, 170, 170), -1);
        rectangle(View_image, Rect(Viewer_center + Point(-2, -2) - bt_dis, Size(12, 12) + Size(4, 4)), Scalar(0, 0, 0), -1);
        rectangle(View_image, Rect(Viewer_center - bt_dis, Size(12, 12)), Scalar(170, 170, 170), -1);
        rectangle(View_image, Rect(Viewer_center + Point(-2, -2) - bt_dis * 2, Size(12, 12) + Size(4, 4)), Scalar(0, 0, 0), -1);
        rectangle(View_image, Rect(Viewer_center - bt_dis * 2, Size(12, 12)), Scalar(170, 170, 170), -1);

	putText(View_image, "<<", Viewer_center - bt_dis * 2 + Point(0, 8), FONT_HERSHEY_SIMPLEX, 0.2, Scalar(0, 0, 0), 2);
	putText(View_image, "<<", Viewer_center - bt_dis * 2 + Point(0, 8), FONT_HERSHEY_SIMPLEX, 0.2, Scalar(255, 255, 255), 1);
        putText(View_image, "<", Viewer_center - bt_dis + Point(2, 8), FONT_HERSHEY_SIMPLEX, 0.3, Scalar(0, 0, 0), 2);
        putText(View_image, "<", Viewer_center - bt_dis + Point(2, 8), FONT_HERSHEY_SIMPLEX, 0.3, Scalar(255, 255, 255), 1);
        putText(View_image, "EXIT", Viewer_center + Point(2, 9), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0, 0, 0), 2);
        putText(View_image, "EXIT", Viewer_center + Point(2, 9), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(255, 255, 255), 1);
        putText(View_image, ">>", Viewer_center + bt_dis * 2 + Point(0, 8) + Point(20, 0), FONT_HERSHEY_SIMPLEX, 0.2, Scalar(0, 0, 0), 2);
        putText(View_image, ">>", Viewer_center + bt_dis * 2 + Point(0, 8) + Point(20, 0), FONT_HERSHEY_SIMPLEX, 0.2, Scalar(255, 255, 255), 1);
        putText(View_image, ">", Viewer_center + bt_dis + Point(2, 8) + Point(20, 0), FONT_HERSHEY_SIMPLEX, 0.3, Scalar(0, 0, 0), 2);
        putText(View_image, ">", Viewer_center + bt_dis + Point(2, 8) + Point(20, 0), FONT_HERSHEY_SIMPLEX, 0.3, Scalar(255, 255, 255), 1);

	glo_Viewer_center = Viewer_center;
}

void Viewer_Click(){
	Viewer_Flag = 1;
	string directoryPath = result_image_addr;
	vector<string> pngFileNames = getPNGFilesInDirectory(directoryPath);

	if(pngFileNames.size() == 0){
		Viewer_Flag = 0;
		Viewer_key = 0;

		return;
	}

	int view_num = pngFileNames.size() - 1;
	while(true){
		string imagePath = directoryPath + "/" + pngFileNames[view_num];
		Mat View_image = imread(imagePath);
		if(!View_image.empty()){
			Viewer_Button(View_image);

			sprintf(Fput_STR, "%06d / %06d", view_num, pngFileNames.size() - 1);
                        putText(View_image, Fput_STR, Point(View_image.cols / 2 - 40, View_image.rows / 2 + 25), FONT_HERSHEY_SIMPLEX, 0.3, Scalar(0, 0, 0), 2);
                        putText(View_image, Fput_STR, Point(View_image.cols / 2 - 40, View_image.rows / 2 + 25), FONT_HERSHEY_SIMPLEX, 0.3, Scalar(255, 255, 255), 1);

			putText(View_image, pngFileNames[view_num], Point(View_image.cols / 2 - 45, View_image.rows / 2 - 10), FONT_HERSHEY_SIMPLEX, 0.3, Scalar(0, 0, 0), 3);
			putText(View_image, pngFileNames[view_num], Point(View_image.cols / 2 - 45, View_image.rows / 2 - 10), FONT_HERSHEY_SIMPLEX, 0.3, Scalar(255, 255, 255), 1);
			imshow("Show_Frame", View_image);
			waitKey(1);

			if(Viewer_key == 1){
				view_num -= 50;
				if(view_num <= -1)	view_num = pngFileNames.size() - 1;
			}
			else if(Viewer_key == 2){
                                view_num--;
                                if(view_num <= -1)       view_num = pngFileNames.size() - 1;
			}
			else if(Viewer_key == 3){
				break;
			}
			else if(Viewer_key == 4){
                                view_num++;
                                if(view_num >= pngFileNames.size()) view_num = 0;

			}
			else if(Viewer_key == 5){
                                view_num += 50;
                                if(view_num >= pngFileNames.size()) view_num = 0;
			}

			Viewer_key = 0;
		}
		else{
			continue;
		}
	}

	Viewer_Flag = 0;
	Viewer_key = 0;
}

void USB_Backup(){
	int mar = Resolution_Size.width;

	putText(Show_Frame, "Please wait...", Resolution_Pt + Point(3, 12) + Point(mar * 2 + 20, -1) + Point(150, 0), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 0), 3);
	putText(Show_Frame, "Please wait...", Resolution_Pt + Point(3, 12) + Point(mar * 2 + 20, -1) + Point(150, 0), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 255, 255), 1);
	imshow("Show_Frame", Show_Frame);
	waitKey(1);

	string path = "/media/cellon/";
	string Image_Path = "/home/cellon/CELLON/Result/";

	try {
		for (const auto& entry : fs::directory_iterator(path)) {
			if (entry.is_directory()) {
				for (const auto& image_name : fs::directory_iterator(Image_Path)) {
					if (image_name.path().extension() == ".PNG" || image_name.path().extension() == ".png") {
						string usb_path = path + entry.path().filename().string() + '/' + image_name.path().filename().string();
						string img_path = Image_Path + image_name.path().filename().string();

						Mat to_usb_image = imread(img_path, IMREAD_COLOR);
						imwrite(usb_path, to_usb_image);
					}
				}
			}
		}
	} catch (const fs::filesystem_error& ex) {
		cerr << "Error accessing directory: " << ex.what() << endl;
	}
}

void Mouse_Event(int event, int x, int y, int flags, void* userdata) {
	if(event == EVENT_LBUTTONDOWN && Viewer_Flag == 1 && Set_Flag != true){		//Viewer_Key
		Point bt_dis = Point(20, 0);
		if((glo_Viewer_center.x <= x) && (glo_Viewer_center.x + 30 >= x) && (glo_Viewer_center.y <= y) && (glo_Viewer_center.y + 12 >= y)){
			Viewer_key = 3;
		}
                else if((glo_Viewer_center.x - bt_dis.x * 2 <= x) && (glo_Viewer_center.x - bt_dis.x * 2 + 12 >= x) &&
			(glo_Viewer_center.y <= y) && (glo_Viewer_center.y + 12 >= y)){
			Viewer_key = 1;
                }
                else if((glo_Viewer_center.x - bt_dis.x <= x) && (glo_Viewer_center.x - bt_dis.x + 12 >= x) && (glo_Viewer_center.y <= y) && (glo_Viewer_center.y + 12 >= y)){
			Viewer_key = 2;
                }
                else if((glo_Viewer_center.x + bt_dis.x + 20 <= x) && (glo_Viewer_center.x + bt_dis.x + 32 >= x) && (glo_Viewer_center.y <= y) && (glo_Viewer_center.y + 12 >= y)){
			Viewer_key = 4;
                }
                else if((glo_Viewer_center.x + bt_dis.x * 2 + 20 <= x) && (glo_Viewer_center.x + bt_dis.x * 2 + 32 >= x) && (glo_Viewer_center.y <= y) && (glo_Viewer_center.y + 12 >= y)){
			Viewer_key = 5;
                }
	}

	if (Setting_Mode_Flag == true && event == EVENT_LBUTTONDOWN && Set_Flag != true && Viewer_Flag == 0) {
		int mar = Resolution_Pt.x + Resolution_Size.width + 10;
		Size Min_Plus_Button_Size = Reset_Button_Size + Size(-8, -4);

		if (((Resolution_Pt.x <= x) && (Resolution_Pt.x + Resolution_Size.width >= x)) &&
			((Resolution_Pt.y <= y) && (Resolution_Pt.y + Resolution_Size.height >= y))) {		//640x480
			char mes[20] = "Res : 640x480";
			Resolution_Event(640, 480, mes);
		}
		else if (((mar <= x) && (mar + Resolution_Size.width >= x)) &&
			((Resolution_Pt.y <= y) && (Resolution_Pt.y + Resolution_Size.height >= y))) {          //1024x576
			char mes[20] = "Res : 1024x576";
			Resolution_Event(1024, 576, mes);
		}
		else if (((mar + Resolution_Size.width + 10 <= x) && (mar + Resolution_Size.width * 2 + 10 >= x)) &&
			((Resolution_Pt.y <= y) && (Resolution_Pt.y + Resolution_Size.height >= y))) {          //1280x720
			char mes[20] = "Res : 1280x720";
			Resolution_Event(1280, 720, mes);
		}
		else if (((mar + Resolution_Size.width + 10 + 92 <= x) && (mar + Resolution_Size.width + 10 + 92  + Rectangle_Size.height >= x)) &&
			((Resolution_Pt.y <= y) && (Resolution_Pt.y + Resolution_Size.height >= y))){		//USB_BackUP Button
			USB_Backup();
		}
		else if ((ROI_Sel_Pt.x <= x) && (ROI_Sel_Pt.x + Min_Plus_Button_Size.width >= x) &&
			((ROI_Sel_Pt.y <= y) && (ROI_Sel_Pt.y + Min_Plus_Button_Size.height >= y))) {		//ROI_1 Button
			ROI_Click(0);
		}
		else if (((ROI_Sel_Pt.x <= x) && (ROI_Sel_Pt.x + Min_Plus_Button_Size.width >= x)) &&
			((ROI_Sel_Pt.y + button_dis <= y) && (ROI_Sel_Pt.y + Min_Plus_Button_Size.height + button_dis >= y))) {		//ROI_2 Button
			ROI_Click(1);
		}
		else if (((ROI_Sel_Pt.x <= x) && (ROI_Sel_Pt.x + Min_Plus_Button_Size.width >= x)) &&
			((ROI_Sel_Pt.y + button_dis * 2 <= y) && (ROI_Sel_Pt.y + Min_Plus_Button_Size.height + button_dis * 2 >= y))) {         //ROI_3 Button
			ROI_Click(2);
		}
		else if (((ROI_Sel_Pt.x <= x) && (ROI_Sel_Pt.x + Min_Plus_Button_Size.width >= x)) &&
			((ROI_Sel_Pt.y + button_dis * 3 <= y) && (ROI_Sel_Pt.y + Min_Plus_Button_Size.height + button_dis * 3 >= y))) {         //ROI_4 Button
			ROI_Click(3);
		}
		else if (((ROI_Sel_Pt.x <= x) && (ROI_Sel_Pt.x + Min_Plus_Button_Size.width >= x)) &&
			((ROI_Sel_Pt.y + button_dis * 4 <= y) && (ROI_Sel_Pt.y + Min_Plus_Button_Size.height + button_dis * 4 >= y))) {         //ROI_5 Button
			ROI_Click(4);
		}
		else if (((ROI_Sel_Pt.x <= x) && (ROI_Sel_Pt.x + Min_Plus_Button_Size.width >= x)) &&
			((ROI_Sel_Pt.y + button_dis * 5 <= y) && (ROI_Sel_Pt.y + Min_Plus_Button_Size.height + button_dis * 5 >= y))) {         //ROI_6 Button
			ROI_Click(5);
		}
		else if (((ROI_Sel_Pt.x <= x) && (ROI_Sel_Pt.x + Min_Plus_Button_Size.width >= x)) &&
			((ROI_Sel_Pt.y + button_dis * 6 <= y) && (ROI_Sel_Pt.y + Min_Plus_Button_Size.height + button_dis * 6 >= y))) {         //ROI_7 Button
			ROI_Click(6);
		}
		else if (((Rectangle_Point_x.x <= x) && (Rectangle_Point_x.x + Rectangle_Size.width >= x)) &&				//- 1
			((Rectangle_Point_x.y + button_dis <= y) && (Rectangle_Point_x.y + button_dis + Rectangle_Size.height >= y))) {
			Plu_Min_Button_Event(0, 0);
		}
		else if (((Rectangle_Point_x.x <= x) && (Rectangle_Point_x.x + Rectangle_Size.width >= x)) &&				//- 2
			((Rectangle_Point_x.y + button_dis * 2 <= y) && (Rectangle_Point_x.y + button_dis * 2 + Rectangle_Size.height >= y))) {
			Plu_Min_Button_Event(0, 1);
		}
		else if (((Rectangle_Point_x.x <= x) && (Rectangle_Point_x.x + Rectangle_Size.width >= x)) &&				//- 3
			((Rectangle_Point_x.y + button_dis * 3 <= y) && (Rectangle_Point_x.y + button_dis * 3 + Rectangle_Size.height >= y))) {
			Plu_Min_Button_Event(0, 2);
		}
		else if (((Rectangle_Point_x.x <= x) && (Rectangle_Point_x.x + Rectangle_Size.width >= x)) &&				//- 4
			((Rectangle_Point_x.y + button_dis * 4 <= y) && (Rectangle_Point_x.y + button_dis * 4 + Rectangle_Size.height >= y))) {
			Plu_Min_Button_Event(0, 3);
		}
		else if (((Rectangle_Point_x.x <= x) && (Rectangle_Point_x.x + Rectangle_Size.width >= x)) &&				//- 5
			((Rectangle_Point_x.y + button_dis * 5 <= y) && (Rectangle_Point_x.y + button_dis * 5 + Rectangle_Size.height >= y))) {
			Plu_Min_Button_Event(0, 4);
		}
		else if (((Rectangle_Point_x.x <= x) && (Rectangle_Point_x.x + Rectangle_Size.width >= x)) &&				//- 6
			((Rectangle_Point_x.y + button_dis * 6 <= y) && (Rectangle_Point_x.y + button_dis * 6 + Rectangle_Size.height >= y))) {
			Plu_Min_Button_Event(0, 5);
		}
		else if (((Rectangle_Point_x.x <= x) && (Rectangle_Point_x.x + Rectangle_Size.width >= x)) &&				//- 7
			((Rectangle_Point_x.y + button_dis * 7 <= y) && (Rectangle_Point_x.y + button_dis * 7 + Rectangle_Size.height >= y))) {
			Plu_Min_Button_Event(0, 6);
		}
		else if (((Rectangle_Point_y.x <= x) && (Rectangle_Point_y.x + Rectangle_Size.width >= x)) &&				//+ 1
			((Rectangle_Point_y.y + button_dis <= y) && (Rectangle_Point_y.y + button_dis + Rectangle_Size.height >= y))) {
			Plu_Min_Button_Event(1, 0);
		}
		else if (((Rectangle_Point_y.x <= x) && (Rectangle_Point_y.x + Rectangle_Size.width >= x)) &&				//+ 2
			((Rectangle_Point_y.y + button_dis * 2 <= y) && (Rectangle_Point_y.y + button_dis * 2 + Rectangle_Size.height >= y))) {
			Plu_Min_Button_Event(1, 1);
		}
		else if (((Rectangle_Point_y.x <= x) && (Rectangle_Point_y.x + Rectangle_Size.width >= x)) &&				//+ 3
			((Rectangle_Point_y.y + button_dis * 3 <= y) && (Rectangle_Point_y.y + button_dis * 3 + Rectangle_Size.height >= y))) {
			Plu_Min_Button_Event(1, 2);
		}
		else if (((Rectangle_Point_y.x <= x) && (Rectangle_Point_y.x + Rectangle_Size.width >= x)) &&				//+ 4
			((Rectangle_Point_y.y + button_dis * 4 <= y) && (Rectangle_Point_y.y + button_dis * 4 + Rectangle_Size.height >= y))) {
			Plu_Min_Button_Event(1, 3);
		}
		else if (((Rectangle_Point_y.x <= x) && (Rectangle_Point_y.x + Rectangle_Size.width >= x)) &&				//+ 5
			((Rectangle_Point_y.y + button_dis * 5 <= y) && (Rectangle_Point_y.y + button_dis * 5 + Rectangle_Size.height >= y))) {
			Plu_Min_Button_Event(1, 4);
		}
		else if (((Rectangle_Point_y.x <= x) && (Rectangle_Point_y.x + Rectangle_Size.width >= x)) &&				//+ 6
			((Rectangle_Point_y.y + button_dis * 6 <= y) && (Rectangle_Point_y.y + button_dis * 6 + Rectangle_Size.height >= y))) {
			Plu_Min_Button_Event(1, 5);
		}
		else if (((Rectangle_Point_y.x <= x) && (Rectangle_Point_y.x + Rectangle_Size.width >= x)) &&				//+ 7
			((Rectangle_Point_y.y + button_dis * 7 <= y) && (Rectangle_Point_y.y + button_dis * 7 + Rectangle_Size.height >= y))) {
			Plu_Min_Button_Event(1, 6);
		}
		else if (((Set_Button_Pt.x <= x) && (Set_Button_Pt.x + Rectangle_Size.width + 20 >= x)) &&				//Set 1
			((Set_Button_Pt.y + button_dis <= y) && (Set_Button_Pt.y + button_dis + Rectangle_Size.height >= y))) {
			Set_Click = 0;
		}
		else if (((Set_Button_Pt.x <= x) && (Set_Button_Pt.x + Rectangle_Size.width + 20 >= x)) &&				//Set 2
			((Set_Button_Pt.y + button_dis * 2 <= y) && (Set_Button_Pt.y + button_dis * 2 + Rectangle_Size.height >= y))) {
			Set_Click = 1;
		}
		else if (((Set_Button_Pt.x <= x) && (Set_Button_Pt.x + Rectangle_Size.width + 20 >= x)) &&				//Set 3
			((Set_Button_Pt.y + button_dis * 3 <= y) && (Set_Button_Pt.y + button_dis * 3 + Rectangle_Size.height >= y))) {
			Set_Click = 2;
		}
		else if (((Set_Button_Pt.x <= x) && (Set_Button_Pt.x + Rectangle_Size.width + 20 >= x)) &&				//Set 4
			((Set_Button_Pt.y + button_dis * 4 <= y) && (Set_Button_Pt.y + button_dis * 4 + Rectangle_Size.height >= y))) {
			Set_Click = 3;
		}
		else if (((Set_Button_Pt.x <= x) && (Set_Button_Pt.x + Rectangle_Size.width + 20 >= x)) &&				//Set 5
			((Set_Button_Pt.y + button_dis * 5 <= y) && (Set_Button_Pt.y + button_dis * 5 + Rectangle_Size.height >= y))) {
			Set_Click = 4;
		}
		else if (((Set_Button_Pt.x <= x) && (Set_Button_Pt.x + Rectangle_Size.width + 20 >= x)) &&				//Set 6
			((Set_Button_Pt.y + button_dis * 6 <= y) && (Set_Button_Pt.y + button_dis * 6 + Rectangle_Size.height >= y))) {
			Set_Click = 5;
		}
		else if (((Set_Button_Pt.x <= x) && (Set_Button_Pt.x + Rectangle_Size.width + 20 >= x)) &&				//Set 7
			((Set_Button_Pt.y + button_dis * 7 <= y) && (Set_Button_Pt.y + button_dis * 7 + Rectangle_Size.height >= y))) {
			Set_Click = 6;
		}
		else if (((Set_Button_Pt.x + 60 <= x) && (Set_Button_Pt.x + 60 + Rectangle_Size.width + 10 >= x)) &&                              //N/P 1
			((Set_Button_Pt.y + button_dis <= y) && (Set_Button_Pt.y + button_dis + Rectangle_Size.height >= y))) {
			NP_Click(0);
		}
		else if (((Set_Button_Pt.x + 60 <= x) && (Set_Button_Pt.x + 60 + Rectangle_Size.width + 10 >= x)) &&                              //N/P 2
			((Set_Button_Pt.y + button_dis * 2 <= y) && (Set_Button_Pt.y + button_dis * 2 + Rectangle_Size.height >= y))) {
			NP_Click(1);
		}
		else if (((Set_Button_Pt.x + 60 <= x) && (Set_Button_Pt.x + 60 + Rectangle_Size.width + 10 >= x)) &&                              //N/P 3
			((Set_Button_Pt.y + button_dis * 3 <= y) && (Set_Button_Pt.y + button_dis * 3 + Rectangle_Size.height >= y))) {
			NP_Click(2);
		}
		else if (((Set_Button_Pt.x + 60 <= x) && (Set_Button_Pt.x + 60 + Rectangle_Size.width + 10 >= x)) &&                              //N/P 4
			((Set_Button_Pt.y + button_dis * 4 <= y) && (Set_Button_Pt.y + button_dis * 4 + Rectangle_Size.height >= y))) {
			NP_Click(3);
		}
		else if (((Set_Button_Pt.x + 60 <= x) && (Set_Button_Pt.x + 60 + Rectangle_Size.width + 10 >= x)) &&                              //N/P 5
			((Set_Button_Pt.y + button_dis * 5 <= y) && (Set_Button_Pt.y + button_dis * 5 + Rectangle_Size.height >= y))) {
			NP_Click(4);
		}
		else if (((Set_Button_Pt.x + 60 <= x) && (Set_Button_Pt.x + 60 + Rectangle_Size.width + 10 >= x)) &&                              //N/P 6
			((Set_Button_Pt.y + button_dis * 6 <= y) && (Set_Button_Pt.y + button_dis * 6 + Rectangle_Size.height >= y))) {
			NP_Click(5);
		}
		else if (((Set_Button_Pt.x + 60 <= x) && (Set_Button_Pt.x + 60 + Rectangle_Size.width + 10 >= x)) &&                              //N/P 7
			((Set_Button_Pt.y + button_dis * 7 <= y) && (Set_Button_Pt.y + button_dis * 7 + Rectangle_Size.height >= y))) {
			NP_Click(6);
		}
		else if (((Title_Point.x <= x) && (Title_Point.x + Reset_Button_Size.width >= x)) &&
			((Title_Point.y + button_dis * 8 <= y) && (Title_Point.y + button_dis * 8 + Reset_Button_Size.height >= y))) {  //Live_Button
			Live_Click();

		}
		else if (((Title_Point.x + 70 <= x) && (Title_Point.x + 70 + Reset_Button_Size.width >= x)) &&
			((Title_Point.y + button_dis * 8 <= y) && (Title_Point.y + button_dis * 8 + Reset_Button_Size.height >= y))) {  //Image Viewer Button
			Viewer_Click();
		}
		else if ((width - 27 <= x) && (width - 5 >= x) && (height - 27 <= y) && (height - 10 >= y)){	//Result Reset Button
			Compare_Total = 0;
			Compare_Bad = 0;
			Compare_Good = 0;
		}
		else if (((Title_Point.x + 190 <= x) && (Title_Point.x + 190 + Reset_Button_Size.width >= x)) &&
			((Title_Point.y + button_dis * 8 <= y) && (Title_Point.y + button_dis * 8 + Reset_Button_Size.height >= y))) {	//Cancel_Button
			Setting_Mode_Flag = !Setting_Mode_Flag;
			if(width != ch_width_temp){
				ch_width_temp = width;
				ch_height_temp = height;

                                Wait_for_Cap(width);
                                cap.release();
                                cap = VideoCapture(WEB_CAM);
                                cap.set(3, width);
                                cap.set(4, height);
			}

			FILE* total_file = fopen(total_data_addr, "r");
			if (total_file != NULL) {
				char line[10];
				fgets(line, sizeof(line), total_file);
				Compare_Total = atoi(line);

				fclose(total_file);
			}

			FILE* good_file = fopen(good_data_addr, "r");
			if (good_file != NULL) {
				char line[10];
				fgets(line, sizeof(line), good_file);
				Compare_Good = atoi(line);

				fclose(good_file);
			}

			FILE* bad_file = fopen(bad_data_addr, "r");
			if (bad_file != NULL) {
				char line[10];
				fgets(line, sizeof(line), bad_file);
				Compare_Bad = atoi(line);

				fclose(bad_file);
			}

			FILE* live_mode_fp = fopen(Live_mode_addr, "r");
			if (live_mode_fp != NULL){
				char line[10];
				fgets(line, sizeof(line), live_mode_fp);
				Live_flag = atoi(line);

				fclose(live_mode_fp);
			}

			for (int i = 0; i < 28; i++)     Inspect_ROI_Point[i] = Inspect_ROI_Point_Temp_List[i];
			for (int i = 0; i < 7; i++) {
				Accuracy_Percent[i] = Accuracy_Percent_Temp[i];
				ROI_flag[i] = ROI_flag_Temp[i];
				NP_flag[i] = NP_flag_Temp[i];
			}

			NP_ROI_Reset();

			State_Message_S[7] = "Cancel";
			State_Message();
			State_Message_S[7] = "RUN";
			State_Message();

			flag = -1;
		}
		else if (((Title_Point.x + 260 <= x) && (Title_Point.x + 260 + Reset_Button_Size.width >= x)) &&
			((Title_Point.y + button_dis * 8 <= y) && (Title_Point.y + button_dis * 8 + Reset_Button_Size.height >= y))) {	//OK_Button
			if(width != ch_width_temp){
				width = ch_width_temp;
				height = ch_height_temp;

				Wait_for_Cap(width);
				cap.release();
				cap = VideoCapture(WEB_CAM);
				cap.set(3, width);
				cap.set(4, height);

				for (int i = 0; i < 28; i++) {
					Inspect_ROI_Point[i] = 0;
					Inspect_ROI_Point_Temp_List[i] = 0;
				}

				FILE* fp_fin = fopen(resol_data_addr, "w");
				sprintf(Fput_STR, "%d\n", width);
				fputs(Fput_STR, fp_fin);
				sprintf(Fput_STR, "%d\n", height);
				fputs(Fput_STR, fp_fin);
				fclose(fp_fin);

				sprintf(Fput_STR, "Res : %dx%d", width, height);
				State_Message_Point.clear();
				State_Message_Point.push_back(Point(15, height - 125));
				State_Message_Point.push_back(Point(15, height - 110));
				State_Message_Point.push_back(Point(15, height - 95));
				State_Message_Point.push_back(Point(15, height - 80));
				State_Message_Point.push_back(Point(15, height - 65));
				State_Message_Point.push_back(Point(15, height - 50));
				State_Message_Point.push_back(Point(15, height - 35));
				State_Message_Point.push_back(Point(15, height - 20));
				State_Message_S[7] = Fput_STR;
			}

			for (int i = 0; i < 28; i++)	Inspect_ROI_Point[i] = Inspect_ROI_Point_Temp_List[i];
			for (int i = 0; i < 7; i++)     Accuracy_Percent_Temp[i] = Accuracy_Percent[i];

			FILE* total_file = fopen(total_data_addr, "w");
			sprintf(Fput_STR, "%d\n", Compare_Total);
			fputs(Fput_STR, total_file);
			fclose(total_file);

                        FILE* good_file = fopen(good_data_addr, "w");
                        sprintf(Fput_STR, "%d\n", Compare_Good);
                        fputs(Fput_STR, good_file);
                        fclose(good_file);

                        FILE* bad_file = fopen(bad_data_addr, "w");
                        sprintf(Fput_STR, "%d\n", Compare_Bad);
                        fputs(Fput_STR, bad_file);
                        fclose(bad_file);

			FILE* fp_ROI = fopen(ROI_flag_addr, "w");
			if (fp_ROI != NULL) {
				for (int i = 0; i < 7; i++) {
					sprintf(Fput_STR, "%d\n", ROI_flag[i]);
					fputs(Fput_STR, fp_ROI);
				}

				fclose(fp_ROI);
			}

			FILE* fp_NP = fopen(NP_flag_addr, "w");
			if (fp_NP != NULL) {
				for (int i = 0; i < 7; i++) {
					sprintf(Fput_STR, "%d\n", NP_flag[i]);
					fputs(Fput_STR, fp_NP);
				}

				fclose(fp_NP);
			}

			FILE* fp_fin = fopen(Acc_addr, "w");
			if (fp_fin != NULL) {
				for (int i = 0; i < 7; i++) {
					sprintf(Fput_STR, "%d\n", Accuracy_Percent_Temp[i]);
					fputs(Fput_STR, fp_fin);
				}

				fclose(fp_fin);
			}

			FILE* fp_pnt = fopen(Point_data_addr, "w");
			if (fp_pnt != NULL) {
				for (int i = 0; i < 28; i++) {
					sprintf(Fput_STR, "%d\n", Inspect_ROI_Point[i]);
					fputs(Fput_STR, fp_pnt);
				}

				fclose(fp_pnt);
			}

			FILE* live_mode_fp = fopen(Live_mode_addr, "w");
		        if (live_mode_fp != NULL){
				sprintf(Fput_STR, "%d\n", Live_flag);
				fputs(Fput_STR, live_mode_fp);

                		fclose(live_mode_fp);
		        }

			for (int i = 0; i < 7; i++) {
				if (Standard_ROI[i].cols > 5 && Standard_ROI[i].rows > 5) {
					char Write_Name[70];
					sprintf(Write_Name, "%sROI_NUM_%d.PNG", img_data_addr, i);

					imwrite(Write_Name, Standard_ROI[i]);
				}
			}

			flag = -1;
			Setting_Mode_Flag = !Setting_Mode_Flag;
			State_Message_S[7] = "OK";
		}

	}

	////ALAM_MARK////
	if (((0 <= x) && (113 >= x)) && ((0 <= y) && (30 >= y)) && event == EVENT_LBUTTONDOWN && Set_Flag != true && Viewer_Flag == 0) {
		Setting_Mode_Flag = !Setting_Mode_Flag;

                FILE* total_file = fopen(total_data_addr, "r");
                if (total_file != NULL) {
                        char line[10];
                        fgets(line, sizeof(line), total_file);
                        Compare_Total = atoi(line);

                        fclose(total_file);
                }

                FILE* good_file = fopen(good_data_addr, "r");
                if (good_file != NULL) {
                        char line[10];
                        fgets(line, sizeof(line), good_file);
                        Compare_Good = atoi(line);

                        fclose(good_file);
                }

                FILE* bad_file = fopen(bad_data_addr, "r");
                if (bad_file != NULL) {
                        char line[10];
                        fgets(line, sizeof(line), bad_file);
                        Compare_Bad = atoi(line);

                        fclose(bad_file);
                }

                FILE* live_mode_fp = fopen(Live_mode_addr, "r");
                if (live_mode_fp != NULL){
                        char line[10];
                        fgets(line, sizeof(line), live_mode_fp);
                        Live_flag = atoi(line);

                        fclose(live_mode_fp);
                }


		for (int i = 0; i < 7; i++) {
			ROI_flag_Temp[i] = ROI_flag[i];
			NP_flag_Temp[i] = NP_flag[i];
		}

		for (int i = 0; i < 28; i++)     Inspect_ROI_Point_Temp_List[i] = Inspect_ROI_Point[i];

		Standard_ROI_Temp.clear();
		for (int i = 0; i < 7; i++) {
			if (Standard_ROI[i].empty())	Standard_ROI_Temp.push_back(Mat::zeros(1, 1, CV_8UC1));
			else	Standard_ROI_Temp.push_back(Standard_ROI[i]);
		}

		if (Setting_Mode_Flag == false) {
			if(width != ch_width_temp){
                                ch_width_temp = width;
                                ch_height_temp = height;

                                Wait_for_Cap(width);
                                cap.release();
                                cap = VideoCapture(WEB_CAM);
                                cap.set(3, width);
                                cap.set(4, height);
                        }

			for (int i = 0; i < 7; i++) {
				Accuracy_Percent[i] = Accuracy_Percent_Temp[i];
				ROI_flag[i] = ROI_flag_Temp[i];
				NP_flag[i] = NP_flag_Temp[i];
			}

			NP_ROI_Reset();
			State_Message_S[7] = "RUN";
			flag = -1;
		}
		else {
			for (int i = 0; i < 7; i++)     Accuracy_Percent_Temp[i] = Accuracy_Percent[i];

			State_Message_S[7] = "SETTING_MODE";
			flag = 3;
		}

	}
	else if (Set_Flag == false && Set_Click != -1 && event == EVENT_LBUTTONUP && Viewer_Flag == 0) {
		Set_Flag = true;
		Set_ROI_Mode();
	}
	else if (event == EVENT_LBUTTONDOWN && Set_Flag == true && Viewer_Flag == 0) {
		temp = Point(x, y);
		circle(Set_Frame, temp, 3, Scalar(0, 0, 255), -1);
		imshow("Show_Frame", Set_Frame);

		State_Message_S[7] = "POINT_CLICK";
		flag++;
	}
	else if ((flags == 33 || flags == EVENT_FLAG_LBUTTON) && event == EVENT_MOUSEMOVE && flag == 4 && Set_Flag == true && Viewer_Flag == 0) {
		rectangle(Set_Frame, Rect(temp, Point(x, y)), Scalar(0, 0, 255), 1);
		circle(Set_Frame, temp, 3, Scalar(0, 0, 255), -1);

		imshow("Show_Frame", Set_Frame);
		Set_Frame = Set_Frame_Clone.clone();
	}

	else if (event == EVENT_LBUTTONUP && flag == 4 && Viewer_Flag == 0) {
		temp2 = Point(x, y);

		Standard_ROI_Temp.clear();
		for (int i = 0; i < 7; i++)	Standard_ROI_Temp.push_back(Standard_ROI[i]);

		Standard_ROI.clear();
		for (int i = 0; i < 28; i += 4) {
			if (i / 4 == Set_Click) {
				Inspect_ROI_Point_Temp_List[i] = temp.x;
				Inspect_ROI_Point_Temp_List[i + 1] = temp.y;
				Inspect_ROI_Point_Temp_List[i + 2] = temp2.x;
				Inspect_ROI_Point_Temp_List[i + 3] = temp2.y;

				Standard_ROI.push_back(Mat(frame, Rect(Point(Inspect_ROI_Point_Temp_List[i], Inspect_ROI_Point_Temp_List[i + 1]),
					Point(Inspect_ROI_Point_Temp_List[i + 2], Inspect_ROI_Point_Temp_List[i + 3]))).clone());
			}
			else	Standard_ROI.push_back(Standard_ROI_Temp[i / 4]);
		}

		flag = 3;
		Set_Flag = false;
		Set_Click = -1;
	}
}

int getFileCount(const string& directory) {
	string command = "ls -1 " + directory + " | wc -l";
	FILE* pipe = popen(command.c_str(), "r");
	if (!pipe) {
		return -1;
	}

	char buffer[128];
	string result = "";
	while (!feof(pipe)) {
		if (fgets(buffer, 128, pipe) != NULL) {
			result += buffer;
		}
	}
	pclose(pipe);

	return stoi(result);
}

string getOldestFile(const string& directory) {
	string command = "ls -1t " + directory + " | tail -1";
	FILE* pipe = popen(command.c_str(), "r");
	if (!pipe) {
		return "";
	}

	char buffer[128];
	string result = "";
	while (!feof(pipe)) {
		if (fgets(buffer, 128, pipe) != NULL) {
		result += buffer;
		}
	}
	pclose(pipe);

	// 개행 문자 제거
	if (!result.empty() && result[result.length() - 1] == '\n') {
		result.erase(result.length() - 1);
	}

	return result;
}

bool deleteFile(const string& filePath) {
	string command = "rm " + filePath;
	int result = system(command.c_str());

	return result == 0;
}

int main() {
	cap = VideoCapture(WEB_CAM);				//WEB_CAM, TEST_VIDEO_1920P
	if (!cap.isOpened()) {
		cout << "Could not find or open Video" << endl;
		return -1;
	}

	if (wiringPiSetupGpio() == -1) {
		cout << "Could Not find or open wiringPi" << endl;
		return -1;
	}

	pinMode(INPUT_DATA_RESET, INPUT);
	pinMode(INPUT_PORT, INPUT);
	for (int i = 0; i < sizeof(Led); i++) {
		pinMode(Led[i], OUTPUT);
		digitalWrite(Led[i], 0);
	}

	Display* disp = XOpenDisplay(NULL);
	Screen* scrn = DefaultScreenOfDisplay(disp);
	int resol_height = scrn->height;
	int resol_width = scrn->width;
	XCloseDisplay(disp);

	namedWindow("Show_Frame", WINDOW_NORMAL);
	resizeWindow("Show_Frame", resol_width, resol_height);
	setMouseCallback("Show_Frame", Mouse_Event);

	width = 640;
	height = 480;

	FILE* fp_resol = fopen(resol_data_addr, "r");
	if (fp_resol != NULL) {
		fscanf(fp_resol, "%d", &width);
		fscanf(fp_resol, "%d", &height);

		fclose(fp_resol);
	}

	ch_width_temp = width;
	ch_height_temp = height;

	cap.set(3, width);
	cap.set(4, height);

	for (int i = 0; i < 10; i++) {
		cap >> frame;
		flip(frame, frame, 1);
		imshow("Show_Frame", frame);
		waitKey(1);
	}

	State_Message_Point.clear();
	State_Message_Point.push_back(Point(15, height - 125));
	State_Message_Point.push_back(Point(15, height - 110));
	State_Message_Point.push_back(Point(15, height - 95));
	State_Message_Point.push_back(Point(15, height - 80));
	State_Message_Point.push_back(Point(15, height - 65));
	State_Message_Point.push_back(Point(15, height - 50));
	State_Message_Point.push_back(Point(15, height - 35));
	State_Message_Point.push_back(Point(15, height - 20));

	State_Message_S.push_back(String("  "));
	State_Message_S.push_back(String("  "));
	State_Message_S.push_back(String("  "));
	State_Message_S.push_back(String("  "));
	State_Message_S.push_back(String("  "));
	State_Message_S.push_back(String("  "));
	State_Message_S.push_back(String("  "));
	State_Message_S.push_back(String("  "));
	State_Message_S[7] = "Res : 640x480";

	Accuracy_Percent[0] = 90;
	Accuracy_Percent[1] = 90;
	Accuracy_Percent[2] = 90;
	Accuracy_Percent[3] = 90;
	Accuracy_Percent[4] = 90;
	Accuracy_Percent[5] = 90;
	Accuracy_Percent[6] = 90;
	FILE* fp_fin = fopen(Acc_addr, "r");
	if (fp_fin != NULL) {
		for (int i = 0; i < 7; i++)	fscanf(fp_fin, "%d", &Accuracy_Percent[i]);
		fclose(fp_fin);
	}

	for (int i = 0; i < 28; i++) {
		Inspect_ROI_Point_Temp_List[i] = 0;
		Inspect_ROI_Point[i] = 0;
	}

	Live_Scalar_List.clear();
	Live_Scalar_List.push_back(Scalar(0, 0, 255));
	Live_Scalar_List.push_back(Scalar(0, 255, 0));

	List_Scalar.clear();
	NP_List_Scalar.clear();
	NP_char.clear();
	Standard_ROI.clear();
	Standard_ROI_Temp.clear();
	for (int i = 0; i < 7; i++) {
		List_Scalar.push_back(Scalar(0, 0, 255, 0));
		NP_List_Scalar.push_back(Scalar(50, 255, 50, 0));
		NP_char.push_back("P");
	}

	FILE* fp_ROI = fopen(ROI_flag_addr, "r");
	if (fp_ROI != NULL) {
		for (int i = 0; i < 7; i++)	fscanf(fp_ROI, "%d", &ROI_flag[i]);

		List_Scalar.clear();
		for (int i = 0; i < 7; i++) {
			if (ROI_flag[i] == 0)
				List_Scalar.push_back(Scalar(0, 0, 255, 0));
			else
				List_Scalar.push_back(Scalar(50, 255, 50, 0));
		}

		fclose(fp_ROI);
	}

	FILE* fp_NP = fopen(NP_flag_addr, "r");
	if (fp_NP != NULL) {
		for (int i = 0; i < 7; i++)     fscanf(fp_NP, "%d", &NP_flag[i]);

		NP_List_Scalar.clear();
		NP_char.clear();
		for (int i = 0; i < 7; i++) {
			if (NP_flag[i] == 0) {
				NP_List_Scalar.push_back(Scalar(0, 0, 255, 0));
				NP_char.push_back("N");
			}
			else {
				NP_List_Scalar.push_back(Scalar(50, 255, 50, 0));
				NP_char.push_back("P");
			}
		}

		fclose(fp_ROI);
	}

	FILE* total_file = fopen(total_data_addr, "r");
	if (total_file != NULL) {
		char line[10];
		fgets(line, sizeof(line), total_file);
		Compare_Total = atoi(line);

		fclose(total_file);
	}

	FILE* good_file = fopen(good_data_addr, "r");
	if (good_file != NULL) {
		char line[10];
		fgets(line, sizeof(line), good_file);
		Compare_Good = atoi(line);

		fclose(good_file);
	}

	FILE* bad_file = fopen(bad_data_addr, "r");
	if (bad_file != NULL) {
		char line[10];
		fgets(line, sizeof(line), bad_file);
		Compare_Bad = atoi(line);

		fclose(bad_file);
	}

	FILE* fp_pnt = fopen(Point_data_addr, "r");
	if (fp_pnt != NULL) {
		for (int i = 0; i < 28; i++)	fscanf(fp_pnt, "%d", &Inspect_ROI_Point[i]);

		fclose(fp_pnt);
	}

	FILE* live_mode_fp = fopen(Live_mode_addr, "r");
	if (live_mode_fp != NULL){
		char line[10];
		fgets(line, sizeof(line), live_mode_fp);
		Live_flag = atoi(line);

		fclose(live_mode_fp);
	}


	for (int i = 0; i < 7; i++) {
		char Fread_Path[70];
		sprintf(Fread_Path, "%sROI_NUM_%d.PNG", img_data_addr, i);

		Mat Fread_ROI_Img = imread(Fread_Path);
		if (Fread_ROI_Img.empty())	Standard_ROI.push_back(Mat::zeros(1, 1, CV_8UC1));
		else	Standard_ROI.push_back(Fread_ROI_Img);
	}

	digitalWrite(Led[0], 1);

	int key = 0;
	int key_temp = 0;
	bool  keyboard_start = false;
	clock_t start, end;
	bool	reset_signal = false;
	while (1) {
		start_signal = !digitalRead(INPUT_PORT);
		if (start_signal == true) {
			delay(50);
			start_signal = !digitalRead(INPUT_PORT);

			if (start_signal == true)
				digitalWrite(Led[0], 0);
		}
		else {
			digitalWrite(Led[0], 1);
			digitalWrite(Led[1], 0);
			digitalWrite(Led[2], 0);
		}

		reset_signal = digitalRead(INPUT_DATA_RESET);
		if (start_signal == false) {
			delay(50);
			reset_signal = digitalRead(INPUT_DATA_RESET);

			if (reset_signal == false) {
				Compare_Total = 0;
				Compare_Good = 0;
				Compare_Bad = 0;

				FILE* total_file = fopen(total_data_addr, "w");
				sprintf(Fput_STR, "%d\n", Compare_Total);
				fputs(Fput_STR, total_file);
				fclose(total_file);

				FILE* good_file = fopen(bad_data_addr, "w");
				sprintf(Fput_STR, "%d\n", Compare_Bad);
				fputs(Fput_STR, good_file);
				fclose(good_file);

				FILE* bad_file = fopen(good_data_addr, "w");
				sprintf(Fput_STR, "%d\n", Compare_Good);
				fputs(Fput_STR, bad_file);
				fclose(bad_file);
			}
		}


		start = clock();

		if(Live_flag == 1){
			cap >> frame;
			if (frame.empty())
				break;

			flip(frame, frame, 1);
		}

		Show_Frame = frame.clone();

		Compare_State(Scalar(0, 255, 255));

		if (flag == 1) {
			circle(Show_Frame, temp, 2, Scalar(0, 0, 255), -1);

			char show_state_run[10] = "[SET]";
			putText(Show_Frame, show_state_run, Point(width / 2, 25) + Point(-120, -2), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 4);
			putText(Show_Frame, show_state_run, Point(width / 2, 25) + Point(-120, -2), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1);
			Center_Marking(width, height);
		}
                else if (flag == 3 && Setting_Mode_Flag == true && Live_flag == true) {
                        Setting_Mode();

                        char show_state_run[10] = "[SET]";
                        putText(Show_Frame, show_state_run, Point(width / 2, 25) + Point(-120, -2), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 4);
                        putText(Show_Frame, show_state_run, Point(width / 2, 25) + Point(-120, -2), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1);

                }
	        else if (flag == 3 && Setting_Mode_Flag == true && Live_flag == false) {
                        Setting_Mode();
			State_Message();
			ALAM_Marking();
			Compare_Result();

                        char show_state_run[10] = "[SET]";
                        putText(Show_Frame, show_state_run, Point(width / 2, 25) + Point(-120, -2), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 4);
                        putText(Show_Frame, show_state_run, Point(width / 2, 25) + Point(-120, -2), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1);

			imshow("Show_Frame", Show_Frame);
		}
		else if (flag == -1 && (start_signal == true ||  keyboard_start == true)) {
			cap >> frame;
			if (frame.empty())
				break;

			flip(frame, frame, 1);

			double		result;
			double		maxv;
			char		maxv_S[11];
			char		result_S[10];
			bool		Compare_Flag = true;
			string		save_roi_maxv[7];

			for (int i = 0; i < 28; i += 4) {
				Inspect_ROI = Mat(frame, Rect(Point(Inspect_ROI_Point[i], Inspect_ROI_Point[i + 1]),
					Point(Inspect_ROI_Point[i + 2], Inspect_ROI_Point[i + 3])));

				if (ROI_flag[i / 4] && (Inspect_ROI.cols > 5 && Inspect_ROI.rows > 5)) {
					Mat Inspect_ROI_blur, Standard_ROI_blur;
					blur(Inspect_ROI, Inspect_ROI_blur, Size(9, 9));
					blur(Standard_ROI[i / 4], Standard_ROI_blur, Size(9, 9));

					double Blue = 0, Green = 0, Red = 0, total_value = 0, total_pixel;
					double std_Blue = 0, std_Green = 0, std_Red = 0, std_total_value = 0;
					total_pixel = Inspect_ROI_blur.cols * Inspect_ROI_blur.rows;
					for (int x = 0; x < Inspect_ROI_blur.rows; x++) {
						for (int y = 0; y < Inspect_ROI_blur.cols; y++) {
							Blue += (int)Inspect_ROI_blur.at<Vec3b>(x, y)[0];
							Green += (int)Inspect_ROI_blur.at<Vec3b>(x, y)[1];
							Red += (int)Inspect_ROI_blur.at<Vec3b>(x, y)[2];
						}
					}

					for (int x = 0; x < Standard_ROI_blur.rows; x++) {
						for (int y = 0; y < Standard_ROI_blur.cols; y++) {
							std_Blue += (int)Standard_ROI_blur.at<Vec3b>(x, y)[0];
							std_Green += (int)Standard_ROI_blur.at<Vec3b>(x, y)[1];
							std_Red += (int)Standard_ROI_blur.at<Vec3b>(x, y)[2];
						}
					}

					int sum = Blue + Green + Red;
					double ratio_blue = (double)Blue / sum;
					double ratio_green = (double)Green / sum;
					double ratio_red = (double)Red / sum;

					int std_sum = std_Blue + std_Green + std_Red;
					double std_ratio_blue = (double)std_Blue / std_sum;
					double std_ratio_green = (double)std_Green / std_sum;
					double std_ratio_red = (double)std_Red / std_sum;

					double diff[3];
					diff[0] = fabs(ratio_blue - std_ratio_blue);
					diff[1] = fabs(ratio_green - std_ratio_green);
					diff[2] = fabs(ratio_red - std_ratio_red);

					maxv = (diff[0] + diff[1] + diff[2]) * 100;

					maxv = 100 - maxv;

					sprintf(maxv_S, "%.2f", maxv);
					save_roi_maxv[i / 4] = to_string(i / 4 + 1) + " : " + maxv_S;

					sprintf(maxv_S, "%d : %.2f", i / 4 + 1, maxv);
					State_Message_S[7] = maxv_S;
					State_Message();

					if (NP_flag[i / 4] == true) {
						if (maxv < Accuracy_Percent[i / 4])	Compare_Flag = false;
					}
					else {
						if (maxv > Accuracy_Percent[i / 4])	Compare_Flag = false;
					}
				}
			}

			end = clock();
			result = (double)(end - start);
			sprintf(result_S, "%.2fms", result / CLOCKS_PER_SEC);
			State_Message_S[7] = result_S;

			Compare_Total++;

			FILE* total_file = fopen(total_data_addr, "w");
			sprintf(Fput_STR, "%d\n", Compare_Total);
			fputs(Fput_STR, total_file);
			fclose(total_file);

			time_t now = time(nullptr);
			tm* currentTime = localtime(&now);
			int year = currentTime->tm_year + 1900;
			int month = currentTime->tm_mon + 1;
			int day = currentTime->tm_mday;
			int hour = currentTime->tm_hour;
			int minute = currentTime->tm_min;
			int second = currentTime->tm_sec;

			char timestamp[20];
			snprintf(timestamp, sizeof(timestamp), "%02d%02d%02d%02d%02d%02d", 
				year % 100, month, day, hour, minute, second);


			int fileCount = getFileCount(result_image_addr);

//			Change Value
			if(5000 < fileCount){
				string oldestFile = getOldestFile(result_image_addr);
				oldestFile = string(result_image_addr) + oldestFile;
				deleteFile(oldestFile);
			}

			Mat write_image;
			Size newSize(320, 240);
			resize(frame, write_image, newSize);

			Point wp  = Point(10, 160);
			for(int i = 0; i < 7; i++){
				if(ROI_flag[i]){
					putText(write_image, save_roi_maxv[i], wp, FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0, 0, 0), 2);
					putText(write_image, save_roi_maxv[i], wp, FONT_HERSHEY_SIMPLEX, 0.4, Scalar(255, 255, 255), 1);
					wp += Point(0, 10);
				}
			}

			for (int j = 0; j < 28; j += 4) {
	                        if (ROI_flag[j / 4]) {
	                                char num_str[2];
	                                int num = j / 4 + 1;
	                                sprintf(num_str, "%d", num);
	                                putText(write_image, num_str, Point(Inspect_ROI_Point[j] / 2, Inspect_ROI_Point[j + 1] / 2) - Point(5, 3), FONT_HERSHEY_SIMPLEX,
	                                        0.5, Scalar(0, 0, 255), 0.5);
	                                rectangle(write_image, Rect(Point(Inspect_ROI_Point[j] / 2, Inspect_ROI_Point[j + 1] / 2),
	                                        Point(Inspect_ROI_Point[j + 2] / 2, Inspect_ROI_Point[j + 3] / 2)), Scalar(0, 0, 255), 1);
	                        }
	                }

			if (Compare_Flag == false) {
				Compare_Bad++;
				Compare_State(Scalar(0, 0, 255));

				sprintf(Fput_STR, "%s%s_NG.PNG", result_image_addr, timestamp);
				imwrite(Fput_STR, write_image);

				FILE* fp_fin = fopen(bad_data_addr, "w");
				sprintf(Fput_STR, "%d\n", Compare_Bad);
				fputs(Fput_STR, fp_fin);
				fclose(fp_fin);

				digitalWrite(Led[2], 1);
			}
			else {
				Compare_Good++;
				Compare_State(Scalar(0, 255, 0));

                                sprintf(Fput_STR, "%s%s_OK.PNG", result_image_addr, timestamp);
                                imwrite(Fput_STR, write_image);

				FILE* fp_fin = fopen(good_data_addr, "w");
				sprintf(Fput_STR, "%d\n", Compare_Good);
				fputs(Fput_STR, fp_fin);
				fclose(fp_fin);

				digitalWrite(Led[1], 1);
			}
			Compare_Flag = true;

			for (int j = 0; j < 28; j += 4) {
				if (ROI_flag[j / 4]) {
					char num_str[2];
					int num = j / 4 + 1;
					sprintf(num_str, "%d", num);
					putText(Show_Frame, num_str, Point(Inspect_ROI_Point[j], Inspect_ROI_Point[j + 1]) - Point(5, 3), FONT_HERSHEY_SIMPLEX,
						0.5, Scalar(0, 0, 255), 0.5);
					rectangle(Show_Frame, Rect(Point(Inspect_ROI_Point[j], Inspect_ROI_Point[j + 1]),
						Point(Inspect_ROI_Point[j + 2], Inspect_ROI_Point[j + 3])), Scalar(0, 0, 255), 1);
				}
			}

			char show_state_run[10] = "[RUN]";
			putText(Show_Frame, show_state_run, Point(width / 2, 25) + Point(-120, -2), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 4);
			putText(Show_Frame, show_state_run, Point(width / 2, 25) + Point(-120, -2), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 1);

			Center_Marking(width, height);

			keyboard_start = false;
			start_signal = !digitalRead(INPUT_PORT);
			while (start_signal) {
				start_signal = !digitalRead(INPUT_PORT);
			}

			imshow("Show_Frame", Show_Frame);
		}
		else if (flag == -1) {
			for (int i = 0; i < 28; i += 4) {
				if (ROI_flag[i / 4]) {
					char num_str[2];
					int num = i / 4 + 1;
					sprintf(num_str, "%d", num);
					putText(Show_Frame, num_str, Point(Inspect_ROI_Point[i], Inspect_ROI_Point[i + 1]) - Point(5, 3), FONT_HERSHEY_SIMPLEX,
						0.5, Scalar(0, 0, 255), 0.5);
					rectangle(Show_Frame, Rect(Point(Inspect_ROI_Point[i], Inspect_ROI_Point[i + 1]),
						Point(Inspect_ROI_Point[i + 2], Inspect_ROI_Point[i + 3])), Scalar(0, 0, 255), 1);
				}
			}

			char show_state_run[10] = "[RUN]";
			putText(Show_Frame, show_state_run, Point(width / 2, 25) + Point(-120, -2), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 4);
			putText(Show_Frame, show_state_run, Point(width / 2, 25) + Point(-120, -2), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 1);

			Center_Marking(width, height);
		}

		State_Message();
		ALAM_Marking();
		Compare_Result();

		if (Set_Flag == false)
			imshow("Show_Frame", Show_Frame);

		key = waitKey(33);
		key_temp += key;
		if(key_temp == 324 || key_temp == 292)
			keyboard_start = true;
		else
			key_temp = 0;

		key_temp = key;
	}

	for (int i = 0; i < sizeof(Led); i++) {
		pinMode(Led[i], OUTPUT);
		digitalWrite(Led[i], 0);
	}

	return 0;
}
