//#include <stdio.h>
#include <iostream>
#include <time.h>
#include "opencv2/opencv.hpp"
#include <wiringPi.h>

#define WEB_CAM		0

using namespace std;
using namespace cv;

unsigned int	Number_of_ROI = 1;
unsigned int	Accuracy_Percent = 90;
unsigned int	Number_of_ROI_Temp;
unsigned int	Accuracy_Percent_Temp;
int		flag = 0;
bool		Setting_Mode_Flag = false;
bool		Config_Flag = false;
unsigned int	Compare_Total, Compare_Good, Compare_Bad;

VideoCapture	cap;
Mat		frame, Show_Frame, Inspect_ROI;
vector<Mat>	Standard_ROI;
Point		temp, temp2;
vector<Point>	Inspect_ROI_Point;
vector<Point>	State_Message_Point;
vector<String>	State_Message_S;
Point		Rectangle_Point_x = Point(90, 90);
Point		Rectangle_Point_y = Point(165, 90);
Size		Rectangle_Size = Size(20, 15);
Point		Reset_Button_Point = Point(135, 230);
Size		Reset_Button_Size = Point(50, 20);
int		button_dis = 30;
double		Message_Font_Size = 0.4;
Point		Resolution_Point = Point(90, 180);
unsigned int	width, height;
int		State_Message_Flag = 0;
char		img_data_addr[40] = "/home/cellon/CELLON/";
char		data_addr[40] = "/home/cellon/CELLON/Log_data.txt";

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
		putText(Show_Frame, State_Message_S[i], State_Message_Point[i], FONT_HERSHEY_DUPLEX, Message_Font_Size, Scalar(0, 0, 0), 2);
		putText(Show_Frame, State_Message_S[i], State_Message_Point[i], FONT_HERSHEY_DUPLEX, Message_Font_Size, Scalar(125, 125, 125), 1);
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

	putText(Show_Frame, Compare_Total_S, Point(width - 135, height - 70), FONT_HERSHEY_DUPLEX, 0.5, Scalar(0, 0, 0), 3);
	putText(Show_Frame, Compare_Total_S, Point(width - 135, height - 70), FONT_HERSHEY_DUPLEX, 0.5, Scalar(0, 255, 0), 1);

	putText(Show_Frame, Compare_Good_S, Point(width - 135, height - 55), FONT_HERSHEY_DUPLEX, 0.5, Scalar(0, 0, 0), 3);
	putText(Show_Frame, Compare_Good_S, Point(width - 135, height - 55), FONT_HERSHEY_DUPLEX, 0.5, Scalar(0, 255, 0), 1);

	putText(Show_Frame, Compare_Bad_S, Point(width - 135, height - 40), FONT_HERSHEY_DUPLEX, 0.5, Scalar(0, 0, 0), 3);
	putText(Show_Frame, Compare_Bad_S, Point(width - 135, height - 40), FONT_HERSHEY_DUPLEX, 0.5, Scalar(0, 255, 0), 1);

}



void ALAM_Marking() {
	putText(Show_Frame, "=", Point(14, 42), FONT_HERSHEY_DUPLEX, 1, Scalar(0, 0, 0), 5);
	putText(Show_Frame, "CELLON", Point(45, 40), FONT_HERSHEY_DUPLEX, 0.6, Scalar(0, 0, 0), 4);
	putText(Show_Frame, "=", Point(14, 42), FONT_HERSHEY_DUPLEX, 1, Scalar(0, 255, 255), 2);
	putText(Show_Frame, "CELLON", Point(45, 40), FONT_HERSHEY_DUPLEX, 0.6, Scalar(0, 255, 255), 1);
}



void Center_Marking(double width, double height) {
	line(Show_Frame, Point(width / 2, height / 2 + 15), Point(width / 2, height / 2 - 15), Scalar(0, 0, 255), 1);
	line(Show_Frame, Point(width / 2 + 15, height / 2), Point(width / 2 - 15, height / 2), Scalar(0, 0, 255), 1);
}

void Setting_Mode() {
	char	Accuracy_Percent_S[5];
	char	Number_of_ROI_S[5];
	Point	Title_Point = Point(5, 102);
	Point	Value_Point = Point(128, 102);

	rectangle(Show_Frame, Rect(Title_Point - Point(2, 30) - Point(3, 3), Title_Point + Point(195, 180) + Point(3, 3)), Scalar(0, 0, 0), -1);
	rectangle(Show_Frame, Rect(Title_Point - Point(2, 30), Title_Point + Point(195, 180)), Scalar(125, 125, 125), -1);

	sprintf(Accuracy_Percent_S, "%d", Accuracy_Percent);
	sprintf(Number_of_ROI_S, "%d", Number_of_ROI);

	putText(Show_Frame, "Accuracy", Title_Point, FONT_HERSHEY_DUPLEX, 0.5, Scalar(0, 0, 0), 3);
	putText(Show_Frame, "Accuracy", Title_Point, FONT_HERSHEY_DUPLEX, 0.5, Scalar(255, 255, 255), 1);
	putText(Show_Frame, "ROI_Num", Title_Point + Point(0, button_dis), FONT_HERSHEY_DUPLEX, 0.5, Scalar(0, 0, 0), 3);
	putText(Show_Frame, "ROI_Num", Title_Point + Point(0, button_dis), FONT_HERSHEY_DUPLEX, 0.5, Scalar(255, 255, 255), 1);

	/*////////////////////		+, - Button		////////////////////*/
	rectangle(Show_Frame, Rect(Rectangle_Point_x + Point(-3, -3), Rectangle_Size + Size(6, 6)), Scalar(0, 0, 0), -1);
	rectangle(Show_Frame, Rect(Rectangle_Point_x + Point(0, button_dis) + Point(-3, -3), Rectangle_Size + Size(6, 6)), Scalar(0, 0, 0), -1);
	rectangle(Show_Frame, Rect(Rectangle_Point_x, Rectangle_Size), Scalar(170, 170, 170), -1);
	rectangle(Show_Frame, Rect(Rectangle_Point_x + Point(0, button_dis), Rectangle_Size), Scalar(170, 170, 170), -1);

	putText(Show_Frame, "-", Rectangle_Point_x + Point(3, 12), FONT_HERSHEY_DUPLEX, 0.5, Scalar(0, 0, 0), 1);
	putText(Show_Frame, "-", Rectangle_Point_x + Point(0, button_dis) + Point(3, 12), FONT_HERSHEY_DUPLEX, 0.5, Scalar(0, 0, 0), 1);


	rectangle(Show_Frame, Rect(Rectangle_Point_y + Point(-3, -3), Rectangle_Size + Size(6, 6)), Scalar(0, 0, 0), -1);
	rectangle(Show_Frame, Rect(Rectangle_Point_y + Point(0, button_dis) + Point(-3, -3), Rectangle_Size + Size(6, 6)), Scalar(0, 0, 0), -1);
	rectangle(Show_Frame, Rect(Rectangle_Point_y, Rectangle_Size), Scalar(170, 170, 170), -1);
	rectangle(Show_Frame, Rect(Rectangle_Point_y + Point(0, button_dis), Rectangle_Size), Scalar(170, 170, 170), -1);

	putText(Show_Frame, "+", Rectangle_Point_y + Point(3, 12), FONT_HERSHEY_DUPLEX, 0.5, Scalar(0, 0, 0), 1);
	putText(Show_Frame, "+", Rectangle_Point_y + Point(0, button_dis) + Point(3, 12), FONT_HERSHEY_DUPLEX, 0.5, Scalar(0, 0, 0), 1);

	//OK Button

	rectangle(Show_Frame, Rect(Resolution_Point - Point(-45, -10 - button_dis - button_dis) + Point(-3, -3),
		Reset_Button_Size + Size(6, 6)), Scalar(0, 0, 0), -1);
	rectangle(Show_Frame, Rect(Resolution_Point - Point(-45, -10 - button_dis - button_dis), Reset_Button_Size), Scalar(170, 170, 170), -1);
	putText(Show_Frame, "OK", Resolution_Point - Point(-55, -25 - button_dis - button_dis), FONT_HERSHEY_DUPLEX, 0.5, Scalar(0, 0, 0), 1);

	//Cancel Button
	rectangle(Show_Frame, Rect(Resolution_Point - Point(15, -10 - button_dis - button_dis) + Point(-3, -3),
		Reset_Button_Size + Size(6, 6)), Scalar(0, 0, 0), -1);
	rectangle(Show_Frame, Rect(Resolution_Point - Point(15, -10 - button_dis - button_dis), Reset_Button_Size), Scalar(170, 170, 170), -1);
	putText(Show_Frame, "Cancel", Resolution_Point - Point(17, -25 - button_dis - button_dis), FONT_HERSHEY_DUPLEX, 0.5, Scalar(0, 0, 0), 1);

	putText(Show_Frame, Accuracy_Percent_S, Value_Point, FONT_HERSHEY_DUPLEX, 0.5, Scalar(0, 0, 0), 3);
	putText(Show_Frame, Accuracy_Percent_S, Value_Point, FONT_HERSHEY_DUPLEX, 0.5, Scalar(255, 255, 255), 1);
	putText(Show_Frame, Number_of_ROI_S, Value_Point + Point(0, button_dis), FONT_HERSHEY_DUPLEX, 0.5, Scalar(0, 0, 0), 3);
	putText(Show_Frame, Number_of_ROI_S, Value_Point + Point(0, button_dis), FONT_HERSHEY_DUPLEX, 0.5, Scalar(255, 255, 255), 1);
	//putText(Show_Frame, "0", Value_Point + Point(0, 100), FONT_HERSHEY_DUPLEX, 0.5, Scalar(0, 0, 0), 3);
	//putText(Show_Frame, "0", Value_Point + Point(0, 100), FONT_HERSHEY_DUPLEX, 0.5, Scalar(255, 255, 255), 1);
}



void Mouse_Event(int event, int x, int y, int flags, void* userdata) {
	if (Setting_Mode_Flag == true && event == EVENT_LBUTTONDOWN) {
		if (((Rectangle_Point_x.x <= x) && (Rectangle_Point_x.x + Rectangle_Size.width >= x)) &&
			((Rectangle_Point_x.y <= y) && (Rectangle_Point_x.y + Rectangle_Size.height >= y))) {		//1-1
			if (Accuracy_Percent >= 90) {
				Accuracy_Percent--;
			}
			else {

				Accuracy_Percent -= 2;
			}

			if (Accuracy_Percent == -1 || Accuracy_Percent == -2) {
				Accuracy_Percent = 0;
			}
		}

		if (((Rectangle_Point_y.x <= x) && (Rectangle_Point_y.x + Rectangle_Size.width >= x)) &&
			((Rectangle_Point_y.y <= y) && (Rectangle_Point_y.y + Rectangle_Size.height >= y))) {		//1-2
			if (Accuracy_Percent >= 90) {
				Accuracy_Percent++;
			}
			else {
				Accuracy_Percent += 2;
			}

			if (Accuracy_Percent == 101) {
				Accuracy_Percent = 100;
			}
		}

		if ((Rectangle_Point_x.x <= x) && (Rectangle_Point_x.x + Rectangle_Size.width >= x) &&

			((Rectangle_Point_x.y + button_dis <= y) && (Rectangle_Point_x.y + button_dis + Rectangle_Size.height >= y))) {		//2-1

			Number_of_ROI--;

			if (Number_of_ROI == 0) {

				Number_of_ROI = 1;

			}



		}

		if (((Rectangle_Point_y.x <= x) && (Rectangle_Point_y.x + Rectangle_Size.width >= x)) &&
			((Rectangle_Point_y.y + button_dis <= y) && (Rectangle_Point_y.y + button_dis + Rectangle_Size.height >= y))) {		//2-2
			Number_of_ROI++;
			if (Number_of_ROI == 6) {
				Number_of_ROI = 5;
			}

		}

		if (((Resolution_Point.x - 15 <= x) && (Resolution_Point.x - 15 + Reset_Button_Size.width >= x)) &&
			((Resolution_Point.y + 10 + button_dis + button_dis <= y) &&
				(Resolution_Point.y + 10 + button_dis + button_dis + Reset_Button_Size.height >= y))) {		//Cancel_Button
			Number_of_ROI = Number_of_ROI_Temp;
			Accuracy_Percent = Accuracy_Percent_Temp;
			Setting_Mode_Flag = !Setting_Mode_Flag;
			State_Message_S[7] = "Cancel";
			State_Message();

			if (Inspect_ROI_Point.size() / 2 == Number_of_ROI) {
				State_Message_S[7] = "COMPARING_MODE";
				flag = -1;
			}
			else {

				State_Message_S[7] = "Cancel";
				flag = 0;
			}
		}

		if (((Resolution_Point.x + 45 <= x) && (Resolution_Point.x + 45 + Reset_Button_Size.width >= x)) &&
			((Resolution_Point.y + 10 + button_dis + button_dis <= y) &&
				(Resolution_Point.y + 10 + button_dis + button_dis + Reset_Button_Size.height >= y))) {		//OK_Button
			if(Inspect_ROI_Point.size() / 2 != Number_of_ROI){
				Inspect_ROI_Point.clear();
				Standard_ROI.clear();
				flag = 0;
			}
			else if (Number_of_ROI == Number_of_ROI_Temp && Accuracy_Percent == Accuracy_Percent_Temp) {
				flag = -1;
			}
			else if (Number_of_ROI != Number_of_ROI_Temp && Accuracy_Percent == Accuracy_Percent_Temp) {
				Inspect_ROI_Point.clear();
				Standard_ROI.clear();
				flag = 0;
			}
			else if (Number_of_ROI == Number_of_ROI_Temp && Accuracy_Percent != Accuracy_Percent_Temp) {
				flag = -1;
			}
			else if (Number_of_ROI != Number_of_ROI_Temp && Accuracy_Percent != Accuracy_Percent_Temp) {
				Inspect_ROI_Point.clear();
				Standard_ROI.clear();
				flag = 0;
			}

			Setting_Mode_Flag = !Setting_Mode_Flag;
			State_Message_S[7] = "OK";
		}
	}

	////ALAM_MARK////
	if (((10 <= x) && (105 >= x)) && ((15 <= y) && (50 >= y)) && event == EVENT_LBUTTONDOWN) {
		Setting_Mode_Flag = !Setting_Mode_Flag;

		Number_of_ROI_Temp = Number_of_ROI;
		Accuracy_Percent_Temp = Accuracy_Percent;

		if (Setting_Mode_Flag == false) {
			if (Inspect_ROI_Point.size() / 2 == Number_of_ROI) {
				State_Message_S[7] = "COMPARING_MODE";
				flag = -1;
			}
			else {
				State_Message_S[7] = "POINT_CLICK";
				flag = 0;
			}
		}
		else {
			State_Message_S[7] = "SETTING_MODE";

		flag = 3;
		}
	}

	if (event == EVENT_LBUTTONDBLCLK && (flag == 0 || flag == 1)) {
		temp = Point(x, y);
		Inspect_ROI_Point.push_back(temp);
		State_Message_S[7] = "POINT_CLICK";
		flag++;
	}

	if (flags == EVENT_FLAG_LBUTTON && event == EVENT_MOUSEMOVE && flag == 1) {
		rectangle(Show_Frame, Rect(temp, Point(x, y)), Scalar(0, 0, 255), 2);
		imshow("Show_Frame", Show_Frame);
	}

	if (event == EVENT_LBUTTONUP && flag == 1) {
		temp2 = Point(x, y);
		if (temp2 != temp) {
			Inspect_ROI_Point.push_back(temp2);
			flag++;
		}
		else if (temp2 == temp) {
			Inspect_ROI_Point.clear();
			flag = 0;
		}

		if (flag == 2) {
			if (Inspect_ROI_Point.size() / 2 == Number_of_ROI) {
				for (int i = 0; i < Inspect_ROI_Point.size(); i += 2) {
					Standard_ROI.push_back(Mat(frame, Rect(Point(Inspect_ROI_Point[i]),
						Point(Inspect_ROI_Point[i + 1]))).clone());
				}

				flag = -1;
			}
			else {
				flag = 0;
			}
		}
	}
}

int main() {
	cap = VideoCapture(WEB_CAM);				//WEB_CAM, TEST_VIDEO_1920P
	if (!cap.isOpened()) {
		cout << "Could not find or open Video" << endl;
		return -1;
	}

	if(wiringPiSetupGpio() == -1){
		cout << "Could Not find or open wiringPi" << endl;
		return -1;
	}

	const int Led[] = {18, 23, 24, 25, 8, 7};
	for(int i = 0; i < sizeof(Led); i++){
		pinMode(Led[i], OUTPUT);
		digitalWrite(Led[i], 0);
	}

	digitalWrite(Led[1], 1);

	namedWindow("Show_Frame", WINDOW_FULLSCREEN);
	setMouseCallback("Show_Frame", Mouse_Event);
	cap.set(3, 640);
	cap.set(4, 480);
	width = 640;
	height = 480;

        for(int i = 0; i < 20; i++){
                cap >> frame;
                imshow("Show_Frame", frame);
		waitKey(1);
        }

	cout << width << endl;
	cout << height << endl;

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

	vector<unsigned int>	Config_Data;
	FILE* fp = fopen(data_addr, "r");
	if (fp != NULL) {
		char line[255];
		while (fgets(line, sizeof(line), fp) != NULL) {
			Config_Data.push_back(atoi(line));
		}

		if (Config_Data.size() >= 1) {
			Number_of_ROI = Config_Data[0];
		}
		if (Config_Data.size() >= 2) {
			Accuracy_Percent = Config_Data[1];
		}
		if (Config_Data.size() >= 3) {
			if((Config_Data.size() - 2) / 4 == Number_of_ROI){
				for (int i = 2; i < Config_Data.size(); i += 2) {
					Point temp_pt = Point(Config_Data[i], Config_Data[i + 1]);
					Inspect_ROI_Point.push_back(temp_pt);
				}

				for(int i = 1; i < Number_of_ROI + 1; i++){
					char Fread_Path[100];

					sprintf(Fread_Path, "%sROI_NUM_%d.PNG", img_data_addr, i);
					Mat ROI_Image = imread(Fread_Path);
					Standard_ROI.push_back(ROI_Image);

				}

				flag = -1;
			}
		}

		fclose(fp);
	}

	int key = 0;
	Compare_Total = 0;
	Compare_Good = 0;
	Compare_Bad = 0;

        clock_t start, end;
	while (1) {
		start = clock();

		cap >> frame;
		if (frame.empty())
			break;

		Show_Frame = frame.clone();
		Compare_State(Scalar(0, 255, 255));

		if (Config_Flag == true) {
			for (int i = 0; i < Inspect_ROI_Point.size(); i += 2) {
				Standard_ROI.push_back(Mat(frame, Rect(Point(Inspect_ROI_Point[i]),
					Point(Inspect_ROI_Point[i + 1]))).clone());
			}

			Config_Flag = false;
		}

		if (flag == 1) {
			circle(Show_Frame, temp, 2, Scalar(0, 0, 255), -1);
		}
		else if (flag == -1 && key == 32) {
			digitalWrite(Led[0], 1);
			double	result;
			double maxv;
			char	maxv_S[11];
			char	result_S[10];
			unsigned char	Standard_ROI_Count = 0;
			bool			Compare_Flag = true;

			for (int i = 0; i < Inspect_ROI_Point.size(); i += 2) {
				Inspect_ROI = Mat(frame, Rect(Point(Inspect_ROI_Point[i]), Point(Inspect_ROI_Point[i + 1])));
				Mat res, res_norm;

				matchTemplate(Inspect_ROI, Standard_ROI[Standard_ROI_Count], res, TM_CCOEFF_NORMED);
				Standard_ROI_Count++;
				normalize(res, res_norm, 0, 255, NORM_MINMAX, CV_8U);

				Point maxloc;
				minMaxLoc(res, 0, &maxv, 0, &maxloc);
				sprintf(maxv_S, "%d : %.2f", Standard_ROI_Count, maxv * 100);
				State_Message_S[7] = maxv_S;
				State_Message();

				if (maxv * 100 < Accuracy_Percent) {
					Compare_Flag = false;
				}
			}

			end = clock();
			digitalWrite(Led[0], 0);
			result = (double)(end - start);
			sprintf(result_S, "%.3fms", result / CLOCKS_PER_SEC);
			State_Message_S[7] = result_S;

			if (Compare_Flag == false) {
				Compare_Bad++;
				Compare_Total++;
				Compare_State(Scalar(0, 0, 255));
				digitalWrite(Led[5], 1);
			}
			else {
				Compare_Good++;
				Compare_Total++;
				Compare_State(Scalar(0, 255, 0));
				digitalWrite(Led[3], 1);
			}

			Compare_Flag = true;

			for (int i = 0; i < Inspect_ROI_Point.size(); i += 2) {
				rectangle(Show_Frame, Rect(Point(Inspect_ROI_Point[i]),
					Point(Inspect_ROI_Point[i + 1])), Scalar(0, 0, 255), 2);
			}
		}
		else if (flag == -1) {
			for (int i = 0; i < Inspect_ROI_Point.size(); i += 2) {
				rectangle(Show_Frame, Rect(Point(Inspect_ROI_Point[i]),
					Point(Inspect_ROI_Point[i + 1])), Scalar(0, 0, 255), 2);
			}
		}
		else if (flag == 3 && Setting_Mode_Flag == true) {
			Setting_Mode();
		}

		else if (flag == 0) {
			for (int i = 0; i < Inspect_ROI_Point.size(); i += 2) {
				rectangle(Show_Frame, Rect(Point(Inspect_ROI_Point[i]),
					Point(Inspect_ROI_Point[i + 1])), Scalar(0, 0, 255), 2);
			}
		}

		State_Message();
		Center_Marking(width, height);
		ALAM_Marking();
		Compare_Result();

		imshow("Show_Frame", Show_Frame);

		key = waitKey(33);
		if (key == 27)	break;

		digitalWrite(Led[3], 0);
		digitalWrite(Led[5], 0);
	}

	for(int i = 1; i < Standard_ROI.size() + 1; i++){
		char Write_Name[100];

		sprintf(Write_Name, "%sROI_NUM_%d.PNG", img_data_addr, i);
		imwrite(Write_Name, Standard_ROI[i - 1]);
	}

	FILE* fp_fin = fopen(data_addr, "w");

	char Fput_STR[100];
	sprintf(Fput_STR, "%d\n", Number_of_ROI);
	fputs(Fput_STR, fp_fin);
	sprintf(Fput_STR, "%d\n", Accuracy_Percent);
	fputs(Fput_STR, fp_fin);

	for (int i = 0; i < Inspect_ROI_Point.size(); i++) {
		sprintf(Fput_STR, "%d\n", Inspect_ROI_Point[i].x);
		fputs(Fput_STR, fp_fin);
		sprintf(Fput_STR, "%d\n", Inspect_ROI_Point[i].y);
		fputs(Fput_STR, fp_fin);
	}

	fclose(fp_fin);

	for(int i = 0; i < sizeof(Led); i++){
		pinMode(Led[i], OUTPUT);
		digitalWrite(Led[i], 0);
        }

	return 0;
}
