
#include <windows.h>
#include "serial.h"
#include <cstdlib>  // For atoi
#include <string>   // For std::stoi
#include <thread>   // For std::thread
#include "NODES.h"
#include <cctype>
#include <iostream>
#include <atomic>
#include <chrono>
#include <mutex>


/*
#include <windows.h>
#include "lib_files/serial.h"
#include <cstdlib>  // For atoi
#include <string>   // For std::stoi
#include <thread>   // For std::thread
#include "lib_files/NODES.h"
#include <cctype>
//#include <iostream>

*/


HBITMAP SLAVE, EXPLOSION, MASTER;  // Handle for the image
int imgX = 0, imgY = 0;  // Image position
int imgX2 = 1000, imgY2 = 100;
bool serialActive = false; // Serial flag
const int NUMBER_OF_NODES = 6;
int baud_rate = 115200;
int slave_number_status = 0;
char flag_type;
SERIAL serial(baud_rate);
HWND g_hwnd; // Global handle to window

//typedef struct{}G;

bool run = true;

int x2 = 400;
int y2 = 400;

std::atomic<int> node_index_num_pointer(0);
//std::atomic<NODES*> current_node;
//std::mutex nodeMutex;

//node_object[0] pati an node_object[1] naka reserve ha master node pati ha explosion 
//node_object[0] = master node
//node_object[1] = explosion

NODES node_object[NUMBER_OF_NODES];
STATUS node_index_status_number;



void init_nodes(NODES* init_nodes, STATUS* node_status){
	

	node_status->node_index_num = 0;

	init_nodes->slave_cooordinates_x = 0;
	init_nodes->slave_cooordinates_y = 0;

	init_nodes->slave_cooordinates_x = 0;
	init_nodes->slave_cooordinates_y = 0;

	init_nodes->explosion_trigger_coordinates_x = 0;
	init_nodes->explosion_trigger_coordinates_y = 0;

	init_nodes->explosion_trigger_flag = false;
	init_nodes->MasterNode = false;
	init_nodes->SlaveNode = false;


}




// Function to Load BMP Image
void LoadBitmaps() {
    SLAVE = (HBITMAP)LoadImage(NULL, "pics/SLAVE_node.bmp", IMAGE_BITMAP, 0, 0, LR_LOADFROMFILE);
    EXPLOSION = (HBITMAP)LoadImage(NULL,"pics/EXPLOSION.bmp", IMAGE_BITMAP, 0, 0, LR_LOADFROMFILE);
	MASTER = (HBITMAP)LoadImage(NULL, "pics/MASTER_node.bmp", IMAGE_BITMAP, 0, 0, LR_LOADFROMFILE);
}

/*
void draw_object(HDC* hdcMem, HDC* hdc, HBITMAP* object_name, BITMAP* bitmap, int* IMG_loc_x, int* IMG_loc_y){

	SelectObject(*hdcMem, object_name);
	GetObject(object_name, sizeof(BITMAP), bitmap);
	BitBlt(*hdc, *IMG_loc_x, *IMG_loc_y, bitmap->bmWidth, bitmap->bmHeight, *hdcMem, 0, 0, SRCCOPY);
	
}
*/


void draw_object(HDC* hdcMem, HDC* hdc, HBITMAP* object_name, BITMAP* bitmap, int IMG_loc_x, int IMG_loc_y) {
    SelectObject(*hdcMem, *object_name);
    GetObject(*object_name, sizeof(BITMAP), bitmap);
    BitBlt(*hdc, IMG_loc_x, IMG_loc_y, bitmap->bmWidth, bitmap->bmHeight, *hdcMem, 0, 0, SRCCOPY);
}


void RenderScene(HWND hwnd) {
    PAINTSTRUCT ps;
    HDC hdc = BeginPaint(hwnd, &ps);
    HDC hdcMem = CreateCompatibleDC(hdc);
    BITMAP bitmap;
	char Slave_buffer1[50];
	char Slave_buffer2[50];
	char Slave_buffer3[50];
	char Slave_buffer4[50];
	char Master_buffer[50];
	char Explosion_buffer[50];

	
	
	SetTextColor(hdc, RGB(255, 0, 0));  // Red color text
    SetBkMode(hdc, TRANSPARENT);
	
	//Snode1
	int X_coor1 = node_object[2].slave_cooordinates_x;
	int y_coor1 = node_object[2].slave_cooordinates_y;
	
	int node_distance1 = X_coor1;
	
	sprintf(Slave_buffer1, "SLAVE Node 1 distance %d\n", node_distance1);
	TextOut(hdc, X_coor1 - 20, y_coor1+50 , Slave_buffer1, strlen(Slave_buffer1));
	draw_object(&hdcMem, &hdc, &SLAVE, &bitmap, X_coor1, y_coor1);
	
	
	
	
	//Snode2
	int X_coor2 = node_object[3].slave_cooordinates_x;
	int y_coor2 = node_object[3].slave_cooordinates_y;
	
	int node_distance2 = X_coor2;
	
	sprintf(Slave_buffer2, "SLAVE Node 2 distance %d\n", node_distance2);
	TextOut(hdc, X_coor2 - 20, y_coor2+50 , Slave_buffer2, strlen(Slave_buffer2));
	draw_object(&hdcMem, &hdc, &SLAVE, &bitmap, X_coor2, y_coor2);
	
	
	
	
	//Snode3
	int X_coor3 = node_object[4].slave_cooordinates_x;
	int y_coor3 = node_object[4].slave_cooordinates_y;
	
	int node_distance3 = X_coor3;
	
	sprintf(Slave_buffer3, "SLAVE Node 3 distance %d\n", node_distance3);
	TextOut(hdc, X_coor3 - 20, y_coor3+50 , Slave_buffer3, strlen(Slave_buffer3));
	draw_object(&hdcMem, &hdc, &SLAVE, &bitmap, X_coor3, y_coor3);
	
	
	
	
	//Snode4
	int X_coor4 = node_object[5].slave_cooordinates_x;
	int y_coor4 = node_object[5].slave_cooordinates_y;
	
	int node_distance4 = X_coor4;
	
	sprintf(Slave_buffer4, "SLAVE Node 4 distance %d\n", node_distance4);
	TextOut(hdc, X_coor4 - 20, y_coor4+50 , Slave_buffer4, strlen(Slave_buffer4));
	draw_object(&hdcMem, &hdc, &SLAVE, &bitmap, X_coor4, y_coor4);
	
	
	
	
	//Enode
	if(node_object[1].explosion_trigger_flag){
	int X_coor_exp = node_object[1].explosion_trigger_coordinates_x;
	int y_coor_exp = node_object[1].explosion_trigger_coordinates_y;
	draw_object(&hdcMem, &hdc, &EXPLOSION, &bitmap, X_coor_exp, y_coor_exp);
	}
	
	
	
	//Mnode
	int X_coor_master = node_object[0].master_coordinates_x;
	int y_coor_master = node_object[0].master_coordinates_y;
	
	int slave_distance = X_coor_master;
	
	//sprintf(Master_buffer, "MASTER Node distance %d\n", slave_distance);
	char sample[] = "(0,0)";
	TextOut(hdc, X_coor_master - 20, y_coor_master+50 , sample/*Master_buffer*/, strlen(sample/*Master_buffer*/));
	draw_object(&hdcMem, &hdc, &MASTER, &bitmap, X_coor_master, y_coor_master);
				



    DeleteDC(hdcMem);
    EndPaint(hwnd, &ps);
}



// Function to read serial data in a separate thread
void SerialReader() {
	while(true){

    if (serial.FindWorkingCOMPort()) {
        serial.begin(1, 1);  // Open the detected COM port
        char buffer[1024];


		//debug
        while (serialActive) {
			int index = node_index_num_pointer.load();
            //serial.read(buffer, sizeof(buffer), serialActive);
			//NODES*	NODE_ptr = current_node.load();
			
			
			if(index >= 0 && index <NUMBER_OF_NODES)
			get_coordinates(serial, buffer, sizeof(buffer), serialActive, &node_object[index], &node_index_status_number, NUMBER_OF_NODES, run);
			//std::lock_guard<std::mutex> lock(nodeMutex);

            if (buffer[0] != '\0') {

                imgX = 4*node_object[index].slave_cooordinates_x;
                imgY = 4*node_object[index].slave_cooordinates_x;
				
				
				if(node_object[index].SlaveNode){
					//debug
					//printf("Status: slave node\n");
					//printf("node number status: %d\n", node_index_status_number.node_index_num);
					node_index_num_pointer.store(node_index_status_number.node_index_num);
					
				}else if(node_object[index].explosion_trigger_flag){
					//debug
					//printf("Status: explosion\n");
					node_index_num_pointer.store(node_index_status_number.node_index_num);
					
				}else if(node_object[index].MasterNode){
					//debug
					//node_object[index].trigger_flag = false;
					//printf("Status: master node\n");
					node_index_num_pointer.store(node_index_status_number.node_index_num);
				}
				
				//debug
				//printf("Float Value: %.2f\n", modifyNodes->slave_cooordinates_x);
				//std::cout<<"Flag type "<<flag_type<<"\n";
				//printf("Flag type %c\n", flag_type);
				//std::cout<<"node number: "<<slave_number_status<<"\n";
				//std::cout<<modifyNodes->slave_cooordinates_y<<"\n";
				
                InvalidateRect(g_hwnd, NULL, TRUE); // Request window repaint


            }


            Sleep(50); // Prevent excessive CPU usage
        }




        serial.stop(); // Close COM port when stopping
    }
	}
}

// Window Procedure
LRESULT CALLBACK WindowProc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam) {

    switch (msg) {
        case WM_CREATE:
            g_hwnd = hwnd;
            LoadBitmaps();
            break;

        case WM_PAINT: {
\
			
			RenderScene(hwnd);
			
			
            break;
        }

        case WM_KEYDOWN:
		
			//baralyuan pa hin pag trigger han serial read
            if (wParam == VK_SPACE && !serialActive) {
                serialActive = true;
				
				//while(1)SerialReader(&node_object[1]);
				//current_node.store(&node_object[1]);
                std::thread readerThread(SerialReader);
				readerThread.detach(); // Start serial reading in a new thread
				
				
				
				/*
				std::this_thread::sleep_for(std::chrono::seconds(3));
				currentName.store(&name[2]);  // Now SerialReader modifies name[2]
				std::cout << "Switched to name[2]" << std::endl;
				*/
				
				
            }
            switch (wParam) {
                case VK_LEFT:  imgX2 -= 10; break;
                case VK_RIGHT: imgX2 += 10; break;
                case VK_UP:    imgY2 -= 10; break;
                case VK_DOWN:  imgY2 += 10; break;
            }
            InvalidateRect(hwnd, NULL, TRUE);
            break;

        case WM_DESTROY:
            serialActive = false; // Stop serial thread
            DeleteObject(SLAVE);
			DeleteObject(EXPLOSION);
			DeleteObject(MASTER);
            PostQuitMessage(0);
            break;

        default:
            return DefWindowProc(hwnd, msg, wParam, lParam);
    }
    return 0;
}

// Entry point
int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow) {
	
	
	//initialize the nodes and set coor values to 0 and flags to false
	for(int i=0; i < NUMBER_OF_NODES; i++){
		init_nodes(&node_object[i], &node_index_status_number);
	}
	
	
    WNDCLASS wc = {0};
    wc.hbrBackground = (HBRUSH)COLOR_WINDOW;
    wc.hCursor = LoadCursor(NULL, IDC_ARROW);
    wc.hInstance = hInstance;
    wc.lpszClassName = "MyWindowClass";
    wc.lpfnWndProc = WindowProc;

    if (!RegisterClass(&wc)) {
        MessageBox(NULL, "Window class registration failed!", "Error", MB_ICONERROR);
        return 0;
    }
	int width = 1080;
	int hieght = 720;
    HWND hwnd = CreateWindow("MyWindowClass", "Serial Port Window", WS_OVERLAPPEDWINDOW | WS_VISIBLE,
                             10, 10, width, hieght, NULL, NULL, hInstance, NULL);

    if (!hwnd) {
        MessageBox(NULL, "Window creation failed!", "Error", MB_ICONERROR);
        return 0;
    }

    MSG msg = {0};
    while (GetMessage(&msg, NULL, 0, 0)) {
        TranslateMessage(&msg);
        DispatchMessage(&msg);
    }

    return 0;
}
