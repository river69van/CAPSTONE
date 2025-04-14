
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

typedef struct{}G;

bool run = true;

int x2 = 400;
int y2 = 400;

std::atomic<int> node_index_num_pointer(1);
//std::atomic<NODES*> current_node;
//std::mutex nodeMutex;


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

	init_nodes->trigger_flag = false;
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


    for(int K = 0; K < NUMBER_OF_NODES; K++){
				int X_coor = node_object[K].slave_cooordinates_x;
				int y_coor = node_object[K].slave_cooordinates_y;
				
				if(node_object[K].SlaveNode)
				draw_object(&hdcMem, &hdc, &SLAVE, &bitmap, X_coor, y_coor);
				
				if(node_object[K].trigger_flag)
				draw_object(&hdcMem, &hdc, &EXPLOSION, &bitmap, X_coor, y_coor);
				
				if(node_object[K].MasterNode)
				draw_object(&hdcMem, &hdc, &MASTER, &bitmap, X_coor, y_coor);
				
			}
	
				



    DeleteDC(hdcMem);
    EndPaint(hwnd, &ps);
}



// Function to read serial data in a separate thread
void SerialReader(/*NODES* modifyNodes*/) {
	while(true){

    if (serial.FindWorkingCOMPort()) {
        serial.begin(1, 1);  // Open the detected COM port
        char buffer[1024];


		//debug
		printf("aadi na");
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
					printf("Status: slave node\n");
					//printf("node number status: %d\n", NODE_ptr->node_number_status);
					printf("node number status: %d\n", node_index_status_number.node_index_num);
					node_index_num_pointer.store(node_index_status_number.node_index_num);
					
				}else if(node_object[index].trigger_flag){
					//debug
					printf("Status: explosion\n");
					
				}else if(node_object[index].MasterNode){
					//debug
					printf("Status: master node\n");
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
			
			/*
            PAINTSTRUCT ps;
            HDC hdc = BeginPaint(hwnd, &ps);

            HDC hdcMem = CreateCompatibleDC(hdc);
			BITMAP bitmap;
			
			
		
			//draw n number of slave objects
			for(int K = 0; K < NUMBER_OF_NODES; K++){
				int X_coor = node_object[K].slave_cooordinates_x;
				int y_coor = node_object[K].slave_cooordinates_y;
				draw_object(&hdcMem, &hdc, &SLAVE, &bitmap, X_coor, y_coor);
			}
			draw_object(&hdcMem, &hdc, &EXPLOSION, &bitmap, imgX, imgX);
			draw_object(&hdcMem, &hdc, &MASTER, &bitmap, x2, y2);
			draw_object(&hdcMem, &hdc, &MASTER, &bitmap, (x2+40), (y2+30));
			
			
            DeleteDC(hdcMem);
            EndPaint(hwnd, &ps);
			*/
			
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
