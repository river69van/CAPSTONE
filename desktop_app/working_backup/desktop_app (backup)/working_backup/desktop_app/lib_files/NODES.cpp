#include <iostream>
#include <windows.h>
#include <stdio.h>
#include <string>
#include "serial.h"
#include "NODES.h"
#include <cstdlib>
#include <cstring>
#include <iostream>



/*
ba balyuan ko sunod an if(trigger_flag)
hin "decode" function pag kita kun kan kanay ine coordinate 

*/


 

void get_self_coordinates(SERIAL& serial_instance, char* outBuffer, size_t bufferSize, bool& flag, NODES *modify_node, bool trigger_flag ){
	/*
	if(trigger_flag)
	serial_instance.read(outBuffer, bufferSize, flag);
	
	
	
	 if (bufferSize == 0 || outBuffer == nullptr) {
		 
        int notDetected = MessageBox(NULL, "Error Invalid buffer", "Error", MB_OK | MB_ICONERROR);
		
		if(notDetected == IDOK || notDetected == IDCANCEL){// to check if the message box is pressed in anyway
			flag = false; // for external trigger for the serial loop to recieve 
		}
        return;
    }
	
	// Tokenize input (format: "123\n456")
	
	 char* token = strtok(outBuffer, "\n"); // Split at \n
    if (token) {
        modify_node->self_coordinates_x = atof(token); // First value
    }

    token = strtok(nullptr, "\n"); // Get next number
    if (token) {
        modify_node->self_coordinates_y = atof(token); // Second value
    }
	
	
	
	
	
	if(outBuffer[0] != '\0'){
		printf("hi");
		std::cout<<"hello\n";
	}
	*/
	
	
	

}



void get_coordinates(SERIAL& serial_instance, char* outBuffer, size_t bufferSize, bool& flag, NODES* modify_node, bool trigger_flag){
	
	/*
	if(trigger_flag)
	//memset(outBuffer, 0, sizeof(outBuffer));  // Clear buffer before reading
	serial_instance.read(outBuffer, bufferSize, flag);
	
	
	
	 if (bufferSize == 0 || outBuffer == nullptr) {
		 
        int notDetected = MessageBox(NULL, "Error Invalid buffer", "Error", MB_OK | MB_ICONERROR);
		
		if(notDetected == IDOK || notDetected == IDCANCEL){// to check if the message box is pressed in anyway
			flag = false; // for external trigger for the serial loop to recieve 
		}
        return;
    }
	//printf("Received: %s\n", outBuffer);
	// Tokenize input (format: "123\n456")
	
	char* token = strtok(outBuffer, "\r\n"); // Split at \n
	
	
	while (token != NULL) {
		
		
    //printf("Token: %s\n", token);  // Debugging: Print token
    //float value = atof(token);
	
	modify_node->slave_cooordinates_x = atof(token); // First value
    //printf("Float Value: %.2f\n", value);
	
	
    token = strtok(NULL, "\r\n");
	modify_node->slave_cooordinates_y = atof(token); // First value
	
	}
	
	//printf("Received: %s\n", token);
	

	
	
	
	
	if(outBuffer[0] != '\0'){
		//printf("hi");
		//std::cout<<"hello\n";
	}
	
	*/
	
	
	
	
	// Read new data if the trigger flag is set
	if (trigger_flag) {
		//memset(outBuffer, 0, bufferSize);
        serial_instance.read(outBuffer, bufferSize, flag);
		//printf("nag read na\n");
    }
	//debug
	//printf("dd na ha may get coor\n");
	if (bufferSize == 0 || outBuffer == nullptr || outBuffer[0] == '\0') {
        int notDetected = MessageBox(NULL, "Error: Invalid buffer", "Error", MB_OK | MB_ICONERROR);
        if (notDetected == IDOK || notDetected == IDCANCEL) {
            flag = false; // Stop external trigger if message box is acknowledged
        }
        return;
    }
	//debug
	//printf("lumapos na ha buffer checker\n");
    char* token = strtok(outBuffer, "\r\n"); // Start tokenizing with '\n' separator
	
	//token format:	flagtype "\n" node_num "\n" 'A' identifier for coordinates "\n" x_coordinates "\n" y_coordinates 
	//token = strtok(nullptr, "\r\n"); ig balhin an token origin pa kadto ha sunod han "\n"
	while(token != NULL){
			
		
		
		if (token == nullptr) return; // Ensure we have data

		// **Detect Arbitrary Flag (First Token)**
		char node_flag = token[0]; // Store the flag (e.g., 'M', 'S', 'E')
		int node_num = 0;


		if (node_flag == 'M' || node_flag == 'S' || node_flag == 'E') {
			token = strtok(nullptr, "\r\n"); // Move to the next token

			// If the next token is a number, store it
			if (token != nullptr && isdigit(token[0])) {
				
				node_num = atoi(token); // node_number pag an status nagin slave_node'S'
				
				token = strtok(nullptr, "\r\n"); // Move past the number
			}
			
			//debug
			//std::cout << "Detected flag: " << node_flag << ", Extra Number: " << node_num << std::endl;
			
			switch(node_flag){
				case 'S':
					modify_node->trigger_flag = false;
					modify_node->MasterNode = false;
					modify_node->SlaveNode = true;
					modify_node->node_number_status = node_num;
					
					//debug
					//std::cout << "Detected flag: " << node_flag << ", Extra Number: " << node_num << std::endl;
					break;
				case 'E':
					modify_node->trigger_flag = true;
					modify_node->MasterNode = false;
					modify_node->SlaveNode = false;
					
					//debug
					//std::cout << "Detected flag: " << node_flag << ", Extra Number: " << node_num << std::endl;
					break;
				case 'M':
					modify_node->trigger_flag = false;
					modify_node->MasterNode = true;
					modify_node->SlaveNode = false;
					
					//debug
					//std::cout << "Detected flag: " << node_flag << ", Extra Number: " << node_num << std::endl;
					break;
				default:
					return;
			}
			
		} else {
			//debug
			//std::cout << "Unknown flag detected!\n";
			//return;
		}

		// **Find and Extract 'A' Coordinates Only**
		while (token != nullptr) {
			if (strcmp(token, "A") == 0) {  
				token = strtok(nullptr, "\r\n"); // Move to first number
				if (token) modify_node->slave_cooordinates_x = atof(token);

				token = strtok(nullptr, "\r\n"); // Move to second number
				if (token) modify_node->slave_cooordinates_y = atof(token);
				
				break; // **Stop parsing after 'A' coordinates**
			}
			
			token = strtok(nullptr, "\r\n"); // Continue scanning for 'A'
		}
		//debug
		//printf("X coor = %2fl\n", modify_node->slave_cooordinates_x );
		//printf("Y coor = %2fl\n", modify_node->slave_cooordinates_y );
	}
	

   
    

    // Debugging output
    if (outBuffer[0] != '\0') {
        //printf("hi\n");
    }
	
	
	
		
}





