#include <iostream>
#include <windows.h>
#include <stdio.h>
#include <string>
#include "serial.h"
#include "NODES.h"
#include <cstdlib>
#include <cstring>
#include <iostream>






 

void calculate_explosion_coordinates(NODES *node_coordinates){
	//an master node amo nala an referrence node
	if(node_coordinates->explosion_trigger_flag){
		
	}
	
	
	
}



void get_coordinates(SERIAL& serial_instance,
		char* outBuffer, 
		size_t bufferSize, 
		bool& flag, 
		NODES* modify_node, 
		STATUS* status_num,
		int max_node_num,		
		bool trigger_flag){
	
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
		//debug
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
				if(node_num >= max_node_num)
					node_num = node_num-1;
				token = strtok(nullptr, "\r\n"); // Move past the number
				//printf("num: %d\n", node_num);
			}
			
			//debug
			//std::cout << "Detected flag: " << node_flag << ", Extra Number: " << node_num << std::endl;
			
			switch(node_flag){
				
				case 'S':
					modify_node->explosion_trigger_flag = false;
					modify_node->MasterNode = false;
					modify_node->SlaveNode = true;
					status_num->node_index_num = node_num;
					printf("Slaves num: %d\n", status_num->node_index_num);
					//debug
					//std::cout << "Detected flag: " << node_flag << ", Extra Number: " << node_num << std::endl;
					break;
				case 'E':
					modify_node->explosion_trigger_flag = true;
					modify_node->MasterNode = false;
					modify_node->SlaveNode = false;
					status_num->node_index_num = node_num;
					printf("Explosion node\n");
					//debug
					//std::cout << "Detected flag: " << node_flag << ", Extra Number: " << node_num << std::endl;
					break;
				case 'M':
					modify_node->explosion_trigger_flag = false;
					modify_node->MasterNode = true;
					modify_node->SlaveNode = false;
					status_num->node_index_num = node_num;
					
					//debug
					
					//printf("Master node dd\n");
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
				
				
				
				//an mga x coordinates
				if(token){
					if(modify_node->SlaveNode){
						
						modify_node->slave_cooordinates_x = atof(token);
						
					}else if(modify_node->MasterNode){
						
						modify_node->master_coordinates_x = atof(token);
						
					}else if(modify_node->explosion_trigger_flag){
						
						modify_node->explosion_trigger_coordinates_x = atof(token);
						
					}
					
				}
				//debug
				//if (token) modify_node->slave_cooordinates_x = atof(token);

				token = strtok(nullptr, "\r\n"); // Move to second number
				
				//printf("na play na\n");
				
				//an mga y coordinates
				if(token){
					//debug
					//printf("goods\n");
					if(modify_node->SlaveNode){
						
						modify_node->slave_cooordinates_y = atof(token);
						
					}else if(modify_node->MasterNode){
						
						modify_node->master_coordinates_y = atof(token);
						
					}else if(modify_node->explosion_trigger_flag){
						
						modify_node->explosion_trigger_coordinates_y = atof(token);
						
					}
					
				}
				//dudug ngan pa pan 3 karan compass values na
				
				
				//if (token) modify_node->slave_cooordinates_y = atof(token);
				
				break; // **Stop parsing after 'A' coordinates**
			}
			
			token = strtok(nullptr, "\r\n"); // Continue scanning for 'A'
			//std::cout<<token<<"\n";
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





