#ifndef NODES_H
#define NODES_H


#include "serial.h"
#include <iostream>
#include <windows.h>
#include <stdio.h>
#include <string>



typedef struct{
	
	

	float self_coordinates_x;
	float self_coordinates_y;
	

	float slave_cooordinates_x;
	float slave_cooordinates_y;
	

	float explosion_trigger_coordinates_x;
	float explosion_trigger_coordinates_y;
	
	//float compas;
	
	
	//flag para han pag check status kun ano na type na node
	bool SlaveNode;
	bool MasterNode;	//is it a masternode or not?
	bool trigger_flag;	//is there an explosion or not?
	int node_number_status;

	
	
}NODES;

typedef struct{
	int node_index_num;
	
		
}STATUS;




void get_self_coordinates(SERIAL& serial_instance, char* outBuffer, size_t bufferSize, bool& flag, NODES *modify_node, bool trigger_flag = false);

void get_coordinates(SERIAL& serial_instance,
	char* outBuffer, 
	size_t bufferSize, 
	bool& flag, 
	NODES *modify_node, 
	STATUS* status_num, 
	int max_node_num, 
	bool trigger_flag = false);



#endif 