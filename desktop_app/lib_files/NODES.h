#ifndef NODES_H
#define NODES_H


#include "serial.h"
#include <iostream>
#include <windows.h>
#include <stdio.h>
#include <string>



typedef struct{
	
	
	//mga self explinatory
	float master_coordinates_x;
	float master_coordinates_y;
	

	float slave_cooordinates_x;
	float slave_cooordinates_y;
	

	float explosion_trigger_coordinates_x;
	float explosion_trigger_coordinates_y;
	
	
	
	
	
	/*
	Future me;
	butangi node to node distance formula (based han karan trip delay tas ppm han gps module tas ig print ha text aadto ine ha may APP.c node_distancex, where 'x' = 1, 2, 3, ...)
	tas an karan node to node coordinates based ha compass module tas han node to node distance.
	*/
	//float compas;
	
	
	//flag para han pag check status kun ano na type na node
	bool SlaveNode;					//Slave ine?
	bool MasterNode;				//is it a masternode or not?
	bool explosion_trigger_flag;	//is there an explosion or not?

	
	
}NODES;

typedef struct{
	
	
	//ma index ine ha mga object node instace from 0 to max node numbers 
	int node_index_num;
	
		
}STATUS;




void get_self_coordinates(NODES *node_coordinates);

void get_coordinates(SERIAL& serial_instance,
					char* outBuffer, 
					size_t bufferSize, 
					bool& flag, 
					NODES *modify_node, 
					STATUS* status_num, 
					int max_node_num, 
					bool trigger_flag = false
					);



#endif 