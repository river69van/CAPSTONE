/*

*	log 18/04/25 23:28 {
	
	Dre ngayan ma aasahan an esp_now_send erratique an iya behaviour danay dre na kaka send nag pipinan masang la kaylangan mag implement catch 
	
	}







*/




#include <math.h>
#include <esp_now.h>
#include <WiFi.h>
#include <Ticker.h>



//change based on node number
#define NODE_number 4

Ticker T;

bool master_starter = false;
volatile int sendStatusCounter = 0;
volatile unsigned long t = 0;
bool start_counting2 = false;
bool start_counting3 = false;
bool start_counting4 = false;
volatile int cycle_wave_counter = -1;

static const int RXPin = 18, TXPin = 19;	//GPIO han ig co connect han GPS module
static const uint32_t GPSBaud = 9600;
volatile bool send_flag = false;
//volatile bool reset_flag;
#if NODE_number == 1
  volatile bool reset_flag = true;
#else
  volatile bool reset_flag = false;
#endif
//node mac addr
uint8_t NODE1[] = {0xCC, 0x7B, 0x5C, 0x36, 0xC1, 0x04};
uint8_t NODE2[] = {0x10, 0x06, 0x1C, 0xB5, 0x2E, 0xDC};
uint8_t NODE3[] = {0x5C, 0x01, 0x3B, 0x94, 0xED, 0x14};
uint8_t NODE4[] = {0xF4, 0x65, 0x0B, 0xE8, 0xF5, 0x94};
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

volatile int initialize_node_cycle_counter = 0;
// The TinyGPSPlus object
//TinyGPSPlus gps;

// The serial connection to the GPS device
//SoftwareSerial ss(RXPin, TXPin);

uint8_t* peerList[] = {NODE1, NODE2, NODE3, NODE4};

uint8_t* peerList_node1[] = { NODE2, NODE3, NODE4};
uint8_t* peerList_node2[] = { NODE3, NODE4};
//uint8_t* peerList_node3[] = { NODE1, NODE2, NODE4};
//uint8_t* peerList_node4[] = { NODE1, NODE2, NODE3};

const int NUM_PEERS = sizeof(peerList) / sizeof(peerList[0]);

typedef struct{

	double N_1_2;
	double N_1_3;
	double N_1_4;
	double N_2_3;
	double N_2_4;
	double N_3_4;

	bool N1_starter;
	bool N2_starter;
	bool N3_starter;
	bool N4_starter;

	double N1_coordinates_x;
	double N1_coordinates_y;

	double N2_coordinates_x;
	double N2_coordinates_y;

	double N3_coordinates_x;
	double N3_coordinates_y;

	double N4_coordinates_x;
	double N4_coordinates_y;


}node_dist;



typedef struct struct_message {
	uint8_t seq;
	int message;		//node number
	double latitude;
	double longitude;
	int self_priority;
	
	//flags
	bool master_node;
	
	bool starter;
	bool send_msg;
	bool is_ack;
	
}__attribute__((packed)) struct_message;

node_dist node_distances_and_status;
struct_message Node_self_message; 		//An sulod mismo na data (tanan na api na an mga flags) na ig se send ha iba 
struct_message From_other_node_msg; 	//An ma ta-tanggap na data mangangadi 



void Interrupt_func() {
	
	sendStatusCounter = 0;
	cycle_wave_counter++;
	send_flag = true;
	From_other_node_msg.starter = false;

}



//just the setup part
void setupPeers() {
	
  for (int i = 0; i < NUM_PEERS; i++) {
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, peerList[i], 6);
    peerInfo.channel = 1;          // Use current WiFi channel
    peerInfo.encrypt = false;      // Set to true if using encryption and peer supports it

    // Avoid adding the same peer twice
    if (!esp_now_is_peer_exist(peerList[i])) {
      if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.print(" Failed to add peer: ");
        for (int j = 0; j < 6; j++) {
          Serial.printf("%02X", peerList[i][j]);
          if (j < 5) Serial.print(":");
        }
        Serial.println();
      } else {
        Serial.print(" Peer added: ");
        for (int j = 0; j < 6; j++) {
          Serial.printf("%02X", peerList[i][j]);
          if (j < 5) Serial.print(":");
        }
        Serial.println();
      }
    }
  }
  
}
//setup code end



void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
	
	//Serial.println("sent");
	
}


void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
	
	memcpy(&From_other_node_msg, incomingData, sizeof(From_other_node_msg));
	
	
	
	//debug
  /*
	Serial.printf("N%d ◀ recv from %02X:%02X:%02X:%02X:%02X:%02X  seq=%u  starter=%d  send_msg=%d  is_ack=%d\n",
              NODE_number,
              info->src_addr[0],info->src_addr[1],info->src_addr[2],
              info->src_addr[3],info->src_addr[4],info->src_addr[5],
              From_other_node_msg.seq,
              From_other_node_msg.starter,
              From_other_node_msg.send_msg,
              From_other_node_msg.is_ack
	);
  */
	
	
	
	
	
	
#if NODE_number == 1

	switch(From_other_node_msg.message){
			
			case 1:
				
			break;
			case 2:
				Serial.println("from N2");
				//if(sendStatusCounter >=5)						// ig verify anay na naka send na ma karan 10 ka beses para bisan mayda packet loss na sasalo han node
				node_distances_and_status.N2_starter = true;	//pag verify na naka recieve na an node(2,3,...)
			break;
			case 3:
				Serial.println("from N3");
				//if(sendStatusCounter >=5)
				node_distances_and_status.N3_starter = true;
			break;
			case 4:
				Serial.printf("from N4, counter: %d\n", sendStatusCounter);
				//if(sendStatusCounter >=5)
				node_distances_and_status.N4_starter = true;
				
				
				
				
				
				if(From_other_node_msg.send_msg){
					
					reset_flag = From_other_node_msg.send_msg;
					Node_self_message.is_ack = true;
					esp_now_send(info->src_addr, (uint8_t *)&Node_self_message, sizeof(Node_self_message));
				}
			break;
			default:
				
			break;
			
		}

#elif NODE_number == 2
	
	switch(From_other_node_msg.message){
			
			case 1:
				/*
				*	once na ma run na ine hiya na line meaning nag reply na tanan na nodes ha N1 ma trigger adi na line 
				*	"if(node_distances_and_status.N2_starter && node_distances_and_status.N3_starter && node_distances_and_status.N4_starter)"
				*/
				esp_now_send(info->src_addr, (uint8_t *)&Node_self_message, sizeof(Node_self_message));
				if(From_other_node_msg.send_msg){
					
					reset_flag = From_other_node_msg.send_msg;
					Node_self_message.is_ack = true;
					esp_now_send(info->src_addr, (uint8_t *)&Node_self_message, sizeof(Node_self_message));
					Node_self_message.is_ack = false;
				}
				
			break;
			case 2:
				
			break;
			case 3:
				Serial.println("from N3");
				//if(sendStatusCounter >=5)
				node_distances_and_status.N3_starter = true;
			break;
			case 4:
				Serial.println("from N4");
				//if(sendStatusCounter >=5)
				node_distances_and_status.N4_starter = true;
			break;
			default:
				
			break;
			
		}
//esp_now_send(info->src_addr, (uint8_t *)&Node_self_message, sizeof(Node_self_message));

#elif NODE_number == 3

	switch(From_other_node_msg.message){
		
		case 1:
			//digitalWrite(17, 1);
			esp_now_send(info->src_addr, (uint8_t *)&Node_self_message, sizeof(Node_self_message));
		break;
		case 2:
		esp_now_send(info->src_addr, (uint8_t *)&Node_self_message, sizeof(Node_self_message));
			if(From_other_node_msg.send_msg){
					
					reset_flag = From_other_node_msg.send_msg;
					Node_self_message.is_ack = true;	//to turn of an node1 ha pag senend han packet to node2 
					esp_now_send(info->src_addr, (uint8_t *)&Node_self_message, sizeof(Node_self_message));
					Node_self_message.is_ack = false;
					
			}
		break;
		case 3:
		
		break;
		case 4:
			Serial.println("from N4");
			//if(sendStatusCounter >=5)
			node_distances_and_status.N4_starter = true;
		break;
		default:
			
		break;
		
	}
	
	
#else

	switch(From_other_node_msg.message){
	
	case 1:
		node_distances_and_status.N1_starter = true;
		esp_now_send(info->src_addr, (uint8_t *)&Node_self_message, sizeof(Node_self_message));
	break;
	case 2:
		esp_now_send(info->src_addr, (uint8_t *)&Node_self_message, sizeof(Node_self_message));
	break;
	case 3:
		Serial.println("from N3");
		esp_now_send(info->src_addr, (uint8_t *)&Node_self_message, sizeof(Node_self_message));
		
		//if(sendStatusCounter >=5)
		esp_now_send(info->src_addr, (uint8_t *)&Node_self_message, sizeof(Node_self_message));
		
		
		if(From_other_node_msg.send_msg){
					Node_self_message.is_ack = true;
					reset_flag = From_other_node_msg.send_msg;
					esp_now_send(info->src_addr, (uint8_t *)&Node_self_message, sizeof(Node_self_message));
					Node_self_message.is_ack = false;
		}
	break;
	case 4:
		
	break;
	default:
		
	break;
	
	}
#endif
	
	
	
}



void setup(){


Node_self_message.send_msg = false;
From_other_node_msg.send_msg = false;
From_other_node_msg.is_ack = false;
Node_self_message.is_ack = false;


#if NODE_number == 1
node_distances_and_status.N2_starter = false;
node_distances_and_status.N3_starter = false;
node_distances_and_status.N4_starter = false;
reset_flag = true;	
#elif NODE_number == 2
reset_flag = false;								//false la anay ine hiya habang dre pa tapos mag send an previous node
node_distances_and_status.N3_starter = false;
node_distances_and_status.N4_starter = false;
#elif NODE_number == 3
node_distances_and_status.N4_starter = false;
reset_flag = false;								//false la anay ine hiya habang dre pa tapos mag send an previous node
#else
reset_flag = false;								//false la anay ine hiya habang dre pa tapos mag send an previous node (NODE4)
#endif

	pinMode(17, OUTPUT);
	Serial.begin(115200);
	WiFi.mode(WIFI_STA);
	setCpuFrequencyMhz(240);  // Set CPU to 240 MHz
	delay(100);

	if (esp_now_init() != ESP_OK) {

		Serial.println(" ESP-NOW init failed");
		return;

	}
	
	
	esp_now_register_send_cb(onDataSent);
	esp_now_register_recv_cb((esp_now_recv_cb_t) onDataRecv);



	setupPeers();

	T.attach(1, Interrupt_func);   // Call reset() every 1 second


	Node_self_message.message = NODE_number;
	Node_self_message.starter = true;			// ig true kay para maka reply na an iba na node ma stop na adi na line "while(!From_other_node_msg.starter)" baga han ACK
	From_other_node_msg.starter = false;		//ma gi gin true ine hiya pag ma receive adi na line "memcpy(&From_other_node_msg, incomingData, sizeof(From_other_node_msg))" tas ma reset ha false pag ma trigger na liwat han ISR
	From_other_node_msg.is_ack = false;
	digitalWrite(17, 0);
}

void loop() {


	
#if NODE_number == 1

	if(node_distances_and_status.N2_starter && node_distances_and_status.N3_starter && node_distances_and_status.N4_starter){ //pan "catch" ine na part pag naka send na an node 1 ha tulo na node (N2, N3, N4) 

		reset_flag = false;	// para dre la anay utro may send an node pag na fulfill na niya an pag send ha mga noded gamit an established pattern (Node1 = send to node 2,3,4; Node2 = send to node 3, 4; Node3 = send to node 4)
		Node_self_message.send_msg = true;	//prepare han pag turn on han N2
		esp_now_send(NODE2, (uint8_t *)&Node_self_message, sizeof(Node_self_message));
		Serial.printf("N%d  flags: reset=%d  send_flag=%d  starter=%d  is_ack=%d  N3_starter=%d  N4_starter=%d\n",
              NODE_number, reset_flag, send_flag,
              From_other_node_msg.starter, From_other_node_msg.is_ack,
              node_distances_and_status.N3_starter,
              node_distances_and_status.N4_starter
	);
		
	}

	if(From_other_node_msg.is_ack){
		node_distances_and_status.N2_starter = false;
		node_distances_and_status.N3_starter = false;
		node_distances_and_status.N4_starter = false;
		Node_self_message.send_msg = false;
		From_other_node_msg.is_ack = false;
		
	}
	
	
	if(send_flag && reset_flag){
		send_flag = false;		//karan ISR la ine flag para pan trigger 1Hz
		//while(!From_other_node_msg.starter){
			
			if(cycle_wave_counter > sizeof(peerList_node1)/sizeof(peerList_node1[0])-1) cycle_wave_counter = 0;
			//for(int k = 0; k != 10; k++){	// ig send ma karan 10 ka beses ha usa na node para bis may ma wawara na packet ma sasalo la gihap han node
			//sendStatusCounter++;			// pag track kun pira na an na send 
			esp_now_send(peerList_node1[cycle_wave_counter], (uint8_t *)&Node_self_message, sizeof(Node_self_message));		//an pag send la han struct message 
			//delay(5);
			//}
			
		//}
		//Serial.println("na run dd");
	}
	

delay(50);
	
#elif NODE_number == 2

	if(!node_distances_and_status.N3_starter && !node_distances_and_status.N4_starter && reset_flag){
		digitalWrite(17, 1);
		
	}

	if(node_distances_and_status.N3_starter && node_distances_and_status.N4_starter){


		reset_flag = false;
		Node_self_message.send_msg = true;
		esp_now_send(NODE3, (uint8_t *)&Node_self_message, sizeof(Node_self_message));
		
		
	}
	
	if(From_other_node_msg.is_ack){
			//node_distances_and_status.N2_starter = false;
			node_distances_and_status.N3_starter = false;
			node_distances_and_status.N4_starter = false;
			Node_self_message.send_msg = false;
			From_other_node_msg.is_ack = false;
			digitalWrite(17, 0);
	}
		

	if(send_flag && reset_flag){
		send_flag = false;
		//while(!From_other_node_msg.starter){
			
			if(cycle_wave_counter > sizeof(peerList_node2)/sizeof(peerList_node2[0])-1) cycle_wave_counter = 0;
			//for(int k = 0; k != 10; k++){
			//sendStatusCounter++;
			esp_now_send(peerList_node2[cycle_wave_counter], (uint8_t *)&Node_self_message, sizeof(Node_self_message));
			//delay(5);
			//}
			
		//}
	digitalWrite(17,1);		
	}
	
	
	//LED debug
	
	
	
#elif NODE_number == 3


	if(node_distances_and_status.N4_starter){

		
		reset_flag = false;
		Node_self_message.send_msg = true;
		esp_now_send(NODE4, (uint8_t *)&Node_self_message, sizeof(Node_self_message));
		
		
		
		
	}
	
	if(From_other_node_msg.is_ack){
			//node_distances_and_status.N2_starter = false;
			//node_distances_and_status.N3_starter = false;
			node_distances_and_status.N4_starter = false;
			Node_self_message.send_msg = false;
			From_other_node_msg.is_ack = false;
			digitalWrite(17, 0);
		}
	
	if(send_flag && reset_flag){
		send_flag = false;
		//while(!From_other_node_msg.starter){
			
			//if(cycle_wave_counter > sizeof(peerList_node2)/sizeof(peerList_node2[0])-1) cycle_wave_counter = 0;
			//for(int k = 0; k != 10; k++){
			//sendStatusCounter++;
			esp_now_send(NODE4, (uint8_t *)&Node_self_message, sizeof(Node_self_message));
			//delay(5);
			//}
			
		//}	
		digitalWrite(17,1);
	}
	
	//LED debug
	
	

#else
	
	if(node_distances_and_status.N1_starter){

		reset_flag = false;
		Node_self_message.send_msg = true;
		esp_now_send(NODE1, (uint8_t *)&Node_self_message, sizeof(Node_self_message));
		
	}
	if(From_other_node_msg.is_ack){
			//node_distances_and_status.N2_starter = false;
			//node_distances_and_status.N3_starter = false;
			node_distances_and_status.N1_starter = false;
			Node_self_message.send_msg = false;
			From_other_node_msg.is_ack = false;
			digitalWrite(17, 0);
		}

	if(send_flag && reset_flag){
		send_flag = false;
		//while(!From_other_node_msg.starter){
			
			//if(cycle_wave_counter > sizeof(peerList_node2)/sizeof(peerList_node2[0])-1) cycle_wave_counter = 0;
			//for(int k = 0; k != 10; k++){
				
			//sendStatusCounter++;
			esp_now_send(NODE1, (uint8_t *)&Node_self_message, sizeof(Node_self_message));
			//delay(5);
			
			//}
			
		//}
		digitalWrite(17,1);
	}
	
	//LED debug

	

#endif	

	
}


