#include <math.h>
#include <esp_now.h>
#include <WiFi.h>
#include <Ticker.h>



//change based on node number
#define NODE_number 2

Ticker T;

bool master_starter = false;
int sendStatusCounter = 0;
volatile unsigned long t = 0;
bool start_counting2 = false;
bool start_counting3 = false;
bool start_counting4 = false;
volatile int cycle_wave_counter = -1;

static const int RXPin = 18, TXPin = 19;	//GPIO han ig co connect han GPS module
static const uint32_t GPSBaud = 9600;
volatile bool send_flag = false;

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
	
	int message;
	double latitude;
	double longitude;
	int self_priority;
	bool master_node;
	bool starter;
	bool send_msg;
	
}__attribute__((packed)) struct_message;

node_dist node_distances;
struct_message Node_self_message; 		//An sulod mismo na data (tanan na api na an mga flags) na ig se send ha iba 
struct_message From_other_node_msg; 	//An ma ta-tanggap na data mangangadi 



void Interrupt_func() {
	
	cycle_wave_counter++;
	send_flag = true;
	From_other_node_msg.starter = false;

}



//just the setup part
void setupPeers() {
	
  for (int i = 0; i < NUM_PEERS; i++) {
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, peerList[i], 6);
    peerInfo.channel = 0;          // Use current WiFi channel
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
	send_flag = true;
	memcpy(&From_other_node_msg, incomingData, sizeof(From_other_node_msg));
	esp_now_send(info->src_addr, (uint8_t *)&Node_self_message, sizeof(Node_self_message));
	switch(From_other_node_msg.message){
		
		case 1:
			
		break;
		case 2:
			Serial.println("from N2");
		break;
		case 3:
			Serial.println("from N3");
		break;
		case 4:
			Serial.println("from N4");
		break;
		default:
			
		break;
		
	}
	
}



void setup(){
	pinMode(17, OUTPUT);
	pinMode(5, OUTPUT);
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

	//T.attach(1, Interrupt_func);   // Call reset() every 1 second


	Node_self_message.message = NODE_number;
	Node_self_message.starter = true;
	From_other_node_msg.starter = false;

}

void loop() {
	
	if(send_flag){
		
		digitalWrite(17, 1);
		delay(1000);
		digitalWrite(17, 0);
		delay(1000);
		
	}
	
}


