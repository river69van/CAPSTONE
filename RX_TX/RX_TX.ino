#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <math.h>
#include <esp_now.h>
#include <WiFi.h>

#define EARTH_RADIUS_KM 6371.0088  // Earth's radius in km
#define INTR_PIN 4

//change based on node number
#define NODE_number 1



bool master_starter = false;
int sendStatusCounter = 0;
volatile unsigned long t = 0;
bool start_counting2 = false;
bool start_counting3 = false;
bool start_counting4 = false;
int cycle_wave_counter = 0;

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
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

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

node_dist node_distances;

typedef struct struct_message {
	
	int message;
	double latitude;
	double longitude;
	int self_priority;
	bool master_node;
	bool starter;
	
}__attribute__((packed)) struct_message;

struct_message Node_self_message; //An sulod mismo na data (tanan na api na an mga flags) na ig se send ha iba 
struct_message From_other_node_msg; //An ma ta-tanggap na data mangangadi 


void IRAM_ATTR gpsPPSInterrupt() {
	
	
	initialize_node_cycle_counter++;
	From_other_node_msg.starter = true;
	
	

#if NODE_number == 1

	if(initialize_node_cycle_counter >= 11)initialize_node_cycle_counter = 11;
	if(initialize_node_cycle_counter >= 10)send_flag = true;

#elif NODE_number == 2

	if(start_counting2){
		
		if(initialize_node_cycle_counter >= 11)initialize_node_cycle_counter = 11;
		if(initialize_node_cycle_counter >= 10)send_flag = true;
		
	}

#elif NODE_number == 3

	if(start_counting2 && start_counting3){
		
		if(initialize_node_cycle_counter >= 11)initialize_node_cycle_counter = 11;
		if(initialize_node_cycle_counter >= 10)send_flag = true;
	}
	
#else
	//Serial.println("error hehehe");
	//Node 4 logics
#endif

	
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


//Amo na ine na part an pag calculate hin distance kada node gamit han coordinates han GPS module (kaylangan 2 na inputs)
//start han code (node-to-node distance)
double degToRad(double deg) {
    return deg * (M_PI / 180.0);
}

double haversine(double lat1, double lon1, double lat2, double lon2) {
    double dLat = degToRad(lat2 - lat1);
    double dLon = degToRad(lon2 - lon1);
    
    double a = sin(dLat / 2) * sin(dLat / 2) +
               cos(degToRad(lat1)) * cos(degToRad(lat2)) *
               sin(dLon / 2) * sin(dLon / 2);

    double c = 2 * atan2(sqrt(a), sqrt(1 - a));

    return EARTH_RADIUS_KM * c;  // Distance in km
}
//end han code (node-to-node distance)

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
	//Serial.println("sent");
}


void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {

if(initialize_node_cycle_counter >= 10){
	sendStatusCounter++;
	memcpy(&From_other_node_msg, incomingData, sizeof(From_other_node_msg));
	
	double distance = haversine(Node_self_message.latitude, 
								Node_self_message.longitude, 
								From_other_node_msg.latitude, 
								From_other_node_msg.longitude);
								
#if NODE_number == 1
	//start_counting2 = true;
	switch(From_other_node_msg.message){
		case 2:
			node_distances.N_1_2 = distance;
		break;
		case 3:
			node_distances.N_1_3 = distance;
		break;
		case 4:
			node_distances.N_1_4 = distance;
		break;
		default:
			Serial.println("error hehe");
		break;
	}
	//esp_now_send(info->src_addr, (uint8_t *)&Node_self_message, sizeof(Node_self_message));
	
#elif NODE_number == 2
	start_counting2 = true;
	switch(From_other_node_msg.message){
		case 1:
			node_distances.N_1_2 = distance;
		break;
		case 3:
			node_distances.N_2_3 = distance;
		break;
		case 4:
			node_distances.N_2_4 = distance;
		break;
		default:
			Serial.println("error hehe");
		break;
	}
	//esp_now_send(info->src_addr, (uint8_t *)&Node_self_message, sizeof(Node_self_message));
#elif NODE_number == 3
	start_counting3 = true;
	switch(From_other_node_msg.message){
		case 1:
			node_distances.N_1_3 = distance;
		break;
		case 2:
			node_distances.N_2_3 = distance;
		break;
		case 4:
			node_distances.N_3_4 = distance;
		break;
		default:
			Serial.println("error hehe");
		break;
	}
	//esp_now_send(info->src_addr, (uint8_t *)&Node_self_message, sizeof(Node_self_message));
#else
	switch(From_other_node_msg.message){
		case 1:
			node_distances.N_1_4 = distance;
		break;
		case 2:
			node_distances.N_2_4 = distance;
		break;
		case 4:
			node_distances.N_3_4 = distance;
		break;
		default:
			Serial.println("error hehe");
		break;
	}
	//esp_now_send(info->src_addr, (uint8_t *)&Node_self_message, sizeof(Node_self_message));
#endif
	
	esp_now_send(info->src_addr, (uint8_t *)&Node_self_message, sizeof(Node_self_message));
	Serial.println("Distance: ");
	Serial.print(distance * 1e3);
	Serial.println("m");
	
	
	
	}
}



void setup(){

	Serial.begin(115200);
	WiFi.mode(WIFI_STA);
	setCpuFrequencyMhz(240);  // Set CPU to 240 MHz
	ss.begin(GPSBaud);
	delay(100);
	t = micros();         // Initialize t
	if (esp_now_init() != ESP_OK) {

		Serial.println(" ESP-NOW init failed");
		return;
    
	}

	esp_now_register_send_cb(onDataSent);
	esp_now_register_recv_cb((esp_now_recv_cb_t) onDataRecv);



	setupPeers();

	pinMode(INTR_PIN, INPUT_PULLUP);
	attachInterrupt(digitalPinToInterrupt(INTR_PIN), gpsPPSInterrupt, FALLING);
  
  //change based on node num
  Node_self_message.message = NODE_number;
  From_other_node_msg.starter = false;
  Node_self_message.starter = true;


}
void loop() {
    // Always process GPS serial data
    while (ss.available() > 0) {
        if (gps.encode(ss.read())) {
            store_data();  
        }
    }

    // When PPS ISR sets the send_flag, do ESP-NOW transmission
    if (send_flag) {
        send_flag = false;

        if (initialize_node_cycle_counter >= 10) {
#if NODE_number == 1
			cycle_wave_counter++;
			for (int i = 0; i < NUM_PEERS-1; i++) {
				
				while(!From_other_node_msg.starter){
					esp_now_send(peerList_node1[i], (uint8_t *)&Node_self_message, sizeof(Node_self_message));
					delay(5);
				}
					
			}
				
#elif NODE_number == 2

            for (int i = 0; i < NUM_PEERS-2; i++) 
				
                while(!From_other_node_msg.starter){
                    esp_now_send(peerList_node2[i], (uint8_t *)&Node_self_message, sizeof(Node_self_message));
					delay(5);
                }
            }
			
#elif NODE_number == 3
			
			while(!From_other_node_msg.starter){
				esp_now_send(NODE4, (uint8_t *)&Node_self_message, sizeof(Node_self_message));
				delay(5);
			}
			
#else
            Serial.println("hehehe error sobra node");
		//Node 4 logics
#endif
        }
    }
}

void store_data(){
  if (gps.location.isValid()){
  Node_self_message.latitude = gps.location.lat();
  Node_self_message.longitude = gps.location.lng();
  //Serial.println(Node_self_message.latitude, 5);
  //Serial.println(Node_self_message.longitude, 5);
  }
}
