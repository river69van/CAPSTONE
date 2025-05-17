/*
    ESP-NOW Network Example
    Lucas Saavedra Vaz - 2024

    This example is based on the ESP-NOW example from the ESP-IDF framework.

    The aim of this example is to demonstrate how to create a network of devices using the ESP-NOW protocol.
    The slave devices will broadcast random data to the master device every 5 seconds and from time to time
    they will ping the other slave devices with a "Hello!" message.

    The master device will receive the data from the slave devices and print it to the Serial Monitor. From time
    to time, the master device will calculate the average of the priorities of the slave devices and send it to
    all the slave devices.

    Each device will have a priority that will be used to decide which device will be the master.
    The device with the highest priority will be the master.

    Flow:
    1. Each device will generate a priority based on its MAC address.
    2. The devices will broadcast their priority on the network.
    3. The devices will listen to the broadcast messages and register the priorities of the other devices.
    4. After all devices have been registered, the device with the highest priority will be the master.
    5. The slave devices will send random data to the master every 5 seconds.
      - Every "REPORT_INTERVAL" messages, the slaves will send a message to the other slaves.
    6. The master device will calculate the average of the data and send it to the slave devices every "REPORT_INTERVAL" messages.
	
------------------------------------------------------ akon logs -----------------------------------------------------------------------
	19/04/2025 02:32 {
		akon solution ha problema na; pag na paparong an naka established na, na node tapos ig turn on utro dre na hiya ma connect.
		ig reset kola an tanan na node ha spacific the counter based pa PPS count hahahaha ez
	}
	
	21/04/2025 05:15{
		Dapat ine na line "if(reset_counter % 5 == Node_Number)" may ig enclose somthing na, na verify
		na ready na tanan na nodes. Tas dapat an pag reset na solution ma depend ha time tikang han GPS module. 
		
	}
	
	23/04/2025 18:04{
		mas maupay ig run nala an send_func_block() ha loop or ig ticker nala tapos an ISR karan timer sync nala 
		tapos pwede liwat ig try and GPS time. 
	}
	
	
	17/05/2025 05:13{
		aayoson kopa an mga struct karan delta time tas an explosion_trigger dapat ma reset tas an mga delta time liwat dapat ma reset. 
	}
	

*/


#include <esp_wifi.h>
#include <esp_now.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include "ESP32_NOW.h"
#include "WiFi.h"
#include <math.h>
#include <esp_mac.h>  // For the MAC2STR and MACSTR macros
#include <Ticker.h>
#include <vector>
#include <xtensa/core-macros.h>
/* Definitions */

#define EARTH_RADIUS_KM 6371.0088 

#define GPS_PPS_pin       34
#define STM_trigger_pin   35

#define INTR_PIN 4

#define Node_Number 1
// Wi-Fi interface to be used by the ESP-NOW protocol
#define ESPNOW_WIFI_IFACE WIFI_IF_STA

// Channel to be used by the ESP-NOW protocol
#define ESPNOW_WIFI_CHANNEL 4

// Number of peers to wait for (excluding this device)
#define ESPNOW_PEER_COUNT 3

// Report to other devices every 5 messages
#define REPORT_INTERVAL 10

/*
    ESP-NOW uses the CCMP method, which is described in IEEE Std. 802.11-2012, to protect the vendor-specific action frame.
    The Wi-Fi device maintains a Primary Master Key (PMK) and several Local Master Keys (LMK).
    The lengths of both PMK and LMK need to be 16 bytes.

    PMK is used to encrypt LMK with the AES-128 algorithm. If PMK is not set, a default PMK will be used.

    LMK of the paired device is used to encrypt the vendor-specific action frame with the CCMP method.
    The maximum number of different LMKs is six. If the LMK of the paired device is not set, the vendor-specific
    action frame will not be encrypted.

    Encrypting multicast (broadcast address) vendor-specific action frame is not supported.

    PMK needs to be the same for all devices in the network. LMK only needs to be the same between paired devices.
*/

// Primary Master Key (PMK) and Local Master Key (LMK)
#define ESPNOW_EXAMPLE_PMK "pmk1234567890123"
#define ESPNOW_EXAMPLE_LMK "lmk1234567890123"



typedef struct {
	
  //for sync puposes 
  uint32_t count;
  uint32_t priority;
  bool ready;
  char str[7];
  
  //data side/mga karan distances tas node num
  int node_number;
  uint32_t data;
  
  //coordinates
  double latitude;
  double longitude;
  
  //explosion variables
  double delta_time;
  //bool is_explosion;
  
  
} __attribute__((packed)) esp_now_data_t;

typedef struct{

	double N_1_2;
	double N_1_3;
	double N_1_4;
	double N_2_3;
	double N_2_4;
	double N_3_4;
	
  double N1_coordinates_x;
  double N1_coordinates_y;
  double N1_delta_time;

  double N2_coordinates_x;
  double N2_coordinates_y;
  double N2_delta_time;

  double N3_coordinates_x;
  double N3_coordinates_y;
  double N3_delta_time;

  double N4_coordinates_x;
  double N4_coordinates_y;
  double N4_delta_time;

	
}node_dist;

struct Point { double x, y; }; //explosion coordinates


/* Global Variables */

uint32_t self_priority = 0;          // Priority of this device
uint8_t current_peer_count = 0;      // Number of peers that have been found
bool device_is_master = false;       // Flag to indicate if this device is the master
bool master_decided = false;         // Flag to indicate if the master has been decided
uint32_t sent_msg_count = 0;         // Counter for the messages sent. Only starts counting after all peers have been found
uint32_t recv_msg_count = 0;         // Counter for the messages received. Only starts counting after all peers have been found
esp_now_data_t new_msg;              // Message that will be sent to the peers
std::vector<uint32_t> last_data(5);  // Vector that will store the last 5 data received


//others
static const int RXPin = 18, TXPin = 19;	//GPIO han ig co connect han GPS module
static const uint32_t GPSBaud = 9600;
volatile bool start_flag = false;
static const int initialize_time_GPS_PPS = 20; 	// 1 min init time for the nodes to establish connection to the satellites 
static const int reset_node = 300;				//reset all node after 5 min 
int initialize_time_GPS_PPS_counter = 0;
volatile int reset_counter = 0;
volatile uint32_t count1   = 0;
volatile uint32_t elapsed  = 0;
volatile bool explosion_trigger = false;
// CPU clock
const uint32_t CPU_HZ = 240000000UL;
const double   time_per_cycle = 1.0 / CPU_HZ;  // = 4.16667e-9
const double speed_of_sound = 1510.0;


// The TinyGPSPlus object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

/* Structs */

// The following struct is used to send data to the peer device.
// We use the attribute "packed" to ensure that the struct is not padded (all data
// is contiguous in the memory and without gaps).
// The maximum size of the complete message is 250 bytes (ESP_NOW_MAX_DATA_LEN).
Ticker T;


node_dist Node_distances_struct;



/* Classes */

// We need to create a class that inherits from ESP_NOW_Peer and implement the _onReceive and _onSent methods.
// This class will be used to store the priority of the device and to send messages to the peers.
// For more information about the ESP_NOW_Peer class, see the ESP_NOW_Peer class in the ESP32_NOW.h file.

/*
void IRAM_ATTR intr_func(){
	
  reset_counter++;
  
  if(initialize_time_GPS_PPS_counter >= initialize_time_GPS_PPS){
	  
	//na cycle ine hiya from 0 to 4 para anti collision 
	if(reset_counter % 10 == Node_Number+1)
    start_flag = true;
	
  }else{
	  
	initialize_time_GPS_PPS_counter++;
	
  }
  
}

*/











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

    return EARTH_RADIUS_KM * c * 1e3;  // Distance in m
}
//end han code (node-to-node distance)



class ESP_NOW_Network_Peer : public ESP_NOW_Peer {
public:
  uint32_t priority;
  bool peer_is_master = false;
  bool peer_ready = false;

  ESP_NOW_Network_Peer(const uint8_t *mac_addr, uint32_t priority = 0, const uint8_t *lmk = (const uint8_t *)ESPNOW_EXAMPLE_LMK)
    : ESP_NOW_Peer(mac_addr, ESPNOW_WIFI_CHANNEL, ESPNOW_WIFI_IFACE, lmk), priority(priority) {}

  ~ESP_NOW_Network_Peer() {}

  bool begin() {
    // In this example the ESP-NOW protocol will already be initialized as we require it to receive broadcast messages.
    if (!add()) {
      log_e("Failed to initialize ESP-NOW or register the peer");
      return false;
    }
    return true;
  }

  bool send_message(const uint8_t *data, size_t len) {
    if (data == NULL || len == 0) {
      log_e("Data to be sent is NULL or has a length of 0");
      return false;
    }

    // Call the parent class method to send the data
    return send(data, len);
  }

  void onReceive(const uint8_t *data, size_t len, bool broadcast) {
    esp_now_data_t *msg = (esp_now_data_t *)data;

    if (peer_ready == false && msg->ready == true) {
      //Serial.printf("Peer " MACSTR " reported ready\n", MAC2STR(addr()));
      peer_ready = true;
    }
	
	
	
	// dd han pag access han mga data ha tanan na nodes
    if (!broadcast) {
      recv_msg_count++;
      if (device_is_master) {
		 
		 
		// pag kuha han coordinates tikang ha iba 
		switch(msg->node_number){
			case 1:
				Node_distances_struct.N1_coordinates_x = msg->latitude;
				Node_distances_struct.N1_coordinates_y = msg->longitude;
				//Serial.printf("node%d lat: %.2f long: %.2f\n",msg->node_number, msg->latitude, msg->longitude);
			break;
			case 2:
				Node_distances_struct.N2_coordinates_x = msg->latitude;
				Node_distances_struct.N2_coordinates_y = msg->longitude;
				//Serial.printf("node%d lat: %.2f long: %.2f\n",msg->node_number, msg->latitude, msg->longitude);
			break;
			case 3:
				Node_distances_struct.N3_coordinates_x = msg->latitude;
				Node_distances_struct.N3_coordinates_y = msg->longitude;
				//Serial.printf("node%d lat: %.2f long: %.2f\n",msg->node_number, msg->latitude, msg->longitude);
			break;
			case 4:
				Node_distances_struct.N4_coordinates_x = msg->latitude;
				Node_distances_struct.N4_coordinates_y = msg->longitude;
				//Serial.printf("node%d lat: %.2f long: %.2f\n",msg->node_number, msg->latitude, msg->longitude);
			break;
			default:
			break;
		}
		  
		/*
        Serial.printf("Received a message from peer " MACSTR "\n", MAC2STR(addr()));
        Serial.printf("  Count: %lu\n", msg->count);
        Serial.printf("  Random data: %lu\n", msg->data);
		*/
		
		
        last_data.push_back(msg->data);
        last_data.erase(last_data.begin());
      } else if (peer_is_master) {
		  
		switch(msg->node_number){
			case 1:
				Node_distances_struct.N1_coordinates_x = msg->latitude;
				Node_distances_struct.N1_coordinates_y = msg->longitude;
				//Serial.printf("node%d lat: %.2f long: %.2f\n",msg->node_number, msg->latitude, msg->longitude);
			break;
			case 2:
				Node_distances_struct.N2_coordinates_x = msg->latitude;
				Node_distances_struct.N2_coordinates_y = msg->longitude;
				//Serial.printf("node%d lat: %.2f long: %.2f\n",msg->node_number, msg->latitude, msg->longitude);
			break;
			case 3:
				Node_distances_struct.N3_coordinates_x = msg->latitude;
				Node_distances_struct.N3_coordinates_y = msg->longitude;
				//Serial.printf("node%d lat: %.2f long: %.2f\n",msg->node_number, msg->latitude, msg->longitude);
			break;
			case 4:
				Node_distances_struct.N4_coordinates_x = msg->latitude;
				Node_distances_struct.N4_coordinates_y = msg->longitude;
				//Serial.printf("node%d lat: %.2f long: %.2f\n",msg->node_number, msg->latitude, msg->longitude);
			break;
			default:
			break;  
		}
		
        //Serial.println("Received a message from the master");
        //Serial.printf("  Average data: %lu\n", msg->data);
      } else {
		  
		  switch(msg->node_number){
			case 1:
				Node_distances_struct.N1_coordinates_x = msg->latitude;
				Node_distances_struct.N1_coordinates_y = msg->longitude;
				//Serial.printf("node%d lat: %.2f long: %.2f\n",msg->node_number, msg->latitude, msg->longitude);
			break;
			case 2:
				Node_distances_struct.N2_coordinates_x = msg->latitude;
				Node_distances_struct.N2_coordinates_y = msg->longitude;
				//Serial.printf("node%d lat: %.2f long: %.2f\n",msg->node_number, msg->latitude, msg->longitude);
			break;
			case 3:
				Node_distances_struct.N3_coordinates_x = msg->latitude;
				Node_distances_struct.N3_coordinates_y = msg->longitude;
				//Serial.printf("node%d lat: %.2f long: %.2f\n",msg->node_number, msg->latitude, msg->longitude);
			break;
			case 4:
				Node_distances_struct.N4_coordinates_x = msg->latitude;
				Node_distances_struct.N4_coordinates_y = msg->longitude;
				//Serial.printf("node%d lat: %.2f long: %.2f\n",msg->node_number, msg->latitude, msg->longitude);
			break;
			default:
			break;
		  }
		  
        //Serial.printf("Peer " MACSTR " says: %s\n", MAC2STR(addr()), msg->str);
      }
    }
  }

  void onSent(bool success) {
    bool broadcast = memcmp(addr(), ESP_NOW.BROADCAST_ADDR, ESP_NOW_ETH_ALEN) == 0;
    if (broadcast) {
      log_i("Broadcast message reported as sent %s", success ? "successfully" : "unsuccessfully");
    } else {
      log_i("Unicast message reported as sent %s to peer " MACSTR, success ? "successfully" : "unsuccessfully", MAC2STR(addr()));
    }
  }
};

/* Peers */

std::vector<ESP_NOW_Network_Peer *> peers;                             // Create a vector to store the peer pointers
ESP_NOW_Network_Peer broadcast_peer(ESP_NOW.BROADCAST_ADDR, 0, NULL);  // Register the broadcast peer (no encryption support for the broadcast address)
ESP_NOW_Network_Peer *master_peer = nullptr;                           // Pointer to peer that is the master

/* Helper functions */

// Function to reboot the device
void fail_reboot() {
  Serial.println("Rebooting in 5 seconds...");
  delay(5000);
  ESP.restart();
}

// Function to check which device has the highest priority
uint32_t check_highest_priority() {
  uint32_t highest_priority = 0;
  for (auto &peer : peers) {
    if (peer->priority > highest_priority) {
      highest_priority = peer->priority;
    }
  }
  return std::max(highest_priority, self_priority);
}

// Function to calculate the average of the data received
uint32_t calc_average() {
  uint32_t avg = 0;
  for (auto &d : last_data) {
    avg += d;
  }
  avg /= last_data.size();
  return avg;
}

// Function to check if all peers are ready
bool check_all_peers_ready() {
  for (auto &peer : peers) {
    if (!peer->peer_ready) {
      return false;
    }
  }
  return true;
}

/* Callbacks */

// Callback called when a new peer is found
void register_new_peer(const esp_now_recv_info_t *info, const uint8_t *data, int len, void *arg) {
  esp_now_data_t *msg = (esp_now_data_t *)data;
  int priority = msg->priority;

  if (priority == self_priority) {
    Serial.println("ERROR! Device has the same priority as this device. Unsupported behavior.");
    fail_reboot();
  }

  if (current_peer_count < ESPNOW_PEER_COUNT) {
    Serial.printf("New peer found: " MACSTR " with priority %d\n", MAC2STR(info->src_addr), priority);
    ESP_NOW_Network_Peer *new_peer = new ESP_NOW_Network_Peer(info->src_addr, priority);
    if (new_peer == nullptr || !new_peer->begin()) {
      Serial.println("Failed to create or register the new peer");
      delete new_peer;
      return;
    }
    peers.push_back(new_peer);
    current_peer_count++;
    if (current_peer_count == ESPNOW_PEER_COUNT) {
      Serial.println("All peers have been found");
      new_msg.ready = true;
    }
  }
}




//----------------ISR----------------------
void IRAM_ATTR GPS_PPS_func() {
	
	reset_counter++;

	if(initialize_time_GPS_PPS_counter >= initialize_time_GPS_PPS){

		//na cycle ine hiya from 0 to 4 para anti collision 
		if(reset_counter % 10 == Node_Number+1)
		start_flag = true;

	}else{

		initialize_time_GPS_PPS_counter++;

	}
	
	if(check_all_peers_ready()){
		
		count1 = xthal_get_ccount();
		
	}
}

void IRAM_ATTR STM_trigger_func() {
	
  if(check_all_peers_ready()){
	  
	  explosion_trigger = true;
	  uint32_t now = xthal_get_ccount();
	  // handle possible wrap‑around
	  if (now >= count1) {
		  
		elapsed = now - count1;
		
	  } else {
		  
		elapsed = (0xFFFFFFFFu - count1) + now + 1;
		
	  }
	  //test_trig = true;
  }
}




/* Main */

void send_func_block(){
	
  if (!master_decided) {
    // Broadcast the priority to find the master
    if (!broadcast_peer.send_message((const uint8_t *)&new_msg, sizeof(new_msg))) {
      Serial.println("Failed to broadcast message");
    }

    // Check if all peers have been found
    if (current_peer_count == ESPNOW_PEER_COUNT) {
      // Wait until all peers are ready
      if (check_all_peers_ready()) {
        Serial.println("All peers are ready");
        // Check which device has the highest priority
        master_decided = true;
        uint32_t highest_priority = check_highest_priority();
        if (highest_priority == self_priority) {
          device_is_master = true;
          Serial.println("This device is the master");
        } else {
          for (int i = 0; i < ESPNOW_PEER_COUNT; i++) {
            if (peers[i]->priority == highest_priority) {
              peers[i]->peer_is_master = true;
              master_peer = peers[i];
              Serial.printf("Peer " MACSTR " is the master with priority %lu\n", MAC2STR(peers[i]->addr()), highest_priority);
              break;
            }
          }
        }
        Serial.println("The master has been decided");
      } else {
        Serial.println("Waiting for all peers to be ready...");
      }
    }
  } else {
    if (!device_is_master) {
      // Send a message to the master
      new_msg.count = sent_msg_count + 1;
	  
      new_msg.data = 1;
	  
      if (!master_peer->send_message((const uint8_t *)&new_msg, sizeof(new_msg))) {
        Serial.println("Failed to send message to the master");
      } else {
        Serial.printf("Sent message to the master. Count: %lu, Data: %lu\n", new_msg.count, new_msg.data);
        sent_msg_count++;
      }

      // Check if it is time to report to peers
      if (sent_msg_count % REPORT_INTERVAL == 0) {
        // Send a message to the peers
        for (auto &peer : peers) {
          if (!peer->peer_is_master) {
            if (!peer->send_message((const uint8_t *)&new_msg, sizeof(new_msg))) {
              Serial.printf("Failed to send message to peer " MACSTR "\n", MAC2STR(peer->addr()));
            } else {
              Serial.printf("Sent message \"%s\" to peer " MACSTR "\n", new_msg.str, MAC2STR(peer->addr()));
            }
          }
        }
      }
    } else {
      // Check if it is time to report to peers
      if (recv_msg_count % REPORT_INTERVAL == 0) {
        // Report average data to the peers
        uint32_t avg = calc_average();
        new_msg.data = avg;
        for (auto &peer : peers) {
          new_msg.count = sent_msg_count + 1;
          if (!peer->send_message((const uint8_t *)&new_msg, sizeof(new_msg))) {
            Serial.printf("Failed to send message to peer " MACSTR "\n", MAC2STR(peer->addr()));
          } else {
            Serial.printf(
              "Sent message to peer " MACSTR ". Recv: %lu, Sent: %lu, Avg: %lu\n", MAC2STR(peer->addr()), recv_msg_count, new_msg.count, new_msg.data
            );
            sent_msg_count++;
          }
        }
      }
    }
  }


}


// pag calculate na han node to node distances
void calc_node_distances(node_dist *distances){
	
	distances->N_1_2 = haversine(distances->N1_coordinates_x, distances->N1_coordinates_y, distances->N2_coordinates_x, distances->N2_coordinates_y );
	distances->N_1_3 = haversine(distances->N1_coordinates_x, distances->N1_coordinates_y, distances->N3_coordinates_x, distances->N3_coordinates_y );
	distances->N_1_4 = haversine(distances->N1_coordinates_x, distances->N1_coordinates_y, distances->N4_coordinates_x, distances->N4_coordinates_y );
	distances->N_2_3 = haversine(distances->N2_coordinates_x, distances->N2_coordinates_y, distances->N3_coordinates_x, distances->N3_coordinates_y );
	distances->N_2_4 = haversine(distances->N2_coordinates_x, distances->N2_coordinates_y, distances->N4_coordinates_x, distances->N4_coordinates_y );
	distances->N_3_4 = haversine(distances->N3_coordinates_x, distances->N3_coordinates_y, distances->N4_coordinates_x, distances->N4_coordinates_y );
}

bool trilaterate(
    const Point& p1, double r1,
    const Point& p2, double r2,
    const Point& p3, double r3,
    Point& out)
{
    double dx = p2.x - p1.x, dy = p2.y - p1.y;
    double d = sqrt(dx*dx + dy*dy);
    if (d == 0) return false;
    double exx = dx / d, exy = dy / d;

    double ix = p3.x - p1.x, iy = p3.y - p1.y;
    double i  = exx*ix + exy*iy;

    double tempX = ix - i*exx, tempY = iy - i*exy;
    double tempDist = sqrt(tempX*tempX + tempY*tempY);
    if (tempDist == 0) return false;
    double eyx = tempX / tempDist, eyy = tempY / tempDist;

    double j = eyx*ix + eyy*iy;

    double x = (r1*r1 - r2*r2 + d*d) / (2*d);
    double y = (r1*r1 - r3*r3 + i*i + j*j) / (2*j) - (i/j)*x;

    out.x = p1.x + x*exx + y*eyx;
    out.y = p1.y + x*exy + y*eyy;
    return true;
}


void computeExplosionRelative(const node_dist& nd, Point& explosionRel) {
    // compute ranges in meters
    double r[5];
    r[1] = nd.N1_delta_time * time_per_cycle * speed_of_sound;
    r[2] = nd.N2_delta_time * time_per_cycle * speed_of_sound;
    r[3] = nd.N3_delta_time * time_per_cycle * speed_of_sound;
    r[4] = nd.N4_delta_time * time_per_cycle * speed_of_sound;

    // absolute positions
    Point absP[5];
    absP[1] = { nd.N1_coordinates_x, nd.N1_coordinates_y };
    absP[2] = { nd.N2_coordinates_x, nd.N2_coordinates_y };
    absP[3] = { nd.N3_coordinates_x, nd.N3_coordinates_y };
    absP[4] = { nd.N4_coordinates_x, nd.N4_coordinates_y };

    // identify master: this code runs on master, so Node_Number is its ID
    Point pMaster = absP[Node_Number];

    // pick any three *other* nodes for trilateration
    int a=0,b=0,c=0, idx=1;
    for (int i = 1; i <= 4 && idx<4; ++i) {
        if (i == Node_Number) continue;
        if (a==0) a = i;
        else if (b==0) b = i;
        else if (c==0) c = i;
        idx++;
    }

    // shift into master‑relative frame
    Point p1 = { absP[a].x - pMaster.x, absP[a].y - pMaster.y };
    Point p2 = { absP[b].x - pMaster.x, absP[b].y - pMaster.y };
    Point p3 = { absP[c].x - pMaster.x, absP[c].y - pMaster.y };
    double r1 = r[a], r2 = r[b], r3 = r[c];

    // do trilateration
    if (!trilaterate(p1, r1, p2, r2, p3, r3, explosionRel)) {
        // fallback: try a different triple if needed...
        explosionRel.x = explosionRel.y = NAN;
    }
}



void setup() {
  uint8_t self_mac[6];

  Serial.begin(115200);
  ss.begin(GPSBaud);
  // Initialize the Wi-Fi module
  WiFi.mode(WIFI_STA);
  WiFi.setChannel(ESPNOW_WIFI_CHANNEL);
  while (!WiFi.STA.started()) {
    delay(100);
  }
  
  esp_err_t err = esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR);
  if (err != ESP_OK) {
	fail_reboot();
    Serial.printf("esp_wifi_set_protocol failed: %d\n", err);
  } else {
    Serial.println("Long‑Range mode enabled");
  }

  Serial.println("ESP-NOW Network Example");
  Serial.println("Wi-Fi parameters:");
  Serial.println("  Mode: STA");
  Serial.println("  MAC Address: " + WiFi.macAddress());
  Serial.printf("  Channel: %d\n", ESPNOW_WIFI_CHANNEL);

  // Generate yhis device's priority based on the 3 last bytes of the MAC address
  WiFi.macAddress(self_mac);
  self_priority = self_mac[3] << 16 | self_mac[4] << 8 | self_mac[5];
  Serial.printf("This device's priority: %lu\n", self_priority);

  // Initialize the ESP-NOW protocol
  if (!ESP_NOW.begin((const uint8_t *)ESPNOW_EXAMPLE_PMK)) {
    Serial.println("Failed to initialize ESP-NOW");
    fail_reboot();
  }

  if (!broadcast_peer.begin()) {
    Serial.println("Failed to initialize broadcast peer");
    fail_reboot();
  }

  // Register the callback to be called when a new peer is found
  ESP_NOW.onNewPeer(register_new_peer, NULL);

  Serial.println("Setup complete. Broadcasting own priority to find the master...");
  memset(&new_msg, 0, sizeof(new_msg));
  strncpy(new_msg.str, "Hello!", sizeof(new_msg.str));
  new_msg.priority = self_priority;
  new_msg.node_number = Node_Number;
  
  //T.attach(1, intr_func);
  
  pinMode(GPS_PPS_pin,     INPUT_PULLUP);
  pinMode(STM_trigger_pin, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(GPS_PPS_pin), GPS_PPS_func, FALLING);
  attachInterrupt(digitalPinToInterrupt(STM_trigger_pin), STM_trigger_func, FALLING);
  
  
  //dummy data
switch(Node_Number){
	case 1:
	
		new_msg.latitude = 1.0;
		new_msg.longitude = 1.1;
		
	break;
	case 2:
	
		new_msg.latitude = 2.0;
		new_msg.longitude = 2.2;
		
	break;
	case 3:
	
		new_msg.latitude = 3.0;
		new_msg.longitude = 3.3;

	break;
	case 4:
	
		new_msg.latitude = 4.0;
		new_msg.longitude = 4.4;
    Node_distances_struct.N4_coordinates_x = 4.0;
    Node_distances_struct.N4_coordinates_y = 4.4;

	break;
	default:
	break;
	
	
}
  
  
}

void loop() {
	
	if(start_flag){
		start_flag = false;
		Serial.println("na gana");
		send_func_block();
		//if(reset_counter >= reset_node)ESP.restart();
		
		if(check_all_peers_ready()){
		print_dist(&Node_distances_struct);
		calc_node_distances(&Node_distances_struct);
		}
	}
	
	
	while (ss.available() > 0){
		
      if (gps.encode(ss.read()))
      getInfo(&Node_distances_struct, &new_msg);
  
    }
	
	
	if (check_all_peers_ready() && explosion_trigger && device_is_master) {
		
    calc_node_distances(&Node_distances_struct);

    Point explosionRel;
    computeExplosionRelative(Node_distances_struct, explosionRel);
    if (!isnan(explosionRel.x)) {
        // explosionRel is in meters relative to master at (0,0)
        Serial.printf(
            "Explosion approx. at relative (%.2f m, %.2f m) from master\n",
            explosionRel.x, explosionRel.y
        );
        explosion_trigger = false;
    } else {
        Serial.println("Trilateration failed (colinear or bad data).");
    }
	}
  

}


void getInfo(node_dist *dist, esp_now_data_t *loc_msg){
	
  if (gps.location.isValid() && gps.hdop.hdop() < 2.0){
	  
  double lat = gps.location.lat();
  double lng = gps.location.lng();
  
  //pag kuha han sefl coordinates tas delta time
  switch(Node_Number){
	case 1:
		dist->N1_coordinates_x = lat;
		dist->N1_coordinates_y = lng;
		dist->N1_delta_time = elapsed;
	break;
	case 2:
		dist->N2_coordinates_x = lat;
		dist->N2_coordinates_y = lng;
		dist->N2_delta_time = elapsed;
	break;
	case 3:
		dist->N3_coordinates_x = lat;
		dist->N3_coordinates_y = lng;
		dist->N3_delta_time = elapsed;
	break;
	case 4:
		dist->N4_coordinates_x = lat;
		dist->N4_coordinates_y = lng;
		dist->N4_delta_time = elapsed;
	break;
	default:
	break;
	
    }
  
  
  //ig se send ha iba an mga coordinates
  loc_msg->latitude = lat;
  loc_msg->longitude = lng;
  if(explosion_trigger){
	loc_msg->delta_time = elapsed * time_per_cycle;
	//explosion_trigger = false;
  }
  
  
  
  
  //Serial.println(Node_self_message.latitude, 5);
  //Serial.println(Node_self_message.longitude, 5);
  }
}



void print_dist(node_dist *dist){
	
	Serial.printf("N1_coordinates_x: %.6f N1_coordinates_y: %.6f\n", dist->N1_coordinates_x, dist->N1_coordinates_y);
	Serial.printf("N2_coordinates_x: %.6f N1_coordinates_y: %.6f\n", dist->N2_coordinates_x, dist->N2_coordinates_y);
	Serial.printf("N3_coordinates_x: %.6f N1_coordinates_y: %.6f\n", dist->N3_coordinates_x, dist->N3_coordinates_y);
	Serial.printf("N4_coordinates_x: %.6f N1_coordinates_y: %.6f\n", dist->N4_coordinates_x, dist->N4_coordinates_y);
	
	
	Serial.printf("N_1_2: %.6f\n", dist->N_1_2);
	Serial.printf("N_1_3: %.6f\n", dist->N_1_3);
	Serial.printf("N_1_4: %.6f\n", dist->N_1_4);
	Serial.printf("N_2_3: %.6f\n", dist->N_2_3);
	Serial.printf("N_2_4: %.6f\n", dist->N_2_4);
	Serial.printf("N_3_4: %.6f\n", dist->N_3_4);

}
