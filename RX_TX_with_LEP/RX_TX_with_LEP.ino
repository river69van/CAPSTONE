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
	

*/
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include "ESP32_NOW.h"
#include "WiFi.h"
#include <math.h>
#include <esp_mac.h>  // For the MAC2STR and MACSTR macros
#include <Ticker.h>
#include <vector>

/* Definitions */

#define EARTH_RADIUS_KM 6371.0088 


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



//others
static const int RXPin = 18, TXPin = 19;	//GPIO han ig co connect han GPS module
static const uint32_t GPSBaud = 9600;
volatile bool start_flag = false;
static const int initialize_time_GPS_PPS = 20; 	// 1 min init time for the nodes to establish connection to the satellites 
static const int reset_node = 300;				//reset all node after 5 min 
int initialize_time_GPS_PPS_counter = 0;
int reset_counter = 0;


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

  double N2_coordinates_x;
  double N2_coordinates_y;

  double N3_coordinates_x;
  double N3_coordinates_y;

  double N4_coordinates_x;
  double N4_coordinates_y;

	
}node_dist;

node_dist Node_distances_struct;

/* Global Variables */

uint32_t self_priority = 0;          // Priority of this device
uint8_t current_peer_count = 0;      // Number of peers that have been found
bool device_is_master = false;       // Flag to indicate if this device is the master
bool master_decided = false;         // Flag to indicate if the master has been decided
uint32_t sent_msg_count = 0;         // Counter for the messages sent. Only starts counting after all peers have been found
uint32_t recv_msg_count = 0;         // Counter for the messages received. Only starts counting after all peers have been found
esp_now_data_t new_msg;              // Message that will be sent to the peers
std::vector<uint32_t> last_data(5);  // Vector that will store the last 5 data received

/* Classes */

// We need to create a class that inherits from ESP_NOW_Peer and implement the _onReceive and _onSent methods.
// This class will be used to store the priority of the device and to send messages to the peers.
// For more information about the ESP_NOW_Peer class, see the ESP_NOW_Peer class in the ESP32_NOW.h file.

//ISR ---------------------------
void IRAM_ATTR intr_func(){
	
  reset_counter++;
  
  if(initialize_time_GPS_PPS_counter >= initialize_time_GPS_PPS){
	  
	//na cycle ine hiya from 0 to 4 para anti collision 
	if(reset_counter % 5 == Node_Number)
    start_flag = true;
	
  }else{
	  
	initialize_time_GPS_PPS_counter++;
	
  }
  
}

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
  
  pinMode(INTR_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(INTR_PIN), intr_func, FALLING);
  
  
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
	
  

}


void getInfo(node_dist *dist, esp_now_data_t *loc_msg){
	
  if (gps.location.isValid() && gps.hdop.hdop() < 2.0){
	  
  double lat = gps.location.lat();
  double lng = gps.location.lng();
  
  //pag kuha han sefl coordinates
  switch(Node_Number){
	case 1:
		dist->N1_coordinates_x = lat;
		dist->N1_coordinates_y = lng;
	break;
	case 2:
		dist->N2_coordinates_x = lat;
		dist->N2_coordinates_y = lng;
	break;
	case 3:
		dist->N3_coordinates_x = lat;
		dist->N3_coordinates_y = lng;
	break;
	case 4:
		dist->N4_coordinates_x = lat;
		dist->N4_coordinates_y = lng;
	break;
	default:
	break;
	
    }
  
  
  //ig se send ha iba an mga coordinates
  loc_msg->latitude = lat;
  loc_msg->longitude = lng;
  
  
  
  
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
