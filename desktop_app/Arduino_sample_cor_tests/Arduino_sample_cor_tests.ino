#include <TimerOne.h>
char flag = 'S';
int num = 0;
int y_coor = 100.0;
int x_coor = 100.0;
int incr = 0;
unsigned long previousMillis = 0;
int x = 0;
int y = 0;
int node;
int lower = 5; // Minimum value
int upper = 500; // Maximum value


//protocol na ine hiya para ma receive han desktop app
void sample_message(char FLAG,  int Xcoor, int Ycoor, int Snum = 0){
	Serial.println(FLAG);
	Serial.println(Snum);
	Serial.println("A");
	Serial.println(Xcoor);
	Serial.println(Ycoor);
	//Serial.println("D");
}


void run(){
	
  int randomNumber1x = random(lower, upper + 1);

  int randomNumber1y = random(lower, upper + 1);


  if(incr > 1 && incr < 6){
	  
	sample_message('S', randomNumber1x, randomNumber1y, incr);
	
  }else if(incr == 1){
	  
	sample_message('E', 200, 300, incr);
	
  }else if(incr == 0){
	  
	sample_message('M', 300, 500, incr);
	
  }
  
}




void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Timer1.initialize(500000); // Set interval to 500,000 microseconds (0.5 sec)
  Timer1.attachInterrupt(run);
  randomSeed(analogRead(0));
}


void loop() {
  
  unsigned long currentMillis = millis();
  
  if (currentMillis - previousMillis >= 1000) {
    previousMillis = currentMillis; // Reset timer
    incr++;
	if(incr>5)
		incr = 0;
  }
  
  //SLAVE();
  //EXPLOSION();
  //MASTER();
  
 
  //delay(100);
  //Serial.println(" ");
  //Serial.println(x, 2);
	//Serial.println(y, 2);
  
  //Serial.println(x);
  //if(x>10)x=0;
  
  
}


