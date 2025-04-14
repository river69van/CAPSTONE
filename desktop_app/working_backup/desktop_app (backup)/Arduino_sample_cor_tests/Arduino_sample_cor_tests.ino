
char flag = 'S';
int slave_num = 1;
float y_coor = 100.0;
float x_coor = 100.0;



void sample_message(char FLAG,  float Xcoor, float Ycoor, int Snum = 0){
	Serial.println(FLAG);
	Serial.println(Snum);
	Serial.println("A");
	Serial.println(Xcoor, 2);
	Serial.println(Ycoor, 2);
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}



float x = 0;
float y = 0;

void loop() {
  x +=1.0;
  y +=1.0;
  // put your main code here, to run repeatedly:
  
  if(x>512 || y >512){
    x = 0;
    y = 0;
    slave_num = 0;

  }
  
  if(x - 60 == 0){
    slave_num++;
  }else if(x - 100 == 0){
    flag = 'S';
    slave_num++;
  }else if(x - 150 == 0){
    slave_num++;
  }else if(x - 200 == 0){
	flag = 'E';
    slave_num = 0;
  }else if(x - 250 == 0){
    flag = 'M';
    slave_num = 0;
  }
  

  //Serial.println(x, 2);
	//Serial.println(y, 2);
  sample_message(flag, x, y, slave_num);
  //Serial.println(x);
  //if(x>10)x=0;
  
  delay(50);
}
