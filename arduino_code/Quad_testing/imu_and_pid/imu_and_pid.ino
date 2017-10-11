#include <Wire.h>

//TODO(): accelerometer offset correction (may be soldered a bit crooced, put it on level surface and mesure offset, than edit the 2 numbers)

long gyro_x_cal, gyro_y_cal, gyro_z_cal;
long loop_timer;

float angle_pitch, angle_roll;
float angle_pitch_output, angle_roll_output;

boolean first_loop;

int reciever_pitch;
int reciever_roll;

float gyro_roll_input;
float gyro_roll;

int pid_max_roll = 255;
int pid_i_gain_roll = 1;
int pid_d_gain_roll = 1;
int pid_p_gain_roll = 1;

float pid_last_roll_d_error = 0;

float pid_i_mem_roll = 0;

void setup() {
  // put your setup code here, to run once:

  Serial.begin(2000000);

  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  angle_pitch = 0;
  angle_roll = 0;
  first_loop = true;
  angle_pitch_output = 0;
  angle_roll_output = 0;

  reciever_pitch = 0;  //MIN=0 MAX=255
  reciever_roll = 0;

  gyro_roll_input = 0;
  gyro_roll = 0;

  setupMPU(); //just so it usees 500°/s and 8g

  gyroCalibration(); //it will just set global gyro_x_cal values (for the 3 axis) (values are in °/s*65.5)

  digitalWrite(13, LOW);

  loop_timer = micros(); //loop starts from now on
}

void loop() {
  // put your main code here, to run repeatedly:

  
  calculateQuadAngles(); //just setes global angle_pitch and angle_roll values to current pitch and roll of the quadcotper (vlues are in °)

  //To dampen the pitch and roll angles a complementary filter is used
  angle_pitch_output = angle_pitch * 15; //0-35 * 15 == 0-525
  angle_roll_output = angle_roll * 15;     

  float pid_pitch_setpoint = 0;
  pid_pitch_setpoint -= angle_pitch_output;
  pid_pitch_setpoint /= 3.0;

  gyro_roll_input = (gyro_roll_input * 0.7) + ((gyro_roll / 65.5) * 0.3);

  calculatePID(pid_pitch_setpoint);


 
  if(angle_pitch_output>=20){
    digitalWrite(13,HIGH);
  }else{
    digitalWrite(13,LOW);  
  }

  //Serial.println(String(angle_pitch_output)+"   "+String(angle_roll_output));

  if(micros() - loop_timer >= 4000){
    Serial.println("Lag");
  }
  while(micros() - loop_timer < 4000);                                 //Wait until the loop_timer reaches 4000us (250Hz) before starting the next loop
  loop_timer = micros();                                               //Reset the loop timer

}




void setupMPU(){
  //Activate the MPU-6050
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x6B);                                                    //Send the requested starting register
  Wire.write(0x00);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
  //Configure the accelerometer (+/-8g)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1C);                                                    //Send the requested starting register
  Wire.write(0x10);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
  //Configure the gyro (500dps full scale)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1B);                                                    //Send the requested starting register
  Wire.write(0x08);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
}

void readMPUdata(int raw_gyro_data[], long raw_acc_data[]){

  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x3B);                                                    //Send the requested starting register
  Wire.endTransmission();                                              //End the transmission
  Wire.requestFrom(0x68,14);                                           //Request 14 bytes from the MPU-6050
  while(Wire.available() < 14);                                        //Wait until all the bytes are received
  raw_acc_data[0] = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_x variable
  raw_acc_data[1] = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_y variable  
  raw_acc_data[2] = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_z variable
  float temperature = Wire.read()<<8|Wire.read();
  raw_gyro_data[0] = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_x variable
  raw_gyro_data[1] = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_y variable
  raw_gyro_data[2] = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_z variable
  
  return;
}

void gyroCalibration(){
  int raw_gyro_data[] = {1,1,1,};
  long raw_acc_data[] = {1,1,1,};
    
  for (int cal_int = 0; cal_int < 2000 ; cal_int ++){                  //Run this code 2000 times
    readMPUdata(raw_gyro_data, raw_acc_data);                                             //Read the raw acc and gyro data from the MPU-6050
    gyro_x_cal += raw_gyro_data[0];                                              //Add the gyro x-axis offset to the gyro_x_cal variable
    gyro_y_cal += raw_gyro_data[1];                                              //Add the gyro y-axis offset to the gyro_y_cal variable
    gyro_z_cal += raw_gyro_data[2];                                              //Add the gyro z-axis offset to the gyro_z_cal variable
    delay(3);                                                          //Delay 3us to simulate the 250Hz program loop
  }
  gyro_x_cal /= 2000;                                                  //Divide the gyro_x_cal variable by 2000 to get the avarage offset
  gyro_y_cal /= 2000;                                                  //Divide the gyro_y_cal variable by 2000 to get the avarage offset
  gyro_z_cal /= 2000;                                                  //Divide the gyro_z_cal variable by 2000 to get the avarage offset


}

void calculateQuadAngles(){
  int raw_gyro_data[] = {1,1,1};
  long raw_acc_data[] = {1,1,1};
  readMPUdata(raw_gyro_data, raw_acc_data);  

  //za PID odstet
  gyro_roll = raw_gyro_data[0];

  raw_gyro_data[0] -= gyro_x_cal;
  raw_gyro_data[1] -= gyro_y_cal;
  raw_gyro_data[2] -= gyro_z_cal;

  //Serial.println(String(raw_gyro_data[0]) +"   "+String(raw_gyro_data[1])+"   "+String(raw_gyro_data[2]));

  //Gyro angle calculations
  //0.0000611 = 1 / (250Hz / 65.5)
  angle_pitch += raw_gyro_data[0] * 0.0000611;                                   //Calculate the traveled pitch angle and add this to the angle_pitch variable
  angle_roll += raw_gyro_data[1] * 0.0000611;                                    //Calculate the traveled roll angle and add this to the angle_roll variable

  

  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
  angle_pitch += angle_roll * sin(raw_gyro_data[2] * 0.000001066);               //If the IMU has yawed transfer the roll angle to the pitch angel
  angle_roll -= angle_pitch * sin(raw_gyro_data[2] * 0.000001066);               //If the IMU has yawed transfer the pitch angle to the roll angel

  //Accelerometer angle calculations
  float acc_total_vector = sqrt( (raw_acc_data[0]*raw_acc_data[0]) + (raw_acc_data[1]*raw_acc_data[1]) + (raw_acc_data[2]*raw_acc_data[2]));  //Calculate the total accelerometer vector
    
  //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
  float angle_pitch_acc = asin(raw_acc_data[1]/acc_total_vector)* 57.296;       //Calculate the pitch angle
  float angle_roll_acc = asin(raw_acc_data[0]/acc_total_vector)* -57.296;       //Calculate the roll angle

  //Serial.println(String(angle_pitch_acc)+"   "+String(angle_roll_acc));

  //Place the MPU-6050 spirit level and note the values in the following two lines for calibration
  angle_pitch_acc -= 0.0;                                              //Accelerometer calibration value for pitch
  angle_roll_acc -= 0.0;                                               //Accelerometer calibration value for roll

  if(!first_loop){                                                 //If the IMU is already started
    angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
    angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;        //Correct the drift of the gyro roll angle with the accelerometer roll angle
  }
  else{                                                                //At first start
    angle_pitch = angle_pitch_acc;                                     //Set the gyro pitch angle equal to the accelerometer pitch angle 
    angle_roll = angle_roll_acc;                                       //Set the gyro roll angle equal to the accelerometer roll angle 
    first_loop = false;                                            //Set the IMU started flag
  }

  Serial.println(String(angle_pitch) +"   "+String(angle_roll));
  
}


void calculatePID(float pid_pitch_setpoint){

  //Roll calculations
  float pid_error_temp = gyro_roll_input - pid_pitch_setpoint; //error between mpu and reciever (0<error<1500?)    in  ° and °
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;   // I -controler (error*I-gain)

  if(pid_i_mem_roll > pid_max_roll)pid_i_mem_roll = pid_max_roll; //to prevent it going out of conrol we need to limit it
  else if(pid_i_mem_roll < pid_max_roll * -1)pid_i_mem_roll = pid_max_roll * -1; //MAX == 400

  //complete PID is calculated (error*P_gain)+(I_controler)+((error-last_error)*D_gain)
  float pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  // P and D may still cause some extreme outputs so they are limited to MAX (400ms pusle)
  if(pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;
  else if(pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1;

  pid_last_roll_d_error = pid_error_temp;// just for the D_controller
  
}
