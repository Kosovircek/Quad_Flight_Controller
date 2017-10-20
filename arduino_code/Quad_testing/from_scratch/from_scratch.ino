

//Still not got the ACC offset

//Still dont understand how pid_inputs (in °/s i guess) gets transformd to pid_output (0-255)

float loop_timer;

int gyro_raw_data[] = {0,0,0}; //raw 8bit values you get from MPU6050 register() VALUES_[°/s *65,5] TODO():add MAX MIN
long acc_raw_data[] = {0,0,0}; //raw 16bit values ou get from MPU6050 register() MAX_MIN_[+-8g] TODO():+-8 is no correct just max


//pid gain values for PITCH and ROLL
float p_gain = 0;
float i_gain = 0;
float d_gain = 0;
//pid gain values YAW
float p_gain_yaw = 0;
float i_gain_yaw = 0;
float d_gain_yaw = 0;
// I-conroller need so save total I-controler values through time so we need a global values
float pid_i_mem_pitch = 0; 
float pid_i_mem_roll = 0;
float pid_i_mem_yaw = 0;
// D-controler needs to save last pid_error so we need global values
float pid_error_last_pitch = 0;
float pid_error_last_roll = 0;
float pid_error_last_yaw = 0;





void setup() {
  // put your setup code here, to run once:


  //Setup Timers (used to generate PWM signal to drive the DC motors)
  //Timer 1 is reserved for milis() function of Arduino
  
  //Timer 2 seup
  //#PWM pins of Timer 2 must be set to OUTPUT for Timer to that is OC2A(PIN10), OC2B(PIN9)
 
  DDRH |= (_BV(PORTH5) | _BV(PORTH4));
  TCCR4A = 10100011;
  //Prescaler Setup (Fastest Mode)
  TCCR4B = 00000001;
  OCR4C = 32767/5;
  OCR4B = 32767/10;

  DDRE |= ( _BV(PORTE5) | _BV(PORTE4) );
  TCCR3A = 10100011;
  //Prescaler Setup (Fastest Mode)
  TCCR3B = 00000001;
  OCR3C = 0;
  OCR3B = 32767;



  //Setup MPU
  Wire.begin();
  Wire.write(0x02);
  Wire.transfer();

  Wire.

  
  

}

void loop() {
  // put your main code here, to run repeatedly:

}

void setupMPU(){
    
}




