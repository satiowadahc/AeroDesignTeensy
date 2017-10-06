/*
 * To increase speed of control system, increase constant Ki
 * To increase the speed decrease from power limit, increse powerConst
 */

//connection and timing parameters
int currentSensorPin = A0;      //voltage from current sensor. Must be scaled. Should be linear-ish
int voltageSensorPin = A1;      //battery voltage sensor
//int pulseInPin = 2;             //servo signal from receiver
//int pulseOutPin = 4;            //output servo pulse to the ESC
//int outputPulseFrequency = 62;  //frequency of output servo pulse should be 62.3286 Hz
//int outputPulsePeriod = 16129;  //corresponds to frequency above  should be 16044 ms
//int current_to_power = 1200;    //scaling factor to convert voltage from current sensor to power
//int pwmFlag = 0;                //if output is high or low
//int confirmationPin = 7;        //pin to check connection to receiver
//unsigned long connectionCount = 0;  //to check connection to receiver

double power;     //power from battery = batteryVoltage*batteryCurrent
double batteryVoltage;  //read from ADC
double batteryCurrent;  //caculate from current sensor reading
double currentSensor;   
volatile unsigned long startTime;   //measure high time of input pwm from receiver
volatile unsigned long currentTime;
volatile unsigned long inputHighTime;

//const double Plim=900; //power limit. Note the system allows going over this number by about 5%

//control system parameters
//double Pconst;
//int err;
//double integral;
//double Ki;
//int outputHighTime;
#define period 16044 //microseconds
double powerConst;

int highMAX = 1622; //Maximum positive servo pulse width (micro seconds), i.e. full speed
int highMIN = 1008;  //Minimum servo pulse width (micro seconds), i.e. stopped

//IntervalTimer timerFrequency; //timer interrupt to run the control system and 
                              //calculate the output servo pulse to the ESC 
//IntervalTimer generatePWM;   //generate PWM output signal to ESC                         

void setup() {
  Serial.begin(9600);
  
  cli();  //stop inturrupts
  // put your setup code here, to run once:
    pinMode(13,OUTPUT);
    digitalWrite(13,HIGH);
//   pinMode(pulseInPin,INPUT);  //input from receiver
//   pinMode(pulseOutPin,OUTPUT);  //output to ESC
    analogReadResolution(16); //analog input 16 bits resolution, values 0 to 65535
    
//   attachInterrupt(pulseInPin, readPWM, CHANGE);   //interrupt on both edges from receiver input
    
    //initializations for measuring input signal from receiver
    startTime = 0;
    currentTime = 0;

    //control system initializations
    integral = 0;
    err = 0;
    outputHighTime = 0;
    inputHighTime = 0;
    
    //determines speed of control system
  //  powerConst = 0.5;
  //  Ki = 0.1;

    //timer interrupts
//    timerFrequency.begin(timer_ISR, period);  //interrupt to update output pwm signal
//    generatePWM.priority(10); //higher priority. 0 to 255. default 128. 0 highest
//    generatePWM.begin(pwm_ISR, period); //interrupt to generate a pwm signal
//    sei();  //start interrupts
      
}

//main loop just reads analog and voltagge signal from sensor
void loop() {
  //currentSensor = analogRead(currentSensorPin);
  //batteryCurrent = currentSensor*3.3/65535*133.33-333.33; 
  batteryCurrent = analogRead(currentSensorPin);
  batteryVoltage = analogRead(voltageSensorPin);
  batteryCurrent = batteryCurrent * 90 / 65535; //ADC 0 to 65535 corresponds to 0 to 90 amps
  batteryVoltage = batteryVoltage * 50 / 65535; //0 to 50 volts
  power = batteryCurrent*batteryVoltage;
  connectionCount = connectionCount + 1;  //incriment every 100ms
  
  Serial.print(power);
  Serial.print("\t");
  Serial.print(inputHighTime);
  Serial.print("\t");
  Serial.print(outputHighTime);
  Serial.print("\t");
  Serial.print(err);
  Serial.print("\t");
  Serial.println(integral);
  
  delay(100);
}

/*
 * Timer ISR to generate output PWM signal to the ESC
 *
void pwm_ISR(void){
  if (connectionCount>2){ //0.3 second delay before stopping motor
    digitalWrite(pulseOutPin,LOW);
    pwmFlag = 0;
  }else if (pwmFlag == 0){
    digitalWrite(pulseOutPin,HIGH);
    pwmFlag = 1;
    generatePWM.begin(pwm_ISR, outputHighTime);
  }else{
    digitalWrite(pulseOutPin,LOW);
    pwmFlag = 0;
    generatePWM.begin(pwm_ISR, period-outputHighTime);
  }
  
}*/

/*  ISR, triggered on both edges of the servo pulse from the receiver
 *  the current time is taken on a positive edge
 *  the end time is taken on the negative edge, and the positive time is calculated
 *  the frequency of pulses is known beforehand and is constant. It is from
 *    the receiver, which would normally go to the ESC
 *
void readPWM() {
    if (digitalRead(pulseInPin) == HIGH){
      startTime = micros();
    }else {
      currentTime = micros();
      inputHighTime = (currentTime - startTime);
    }
  connectionCount = 0;  //reset to confirm connection
}*/

/* Timer ISR to run control system to calculate the 
 *    servo pulse high time
 *
void timer_ISR(void)
{
  err = (inputHighTime - outputHighTime);
  
  if(power>Plim){
    Pconst = powerConst*(Plim-power)-err;
  }else{
    Pconst = 0;
  }
  err = err+Pconst;
  
  integral = integral + (err*Ki);
  

  outputHighTime = (int)integral;

  //apply boundary to output high time
  if (outputHighTime > highMAX){
    outputHighTime = highMAX;
    integral = outputHighTime;    //reset integral
  }else if (outputHighTime < highMIN){
    outputHighTime = highMIN;
    integral = outputHighTime;  //reset integral
  }

}*/

