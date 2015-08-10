/*
ASCII Serial Commands

FORWARD = 'f', [0-255][0-255] (speed for each wheel) 
BACKWARD = 'b', [0-255][0-255] (speed for each wheel)
LEFT = 'l', [0-255][0-255] (speed for each wheel)
RIGHT = 'r', [0-255][0-255] (speed for each wheel)
LEFTTIMED = 'z'. [0-255][0-255] (speed for each wheel) [0-255][0-255] DWORD milliseconds
RIGHTTIMED = 'e'. [0-255] (speed) [0-255][0-255] DWORD milliseconds
CAM = 'v', [0-255] (servo angle)  
CAM RELEASE = 'w' (servo release)
CAM HORIZ SET = 'm' [0-255] (write horizontal position to eeprom)
STOP = 's' (DC motors stop)
HARD_STOP = 'h' (DC motors stop with brake)
GET VERSION = 'y'
GET FIRMWARE = 'x'
FWD FLOOD LIGHT = 'q', [0-255] (intensity)
REAR FLOOD LIGHT = 'o', [0-255] (intensity)
SPOT LIGHT = 'p', [0-255] (intensity)
ODOMETRY_START = 'i' (start recording encoder and gyro, zero values)
ODOMETRY_STOP = 'j' (stop recording encoder and gyro, and report)
ODOMETRY_REPORT = 'k' (report current encoder and gyro counts, then zero counts)
PING = 'c' (heartbeat)
EEPROM_CLEAR = '8' (set entire eeprom bank to 0)
EEPROM_READ = '9' (read camhoriz position) 
*/

/*
 * gyro function
 * 
 * config:
 * read z only, 500Mz rate, 250 degrees-per-second resolution
 * write to 256 value FIFO buffer, overwrite continuously, trigger interrupt at 126 full
 * 
 * when motors stopped, calibrate zero every 2 seconds
 * 
 * angle reading is cummulative, as gyro produces rate data (degrees per second)
 * if active (not stopped, and readangle boolean true), 
 * check if fifo threshold reached every loop, if so read up to threshold and add to angle
 * 
 * if info requested or stop detected, pring angle to serial, start from 0 again
 * 
 * 
 */
 
#include <I2C.h> 
#include <Servo.h> 
#include <EEPROM.h>

#define GYRO 0x58
#define WHIGH 0 // 0 = default, reverse 0/1 if wheels wired backwards
#define WLOW 1  // 1 = default, reverse 0/1 if wheels wired backwards

// hbridge 
const int pwmA = 3; // pwm
const int in1 = 2;
const int in2 = 4;
const int pwmB = 11; // pwm
const int in3 = 7;
const int in4 = 8;

// lights 
const int spotPin = 6; // pwm
const int floodPWMPin = 5; // pwm
const int fwdFloodPin = 12;
const int rearFloodPin = 9;

// servo
const int servoPin = 10; // pwm
Servo camservo;  
const int eepromAddress = 0;

// encoder
const int encA = A0;

// spare 13, A2, A3, A1


// timers 
unsigned long time = 0;
unsigned long lastcmd = 0;
const unsigned long hostTimeout = 10000; // stop motors if no steady ping from host
unsigned long stoptime = 0;

// command byte buffer 
const int MAX_BUFFER = 32;
int buffer[MAX_BUFFER];
int commandSize = 0;

//pins 5,6 timer 0 factor: 62500/1 >> 64
// 62500/8 >> 8
// 62500/256 >> 0.25
const double timemult = 64;

// gyro
boolean readAngle = false;
double angle = 0;
// const double calibrationComp = 1.047; // 1.047; // 1.094;
const unsigned long gyroZeroInterval = 2000;
unsigned long lastGyroZero = 0;
int zOff = 0;
const unsigned long gyroSampleRate = 2; // 500hz
const int gyroThreshold = 126;
int gyroZ[gyroThreshold/2];
// const unsigned long gyroReadFifoInterval = gyroThreshold/2*gyroSampleRate;
// unsigned long lastGyroFifoRead = 0;
// end of gyro

// encoder
volatile boolean encoderPinAtZero = false;
volatile int encoderTicks = 0;
const int gearRatio = 180; 
volatile boolean readEncoder = false;
volatile unsigned long lastEncoderTick = 0;
volatile boolean stopdetected = false;
// end of encoder

// stop detect control
int directioncmd = 0; // 0=stop, 1=forward, 2=backward, 3=left, 4=right
boolean stopped = true;
boolean stopPending = false;
unsigned long stopCommand = 0;
const unsigned long allowforstop = 1000;


void setup() { 

	// horiz servo
	int m = (int) EEPROM.read(eepromAddress);
	if (m < 30 || m > 100) { // not within range where it looks like its been set properly before 		
		m = 70;  //default
		EEPROM.write(eepromAddress, m);
	} 
	camservo.attach(servoPin);  
	camservo.write(m);
	delay(500);
	camservo.detach();
  
	pinMode(pwmA, OUTPUT);
	pinMode(pwmB, OUTPUT);
	pinMode(in1, OUTPUT); 
	pinMode(in2, OUTPUT); 
	pinMode(in3, OUTPUT); 
	pinMode(in4, OUTPUT); 

	pinMode(rearFloodPin, OUTPUT);  
	pinMode(spotPin, OUTPUT);  
	pinMode(fwdFloodPin, OUTPUT); 
	
	pinMode(encA, INPUT); 

	//pwm frequencies setup (from http://playground.arduino.cc/Code/PwmFrequency)
	TCCR2B = TCCR2B & 0b11111000 | 0x07;  // 30 Hz pin 3, 11 wheels
	TCCR0B = TCCR0B & 0b11111000 | 0x01; // pins 5, 6 62500/1 = 62kHz lights
	
	// motors fwd, off
	analogWrite(pwmA, 0); 
	analogWrite(pwmB, 0);
	digitalWrite(in1, HIGH);
	digitalWrite(in2, LOW);
	digitalWrite(in3, HIGH);
	digitalWrite(in4, LOW);
	
	// gyro seup
	I2c.begin();  
	I2c.setSpeed(1); // fast
	I2c.write(GYRO, 0x00, 204); // POWER_CFG  11 001 1 0 0  250dps, Pwr Normal, Zon
	I2c.write(GYRO, 0x01, 32);  // SENSE_CFG1 00 1000 0 0 50Hz LPF 
	I2c.write(GYRO, 0x02, 19);  // SENSE_CFG2   500Hz sample rate
	I2c.write(GYRO, 0x13, 32); // DR_CFG   00 10 00 0 0   STATUS-DATA_READY , TEMP_OFF
	// I2c.write(GYRO, 0x13, 0); // DR_CFG   00 00 00 0 0   ALL-DATA_READY , TEMP_OFF
	I2c.write(GYRO, 0x18, 84); // FIFO_CFG 01 0 1 0100    Normal_mode, overruntrue, store Z
	// I2c.write(GYRO, 0x18, 68); // FIFO_CFG 01 0 0 0100    Normal_mode, overrunfalse, store Z
	int n= gyroThreshold/2-1;
	I2c.write(GYRO, 0x17, n);	// FIFO_TH   
	
	// encoder read interrupt setup
	cli();		// switch interrupts off while messing with their settings  
	PCICR =0x02;          // Enable PCINT1 interrupt
	PCMSK1 = 0b00000001; // mask A0
	sei();   // switch back on
	
	Serial.begin(115200);
	Serial.println("<reset>"); 

}


void loop(){

	time = millis() / timemult;

	if (readAngle && !stopped) { // && time - gyroReadFifoInterval > lastGyroFifoRead) { 
		if (checkGyroFIFOFull()) {
			// lastGyroFifoRead =  time;
			getGyroFIFOcontents(); // this causes noticeable lag in java accell especially if spd fast
			boolean checkstop = false;
			if (stopPending && (directioncmd ==3 || directioncmd==4) && time- lastcmd > 80) checkstop = true;
			boolean callstopdetect = false;

			for (int i=0; i<gyroThreshold/2; i++ ){
				int z = gyroZ[i]+zOff;
				double degPerSec = (double) z * 250/0x7fff * -1; // negate because typically mounted upside down
				angle += degPerSec * 2/1000;
				if (checkstop) {
					if (abs(z)<150) {
					callstopdetect = true;
					}
				}
			}

			if (callstopdetect)  stopDetect();
		}
	}
	
	// periodically determine gyro zero offset when stopped
	if (time - gyroZeroInterval > lastGyroZero && time >= gyroZeroInterval) {
		if (stopped) {
			getZ(); // clear old data... required???
			zOff = 0 - getZ();
		}
		lastGyroZero = time;
	}
	
	// allow for slow down to complete stop, timed
	if (stopPending && time - allowforstop > stopCommand && readAngle && readEncoder) {
		Serial.println("<stopdetectfail>"); // TODO: testing only 
		stopDetect();
	}
	
	if (stopdetected) { // used by interrupt only 
		stopdetected = false;
		if (readEncoder && stopPending && (directioncmd==1 || directioncmd==2) ) {
			stopDetect();
		}
	}
	
	if (stoptime != 0 && time > stoptime) {
		stop();
		stoptime = 0;
	}
	
	// manage serial input
	if( Serial.available() > 0) {
		lastcmd = time;
		manageCommand();
	}
	else if (time - hostTimeout > lastcmd ){ 
		 //if no comm with host, stop motors
		 allOff();
		 lastcmd = time; 
	}

}

// 
// buffer and/or execute commands from host controller 
//
void manageCommand() {
	int input = Serial.read();
	if((input == 13) || (input == 10)){
		if(commandSize > 0){
			  parseCommand();
			  commandSize = 0; 
		}
	} 
	else {
		buffer[commandSize++] = input;
	}
}

void allOff() {
	analogWrite(floodPWMPin, 0);
	analogWrite(spotPin, 0);
	analogWrite(pwmA, 0); 
	analogWrite(pwmB, 0);
	camservo.detach();
}

void parseCommand(){
  
	if (buffer[0] == 'q'){ // forward floodlight
		analogWrite(floodPWMPin, buffer[1]);
		digitalWrite(fwdFloodPin, LOW);
		digitalWrite(rearFloodPin, HIGH);
	}
  
	else if (buffer[0] == 'o'){ // floodlight
		analogWrite(floodPWMPin, buffer[1]);
		digitalWrite(fwdFloodPin, HIGH);
		digitalWrite(rearFloodPin, LOW);
	}
  
	else if (buffer[0] == 'p'){ // spotlight
		analogWrite(spotPin, buffer[1]);
	}

	else if(buffer[0] == 'v') { // camera to position
		camservo.attach(servoPin);  
		camservo.write(buffer[1]); 
	}

	else if(buffer[0]== 'w') camservo.detach();
	
	else if (buffer[0] == 'm') {  // camera horiz position to eeprom
		EEPROM.write(eepromAddress, buffer[1]);
	}

	// always set speed on each move command 
	else if(buffer[0] == 'f' || buffer[0] == 'b' || buffer[0] == 'l' || buffer[0] == 'r'
			|| buffer[0] == 'z' || buffer[0] == 'e') {
		analogWrite(pwmA, buffer[1]); 
		analogWrite(pwmB, buffer[2]);

		if (buffer[0] == 'f') { // forward
			digitalWrite(in1, WLOW);
			digitalWrite(in2, WHIGH);
			digitalWrite(in3, WHIGH);
			digitalWrite(in4, WLOW);
			
			stopPending = false;
			stopped = false;
			directioncmd =1;
		}

		else if (buffer[0] == 'b') { // backward
			digitalWrite(in1, WHIGH);
			digitalWrite(in2, WLOW);
			digitalWrite(in3, WLOW);
			digitalWrite(in4, WHIGH);
			
			stopPending = false;
			stopped = false;
			directioncmd = 2;
		}

		else if (buffer[0] == 'l') { // left
			digitalWrite(in1, WHIGH);
			digitalWrite(in2, WLOW);
			digitalWrite(in3, WHIGH);
			digitalWrite(in4, WLOW);
			
			stopPending = false;
			stopped = false;
			directioncmd=3;
		} 
		
		else if (buffer[0] == 'r') { // right
			digitalWrite(in1, WLOW);
			digitalWrite(in2, WHIGH);
			digitalWrite(in3, WLOW);
			digitalWrite(in4, WHIGH);
			
			stopPending = false;
			stopped = false;
			directioncmd=4;
		}
		
		else if (buffer[0] == 'z') { // lefttimed
			digitalWrite(in1, WHIGH);
			digitalWrite(in2, WLOW);
			digitalWrite(in3, WHIGH);
			digitalWrite(in4, WLOW);
			
			stopPending = false;
			stopped = false;
			directioncmd=3;
			// unsigned long delay = buffer[3]<<8 | buffer[4];
			// if (delay > 1000) { delay = 1000; }
			// stoptime = time + delay;
			stoptime = time + (buffer[3]<<8 | buffer[4]);
		}
		
		else if (buffer[0] == 'e') { // righttimed
			digitalWrite(in1, WLOW);
			digitalWrite(in2, WHIGH);
			digitalWrite(in3, WLOW);
			digitalWrite(in4, WHIGH);
			
			stopPending = false;
			stopped = false;
			directioncmd=4;
			// unsigned long delay = buffer[3]<<8 | buffer[4];
			// if (delay > 1000) { delay = 1000; }
			// stoptime = time + delay;
			stoptime = time + (buffer[3]<<8 | buffer[4]);
		}
		
	}

	else if (buffer[0] == 's') { // stop
		stop();
	}
	
	else if (buffer[0] == 'h') { // hard stop
		digitalWrite(in1, 0);
		digitalWrite(in2, 0);
		digitalWrite(in3, 0);
		digitalWrite(in4, 0);
		if (!stopped) {
			stopCommand = time;
			stopPending = true;	
		}
	}

	else if(buffer[0] == 'x') Serial.println("<id::malg>");

	else if(buffer[0] == 'y') version();
	
	else if (buffer[0] == 'c') Serial.println(""); // ping, single character to make it quick
  
	else if (buffer[0] == 'i') { // gyro and encoder start
		encoderPinAtZero = false;
		encoderTicks = 0;
		readEncoder = true;	
		
		readAngle = true;
		angle = 0;
		stopdetected = false;
		stopPending = false;
		stopped = true;
		directioncmd = 0;
	}
	else if (buffer[0] == 'j') { // gyro and encoder stop and report
		printMoved();
		readEncoder = false;
		encoderPinAtZero = false;
		readAngle = false;
	}
	else if (buffer[0] == 'k') { // ODOMETRY_REPORT 
		printMoved();	
	}
	
	else if (buffer[0] == '8') { // clear eeprom
		for (int i = 0; i < 512; i++)
			EEPROM.write(i, 0);
		Serial.println("<eeprom erased>");
	}
	else if (buffer[0] == '9') { // read eeprom
		int i = EEPROM.read(eepromAddress);
		Serial.print("<horiz_eeprom: ");
		Serial.print(i);
		Serial.println(">");
	}
	
/* end of command buffer[0] list */	

	
}

int getZ() { // requires DR_CFG   00 10 00 0 0   STATUS - DATA_READY
	
	byte zreg[2];
	
	while (true) {
		I2c.read(GYRO, 0x22, 1);  // SYSTEM_STATUS
		int status = I2c.receive();
		if (status !=0) { // new data available
			I2c.read(GYRO, 0x27, 2);
			for (int i=0; i<2; i++) {  zreg[i] = I2c.receive();  }
			break;
		}
	}

	int z = zreg[0] << 8 | zreg[1];
	
	return z;
} 

boolean checkGyroFIFOFull() {
	I2c.read(GYRO, 0x3D, 1); // FIFO_STATUS
	byte status = I2c.receive();
	// if (bitRead(status, 1)==1) Serial.print("<fifo full>"); // TODO: testing
	if (bitRead(status, 2)==1) return true; // above threshold
	else return false;
}

void getGyroFIFOcontents() {
	byte zb[gyroThreshold]; // samples are 16bit
	for (int i=0; i<gyroThreshold; i++) {
		I2c.read(GYRO, 0x3E, 1); // FIFO_DATA
		zb[i] = I2c.receive();  
	}

	int n=0;
	for (int i=0; i<gyroThreshold; i+=2) {
		gyroZ[n] = zb[i]<<8 | zb[i+1];	
		n++;
	}
}

void printMoved() {
	double revs = 0;
	if (directioncmd ==1 || directioncmd == 2) {
		revs = (double) encoderTicks/gearRatio;
		if (directioncmd==2) revs *= -1;
	}
	encoderTicks = 0;
	double a = angle; // * calibrationComp;
	angle = 0;
	Serial.print("<moved ");
	Serial.print(revs);
	Serial.print(" ");
	Serial.print(a);
	Serial.println(">");
}


void stop() {
	analogWrite(pwmA, 0); 
	analogWrite(pwmB, 0);
	
	if (!stopped) {
		stopCommand = time;
		stopPending = true;	
	}
}

void stopDetect() {
	printMoved(); 
	Serial.println("<stop>");
	stopPending = false;
	stopped = true;
	directioncmd = 0;
	lastGyroZero = time; // in case still moving a bit...
}

ISR(PCINT1_vect) {
	if (!readEncoder) return; 
	
	int m = digitalRead(encA);
	if (m == LOW) encoderPinAtZero = true; 
	else {
		if (encoderPinAtZero) { // tick
			encoderPinAtZero = false;
			encoderTicks ++;
			unsigned long t = millis()/timemult;
			if (t - lastEncoderTick > 25) { stopdetected = true; }
			lastEncoderTick = t;
		}
	}
}

void version() {
	Serial.println("<version:0.127>"); 
}

