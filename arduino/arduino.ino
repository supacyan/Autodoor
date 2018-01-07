/******************************************************************************
 * This program automates a door for keyless unlocking                         *
 * Developed by: Chauncey Yan, Sam Quinn, Ashley Greenacre, and Chris Harper.  *
 * 05/13/2014, Rebooted Jan 2018 by Chauncey                                   *
 ******************************************************************************/
#include <Wire.h>
#include <Servo.h>
#include <SoftwareSerial.h>
#include "pitches.h"

// XBee Object
SoftwareSerial XBee(2, 3); // RX, TX

// Debug Toggle
#define DEBUG true

// Lock angle definitions
#define LOCK            100
#define UNLOCK          0

// Time Definitions
#define SYS_WAIT	1000        // Short pause to allow system to catch up	
#define RUN_WAIT	400         // Time to wait before starting loop again
#define CAL_WAIT	1500        // Time to wait for the calibrator
#define DSR_WAIT	500         // Delay before locking after the door sensor is triggered
#define AFT_WAIT	1000        // Time to wait to allow doorlock to complete its task
#define ERR_WAIT	1000        // Time to wait to redo after ERROR
#define STAT_WAIT	100         // Time to wait to redo befor read the pot in lock_stat()

// Functions declarations
extern int lock_stat();
extern int lock(int lock_pos);
extern void print_info();
extern void calibrate();
extern void calibrate_unlock();
extern void calibrate_lock();
extern int door_position();
extern void range_detector();
extern void door_timeout();
extern void serial_monitor();
extern void xbee_update();
extern void melody_tone();

// Pinout
int servo_pin = 9;          // Digital pin to control the servo
int pot_pin = A5;           // analog pin used to connect the potentiometer
int trig_pin = 12;          // Ultrasonic sensor trig singal out 
int echo_pin = 11;          // ultrasonic sensor echo singal in 
int buzzer_pin = 8;         // buzzer pin
int buzzer_gnd = 10;        // buzzer GND
int door_pin = 13;          // door detector pin
int xbee_door = 6;          // XBee door pin   DIO04 xbee_pin 11    
int xbee_obj = 5;           // XBee object pin DIO07 xbee_pin 12        
int xbee_lock = 4;          // XBee lock pin   DIO09 xbee_pin 13        
int xbee_buzz = A3;         // XBee buzz pin   DIO03 xbee_pin 17
int xbee_net = A2;          // XBee net pin    DIO05 xbee_pin 15
int xbee_master = A1;       // XBee master pin DIO06 xbee_pin 16
int xbee_locc = 7;          // XBee ctrl pin   DI11 xbee_pin 07

// Variable declearation
int lock_status;            // Variable for lock status
int pot_val = -1;           // variable to read the value from the analog pin 
int pot_mid = 190;          // pot variable for calibrate bidirectional
int pot_lock = 250;         // pot value for lock
int pot_unlock = 0;         // pot value for unlock
int pot_tole = 100;         // pot variable for pot tolerance
int duration, distance;     // Ultrasonic unlock sensor
bool door_status = true;    // door detector value
int gc = 30;                // global counter for door timeout
int led_cool[2] = {255, 0}; // additional led brightness control
byte input;                 // input variable from XBee
bool obj_status = false;    // object detection
bool buzzer_toggle = true;  // buzzer switch to turn tones on and off
bool online_toggle = true;  // internet switch to turn net control on and off
bool master_toggle = true;  // master switch to turn system on and off
bool lock_toggle   = true;  // lock control toggle

// Servo Object
Servo door;

// Initial setup
void setup(){
    // Start the port for output
    XBee.begin(9600);
    
    pinMode(door_pin,   INPUT);
    pinMode(xbee_buzz,  INPUT);
    pinMode(xbee_net,   INPUT);
    pinMode(xbee_master,INPUT);
    pinMode(xbee_locc,  INPUT);
    pinMode(echo_pin,   INPUT);
    pinMode(trig_pin,   OUTPUT);
    pinMode(buzzer_pin, OUTPUT);
    pinMode(buzzer_gnd, OUTPUT);
    pinMode(xbee_door,  OUTPUT);
    pinMode(xbee_obj,   OUTPUT);
    pinMode(xbee_lock,  OUTPUT);
    digitalWrite(buzzer_gnd,    LOW);       // pseudo gnd
    digitalWrite(echo_pin,      HIGH);      // turn on pullup resistors
    digitalWrite(door_pin,      HIGH);      // turn on pullup resistors
    digitalWrite(xbee_buzz,     HIGH);      // turn on pullup resistors
    digitalWrite(xbee_net,      HIGH);      // turn on pullup resistors
    digitalWrite(xbee_master,   HIGH);      // turn on pullup resistors
    digitalWrite(xbee_locc,     HIGH);      // turn on pullup resistors
    
    calibrate();    // Calibrates the definitions of the potentiometer values
    if (pot_unlock > 475) calibrate();

    delay(SYS_WAIT);
}

// Main program loop
void loop() {
    if (master_toggle){
        if (online_toggle){
            serial_monitor();
            xbee_unlock();
        }
        range_detector();
        door_timeout();
    }
    xbee_update();
    delay(RUN_WAIT); // Run again in RUN_WAIT ms
}

void serial_monitor(){
    if (XBee.available() > 0) {
        input = XBee.read();
        if (input == '0' || input == '1' || input == '2' || input == '3'){
            if( input == '1') {
                if (lock(1) != 1) {
                    XBee.println("ERROR: Could not execute command LOCK");
                    errorTone();
                    delay(ERR_WAIT);
                    if (lock(1) != 1) {
                        XBee.println("ERROR: Could not execute command LOCK");
                        errorTone();
                    }
                }
            } else if ( input == '0') {
                if (lock(0) != 0) {
                    XBee.println("ERROR: Could not execute command UNLOCK.");
                    errorTone();
                    delay(ERR_WAIT);
                    if (lock(0) != 0) {
                        XBee.println("ERROR: Could not execute command UNLOCK twice.");
                        errorTone();
                    }
                }
            } else if ( input == '2' ){
                lock_status = lock_stat();
                print_info();
                if (lock_status == 1) {
                    XBee.println("LOCKED");
                } else if (lock_status == 0){
                    XBee.println("UNLOCKED");
                } else {
                    XBee.println("ERROR");
                }
            } else {
                calibrate();
            }
        } else if (byte(input) == 13){
            //XBee.println("s/e");
        } else {
            XBee.println("ERROR: Unreconnized command: ");
            byte out = input;
            XBee.print("(");
            XBee.print(out);
            XBee.print(")");
        }
    }
}

void xbee_unlock(){
    if (lock_toggle == true && digitalRead(xbee_locc) == 0){ // try to unlock 
        if (lock(0) != 0) {
            XBee.println("ERROR: Could not execute command UNLOCK.");
            errorTone();
            delay(ERR_WAIT);
            if (lock(0) != 0) {
                XBee.println("ERROR: Could not execute command UNLOCK twice.");
                errorTone();
            }
        }
    } else if (lock_toggle == false && digitalRead(xbee_locc) == 1){ // try to lock
        if (lock(1) != 1) {
            XBee.println("ERROR: Could not execute command LOCK");
            errorTone();
            delay(ERR_WAIT);
            if (lock(1) != 1) {
                XBee.println("ERROR: Could not execute command LOCK");
                errorTone();
            }
        }
    } 
}

// Returns the position of the door.
int door_position() {
    door_status = digitalRead(door_pin)?true:false; 
    return door_status;
}

void calibrate() {
    if (buzzer_toggle)
        melody_tone();
    if (analogRead(pot_pin) < pot_mid) {
        calibrate_unlock();
        calibrate_lock();
    } else {
        calibrate_lock();
        calibrate_unlock();
        calibrate_lock();
    }
}

void calibrate_unlock () {
    //unlock the door to read the potvalue 
    door.attach(9);
    door.write(UNLOCK);
    delay(CAL_WAIT);
    door.detach();
    // read the value of the potentiometer
    pot_unlock = analogRead(pot_pin); 
    // print out the value to the XBee monitor
    XBee.print("Defined unlock: ");
    XBee.println(pot_unlock);
}

void calibrate_lock () {
    //lock the door to read the potvalue 
    door.attach(9);
    door.write(LOCK);
    delay(CAL_WAIT);
    door.detach();
    // read the value of the potentiometer
    pot_lock = analogRead(pot_pin); 
    // print out the value to the XBee monitor
    XBee.print("Defined lock: ");
    XBee.println(pot_lock);
}

/******************************************************************************
 * Determines the current state of the door
 *
 * Tasks:
 * 1.)   Read analog data from the servos internal potentiometer
 * 2.)   Map the potentiometer data to an angle
 * 3.)   If angle is close to the defined LOCK value return 1
 * 4.)   If angle is close to the defined UNLOCK value return 0
 * 5.)   If the lock is in an indeterminate state then return -1
 *
 ******************************************************************************/
int lock_stat() {
    int rv = -1;
    
    delay(STAT_WAIT);              // prevent it from reading bad value
    pot_val = analogRead(pot_pin); // read the value of the potentiometer

    if(pot_val > 200 && pot_val < (pot_lock + pot_tole)){
        rv = 1;
    } else if(pot_val > (pot_unlock - pot_tole) && pot_val < (pot_unlock + pot_tole)) {
        rv = 0;
    } else {
        rv = -1;
    }
    
    return rv;
}

/******************************************************************************
 * Print the current state of the servo motor
 *
 * Tasks:
 * 1.)   Read analog input from the potentiometer of the servo
 * 2.)   Map the potentiometer data to angles
 * 3.)   Check current lock status
 * 4.)   Print all data out through XBee
 *
 ******************************************************************************/
void print_info() {

    pot_val = analogRead(pot_pin); // read the value of the potentiometer

    //print out the value to the XBee monitor

    XBee.print("pot_val: ");
    XBee.println(pot_val);
    XBee.print("pot_unlock: ");
    XBee.println(pot_unlock);
    XBee.print("pot_lock: ");
    XBee.println(pot_lock);
    XBee.print("lock_status: ");
    XBee.println(lock_status);
    XBee.print("door_status: "); 
    XBee.println(door_status); 
    XBee.print("obj_status: "); 
    XBee.println(obj_status); 
    XBee.print("buzzer_toggle: "); 
    XBee.println(buzzer_toggle); 
    XBee.print("online_toggle: "); 
    XBee.println(online_toggle); 
    XBee.print("master_toggle: "); 
    XBee.println(master_toggle); 
    XBee.print("lock_toggle: "); 
    XBee.println(lock_toggle); 

}

/******************************************************************************
 * Will either lock (1) or unlock (0) the door 
 * 
 * Tasks:
 * 1.)   Attach to the Servo motor
 * 2.)   Read the curent position of the lock
 * 3.)   If the door is already in its desired location do nothing
 * 4.)   If the door is not in the desired location then set the servo angle
 * 5.)   Move the servo to the desired location
 * 6.)   Detach the servo to allow manual locking and unlocking.
 *
 ******************************************************************************/
int lock(int lock_pos) {

    int l_status = lock_stat();
    int angle;
    
    if (lock_pos == 1) {
        XBee.println("----LOCKING----");
    } else if (lock_pos == 0) {
        XBee.println("----UNLOCKING----");
    } else {
        XBee.print("Unrecognized command for lock():");
        XBee.println(lock_pos);
    }
    // Read the position of the lock currently
    if (l_status == lock_pos) {
        XBee.println("ALREADY ins desired state.");
        return lock_pos;
    } else {
        //print_info();
        if (lock_pos == 1) {
            angle = LOCK;
        } else if (lock_pos == 0) {
            angle = UNLOCK;
        }
    }

    // set the servo position  
    if (angle == LOCK) {
        door.attach(9);
        door.write(LOCK);
        if (buzzer_toggle)
            locktone();
    } else {
        door.attach(9);
        door.write(UNLOCK);
        if (buzzer_toggle)
            unlocktone();
    }
    delay(AFT_WAIT);
    // Detach servo so manual override of the door can take place
    door.detach();
    return lock_stat();
}

void range_detector() {
    // Get the distance value from the ultrasonic sensor
    // issue : the first value will be really small
    digitalWrite(trig_pin, HIGH);         // transmit sound wave out
    delayMicroseconds(10);             	 // transmit last 10 uS
    digitalWrite(trig_pin, LOW);          // stop transmit
    duration = pulseIn(echo_pin, HIGH);   // read from echo pin for travel duration
    distance = (duration/2) / 29.1;      // calculate distance
    // trying to solve the issue above
    
    if ( gc == 30 )
        distance = 15;

    if (distance >= 15 || distance <= 0){
        obj_status = false;
    } else {
        XBee.println("Object detected");      // Noise
        
        // make sure there is a solid object;
        digitalWrite(trig_pin, HIGH);         // transmit sound wave out
        delayMicroseconds(10);                // transmit last 10 uS
        digitalWrite(trig_pin, LOW);          // stop transmit
        duration = pulseIn(echo_pin, HIGH);   // read from echo pin for travel duration
        distance = (duration/2) / 29.1;      // calculate distance

        if (distance >= 15 || distance <= 0){
            obj_status = false;
        } else {
            XBee.println("Solid object detected");      // unlock the door
            obj_status = true;
            if (lock_stat() == 1){ 
                if (lock(0) != 0) {
                    XBee.println("ERROR: Could not execute command UNLOCK");
                    errorTone();
                    delay(ERR_WAIT);
                    if (lock(0) != 0) {
                        XBee.println("ERROR: Could not execute command UNLOCK");
                        errorTone();
                    }
                }
            }
            delay(AFT_WAIT);
        }
    }
}

void door_timeout(){
    // Check whether the door is open or closed using the Magetic door sensor.
    // Waits till the door is closed before locking.
    if (door_position()){ // true means door is open, once its closed, only (30-10)*0.4s to lock
        gc = 20;
    }
    // check if the door is unlocked. 
    lock_status = lock_stat();
    if (lock_status != 1){
        
    // lock it after about 12 (30*0.4) seconds 
    // if no more interaction detected.
        if ( gc >= 30 ){
            lock(1);
        } else {
            gc++;
        }
    } else {
        gc = 0;
    }
}

void xbee_update(){
    digitalWrite(xbee_door, door_status);
    digitalWrite(xbee_obj, obj_status);
    digitalWrite(xbee_lock, lock_status);
    buzzer_toggle = digitalRead(xbee_buzz)?true:false;
    online_toggle = digitalRead(xbee_net)?true:false;
    master_toggle = digitalRead(xbee_master)?true:false;
    lock_toggle = digitalRead(xbee_locc)?true:false;
}

void melody_tone() {
    // notes in the melody:
    int melody[] = {
    NOTE_C4, NOTE_G3,NOTE_G3, NOTE_A3, NOTE_G3,0, NOTE_B3, NOTE_C4};

    // note durations: 4 = quarter note, 8 = eighth note, etc.:
    int noteDurations[] = { 4, 8, 8, 4,4,4,4,4 };
    
    // iterate over the notes of the melody:
    for (int thisNote = 0; thisNote < 8; thisNote++) {
    
        // to calculate the note duration, take one second 
        // divided by the note type.
        //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
        int noteDuration = 800/noteDurations[thisNote];
        tone(8, melody[thisNote],noteDuration);

        // to distinguish the notes, set a minimum time between them.
        // the note's duration + 30% seems to work well:
        int pauseBetweenNotes = noteDuration * 1.30;
        delay(pauseBetweenNotes);
        
        // stop the tone playing:
        noTone(8);
    }
}

void unlocktone(){ // keep these two functions under 1000 ms
    tone(buzzer_pin, 800);          // play 400 Hz tone for 500 ms
    delay(250);
    tone(buzzer_pin, 600);          // play 800Hz tone for 500ms
    delay(250);
    tone(buzzer_pin, 800);          // play 400 Hz tone for 500 ms
    delay(250);
    //tone(buzzer_pin, 400);          // play 800Hz tone for 500ms
    //delay(250);
    noTone(buzzer_pin);
}
void locktone(){ // keep these two funtion under 1000 ms
    tone(buzzer_pin, 600);          // play 400 Hz tone for 500 ms
    delay(250);
    tone(buzzer_pin, 800);          // play 800Hz tone for 500ms
    delay(250);
    tone(buzzer_pin, 600);          // play 400 Hz tone for 500 ms
    delay(250);
    //tone(buzzer_pin, 400);          // play 800Hz tone for 500ms
    //delay(250);
    noTone(buzzer_pin);
}
void errorTone(){ 
    tone(buzzer_pin, 1000);
    delay(100);
    tone(buzzer_pin, 600);
    delay(100);
    noTone(buzzer_pin); 
}
