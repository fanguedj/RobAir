/*
 * Reads data from infrared and ultrasound sensors, prints results on Serial.
 *
 * Format:
 * INFRA val_1 val_2 ... (4 front first, 4 rear)
 * ULTRA val_1 val_2 ... (2 front, 2 rear)
 *
 * For arduino mega2560 in Rob'Air rev2 (pots de fleurs violets)
 */
#include <NewPing.h>

#define INFRA_NB      4 // Number of infrared sensors. 2 front, 2 rear
#define INFRA_LATENCY 100 // Milliseconds between sensor read (at least 2ms !)
#define SONAR_NUM     8 // Number of ultrasonic sensors. 4 front, 4 rear
#define MAX_DISTANCE 100 // Maximum distance (in cm) to ping.
#define PING_INTERVAL 33 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).

// Pin number associated with each sensor, in the order same order as the
// "Details" section of the InfraredPotholes.msg message definition.
const int INFRA_PINS[INFRA_NB] = {A0, A1, A2};

unsigned long pingTimer[SONAR_NUM]; // Holds the times when the next ping should happen for each sensor.
unsigned int cm[SONAR_NUM];         // Where the ping distances are stored.
uint8_t currentUltrasoundSensor = 0;          // Keeps track of which sensor is active.
unsigned long infraredTimer;

NewPing sonar[SONAR_NUM] = {     // Ultrasonic Sensor object array. Each sensor's trigger pin, echo pin, and max distance to ping.
//each sensor localisation si done from the kinect Point of view
  //FRONT
  NewPing(52, 53, MAX_DISTANCE), //LEFT Sensor
  NewPing(50, 51, MAX_DISTANCE), //FRONT LEFT Sensor
  NewPing(48, 49, MAX_DISTANCE), //FRONT RIGHT Sensor
  NewPing(46, 47, MAX_DISTANCE)  //RIGH Sensor
  //REAR
  NewPing(44, 45, MAX_DISTANCE), //RIGH Sensor
  NewPing(42, 43, MAX_DISTANCE), //REAR RIGHT Sensor
  NewPing(40, 41, MAX_DISTANCE), //REAR LEFT Sensor
  NewPing(38, 39, MAX_DISTANCE)  //LEFT sensor
  /*NewPing(31, 32, MAX_DISTANCE),
  NewPing(34, 33, MAX_DISTANCE),
  NewPing(35, 36, MAX_DISTANCE),
  NewPing(37, 38, MAX_DISTANCE),
  NewPing(39, 40, MAX_DISTANCE),
  NewPing(50, 51, MAX_DISTANCE),
  NewPing(52, 53, MAX_DISTANCE)*/
};

/*
    Code sample for 139741 Arduino Infrared Obstacle Avoidance Detection Photoelectric Sensor

    Output current: 100mA
    Working voltage: DC 5V
    Current consumption: DC < 25mA
    Response time: < 2ms
    Sensing angle: Less than 15 degree
    Detecting range: 3~80 cm
    Working temperature: -25~55'C
    Wiring:
        red (DC 4.5~5V power high level)
        yellow (Pin A0 microcontroller)
        green (GND 0V power low level) 

    This code is in the public domain.

*/
// Reads a value from an infrared sensor and returns 0 for no hole, 1 for hole.
int infraredRead(int pin) {
    int sensorValue = analogRead(pin);
    // The read value should be 0 or 1023 (binary)
    if (sensorValue < 512) {
        // The ground is close, i.e. no hole
        return 0;
    } else {
        return 1;
    }
}

int ultrasoundRead(int sen) {
    return cm[sen];
}

void setup() {
    // initialize serial communications at 115200 bps:
    Serial.begin(115200); 
    infraredTimer = millis() + 75;
    pingTimer[0] = millis() + 75;           // First ping starts at 75ms, gives time for the Arduino to chill before starting.
    for (uint8_t i = 1; i < SONAR_NUM; i++) // Set the starting time for each sensor.
      pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;
}

void loop() {
    // Infrared
    if( millis() >= infraredTimer )
    {
      
      infraredTimer += INFRA_LATENCY;
      Serial.print("INFRA ");
      for (int i = 0; i < INFRA_NB; ++i) {
          Serial.print(infraredRead(INFRA_PINS[i]), DEC);
          Serial.print(" ");
      }
      Serial.println();
    }
    
    // Utlrasound
    for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop through all the sensors.
      if (millis() >= pingTimer[i]) {         // Is it this sensor's time to ping?
        pingTimer[i] += PING_INTERVAL * SONAR_NUM;  // Set next time this sensor will be pinged.
        if (i == 0 && currentUltrasoundSensor == SONAR_NUM - 1) {
            Serial.print("ULTRA ");
            for(int j = 0; j < SONAR_NUM; j++) {
                Serial.print(ultrasoundRead(j), DEC); // Sensor ping cycle complete, do something with the results.
                Serial.print(" ");
            }
            Serial.println();
        }
        sonar[currentUltrasoundSensor].timer_stop();          // Make sure previous timer is canceled before starting a new ping (insurance).
        currentUltrasoundSensor = i;                          // Sensor being accessed.
        cm[currentUltrasoundSensor] = 0;                      // Make distance zero in case there's no ping echo for this sensor.
        sonar[currentUltrasoundSensor].ping_timer(echoCheck); // Do the ping (processing continues, interrupt will call echoCheck to look for echo).
      }
    }

    // wait 2 milliseconds before the next loop
    // for the analog-to-digital converter to settle
    // after the last reading:
    delay(2);                     
}

void echoCheck() { // If ping received, set the sensor distance to array.
  if (sonar[currentUltrasoundSensor].check_timer())
    cm[currentUltrasoundSensor] = sonar[currentUltrasoundSensor].ping_result / US_ROUNDTRIP_CM;
}

void oneSensorCycle() { // Sensor ping cycle complete, do something with the results.

}
