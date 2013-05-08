/*
 * Reads data from infrared and ultrasound sensors, prints results on Serial.
 *
 * Format:
 * INFRA val_1 val_2 ...
 * ULTRA val_1 val_2 ...
 */

// Number of infrared sensors.
const int INFRA_NB = 3;
// Pin number associated with each sensor, in the order same order as the
// "Details" section of the InfraredPotholes.msg message definition.
const int INFRA_PINS[INFRA_NB] = {A0, A1, A2};

// Number of ultrasound sensors.
const int ULTRA_NB = 0;
// Pin number associated with each sensor, in the order same order as the
// "Details" section of the InfraredPotholes.msg message definition.
// TODO const int ULTRA_PINS[ULTRA_NB] = {A0, A1, A2};


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
    if (sensorValue >= 512) {
        // The ground is close, i.e. no hole
        return 0;
    } else {
        return 1;
    }
}

// TODO
int ultrasoundRead(int pin) {
    return 0;
}

void setup() {
    // initialize serial communications at 9600 bps:
    Serial.begin(9600); 
}

void loop() {
    // Infrared
    Serial.print("INFRA ");
    for (int i = 0; i < INFRA_NB; ++i) {
        Serial.print(infraredRead(INFRA_PINS[i]), DEC);
        Serial.print(" ");
    }
    Serial.println();
    
    // Utlrasound
    Serial.print("ULTRA ");
    for (int i = 0; i < ULTRA_NB; ++i) {
        Serial.print(ultrasoundRead(ULTRA_PINS[i]), DEC);
        Serial.print(" ");
    }
    Serial.println();

    // wait 2 milliseconds before the next loop
    // for the analog-to-digital converter to settle
    // after the last reading:
    delay(2);                     
}
