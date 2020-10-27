#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ADS1015.h>

/* ToDo */

// ToDo: Add switch between 5V Power Supply GND and GND for USB debugging
// ToDo: Add ADC
// ToDo: Increase ADC accuracy
// ToDo: Add display and [voltage, mode, settings] input

/* ToDo */

/* ADS1115 15 bit analog digital converter */
Adafruit_ADS1115 ads(0x4a); // A4 - SDA; A5 - SCL; 0x4a: ADDR - SDA
double adsC = 0.000125f;

/* Debug */
bool debugMode = false;
bool advancedDebugMode = false;

/* Charger Settings */
const double current = 4.5;
//const double current = 9;

/* Battery Settings */
const int series = 18;
// const int series = 26;
const int parallel = 1;
// const int parallel = 8;

/* Cell Specifications */
const double voltageNominal = 3.7; // V
const double voltageMax = 4.2; // V
const double capacity = 20; // Ah
// const double capacity = 5; // Ah

/* Pin Settings */
const int voltageSettingIn = A2; // LOW: Charge to 100%; HIGH: Charge to 80%
const int switchIn = A1; // Switch to turn charging on/off
const int voltageIn = A0; // Voltage from battery/charger voltage divider - replaced by ADS1115
const int statusOut = 5; // Status voltmeter (0-1V out)
const int relayNeutralOut = 6; // Relay Neutral wire
const int relayLiveOut = 7; // Relay Live wire
const int relaySwitchOut = 8; // Relay charger switch
const int relayVoltageOut = 9; // Relay positive voltage wire

/* Voltage Divider Settings */
const double maxInputVoltage = 126.729078; // Determine this experimentally

/* General Stuff */
int vIns[60] = {};
int counter = 0;

/* Mode = 0 - Turn charger off when this voltage is reached */
double voltageMin = 60; // Wait until >= this voltage is measured before starting charging, to verify battery connection
double voltageCutoff = 81; // For experimenting, put this impossibly high
double battery80Voltage = series * 4.025; // 72.45V - Will drop to approx. 72.4V after charging (4.02V per cell)
double battery100Voltage = series * 4.185; // 75.33V - Close to 100% so BMS doesn't cut power

/* Mode = 1 - Turn charger off when this percentage is reached - ToDo: Calculate with specific cell voltage graph, otherwise don't use this yet! */
double percentageCutoff = 80;
double batteryMinVoltage = series * voltageNominal;
double batteryMaxVoltage = series * voltageMax;

/* Mode = 2 - Turn charger off when certain Ah is reached (CC only) */
double ahCutoff = 4;

/* Mode = 3 - Turn charger off when certain Wh is reached (CC only) */
double whCutoff = 530;

int mode = 2;
int state = 0;

unsigned long start = 0;
double startStatus = 0;

void chargerOn() {
    Serial.print("Turning charger on...");

    digitalWrite(relayNeutralOut, LOW);
    delay(500);
    digitalWrite(relayLiveOut, LOW);
    delay(1500); // Wait for capacitors to charge
    digitalWrite(relaySwitchOut, LOW);
    delay(1500);
    digitalWrite(relayVoltageOut, LOW);
    delay(1500); // Wait for voltage to stabilize before measuring
    Serial.println(" done!");

    state = 1;

    start = millis();
    startStatus = 0;
}

void chargerOff() {
    Serial.print("Turning charger off...");

    digitalWrite(relayVoltageOut, HIGH);
    delay(500);
    digitalWrite(relaySwitchOut, HIGH);
    delay(500);
    digitalWrite(relayLiveOut, HIGH);
    delay(500);
    digitalWrite(relayNeutralOut, HIGH);
    delay(3000); // Wait for capacitors to discharge
    Serial.println(" done!");

    state = 2;
}

void setStatus(double status) {
    int stOut = ((int) (status * ((double) 256 / 5))) - 1;
    if (debugMode) {
        Serial.print("stOut: " + String(stOut));
    }
    if (status < 0) stOut = 0;
    if (status > 1) stOut = (int) ((double) 256 / 5);
    if (debugMode) {
        Serial.println(", " + String(stOut));
    }
    analogWrite(statusOut, stOut);
}

void setup() {
    Serial.begin(115200);

    pinMode(switchIn, INPUT_PULLUP);
    pinMode(voltageSettingIn, INPUT_PULLUP);

    digitalWrite(statusOut, LOW);
    digitalWrite(relayNeutralOut, HIGH);
    digitalWrite(relayLiveOut, HIGH);
    digitalWrite(relaySwitchOut, HIGH);
    digitalWrite(relayVoltageOut, HIGH);

    pinMode(statusOut, OUTPUT);
    pinMode(relayNeutralOut, OUTPUT);
    pinMode(relayLiveOut, OUTPUT);
    pinMode(relaySwitchOut, OUTPUT);
    pinMode(relayVoltageOut, OUTPUT);

    ads.setGain(GAIN_ONE);
    ads.begin();

    for (int i = 0; i < 60; i++) {
        vIns[i] = 0;
    }

    if (advancedDebugMode) {
        digitalWrite(relayVoltageOut, LOW);
    }
}

double getVoltage(double vIn) {
    double voltage = (((double) vIn * adsC) / 4.096F) * maxInputVoltage;
    return voltage;
}

void loop() {
    int16_t vIn = ads.readADC_SingleEnded(0);

    if (vIn <= 0) vIn = 1;

    double voltage = getVoltage(vIn);

    vIns[counter] = vIn;

    if (counter == 60) {
        counter = 0;
    } else {
        counter++;
    }

    double dividend = 0;
    double divisor = 0;

    for (int currentVIn : vIns) {
        if (currentVIn > 0) {
            dividend += currentVIn;
            divisor++;
        }
    }

    double averageVIn = (double) dividend / (double) divisor;

    if (advancedDebugMode) {
        Serial.println("");
        Serial.println("vIn            : " + String(vIn));
        Serial.println("Voltage        : " + String(voltage));
        Serial.println("Average vIn    : " + String(averageVIn));
        Serial.println("Average Voltage: " + String(getVoltage(averageVIn)));
    }

    voltage = getVoltage(averageVIn);

    if (state == 0) { // Waiting for switch to be turned on // ToDo: And in settings mode. Change cutoff voltage/percentage
        Serial.println("Waiting...");
        if (digitalRead(switchIn) == LOW && (voltage >= voltageMin || mode == 2)) {
            Serial.println("Switch turned on!");
            chargerOn();
        }
    } else if (state == 1) { // Charging - Waiting for charging to be finished or switch to be turned off
        if (digitalRead(switchIn) == HIGH) {
            chargerOff();
            return;
        }

        double seconds = ((double) (millis() - start)) / 1000;
        double ah = ((double) (current * seconds)) / 3600;
        double wh = ah * batteryMinVoltage;

        Serial.print("vIn: " + String(vIn));
        Serial.print("Vol: " + String(voltage));
        Serial.print("Ah : " + String(ah));
        Serial.println("Wh : " + String(wh));
        Serial.println();

        if (mode == 0) {
            if (startStatus == 0) {
                startStatus = voltage;
            } else {
                setStatus(((voltage - startStatus)) / (voltageCutoff - startStatus));
            }

            if (digitalRead(voltageSettingIn) == HIGH) {
                voltageCutoff = battery80Voltage;
            } else {
                voltageCutoff = battery100Voltage;
            }
            
            if (voltage >= voltageCutoff) {
                Serial.println("Voltage reached " + String(voltage) + " (+" + String(voltage - voltageCutoff) +
                               "), turning charger off...");
                chargerOff();
            }
        } else if (mode == 1) {
            double voltageMinMaxDifference = batteryMaxVoltage - batteryMinVoltage;
            double currentPercentage = (((double) (voltage - batteryMinVoltage)) / voltageMinMaxDifference) * 100;

            if (startStatus == 0) {
                startStatus = currentPercentage;
            } else {
                setStatus(((currentPercentage - startStatus)) / (percentageCutoff - startStatus));
            }

            if (currentPercentage >= percentageCutoff) {
                Serial.println(
                        "Percentage reached " + String(currentPercentage) + " at " + String(voltage) + " volts (+" +
                        String(currentPercentage - percentageCutoff) + "), turning charger off...");
                chargerOff();
            }
        } else if (mode == 2) {
            if (startStatus == 0) {
                startStatus = ah;
            } else {
                setStatus(((ah - startStatus)) / (ahCutoff - startStatus));
            }

            Serial.println(
                    "Reached " + String(ah) + "/" + String(ahCutoff) + " amp hours (" + String((ah / ahCutoff) * 100) +
                    "%)");

            if (ah >= ahCutoff) {
                Serial.println("Ampere hours reached " + String(ah) + " (+" + String(ah - ahCutoff) +
                               "), turning charger off...");
                chargerOff();
            }
        } else if (mode == 3) {
            if (startStatus == 0) {
                startStatus = wh;
            } else {
                setStatus(((wh - startStatus)) / (whCutoff - startStatus));
            }

            if (wh > whCutoff) {
                Serial.println("Watt hours reached " + String(wh) + " (+" + String(wh - whCutoff) +
                               "), turning charger off...");
                chargerOff();
            }
        }
    } else if (state == 2) { // Finished Charging - Waiting for switch to be turned off
        if (digitalRead(switchIn) == HIGH) {
            Serial.println("Switch turned off, resetting!");
            state = 0;
        }
    }
}
