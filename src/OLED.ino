/*
  CosmicWatch Desktop Muon Detector Arduino Code

  This code does not use the microSD card reader/writer, but does used the OLED screen.

  Questions?
  Spencer N. Axani
  saxani@mit.edu

  Requirements: Sketch->Include->Manage Libraries:
  SPI, EEPROM, SD, and Wire are probably already installed.
  1. Adafruit SSD1306     -- by Adafruit Version 1.0.1
  2. Adafruit GFX Library -- by Adafruit Version 1.0.2
  3. TimerOne             -- by Jesse Tane et al. Version 1.1.0
*/

#include <Wire.h>
#include <Arduino.h>
#include <MIDI.h>

const byte OLED = 1; // Turn on/off the OLED [1,0]

const int SIGNAL_THRESHOLD = 50; // Min threshold to trigger on. See calibration.pdf for conversion to mV.
const int RESET_THRESHOLD = 25;

const int LED_BRIGHTNESS = 255; // Brightness of the LED [0,255]

const long double calibration[] = {-9.085681659276021e-27, 4.6790804314609205e-23, -1.0317125207013292e-19,
                                   1.2741066484319192e-16, -9.684460759517656e-14, 4.6937937442284284e-11, -1.4553498837275352e-08,
                                   2.8216624998078298e-06, -0.000323032620672037, 0.019538631135788468, -0.3774384056850066, 12.324891083404246};

const int cal_max = 1023;

unsigned long time_stamp = 0L;
unsigned long measurement_deadtime = 0L;
unsigned long time_measurement = 0L; // Time stamp
unsigned long interrupt_timer = 0L;  // Time stamp
int start_time = 0L;                 // Reference time for all the time measurements
unsigned long total_deadtime = 0L;   // total measured deadtime
unsigned long waiting_t1 = 0L;
unsigned long measurement_t1;
unsigned long measurement_t2;

float sipm_voltage = 0;
long int muon_count = 0L; // A tally of the number of muon counts observed
float temperatureC;

MIDI_CREATE_DEFAULT_INSTANCE();

void setup()
{
    analogReference(EXTERNAL);
    ADCSRA &= ~(bit(ADPS0) | bit(ADPS1) | bit(ADPS2)); // clear prescaler bits
    ADCSRA |= bit(ADPS0) | bit(ADPS1);                 // Set prescaler to 8
    Serial.begin(9600);
    MIDI.begin(MIDI_CHANNEL_OMNI);
    pinMode(3, OUTPUT);

    digitalWrite(3, LOW);

    Serial.println(F("##########################################################################################"));
    Serial.println(F("### CosmicWatch: The Desktop Muon Detector"));
    Serial.println(F("### Questions? saxani@mit.edu"));
    Serial.println(F("### Comp_date Comp_time Event Ardn_time[ms] ADC[0-1023] SiPM[mV] Deadtime[ms] Temp[C] Name"));
    Serial.println(F("##########################################################################################"));

    delay(900);
    start_time = millis();
}

void loop()
{
    while (1)
    {
        if (analogRead(A0) > SIGNAL_THRESHOLD)
        {

            // Make a measurement of the pulse amplitude
            int adc = analogRead(A0);

measureTemperature();

            measureDeadTime();

            measurement_t1 = micros();

            analogWrite(3, LED_BRIGHTNESS);
            sipm_voltage = get_sipm_voltage(adc);
            Serial.println((String)muon_count + " " + time_stamp + " " + adc + " " + sipm_voltage + " " + measurement_deadtime + " " + temperatureC);
            MIDI.sendNoteOn((int)sipm_voltage*127, 127, 1);
            digitalWrite(3, LOW);
            while (analogRead(A0) > RESET_THRESHOLD)
            {
                continue;
            }
            total_deadtime += (micros() - measurement_t1) / 1000.;
        }
    }
}

void measureTemperature()
{
            // Wait for ~8us
            analogRead(A3);

            // Measure the temperature, voltage reference is currently set to 3.3V
            temperatureC = (((analogRead(A3) + analogRead(A3) + analogRead(A3)) / 3. * (3300. / 1024)) - 500.) / 10.;
}

void measureDeadTime()
{
    // Measure deadtime
    measurement_deadtime = total_deadtime;
    time_stamp = millis() - start_time;
}

// This function converts the measured ADC value to a SiPM voltage via the calibration array
float get_sipm_voltage(float adc_value)
{
    float voltage = 0;
    for (unsigned int i = 0; i < (sizeof(calibration) / sizeof(float)); i++)
    {
        voltage += calibration[i] * pow(adc_value, (sizeof(calibration) / sizeof(float) - i - 1));
    }
    return voltage;
}
