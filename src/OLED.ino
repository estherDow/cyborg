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
float last_sipm_voltage = 0;
float temperatureC;

byte waiting_for_interupt = 0;
byte SLAVE;
byte MASTER;
byte keep_pulse = 0;

void setup()
{
    analogReference(EXTERNAL);
    ADCSRA &= ~(bit(ADPS0) | bit(ADPS1) | bit(ADPS2)); // clear prescaler bits
    ADCSRA |= bit(ADPS0) | bit(ADPS1);                 // Set prescaler to 8
    Serial.begin(9600);

    pinMode(3, OUTPUT);
    pinMode(6, INPUT);
    if (digitalRead(6) == HIGH)
    {
        SLAVE = 1;
        MASTER = 0;
        digitalWrite(3, HIGH);
        delay(1000);
    }

    else
    {
        delay(10);
        MASTER = 1;
        SLAVE = 0;
        pinMode(6, OUTPUT);
        digitalWrite(6, HIGH);
    }

    digitalWrite(3, LOW);
    if (MASTER == 1)
    {
        digitalWrite(6, LOW);
    }

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

            // If Master, send a signal to the Slave
            if (MASTER == 1)
            {
                digitalWrite(6, HIGH);
                muon_count++;
                keep_pulse = 1;
            }

            // Wait for ~8us
            analogRead(A3);

            // If Slave, check for signal from Master

            if (SLAVE == 1)
            {
                if (digitalRead(6) == HIGH)
                {
                    keep_pulse = 1;
                    muon_count++;
                }
            }

            // Wait for ~8us
            analogRead(A3);

            // If Master, stop signalling the Slave
            if (MASTER == 1)
            {
                digitalWrite(6, LOW);
            }

            // Measure the temperature, voltage reference is currently set to 3.3V
            temperatureC = (((analogRead(A3) + analogRead(A3) + analogRead(A3)) / 3. * (3300. / 1024)) - 500.) / 10.;

            // Measure deadtime
            measurement_deadtime = total_deadtime;
            time_stamp = millis() - start_time;

            // If you are within 15 miliseconds away from updating the OLED screen, we'll let if finish
            if ((interrupt_timer + 1000 - millis()) < 15)
            {
                waiting_t1 = millis();
                waiting_for_interupt = 1;
                delay(30);
                waiting_for_interupt = 0;
            }

            measurement_t1 = micros();

            if (MASTER == 1)
            {
                analogWrite(3, LED_BRIGHTNESS);
                sipm_voltage = get_sipm_voltage(adc);
                last_sipm_voltage = sipm_voltage;
                Serial.println((String)muon_count + " " + time_stamp + " " + adc + " " + sipm_voltage + " " + measurement_deadtime + " " + temperatureC);
            }

            if (SLAVE == 1)
            {
                if (keep_pulse == 1)
                {
                    analogWrite(3, LED_BRIGHTNESS);
                    sipm_voltage = get_sipm_voltage(adc);
                    last_sipm_voltage = sipm_voltage;
                    Serial.println((String)muon_count + " " + time_stamp + " " + adc + " " + sipm_voltage + " " + measurement_deadtime + " " + temperatureC);
                }
            }

            keep_pulse = 0;
            digitalWrite(3, LOW);
            while (analogRead(A0) > RESET_THRESHOLD)
            {
                continue;
            }
            total_deadtime += (micros() - measurement_t1) / 1000.;
        }
    }
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
