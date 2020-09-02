/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 * 29 Dec 2016 Modified by Zaki to cater for RF95 + Arduino 
 * 15 Dec 2017 Do voltage sensing and send Uplink to TTN
 * 20 Feb 2018 Added downlink handler and check for first byte for downlink, line 136-158
 *******************************************************************************/
#include <SPI.h>
#include <lmic.h>
#include <hal/hal.h>
#include <ArduinoJson.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h> 
#include <dht.h>

#define DHT22_PIN 3

/*********************************Pulse Sensor********************************************************/
//  Variables
int pulsePin = 5;                 // Pulse Sensor purple wire connected to analog pin A5
int blinkPin = 13;                // pin to blink led at each beat. LED at D13

// Volatile Variables, used in the interrupt service routine!
volatile int BPM;                   // int that holds raw Analog in 0. updated every 2mS
volatile int Signal;                // holds the incoming raw data
volatile int IBI = 600;             // int that holds the time interval between beats! Must be seeded! 
volatile boolean Pulse = false;     // "True" when User's live heartbeat is detected. "False" when not a "live beat". 
volatile boolean QS = false;        // becomes true when Arduoino finds a beat.
volatile int rate[10];                      // array to hold last ten IBI values
volatile unsigned long sampleCounter = 0;   // used to determine pulse timing
volatile unsigned long lastBeatTime = 0;    // used to find IBI
volatile int P = 512;                     // used to find peak in pulse wave, seeded
volatile int T = 512;                     // used to find trough in pulse wave, seeded
volatile int thresh = 525;                // used to find instant moment of heart beat, seeded
volatile int amp = 100;                   // used to hold amplitude of pulse waveform, seeded
volatile boolean firstBeat = true;        // used to seed rate array so we startup with reasonable BPM
volatile boolean secondBeat = false;      // used to seed rate array so we startup with reasonable BPM
/**********************************************************************************************************/
                          
dht DHT;
TinyGPSPlus gps;
SoftwareSerial gpsSerial(4,5); // (Rx,Tx) ...note Rx connect to Tx at GPS module and Tx connects to Rx at GPS module
//PulseSensorPlayground pulseSensor;  // Creates an instance of the PulseSensorPlayground object called "pulseSensor"

volatile float averageVcc = 0.0;

static const PROGMEM u1_t NWKSKEY[16] = { 0xF5, 0x6B, 0x4B, 0xBA, 0x9B, 0xA6, 0xF3, 0xF7, 0x49, 0x73, 0x76, 0xFD, 0x88, 0x9D, 0x6B, 0x0C };
static const u1_t PROGMEM APPSKEY[16] = { 0xA5, 0x7D, 0xC5, 0x2E, 0x1D, 0xA1, 0x63, 0xD5, 0xF9, 0xC4, 0xAD, 0x6B, 0x04, 0x8E, 0x96, 0x10 };
static const u4_t DEVADDR = 0x260119E7 ; 
   
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

/********************************************* ABP Section Ends ***************************************************************************/

static osjob_t sendjob;

/* Schedule TX every this many seconds (might become longer due to duty
   cycle limitations). */
const unsigned TX_INTERVAL = 2; //every 2secs

/* Pin mapping */
const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 9,
    .dio = {2, 6, 7},
};

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {        
        case EV_TXCOMPLETE:
            Serial.println("EV_TXCOMPLETE (includes waiting for RX windows)");
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        
         default:
            //Serial.println(F("Unknown event"));
            break;
    }
}

void do_send(osjob_t* j){
    
        static uint8_t message[11];
    
        /* Check if there is not a current TX/RX job running */
        if (LMIC.opmode & OP_TXRXPEND) {
           //Serial.println(F("OP_TXRXPEND, not sending"));
        } 
        else 
        {
            /********** GPS Neo-6MV2 **************/
            for (unsigned long start = millis(); millis() - start < 1000;) //Just for 1 second, get GPS location
            {
                gpsSerial.listen();
                if (gpsSerial.available()) // check for gps data
                {
                    if (gps.encode(gpsSerial.read()))
                    {
                        Serial.print(F("Location: ")); 
                        if (gps.location.isValid())
                        {   
                            float flat = gps.location.lat();
                            int32_t latitude = flat * 10000; // to make it as round number                    
                            Serial.print(flat, 4); // up to 4 decimal points
                            message[0] = latitude >> 16;
                            message[1] = latitude >> 8;
                            message[2] = latitude;

                            float flon = gps.location.lng();
                            int32_t longitude = flon * 10000; // to make it as round number                  
                            Serial.print(F(","));
                            Serial.print(flon, 4); // up to 4 decimal points
                            message[3] = longitude >> 16;
                            message[4] = longitude >> 8;
                            message[5] = longitude;
                        }
                        else
                        {
                            Serial.print("INVLD");
                        }
                        Serial.println();
                    }
                }
                if (millis() > 5000 && gps.charsProcessed() < 10)
                {
                    Serial.println("No GPS:chck wiring");
                }
            }
            /**************************************************************/
            
            /***************  DHT22  **************************************/
            // READ DATA
            Serial.println("DHT22 :");
            delay(100);
            int chk = DHT.read22(DHT22_PIN);
            delay(100);          
            volatile float humid_float = (float)DHT.humidity;
            delay(100);
            volatile float temp_float = (float)DHT.temperature;
            delay(100);
            uint16_t humidity = round(humid_float * 100);
            uint16_t temperature = round(temp_float * 100);
            Serial.print("Humidity :");
            Serial.println(humid_float);
            Serial.print("Temperature :");
            Serial.println(temp_float);

            if(humid_float >= 0 && humid_float <= 100)
            {
                message[6] = highByte(humidity);
                message[7] = lowByte(humidity);
                message[8] = highByte(temperature);
                message[9] = lowByte(temperature);   
            }         
            /***************************************************************/

            /**************** Pulse Sensor *********************************/
            cli();  //disable interrupts

            /*sets up to read Pulse Sensor signal every 2mS*/ 
            TCCR2A = 0x02;     // DISABLE PWM ON DIGITAL PINS 3 AND 11, AND GO INTO CTC MODE
            TCCR2B = 0x06;     // DON'T FORCE COMPARE, 256 PRESCALER 
            OCR2A = 0X7C;      // SET THE TOP OF THE COUNT TO 124 FOR 500Hz SAMPLE RATE
            TIMSK2 = 0x02;     // ENABLE INTERRUPT ON MATCH BETWEEN TIMER2 AND OCR2A

            for (unsigned long startpulse = millis(); millis() - startpulse < 5000;) //Just for 5 second, get pulse rate
            {
                //Serial.println(F("Pulse Sensor is started"));
                sei();  //enables interrupt
                if (QS == true) // A Heartbeat Was Found
                {     
                      if (BPM < 150 && BPM > 0)
                      {   
                          Serial.print("BPM: ");
                          Serial.println(BPM);
                          message[10] = BPM;  
                      }
                      else 
                      {
                          message[10] = 0;
                      }
                      
                      QS = false; // reset the Quantified Self flag for next time    
                }
 
                delay(20); //  take a break
            }            

            TCCR2A = 0x02;     // DISABLE PWM ON DIGITAL PINS 3 AND 11, AND GO INTO CTC MODE
            TCCR2B = 0x06;     // DON'T FORCE COMPARE, 256 PRESCALER 
            OCR2A = 0X7C;      // SET THE TOP OF THE COUNT TO 124 FOR 500Hz SAMPLE RATE
            TIMSK2 = 0x00;     // DISABLE TIMER COMPARE INTERRUPT
            sei();
            
            /***************************************************************************************************/

            
            LMIC_setTxData2(1, message, sizeof(message), 0);        
            Serial.println(F("Packet queued"));
            //Print Freq being used/
            Serial.print("TransmitChanel : ");Serial.println(LMIC.txChnl);     
            
        }
    /* Next TX is scheduled after TX_COMPLETE event. */
}

void setup() {
    Serial.begin(115200); 
    delay(100);
    Serial.println("Start");   
    
    gpsSerial.begin(9600); 
    delay(100);// connect gps sensor
    Serial.println(F("Initializing GPS..."));

    pinMode(blinkPin,OUTPUT);         // pin that will blink to your heartbeat!
    pinMode(pulsePin,INPUT);
    
    LoraInitialization();  // Do all Lora Init Stuff
    /* Start job */
    do_send(&sendjob);
}

void loop() {   
    os_runloop_once();   
}


void LoraInitialization() {
    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);

    LMIC_setupChannel(0, 923200000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF12,14);

    // Start job
    do_send(&sendjob);
}


ISR(TIMER2_COMPA_vect) //triggered when Timer2 counts to 124
{  
  Signal = analogRead(pulsePin);              // read the Pulse Sensor 
  sampleCounter += 2;                         // keep track of the time in mS with this variable
  int N = sampleCounter - lastBeatTime;       // monitor the time since the last beat to avoid noise
                                              //  find the peak and trough of the pulse wave
  if(Signal < thresh && N > (IBI/5)*3) // avoid dichrotic noise by waiting 3/5 of last IBI
    {      
      if (Signal < T) // T is the trough
      {                        
        T = Signal; // keep track of lowest point in pulse wave 
      }
    }

  if(Signal > thresh && Signal > P)
    {          // thresh condition helps avoid noise
      P = Signal;                             // P is the peak
    }                                        // keep track of highest point in pulse wave

  //  NOW IT'S TIME TO LOOK FOR THE HEART BEAT
  // signal surges up in value every time there is a pulse
  if (N > 250)
  {                                   // avoid high frequency noise
    if ( (Signal > thresh) && (Pulse == false) && (N > (IBI/5)*3) )
      {        
        Pulse = true;                               // set the Pulse flag when we think there is a pulse
        digitalWrite(blinkPin,HIGH);                // turn on pin 13 LED
        IBI = sampleCounter - lastBeatTime;         // measure time between beats in mS
        lastBeatTime = sampleCounter;               // keep track of time for next pulse
  
        if(secondBeat)
        {                        // if this is the second beat, if secondBeat == TRUE
          secondBeat = false;                  // clear secondBeat flag
          for(int i=0; i<=9; i++) // seed the running total to get a realisitic BPM at startup
          {             
            rate[i] = IBI;                      
          }
        }
  
        if(firstBeat) // if it's the first time we found a beat, if firstBeat == TRUE
        {                         
          firstBeat = false;                   // clear firstBeat flag
          secondBeat = true;                   // set the second beat flag
          sei();                               // enable interrupts again
          return;                              // IBI value is unreliable so discard it
        }   
      // keep a running total of the last 10 IBI values
      word runningTotal = 0;                  // clear the runningTotal variable    

      for(int i=0; i<=8; i++)
        {                // shift data in the rate array
          rate[i] = rate[i+1];                  // and drop the oldest IBI value 
          runningTotal += rate[i];              // add up the 9 oldest IBI values
        }

      rate[9] = IBI;                          // add the latest IBI to the rate array
      runningTotal += rate[9];                // add the latest IBI to runningTotal
      runningTotal /= 10;                     // average the last 10 IBI values 
      BPM = 60000/runningTotal;               // how many beats can fit into a minute? that's BPM!
      QS = true;                              // set Quantified Self flag 
      // QS FLAG IS NOT CLEARED INSIDE THIS ISR
    }                       
  }

  if (Signal < thresh && Pulse == true)
    {   // when the values are going down, the beat is over
      digitalWrite(blinkPin,LOW);            // turn off pin 13 LED
      Pulse = false;                         // reset the Pulse flag so we can do it again
      amp = P - T;                           // get amplitude of the pulse wave
      thresh = amp/2 + T;                    // set thresh at 50% of the amplitude
      P = thresh;                            // reset these for next time
      T = thresh;
    }

  if (N > 2500)
    {                           // if 2.5 seconds go by without a beat
      thresh = 512;                          // set thresh default
      P = 512;                               // set P default
      T = 512;                               // set T default
      lastBeatTime = sampleCounter;          // bring the lastBeatTime up to date        
      firstBeat = true;                      // set these to avoid noise
      secondBeat = false;                    // when we get the heartbeat back
    }
    
}// end isr

