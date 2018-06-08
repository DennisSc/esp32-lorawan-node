//#include <Arduino.h>
#include <SPI.h>
#include <SSD1306.h>
#include <HardwareSerial.h>
#include <lmic.h>
#include <hal/hal.h>
#include <ByteConvert.hpp>

//initialize OLED of TTGO ES32 Devboard
SSD1306 display(0x3c, 5, 4); // instance for the OLED. Addr, SDA, SCL

//little endian
static const u1_t PROGMEM APPEUI[8]= { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8]= { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

//big endian
static const u1_t PROGMEM APPKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

uint8_t ledPin = 16; // Onboard LED reference

static uint8_t mydata[] = "ABCD12";
static osjob_t sendjob;
static osjob_t randomjob;

//persistently store global variables in RTC memory
RTC_DATA_ATTR int bootCount = 0;
RTC_DATA_ATTR byte rndom = 0;

// Schedule TX interval every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 15;

// Schedule RND generation every this many seconds
const unsigned RND_INTERVAL = 1;



// LMIC pin mapping
const lmic_pinmap lmic_pins = {   

    //.nss = 10,
    .nss = 21,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 35,
    //.rst = LMIC_UNUSED_PIN,
    .dio = {26,33,LMIC_UNUSED_PIN}
    //.dio = {2, 6, 7},
};



void do_random(osjob_t* j)
{
    int adcval = analogRead(0);
    mydata[4] = highByte(adcval);
    mydata[5] = lowByte(adcval);

    rndom = random(0,255);
    mydata[3] = rndom;

    

    display.setColor(BLACK);
    display.fillRect(0,45,128,14);
    display.setColor(WHITE);
    String msg = "";
    msg = msg + "ADC: ";
    msg = msg + String(adcval);
    msg = msg + ", RND: ";
    msg = msg + String(rndom);

    display.drawString(0,45, msg);
    display.display();

    os_setTimedCallback(&randomjob, os_getTime()+sec2osticks(RND_INTERVAL), do_random);
    
}


//char conversion helper
char valToHex(uint8_t val) {
    if ((val & 0x0f) < 10)
      return ('0' + val);
    else
      return ('a' + (val - 10));
  }

//byte conversion helper
String byteToHexString(uint8_t b) {
    String buffer = "";
    buffer += valToHex(b & 0x0f);
    b >>= 4;
    buffer = valToHex(b & 0x0f) + buffer;
    return buffer;
  }



//main TTN TX function
void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
        digitalWrite(ledPin, HIGH);

        display.setColor(BLACK);
        display.fillRect(0,31,128,12);
        display.setColor(WHITE);  
        display.drawString(0,31, "");
        display.display();
        
    } else {
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0);
        Serial.println(F("Packet queued"));
        digitalWrite(ledPin, LOW);

         display.setColor(BLACK);
        display.fillRect(0,31,128,12);
        display.setColor(WHITE);
        display.drawString(0,31, "Sending packet ");
        display.display();
    }
    // Next TX is scheduled after TX_COMPLETE event.
}


// LMIC event handler
void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            display.setColor(BLACK);
            display.fillRect(0,11,128,12);
            display.setColor(WHITE);
            display.drawString(0,10,"Joining...");
            display.display(); // display whatever is in the buffer
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            display.setColor(BLACK);
            display.fillRect(0,11,128,12);
            display.setColor(WHITE);
            display.drawString(0,10,"Joined.");
            display.display(); // display whatever is in the buffer
            // Disable link check validation (automatically enabled
            // during join, but not supported by TTN at this time).
            LMIC_setLinkCheckMode(0);
    
            //start random generator after joined
            os_setTimedCallback(&randomjob, os_getTime()+sec2osticks(RND_INTERVAL), do_random);
    
            break;
        case EV_RFU1:
            Serial.println(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            
            if (LMIC.dataLen) {
              display.setColor(BLACK);
              display.fillRect(0,11,128,12);
              display.fillRect(0,21,128,12);
              
              display.setColor(WHITE);
              String text = "Received ";
              text += LMIC.dataLen;
              text += " bytes:";
              display.drawString(0,10,text);
              
              String buffer = "";
              for (size_t i = 0;i < (LMIC.dataLen);i++)
                  buffer += byteToHexString(LMIC.frame[LMIC.dataBeg + i]);
              //Serial.println(buffer); 
              display.drawString(0,20,buffer);
              

              display.display(); // display whatever is in the buffer
              Serial.print(F("Received "));
              Serial.print(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
              // data received in rx slot after tx
              Serial.print(F("Data Received: "));
              Serial.println(buffer);
              //Serial.write(LMIC.frame+LMIC.dataBeg, LMIC.dataLen);
              /*Serial.println();
              for (int i = 0; i < LMIC.dataLen; i++) {
                if (LMIC.frame[LMIC.dataBeg + i] < 0x10) {
                Serial.print(F("0"));
                }
              Serial.print(LMIC.frame[LMIC.dataBeg + i], HEX);
              }*/
            
            }
            else
            {  display.setColor(BLACK);
              display.fillRect(0,11,128,12);
              display.fillRect(0,21,128,12);
              display.setColor(WHITE);
              display.display(); // display whatever is in the buffer
            }
            
            //not sending:led off
            digitalWrite(ledPin, HIGH);
            
            display.setColor(BLACK);
            display.fillRect(0,31,128,12);
            display.setColor(WHITE);
                
            display.drawString(0,31, "...");
            display.display();
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            //os_setTimedCallback(&randomjob, os_getTime()+sec2osticks(RND_INTERVAL), do_random);
    
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
         default:
            Serial.println(F("Unknown event"));
            break;
    }
}


//setup
void setup() {
    Serial.begin(115200);
    Serial.println(F("Starting"));

    //incrment boot counter after wakeup from deep sleep
    ++bootCount;
    Serial.println("Boot number: " + String(bootCount));
    rndom = random(0,255);
    Serial.println("Random number: " + String(rndom));
    
    pinMode(ledPin, OUTPUT);
    pinMode(35, INPUT);

    display.init(); // initialise the OLED
    display.clear();
    //display.flipScreenVertically(); // does what is says
    display.setFont(ArialMT_Plain_10); // does what is says
    // Set the origin of text to top left
    //display.setTextAlignment(TEXT_ALIGN_CENTER_BOTH);
    display.drawString(0, 0, "esp32 LoRaWAN Node");
    
    display.drawString(0,10,"Starting...");
    display.display(); // display whatever is in the buffer
    

    for (int i = 0; i< 3; i++){
      mydata[i] = 65+i; //ABC in ASCII
      }
    
    //pinMode(25, INPUT);
    analogReadResolution(11);
    analogSetAttenuation(ADC_11db); 

    //test
    int adcval = analogRead(0);
    Serial.print("adcval: ");
    Serial.println(adcval);
    mydata[4] = highByte(adcval);
    mydata[5] = lowByte(adcval);

    randomSeed(adcval);
    
    // LMIC init begins here

    #ifdef VCC_ENABLE
    // For Pinoccio Scout boards
    pinMode(VCC_ENABLE, OUTPUT);
    digitalWrite(VCC_ENABLE, HIGH);
    delay(1000);
    #endif

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // Start job (sending automatically starts OTAA too)
    do_send(&sendjob);
    //do_random(&randomjob);
    
    // put your setup code here, to run once:
}

//main loop
void loop() {
    

    os_runloop_once();
    // put your main code here, to run repeatedly:

    //delay(100);
}