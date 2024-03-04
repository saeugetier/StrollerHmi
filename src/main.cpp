#include "esp32-hal-adc.h"
#include <Arduino.h>
#include <U8g2lib.h>
#include <hal/uart_types.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

#include <PJONThroughSerial.h>

#include <pb_decode.h>
#include <pb_encode.h>
#include "steering.pb.h"

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

#if defined(ESP8266)|| defined(ESP32) || defined(AVR)
#include <EEPROM.h>
#endif

//#include <LiteLED.h>

// Choose the LED type from the list below.
// Comment out all but one LED_TYPE.
// #define LED_TYPE        LED_STRIP_WS2812
// #define LED_TYPE        LED_STRIP_SK6812
#define LED_TYPE        LED_STRIP_APA106
// #define LED_TYPE        LED_STRIP_SM16703

#define LED_TYPE_IS_RGBW 0   // if the LED is an RGBW type, change the 0 to 1

#define LED_GPIO 42     // change this number to be the GPIO pin connected to the LED

#define LED_BRIGHT 30   // sets how bright the LED is. O is off; 255 is burn your eyeballs out (not recommended)

//LiteLED myLED( LED_TYPE, LED_TYPE_IS_RGBW );    // create the LiteLED object; we're calling it "myLED"


#define CONVERSIONS_PER_PIN 5

uint8_t adc_pins[] = {0, 1, 2, 3}; 
const uint8_t adc_pins_count = sizeof(adc_pins) / sizeof(uint8_t);
volatile bool adc_coversion_done = false;
adc_continuos_data_t * result = NULL;

uint16_t zero_point[adc_pins_count] = {(1<<11)};

uint8_t tare_counter = 0;

bool tare = false;

void ARDUINO_ISR_ATTR adcComplete() {
  adc_coversion_done = true;
}

U8G2_SH1106_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

constexpr uint8_t HMI_DEVICE_ID = 42;
constexpr uint8_t LEFT_MOTOR_ID = 43;
constexpr uint8_t RIGHT_MOTOR_ID = 44;

PJONThroughSerial pjon(HMI_DEVICE_ID);

// End of constructor list

void u8g2Task(void* pvParameters);

void receiver_function(uint8_t *payload, uint16_t length, const PJON_Packet_Info &packet_info) {
  pb_istream_t istream;

  istream = pb_istream_from_buffer(payload, length);  
}

void error_handler(uint8_t code, uint16_t data, void *custom_pointer) 
{
  Serial.printf("error %d\n", code);
}

BLEServer *pServer = nullptr;

void setup(void) {

  delay(1000);

  Serial.begin(115200);
  Serial0.begin(38400);
  Serial0.setMode(UART_MODE_RS485_HALF_DUPLEX);

  pjon.set_receiver(receiver_function);
  pjon.set_error(error_handler);
  pjon.strategy.set_serial(&Serial0);
  pjon.begin();
  pinMode(A0, INPUT);
  digitalRead(A0);

  BLEDevice::init("Strollomat 5000");
  pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);
  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

  pCharacteristic->setValue("Hello World says Neil");
  pService->start();
  // BLEAdvertising *pAdvertising = pServer->getAdvertising();  // this still is working for backward compatibility
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();

  pinMode(10, OUTPUT);

  pinMode(8, OUTPUT);

  xTaskCreate(u8g2Task, "u8g2", 1000, nullptr, 5, nullptr);

  analogContinuousSetWidth(12);

  analogSetAttenuation(ADC_11db);

  analogContinuous(adc_pins, adc_pins_count, CONVERSIONS_PER_PIN, 20000, &adcComplete);

  analogContinuousStart();  

  tare_counter = 0;
  tare = false;  

   for (int i = 0; i < adc_pins_count; i++) {
      zero_point[i] = 0;
    }
}



uint8_t width = 5;
uint8_t width_state = 1;
uint8_t border = 0;
uint8_t border_state = 1;
uint8_t emoticons = 0;
uint8_t emoticons_state = 1;
uint8_t offset = 0;
uint8_t offset_state = 1;

void wave(uint8_t min, uint8_t max, uint8_t &v, uint8_t &state) {
  if ( state ) {
    v++;
    if ( v >= max )
      state = 0;
  } else {
    v--;
    if ( v <= min )
      state = 1;
  }
}

int iteration = 0;
uint8_t pwm = 0;
uint8_t pwm_state = 1;

float load_a = -1.0f;
float load_b = 0.0f;

void loop(void) {
  if(!tare)
  {
      // Check if conversion is done and try to read data
    if (adc_coversion_done == true) {
      // Set ISR flag back to false
      adc_coversion_done = false;
      // Read data from ADC
      if (analogContinuousRead(&result, 0)) {

          // Optional: Stop ADC Continuous conversions to have more time to process (print) the data
          analogContinuousStop();

          for (int i = 0; i < adc_pins_count; i++) {
            zero_point[i] += result[i].avg_read_raw;
          }

          tare_counter++;

          // Optional: If ADC was stopped, start ADC conversions and wait for callback function to set adc_coversion_done flag to true
          analogContinuousStart();
      }
      else {
          Serial.println("Error occured during reading data. Set Core Debug Level to error or lower for more informations.");
      }
    }

    if(tare_counter >= 8)
    {
      

      for (int i = 0; i < adc_pins_count; i++) {
        zero_point[i] = zero_point[i] / 8;
      }

      tare_counter = 0;
      tare = true;
    }
  }
  else
  {
    // normal operation
    
    // Check if conversion is done and try to read data
    if (adc_coversion_done == true) {
      // Set ISR flag back to false
      adc_coversion_done = false;
      // Read data from ADC
      if (analogContinuousRead(&result, 0)) {

          // Optional: Stop ADC Continuous conversions to have more time to process (print) the data
          analogContinuousStop();

          for (int i = 0; i < adc_pins_count; i++) {
            Serial.print( result[i].avg_read_raw - zero_point[i] );
            Serial.print( ",");
          }
          Serial.print("\n");

          SteeringMessage message = SteeringMessage_init_zero;

          load_a = message.force = result[0].avg_read_raw / 10.0f;

          uint8_t buffer[64];
          pb_ostream_t ostream = pb_ostream_from_buffer(buffer, sizeof(buffer));

          pb_encode(&ostream, SteeringMessage_fields, &message);
          pjon.send(LEFT_MOTOR_ID, buffer, ostream.bytes_written);

          message = SteeringMessage_init_zero;

          load_b = message.force = result[1].avg_read_raw / 10.0f;

          ostream = pb_ostream_from_buffer(buffer, sizeof(buffer));

          pb_encode(&ostream, SteeringMessage_fields, &message);
          pjon.send(RIGHT_MOTOR_ID, buffer, ostream.bytes_written);

          // Optional: If ADC was stopped, start ADC conversions and wait for callback function to set adc_coversion_done flag to true
          analogContinuousStart();
      }
      else {
          Serial.println("Error occured during reading data. Set Core Debug Level to error or lower for more informations.");
      }      
    }
    
    delay(1);
  }


  if(iteration == 100)
  {
    wave(7, u8g2.getDisplayWidth()/2, width, width_state); 
    wave(0, 5<<2, border, border_state);
    wave(0, 6<<2, emoticons, emoticons_state);
    wave(0, 11<<4, offset, offset_state);

    iteration = 0;
  }
  
  iteration++;

  pjon.update();
  pjon.receive();
}
  

void u8g2Task(void* pvParameters)
{
  u8g2.begin();

  while(1)
  {
      u8g2_uint_t x = u8g2.getDisplayWidth()/2;
      u8g2_uint_t y = u8g2.getDisplayHeight()/2;
      char text[2] = " ";
      y += offset>>4;
      text[0] = (emoticons>>2)+48;
      u8g2.setFont(u8g2_font_battery19_tn);
      u8g2.firstPage();
      do {
        u8g2.setFont(u8g2_font_battery19_tn);
        u8g2.drawButtonUTF8(x, y, U8G2_BTN_HCENTER | (border>>2), width, 1, 1, text);
        u8g2.setFont(u8g2_font_logisoso32_tf);
        u8g2.setCursor(0, 64);
        u8g2.print(load_a);
        u8g2.setCursor(0, 32);
        u8g2.print(load_b);
      } while ( u8g2.nextPage() );
  }
}
