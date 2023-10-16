#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <openevse.h>
#include "Adafruit_GFX.h"
#include "Adafruit_RA8875.h"
#include "MFRC522_I2C.h"
#include "SoftwareSerial.h"
#include <SD.h>
#include "esp_task_wdt.h"
#include "buzzer.h"
#include "Font.h"
#include <U8g2_for_Adafruit_GFX.h>


/* RA8875 SPI */
#define SPI_SCK 11
#define SPI_MISO 10
#define SPI_MOSI 9

// LCD
#define LCD_RST 7
#define LCD_CS 15
#define LCD_INT 16

// ETH
#define ETH_RST 14
#define ETH_CS 12
#define ETH_INT 13

// SDCard
#define MSD_CS 4

// Beeper
#define BEEPER 2
#define RESOLUTION 8
#define PWMCHANNEL 0

// LED
#define LED1 40
#define LED2 41
#define LED3 42

/* RC522 & RC532 */
#define RFID_SDA 38
#define RFID_SCL 39

/* UART to ATmega32*/
#define ATMEGA32_RX 18
#define ATMEGA32_TX 17
#define ATMEGA32_BAUD 115200

/*EMBUTTOM*/
#define BUTTON_1 8
#define BUTTON_2 19
#define BUTTON_3 20 

#define DEBUG_PORT Serial
#define RAPI_PORT Serial1

#define POLL_TIME (5 * 1000)

#define EMERGENCY_STOP 20
#define SCHEDULED_CHARGING 21

EspSoftwareSerial::UART swSer1;
RapiSender rapiSender(&RAPI_PORT);

uint32_t next_status = 0;
String uuid="";
Adafruit_RA8875 tft = Adafruit_RA8875(LCD_CS, LCD_RST);
U8G2_FOR_ADAFRUIT_GFX u8g2_for_adafruit_gfx;
MFRC522 mfrc522(0x28, 0x28);
File RFID_DATA;
Buzzer buzzer = Buzzer(BEEPER,PWMCHANNEL,RESOLUTION);
static int lcd_state;
static int layer = 0;

void memorytolayer(int L){
  if(L == 0){
    tft.writeCommand(0x41);
    // Set bit 1
    tft.writeData(0x00);
  }
  else{
    tft.writeCommand(0x41);
    // Set bit 1
    tft.writeData(0x01);
  }
  delay(17);
}

void displaytolayer(int L){
  if(L == 0){
    tft.writeCommand(0x52);
    // Set bit 1
    tft.writeData(0x00);
  }
  else  {
    tft.writeCommand(0x52);
    // Set bit 1
    tft.writeData(0x01);
  }
  delay(17);
}

void LCD_display(int state){

  if(lcd_state!=state){
  memorytolayer(layer);
  lcd_state = state;
  tft.fillScreen(RA8875_BLACK);
  }
  else{
    return;
  }
  
  switch (state)
  {
  case OPENEVSE_STATE_STARTING:
    /*開機*/
    u8g2_for_adafruit_gfx.setFontMode(2);                 
    u8g2_for_adafruit_gfx.setFontDirection(0);           
    u8g2_for_adafruit_gfx.setForegroundColor(RA8875_WHITE);      
    u8g2_for_adafruit_gfx.setFont(evse_text1_40);  
    u8g2_for_adafruit_gfx.setCursor(180,145);                
    u8g2_for_adafruit_gfx.print(F("開機中"));     
    break;

  case OPENEVSE_STATE_NOT_CONNECTED:
    /*未檢測到車輛*/
    u8g2_for_adafruit_gfx.setFontMode(2);                 
    u8g2_for_adafruit_gfx.setFontDirection(0);            
    u8g2_for_adafruit_gfx.setForegroundColor(RA8875_WHITE);
    u8g2_for_adafruit_gfx.setFont(evse_text1_40);   
    u8g2_for_adafruit_gfx.setCursor(0,40);                
    u8g2_for_adafruit_gfx.print(F("社區型智慧充電樁"));
    u8g2_for_adafruit_gfx.setFont(evse_text1_30);  
    u8g2_for_adafruit_gfx.setCursor(0,90);                
    u8g2_for_adafruit_gfx.print(F("未偵測到電動車"));
    u8g2_for_adafruit_gfx.setCursor(240,240);
    u8g2_for_adafruit_gfx.print(F("車位:"));
    break;

  case OPENEVSE_STATE_CONNECTED:
    /*檢測到車輛*/
    u8g2_for_adafruit_gfx.setFontMode(2);                 
    u8g2_for_adafruit_gfx.setFontDirection(0);            
    u8g2_for_adafruit_gfx.setForegroundColor(RA8875_WHITE);     
    u8g2_for_adafruit_gfx.setFont(evse_text1_40);  
    u8g2_for_adafruit_gfx.setCursor(0,40);               
    u8g2_for_adafruit_gfx.print(F("社區型智慧充電樁"));
    u8g2_for_adafruit_gfx.setFont(evse_text1_30); 
    u8g2_for_adafruit_gfx.setCursor(0,90);                
    u8g2_for_adafruit_gfx.print(F("偵測到電動車"));
    u8g2_for_adafruit_gfx.setCursor(240,240);
    u8g2_for_adafruit_gfx.print(F("車位:"));
    break;

  case OPENEVSE_STATE_CHARGING:
    /*預約充電模式充電中*/
    u8g2_for_adafruit_gfx.setFontMode(2);                 
    u8g2_for_adafruit_gfx.setFontDirection(0);            
    u8g2_for_adafruit_gfx.setForegroundColor(RA8875_WHITE);      
    u8g2_for_adafruit_gfx.setFont(evse_text1_40);  
    u8g2_for_adafruit_gfx.setCursor(0,40);                
    u8g2_for_adafruit_gfx.print(F("社區型智慧充電樁"));
    u8g2_for_adafruit_gfx.setFont(evse_text1_30);  
    u8g2_for_adafruit_gfx.setCursor(0,90);                
    u8g2_for_adafruit_gfx.print(F("偵測到電動車\n充電模式充電中"));
    u8g2_for_adafruit_gfx.setCursor(240,240);
    u8g2_for_adafruit_gfx.print(F("車位:"));
    break;
  
  case OPENEVSE_STATE_VENT_REQUIRED:
    /*啟動車輛需要空調保護*/
    u8g2_for_adafruit_gfx.setFontMode(2);                 
    u8g2_for_adafruit_gfx.setFontDirection(0);            
    u8g2_for_adafruit_gfx.setForegroundColor(RA8875_WHITE);     
    u8g2_for_adafruit_gfx.setFont(evse_text1_40);  
    u8g2_for_adafruit_gfx.setCursor(0,40);               
    u8g2_for_adafruit_gfx.print(F("社區型智慧充電樁"));
    u8g2_for_adafruit_gfx.setFont(evse_text1_30);  
    u8g2_for_adafruit_gfx.setCursor(0,90);                
    u8g2_for_adafruit_gfx.print(F("停止充電!!/n錯誤:車輛需要空調"));
    u8g2_for_adafruit_gfx.setCursor(240,240);
    u8g2_for_adafruit_gfx.print(F("車位:"));
    break;

  case OPENEVSE_STATE_DIODE_CHECK_FAILED:
    /*二極體測試錯誤*/
    u8g2_for_adafruit_gfx.setFontMode(2);                 
    u8g2_for_adafruit_gfx.setFontDirection(0);            
    u8g2_for_adafruit_gfx.setForegroundColor(RA8875_WHITE);     
    u8g2_for_adafruit_gfx.setFont(evse_text1_40);  
    u8g2_for_adafruit_gfx.setCursor(0,40);               
    u8g2_for_adafruit_gfx.print(F("社區型智慧充電樁"));
    u8g2_for_adafruit_gfx.setFont(evse_text1_30);  
    u8g2_for_adafruit_gfx.setCursor(0,90);                
    u8g2_for_adafruit_gfx.print(F("錯誤:二極體測試錯誤"));
    u8g2_for_adafruit_gfx.setCursor(240,240);
    u8g2_for_adafruit_gfx.print(F("車位:"));
    break;

  case OPENEVSE_STATE_GFI_FAULT:
    /*啟動GFCI漏電保護*/
    u8g2_for_adafruit_gfx.setFontMode(2);                 
    u8g2_for_adafruit_gfx.setFontDirection(0);            
    u8g2_for_adafruit_gfx.setForegroundColor(RA8875_WHITE);     
    u8g2_for_adafruit_gfx.setFont(evse_text1_40);  
    u8g2_for_adafruit_gfx.setCursor(0,40);               
    u8g2_for_adafruit_gfx.print(F("社區型智慧充電樁"));
    u8g2_for_adafruit_gfx.setFont(evse_text1_30);  
    u8g2_for_adafruit_gfx.setCursor(0,90);                
    u8g2_for_adafruit_gfx.print(F("錯誤:漏電產生/n啟動GFCI漏電保護"));
    u8g2_for_adafruit_gfx.setCursor(240,240);
    u8g2_for_adafruit_gfx.print(F("車位:"));
    break;

  case OPENEVSE_STATE_NO_EARTH_GROUND:
    /*充電樁未接地錯誤*/
    u8g2_for_adafruit_gfx.setFontMode(2);                 
    u8g2_for_adafruit_gfx.setFontDirection(0);            
    u8g2_for_adafruit_gfx.setForegroundColor(RA8875_WHITE);     
    u8g2_for_adafruit_gfx.setFont(evse_text1_40);  
    u8g2_for_adafruit_gfx.setCursor(0,40);               
    u8g2_for_adafruit_gfx.print(F("社區型智慧充電樁"));
    u8g2_for_adafruit_gfx.setFont(evse_text1_30);  
    u8g2_for_adafruit_gfx.setCursor(0,90);                
    u8g2_for_adafruit_gfx.print(F("錯誤:未接地錯誤"));
    u8g2_for_adafruit_gfx.setCursor(240,240);
    u8g2_for_adafruit_gfx.print(F("車位:"));
    break;

  case OPENEVSE_STATE_STUCK_RELAY:
    /*繼電器測試錯誤*/
    u8g2_for_adafruit_gfx.setFontMode(2);                 
    u8g2_for_adafruit_gfx.setFontDirection(0);            
    u8g2_for_adafruit_gfx.setForegroundColor(RA8875_WHITE);     
    u8g2_for_adafruit_gfx.setFont(evse_text1_40);  
    u8g2_for_adafruit_gfx.setCursor(0,40);               
    u8g2_for_adafruit_gfx.print(F("社區型智慧充電樁"));
    u8g2_for_adafruit_gfx.setFont(evse_text1_30);  
    u8g2_for_adafruit_gfx.setCursor(0,90);                
    u8g2_for_adafruit_gfx.print(F("錯誤:繼電器測試錯誤"));
    u8g2_for_adafruit_gfx.setCursor(240,240);
    u8g2_for_adafruit_gfx.print(F("車位:"));
    break;

  case OPENEVSE_STATE_GFI_SELF_TEST_FAILED:
    /*GFCI漏電保護自檢錯誤*/
    u8g2_for_adafruit_gfx.setFontMode(2);                 
    u8g2_for_adafruit_gfx.setFontDirection(0);            
    u8g2_for_adafruit_gfx.setForegroundColor(RA8875_WHITE);     
    u8g2_for_adafruit_gfx.setFont(evse_text1_40);  
    u8g2_for_adafruit_gfx.setCursor(0,40);               
    u8g2_for_adafruit_gfx.print(F("社區型智慧充電樁"));
    u8g2_for_adafruit_gfx.setFont(evse_text1_30);  
    u8g2_for_adafruit_gfx.setCursor(0,90);                
    u8g2_for_adafruit_gfx.print(F("錯誤:GFCI漏電保護自我測試錯誤"));
    u8g2_for_adafruit_gfx.setCursor(240,240);
    u8g2_for_adafruit_gfx.print(F("車位:"));
    break;

  case OPENEVSE_STATE_OVER_TEMPERATURE:
    /*啟動溫度保護*/
    u8g2_for_adafruit_gfx.setFontMode(2);                 
    u8g2_for_adafruit_gfx.setFontDirection(0);            
    u8g2_for_adafruit_gfx.setForegroundColor(RA8875_WHITE);     
    u8g2_for_adafruit_gfx.setFont(evse_text1_40);  
    u8g2_for_adafruit_gfx.setCursor(0,40);               
    u8g2_for_adafruit_gfx.print(F("社區型智慧充電樁"));
    u8g2_for_adafruit_gfx.setFont(evse_text1_30);  
    u8g2_for_adafruit_gfx.setCursor(0,90);                
    u8g2_for_adafruit_gfx.print(F("錯誤:溫度過高,停止充電!!"));
    u8g2_for_adafruit_gfx.setCursor(240,240);
    u8g2_for_adafruit_gfx.print(F("車位:"));
    break;

  case OPENEVSE_STATE_OVER_CURRENT:
    /*啟動電流保護*/
    u8g2_for_adafruit_gfx.setFontMode(2);                 
    u8g2_for_adafruit_gfx.setFontDirection(0);            
    u8g2_for_adafruit_gfx.setForegroundColor(RA8875_WHITE);     
    u8g2_for_adafruit_gfx.setFont(evse_text1_40);  
    u8g2_for_adafruit_gfx.setCursor(0,40);               
    u8g2_for_adafruit_gfx.print(F("社區型智慧充電樁"));
    u8g2_for_adafruit_gfx.setFont(evse_text1_30);  
    u8g2_for_adafruit_gfx.setCursor(0,90);                
    u8g2_for_adafruit_gfx.print(F("錯誤:充電電流過高,停止充電!!"));
    u8g2_for_adafruit_gfx.setCursor(240,240);
    u8g2_for_adafruit_gfx.print(F("車位:"));
    break;

   case EMERGENCY_STOP:
    /*緊急停止*/
    u8g2_for_adafruit_gfx.setFontMode(2);                
    u8g2_for_adafruit_gfx.setFontDirection(0);            
    u8g2_for_adafruit_gfx.setForegroundColor(RA8875_WHITE);      
    u8g2_for_adafruit_gfx.setFont(evse_text1_40);  
    u8g2_for_adafruit_gfx.setCursor(160,145);              
    u8g2_for_adafruit_gfx.print(F("緊急停止!!"));
    break;

  case SCHEDULED_CHARGING:
    /*預約充電模式等待中*/
    u8g2_for_adafruit_gfx.setFontMode(2);                 
    u8g2_for_adafruit_gfx.setFontDirection(0);            
    u8g2_for_adafruit_gfx.setForegroundColor(RA8875_WHITE);     
    u8g2_for_adafruit_gfx.setFont(evse_text1_40);  
    u8g2_for_adafruit_gfx.setCursor(0,40);               
    u8g2_for_adafruit_gfx.print(F("社區型智慧充電樁"));
    u8g2_for_adafruit_gfx.setFont(evse_text1_30);  
    u8g2_for_adafruit_gfx.setCursor(0,90);                
    u8g2_for_adafruit_gfx.print(F("偵測到電動車\n執行預約充電模式\n再感應一次卡片停止充電"));
    u8g2_for_adafruit_gfx.setCursor(240,240);
    u8g2_for_adafruit_gfx.print(F("車位:"));
    break;

  case OPENEVSE_STATE_SLEEPING:
    /*待機睡眠模式*/
    u8g2_for_adafruit_gfx.setFontMode(2);                
    u8g2_for_adafruit_gfx.setFontDirection(0);            
    u8g2_for_adafruit_gfx.setForegroundColor(RA8875_WHITE);      
    u8g2_for_adafruit_gfx.setFont(evse_text1_40);  
    u8g2_for_adafruit_gfx.setCursor(160,145);              
    u8g2_for_adafruit_gfx.print(F("待機模式"));
    break;

  case OPENEVSE_STATE_DISABLED:
    /*等待刷卡開機*/
    u8g2_for_adafruit_gfx.setFontMode(2);                 
    u8g2_for_adafruit_gfx.setFontDirection(0);            
    u8g2_for_adafruit_gfx.setForegroundColor(RA8875_WHITE);     
    u8g2_for_adafruit_gfx.setFont(evse_text1_40);  
    u8g2_for_adafruit_gfx.setCursor(0,40);               
    u8g2_for_adafruit_gfx.print(F("社區型智慧充電樁"));
    u8g2_for_adafruit_gfx.setFont(evse_text1_30);  
    u8g2_for_adafruit_gfx.setCursor(0,90);                
    u8g2_for_adafruit_gfx.print(F("請感應一次卡片，開啟充電模式"));
    u8g2_for_adafruit_gfx.setCursor(240,240);
    u8g2_for_adafruit_gfx.print(F("車位:"));
    break;

  default:
    
    break;
  }
  displaytolayer(layer);
  layer ^= 1;
}

void IRAM_ATTR EM() {
    
    rapiSender.sendCmd("$FD");
    DEBUG_PORT.println("SHUT DOWN EVSE!!!!");
    LCD_display(EMERGENCY_STOP);
    delay(1000);
    LCD_display(OPENEVSE_STATE_DISABLED);
}


void setup() {
  // put your setup code here, to run once:
  
  Serial.begin(115200);
  Serial1.begin(ATMEGA32_BAUD, SERIAL_8N1, ATMEGA32_RX, ATMEGA32_TX);
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);

  while (!Serial && millis() < 5000);
  delay(500);

  log_e("[Setup] EVSE-S3 Setup Phase");

  pinMode(BUTTON_1, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(BUTTON_1), EM, RISING);

  pinMode(LED1,OUTPUT);
  pinMode(LED2,OUTPUT);
  pinMode(LED3,OUTPUT);
  digitalWrite(LED1,HIGH);
  digitalWrite(LED2,HIGH);
  digitalWrite(LED3,HIGH);

  buzzer.begin();

  Wire.begin(RFID_SDA, RFID_SCL);
  
  // i2c Scan
  int count = 0;
  log_e("[Setup] Starting I2C scan:");
  for (byte i = 0; i < 128; i++)
  {
    Wire.beginTransmission(i);       // Begin I2C transmission Address (i)
    byte error = Wire.endTransmission();
    if (error == 0) // Receive 0 = success (ACK response)
    {
      log_e("[Setup] \tFound address: %d", i);
      count++; 
    }
  }
  log_e("[Setup] \tFound %d devices(s).", count);


  log_e("[Setup] Init ATmega328p UART");
  Serial1.begin(115200);
  rapiSender.setStream(&Serial1);

  log_e("[Setup] Init RFID Reader");
  mfrc522.PCD_Init();

  log_e("[Setup] Looking up RA8875");
  /* Initialize the display using 'RA8875_480x80', 'RA8875_480x128', 'RA8875_480x272' or 'RA8875_800x480' */
  if (!tft.begin(RA8875_480x272)) {
    log_e("[Setup] RA8875 Not Found: Freezed");
  } else {
    log_e("[Setup] Found RA8875");
  }
  u8g2_for_adafruit_gfx.begin(tft); 

  tft.displayOn(true);
  tft.GPIOX(true);      // Enable TFT - display enable tied to GPIOX
  tft.PWM1config(true, RA8875_PWM_CLK_DIV1024); // PWM output for backlight
  tft.PWM1out(255);
  tft.fillScreen(RA8875_BLACK);

  tft.textMode();
  LCD_display(OPENEVSE_STATE_STARTING);

  digitalWrite(LED1,LOW);
  digitalWrite(LED2,LOW);
  
  if (!SD.begin(MSD_CS)) {
    Serial.println("Card initialization failed!");
    return;
  }
  Serial.println("Card initialized successfully.");

  // 創建RFID文件
  RFID_DATA=SD.open("/RFID.txt");
  RFID_DATA.close();

  /* W5500 */
  //Ethernet.init();
  log_e("[Setup] EVSE-S3 Setup Phase: Finish");
  
}
char c2h(char c)
{  
  return "0123456789ABCDEF"[0x0F & (unsigned char)c];
}
const char *get_state_name(uint8_t state)
{
  const char *estate = "";
  switch (state) {
    case OPENEVSE_STATE_STARTING:
      estate = "Starting";
      break;
    case OPENEVSE_STATE_NOT_CONNECTED:
      estate = "Not Connected";
      break;
    case OPENEVSE_STATE_CONNECTED:
      estate = "EV Connected";
      break;
    case OPENEVSE_STATE_CHARGING:
      estate = "Charging";
      break;
    case OPENEVSE_STATE_VENT_REQUIRED:
      estate = "Vent Required";
      break;
    case OPENEVSE_STATE_DIODE_CHECK_FAILED:
      estate = "Diode Check Failed";
      break;
    case OPENEVSE_STATE_GFI_FAULT:
      estate = "GFCI Fault";
      break;
    case OPENEVSE_STATE_NO_EARTH_GROUND:
      estate = "No Earth Ground";
      break;
    case OPENEVSE_STATE_STUCK_RELAY:
      estate = "Stuck Relay";
      break;
    case OPENEVSE_STATE_GFI_SELF_TEST_FAILED:
      estate = "GFCI Self Test Failed";
      break;
    case OPENEVSE_STATE_OVER_TEMPERATURE:
      estate = "Over Temperature";
      break;
    case OPENEVSE_STATE_OVER_CURRENT:
      estate = "Over Current";
      break;
    case OPENEVSE_STATE_SLEEPING:
      estate = "Sleeping";
      break;
    case OPENEVSE_STATE_DISABLED:
      estate = "Disabled";
      break;
    default:
      estate = "Invalid";
      break;

  }

  return estate;
}
void loop() {
  // put your main code here, to run repeatedly:

  rapiSender.loop();

  if(millis() > next_status)
  {
    next_status = millis() + POLL_TIME;
    if(OpenEVSE.isConnected())
    {
      if (uuid.length() == 0) {
        rapiSender.sendCmd("$FD");
      } else {
        rapiSender.sendCmd("$FE");
      }
      OpenEVSE.getStatus([](int ret, uint8_t evse_state, uint32_t session_time, uint8_t pilot_state, uint32_t vflags)
      {
        if(RAPI_RESPONSE_OK == ret)
        {
          DEBUG_PORT.printf("evse_state = %02x, session_time = %d, pilot_state = %02x, vflags = %08x\n", evse_state, session_time, pilot_state, vflags);
          DEBUG_PORT.printf("EVSE state: %s\n", get_state_name(evse_state));
          DEBUG_PORT.printf("Pilot state: %s\n", get_state_name(pilot_state));
          // tft.textSetCursor(50, 150);
          // tft.textEnlarge(1);
          // tft.textWrite(get_state_name(evse_state));
          LCD_display(evse_state);
          if(evse_state==OPENEVSE_STATE_CHARGING){
            digitalWrite(LED2,HIGH);
          }
          else{
            digitalWrite(LED2,LOW);
          }
        }
      });

      OpenEVSE.getChargeCurrentAndVoltage([](int ret, double amps, double volts) {
        if (RAPI_RESPONSE_OK == ret) {

          // tft.textTransparent(RA8875_WHITE);
          // tft.textColor(RA8875_RED, RA8875_WHITE);

          // char* volStr = "";
          // sprintf(volStr, "Voltage: %f V", volts);
          // log_e("Display to: %s", volStr);
          // tft.textSetCursor(50, 50);
          // tft.textEnlarge(1);
          // tft.textWrite(volStr);

          // char* curStr = "";
          // sprintf(curStr, "Current: %f A", amps);
          // log_e("Display to: %s", curStr);
          // tft.textSetCursor(50, 100);
          // tft.textEnlarge(1);
          // tft.textWrite(curStr);
        }
      });
    }
    else
    {
      OpenEVSE.begin(rapiSender, [](bool connected)
      {
        if(connected)
        {
          DEBUG_PORT.printf("Connected to OpenEVSE\n");
        } else {
          DEBUG_PORT.println("OpenEVSE not responding or not connected");
        }
      });
    }
  }
  


  if ( !mfrc522.PICC_IsNewCardPresent() || !mfrc522.PICC_ReadCardSerial() ) {
    delay(50);
    return;
  }
  uuid = "";
  
  for (byte i = 0; i < mfrc522.uid.size; i++) {
    
    log_e("i=%d, byte=%d", i, mfrc522.uid.uidByte[i]);
    char sec[3] ={0};
    sprintf(sec, "%02X", mfrc522.uid.uidByte[i]);
    sec[2]=0;
    log_e("SEC: %s", sec);

    uuid.concat(sec);
  }

  if(uuid=="15F3A83C"){
    buzzer.Success();
  }
  else{
    buzzer.Fail();
  }
  

  //Save RFID data to SD Card
  // RFID_DATA=SD.open("/RFID.txt", FILE_WRITE);
  // RFID_DATA.println(uuid);
  // RFID_DATA.close();
  // RFID_DATA=SD.open("/RFID.txt");
  // log_e("SD_CARD text:");
  // while (RFID_DATA.available()) {
  //     Serial.write(RFID_DATA.read());
      
  //   }
  // RFID_DATA.close();

  //Display RFID on LCD  
  log_e("Card ID: %s", uuid);
  u8g2_for_adafruit_gfx.setFontMode(2);                 
  u8g2_for_adafruit_gfx.setFontDirection(0);            
  u8g2_for_adafruit_gfx.setForegroundColor(RA8875_WHITE);      
  u8g2_for_adafruit_gfx.setFont(evse_text1_30);
  u8g2_for_adafruit_gfx.setCursor(0,230);                
  u8g2_for_adafruit_gfx.print(F("卡號:"));
  tft.textSetCursor(80, 220);
  tft.textTransparent(RA8875_WHITE);
  tft.textColor(RA8875_WHITE, RA8875_BLACK);
  tft.textEnlarge(1);
  tft.textWrite(uuid.c_str());
  uuid = " ";
  vTaskDelay(2000);
  
}
