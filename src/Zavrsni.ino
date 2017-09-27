#include <TinyGPS++.h>
#include <GUI.h>

#include <stdio.h>
#include <ArduinoJson.h>

#include "Simboli.h"
#include "Projekcije.h"

#define degToRad(angleDegrees) (angleDegrees * M_PI / 180.0)
#define EARTH_RADIUS 6371


char map_request[350] = "GET http://maps.googleapis.com/maps/api/staticmap?center=43.856542,18.397237&zoom=12&size=320x240&format=jpg-baseline&maptype=roadmap&key=---------------HTTP/1.1\r\nHost: maps.googleapis.com\r\nUser-agent: ESP8266\r\nConnection: Close\r\n\r\n";
char aprs_request[250] = "GET http://api.aprs.fi/api/get?name=Grdonj,ER-E71ETF,Saraj.,E74OF-C&what=loc&apikey=-------------------------&format=json HTTP/1.0\r\nHost: api.aprs.fi\r\nUser-agent: ESP8266\r\nConnection: Close\r\n\r\n";

char jBuffer[1576];
unsigned char jpgFile[15000];

//Lista stanica
char stanica_1[12]  = "ER-E71ETF";
char stanica_2[12]  = "E74OF-C";
char stanica_3[12]  = "Grdonj";
char stanica_4[12]  = "Saraj.";

char *tabela[] = {stanica_1, stanica_2, stanica_3, stanica_4};

GUI gui;

TinyGPSPlus gps;

double c_lat = 43.856542;
double c_lng = 18.397237;

uint8_t ZOOM = 12;

static uint32_t json_buffer_index = 0;
static uint32_t jpg_buffer_index = 0;

static uint8_t selection_menu = 1;
static uint8_t selection_list = 0;

static bool procitano = false;
static bool pocetak_jpg = false;
static bool pocetak_json = false;
static bool request_sent = false;


enum states {STATE_1, STATE_2, STATE_3, STATE_4, STATE_5, STATE_6, MAX_STATES} current_state;
enum events {EVENT_1, EVENT_2, MAX_EVENTS} new_event;
enum tip_citanja {JPEG_DATA, JSON, NONE};
//enum json_funkcije {};

static events event_status = MAX_EVENTS;
static tip_citanja citanje = NONE;

void pocetni_s1_e1();
void pocetni_s1_e2();

void aprsSelection_s2_e1();
void aprsSelection_s2_e2();

void aprsInfo_s3_e1();
void aprsInfo_s3_e2();

void gpsInfo_s4_e1();
void gpsInfo_s4_e2();

void prikazJedneLokacije_s5_e1();
void prikazJedneLokacije_s5_e2();

void prikazSvihLokacija_s6_e1();
void prikazSvihLokacija_s6_e2();

//Tabela stanja za funkcionalnosti (korisnički interfejs)
void (*const state_table [MAX_STATES][MAX_EVENTS]) (void) = {
  { pocetni_s1_e1, pocetni_s1_e2 },
  { aprsSelection_s2_e1, aprsSelection_s2_e2 },
  { aprsInfo_s3_e1, aprsInfo_s3_e2 },
  { gpsInfo_s4_e1, gpsInfo_s4_e2 },
  { prikazJedneLokacije_s5_e1, prikazJedneLokacije_s5_e2},
  { prikazSvihLokacija_s6_e1, prikazSvihLokacija_s6_e2}
};


void sendRequest(char *http_request, char *host);
bool timeout(int timeout, long vrijemePoziva);
void jsonProcitan();
void overlay_list(const JsonObject& root);
void overlay(const JsonObject& root, uint8_t item);


int current_;         // Current state of the button
                    
long millis_held;    // How long the button was held (milliseconds)
long secs_held;      // How long the button was held (seconds)
long prev_millis_held; // How long the button was held in the previous check
byte previous = HIGH;
unsigned long firstTime; // how long since the button was first pressed 

int current1;        
                    
long millis_held1;    
long secs_held1;      
long prev_millis_held1; 
byte previous1 = HIGH;
unsigned long firstTime1;


void setup() {
  Serial.begin(115200);         //PC Serial
  Serial1.begin(115200);        //CMD Serial
  Serial2.begin(9600);          //GPS Serial
  Serial3.begin(115200);        //WIFI Serial
  
  pinMode(PUSH1, INPUT_PULLUP);
  pinMode(PUSH2, INPUT_PULLUP);

  gui.begin(PD_1, PE_1, PE_4, PE_2);

  current_state = STATE_1;
  event_status = EVENT_1;

  long current = millis();
  Serial3.print("AT+CWMODE=1\r\n");

  delay(1000);

  //AP, SSID i password
  Serial3.println("AT+CWJAP_CUR=\"SSID\",\"Password\"");

  current = millis();

  delay(3000);
  gui.DrawMainScreen(selection_menu);
}

void loop() {
  //https://playground.arduino.cc/Code/HoldButton
  current_ = digitalRead(PUSH1);
  if (current_ == LOW && previous == HIGH && (millis() - firstTime) > 200) {
    firstTime = millis();
  }

  millis_held = (millis() - firstTime);

  if (millis_held > 200) {
    if (current_ == HIGH && previous == LOW) {
      if (millis_held <= 500) {
        new_event = EVENT_1;
        switchState();
      }

      if (millis_held >= 1000 && millis_held <= 6000) {
        if(current_state == STATE_5 || current_state == STATE_6) {
            if(ZOOM > 1) {
              ZOOM--;
              setZoom(ZOOM);
              new_event = EVENT_1;
              switchState();
            }
        }
      }
    }
  }
  previous = current_;
  prev_millis_held = millis_held;


  //Drugo dugme
  current1 = digitalRead(PUSH2);
  if (current1 == LOW && previous1 == HIGH && (millis() - firstTime1) > 200) {
    firstTime1 = millis();
  }

  millis_held1 = (millis() - firstTime1);

  if (millis_held1 > 150) {
    if (current1 == HIGH && previous1 == LOW) {
      if (millis_held1 <= 500) {
          new_event = EVENT_2;
          switchState();
      }
      
      if (millis_held1 >= 1000 && millis_held1 <= 6000) {
           if(current_state == STATE_5 || current_state == STATE_6) {
            if(ZOOM < 20) {
              ZOOM++;
              setZoom(ZOOM);
              new_event = EVENT_1;
              switchState();
            }
        }
      }
    }
  }

  previous1 = current1;
  prev_millis_held1 = millis_held1;
    

}

void pocetni_s1_e1() {
  if (selection_menu >= 3)
    selection_menu = 0;
  gui.DrawMainScreen(++selection_menu);
}

void pocetni_s1_e2() {
  gui.ClearScreen();
  switch (selection_menu) {
    case 1:
      citanje = JSON;
      current_state = STATE_2;
      sendRequest(aprs_request, "api.aprs.fi");
      break;
    case 2:
      current_state = STATE_4;
      break;
    case 3:
      
      current_state = STATE_6;
      break;
    default:
      current_state = STATE_1;
      break;
  }
  new_event = EVENT_1;
  selection_menu--;
  switchState();
}


void aprsSelection_s2_e1() {
  current_state = STATE_2;
  if (selection_list >= 4)
    selection_list = 0;
  gui.DrawIndicator(310, (++selection_list)*50, YELLOW, true, BLACK);
}

void aprsSelection_s2_e2() {
  citanje = JSON;

  current_state = STATE_3;
  new_event = EVENT_1;
  delay(20);
  switchState();
}


// Ovo stanje prikazuje detaljne informacije o objektu.

void aprsInfo_s3_e1() {
  citanje = JSON;
  sendRequest(aprs_request, "api.aprs.fi");
  current_state = STATE_3;
}



void aprsInfo_s3_e2() {
  current_state = STATE_5;
  new_event = EVENT_1;
  citanje = JPEG_DATA;
  switchState();
}

void gpsInfo_s4_e1() {
  gui.DrawGPSInfo(gps);
}

void gpsInfo_s4_e2() {
  current_state = STATE_1;
  new_event = EVENT_1;
  switchState();
}


void prikazJedneLokacije_s5_e1() {
  if(gps.location.isValid() && gps.location.age() < 10000) {
    c_lat = gps.location.lat();
    c_lng = gps.location.lng();
  }
  setCenter(&c_lat,&c_lng);
  Serial.println(map_request);
  citanje = JPEG_DATA;
  sendRequest(map_request, "maps.googleapis.com");
}

void prikazJedneLokacije_s5_e2() {
  current_state = STATE_1;
  new_event = EVENT_1;
  selection_list = 0;
  switchState();
}

void prikazSvihLokacija_s6_e1() {
  if(gps.location.isValid() && gps.location.age() < 10000) {
    c_lat = gps.location.lat();
    c_lng = gps.location.lng();
  }
  setCenter(&c_lat,&c_lng);
  
  citanje = JPEG_DATA;
  sendRequest(map_request, "maps.googleapis.com");
}

void prikazSvihLokacija_s6_e2() {
  current_state = STATE_1;
  new_event = EVENT_1;
  switchState();
}

void switchState() {
  if (((new_event >= 0) && (new_event < MAX_EVENTS))
      && ((current_state >= 0) && (current_state < MAX_STATES))) {
    state_table [current_state][new_event] ();
  } else {
    gui.DisplayError("Invalid state!");
    delay(50);
    current_state = STATE_1;
    new_event = EVENT_1;
    state_table [current_state][new_event] ();
  }
}


void serialEvent1() {
  static bool prosljedjivanje = false;
  static bool ST_Pocetak = false;
  static char naziv[12];
  static uint8_t brojac = 0;
  uint8_t broj = 0;
  if(Serial.available()) {
    char c = Serial.read();
    
    if(c == 'A' && Serial.peek() == 'T') {
      prosljedjivanje = true;
    }
    
    if(prosljedjivanje) {
       Serial3.write(c);
       if(c == '\r' && Serial.peek() == '\n') {
          Serial3.write("\r\n");
          Serial.write("KRAJ AT");
          prosljedjivanje = false;
          return;
        }
    }
    
    static uint8_t state = 0;
    if (c == 'S') { state = 1; }
    else if (state == 1 && c == 'T') { state = 2; }
    else if (state == 2 && (c >= '0' && c<='9')) { state = 0; 
      ST_Pocetak = true;
      broj = c - '0';

      if(broj < 0 || broj >= 4)
        return;
        
    }
    if(ST_Pocetak) {
      if( c == '\r' || Serial.peek() == '\n') {      
         
         ST_Pocetak = false;
         brojac = 0;

          memmove(naziv, naziv+1, strlen(naziv));
          strncpy(tabela[broj], naziv,12);
         updateAPRS();
         return;
      }

      if(brojac >= 11) {
        Serial.println("Prekoracenje");
        return;
      }
      
      naziv[brojac++] = c;
      
      naziv[brojac] = '\0';
    }

  }
}

void serialEvent3() {
  char c = Serial3.read();
  Serial.write(c);
  switch (citanje) {
    case JSON: {
        static uint8_t state = 0;
        if (c == '+') { state = 1; }
        else if (state == 1 && c == 'I') { state = 2; }
        else if (state == 2 && c == 'P') { state = 3; }
        else if (state == 3 && c == 'D') { state = 4; }
        else if (state == 4 && c == ',') { state = 0;
               request_sent = true;
        }

        if (!request_sent)
          return;
        if (c == '{' && !pocetak_json)
          pocetak_json = true;

        if (pocetak_json) {
          jBuffer[json_buffer_index++] = c;
          if (jBuffer[json_buffer_index - 2] == '\r'
              && jBuffer[json_buffer_index - 1] == '\n' && !procitano) {
            procitano = true;
            request_sent = false;
            json_buffer_index = 0;
            jsonProcitan();
          }
        }

        break;
      }

    case JPEG_DATA: {
        static uint8_t state = 0;
        jpgFile[jpg_buffer_index] = c;

        if (jpgFile[jpg_buffer_index - 1] == 0xFF && jpgFile[jpg_buffer_index] == 0xD8) {
          pocetak_jpg = true;
          jpgFile[0] = 0xFF;
          jpgFile[1] = 0xD8;
          jpg_buffer_index = 1;
        }

        if (jpgFile[jpg_buffer_index] == '+') { state = 1; }
        else if (state == 1 && jpgFile[jpg_buffer_index] == 'I') { state = 2; }
        else if (state == 2 && jpgFile[jpg_buffer_index] == 'P') { state = 3; }
        else if (state == 3 && jpgFile[jpg_buffer_index] == 'D') { state = 4; }
        else if (state == 4 && jpgFile[jpg_buffer_index] == ',') {
          state = 0;
          jpg_buffer_index -= 7;
          while (Serial3.read() != ':');
        } else {
          state = 0;
        }

        if (jpg_buffer_index > 0)
          if (jpgFile[jpg_buffer_index - 1] == 0xFF && jpgFile[jpg_buffer_index] == 0xD9) {
            procitano = true;
            jpegProcitan();
          }
        if (jpg_buffer_index > 16000 && pocetak_jpg == false)
          jpg_buffer_index = 0;

        jpg_buffer_index++;
        break;
      }

    case NONE: {
        Serial.print(c);
        break;
      }
    default: {
        Serial.print("DEFAULT");
        break;
      }
  }
}

bool timeout(int timeout, long vrijemePoziva) {
  if (millis() - vrijemePoziva < timeout)
    return false;
  return true;
}

void jsonProcitan() {
   int i = strlen(jBuffer);
   while(jBuffer[i] != '}' && i > 0)
    jBuffer[i--] = '\0';
   
  StaticJsonBuffer<1576> jsonBuffer;
  JsonObject& root = jsonBuffer.parseObject(jBuffer);

  if (!root.success()) {
    gui.DisplayError("JSON Parsing problems");
    return;
  }

  uint8_t found = root["found"];

  switch (current_state) {
    case STATE_2:
      gui.DrawAprsList(root);
      break;

    case STATE_3:
      gui.DrawAprsInfo(root, selection_list-1);
      break;
    case STATE_5:
      overlay(root, selection_list-1);
      break;
    case STATE_6:
      overlay_list(root);
      break;
    default:
      break;
  }
  
  memset(jBuffer, '\0', sizeof(char) * 1576);
  json_buffer_index = 0;
  pocetak_json = false;
  request_sent = false;
  procitano = false;
}

void jpegProcitan() {
  gui.RenderImage(jpgFile, jpg_buffer_index, 0, 0);
  gui.DrawCenter(RED);
  json_buffer_index = 0;
  pocetak_json = false;
  request_sent = false;
  procitano = false;
  long current;
  memset(jBuffer, '\0', sizeof(char) * 1576);

  switch (current_state) {
    case STATE_5:
      citanje = JSON;
      delay(100);
      sendRequest(aprs_request, "api.aprs.fi");
      break;
    case STATE_6:
      citanje = JSON;
      delay(100);
      sendRequest(aprs_request, "api.aprs.fi");
      break;
  }
  jpg_buffer_index = 0;
}

void overlay_list(const JsonObject& root) {
  double dst = 0;
  double dst_obj = 0;
  for (uint8_t i = 0; i < root["found"]; i++) {
    double object_lat = root["entries"][i]["lat"];
    double object_lng = root["entries"][i]["lng"];
    gui.DrawSymbol(ZOOM, object_lat, object_lng, c_lat, c_lng, root["entries"][i]["symbol"], root["entries"][i]["name"]);
  }
}

void overlay(const JsonObject& root, uint8_t item) {
  double object_lat = root["entries"][item]["lat"];
  double object_lng = root["entries"][item]["lng"];
  gui.DrawSymbol(ZOOM, object_lat, object_lng, c_lat, c_lng, root["entries"][item]["symbol"], root["entries"][item]["name"]);
}

void sendRequest(char *http_request, char *host) {
  Serial3.print("AT+CIPCLOSE\r\n");
  delay(150);
  Serial3.print("AT+CIPSTART=\"TCP\",\"");
  Serial3.print(host);
  Serial3.print("\",80\r\n");
  delay(450);
  Serial3.print("AT+CIPSEND=");
  delay(150);
  Serial3.print(strlen(http_request));
  Serial.println(http_request);
  Serial3.print("\r\n");
  delay(50);
  Serial3.print(http_request);
  delay(15);
}

//GPS
void serialEvent2() {
  while (Serial2.available())
    gps.encode(Serial2.read());
}


void updateAPRS() {
    uint16_t aprs_request_len = strlen(aprs_request);
    uint16_t buff_len = 0;

    uint16_t i=0, j = 0, k = 0;
    while (aprs_request[++i] != '?');

    i++;
    i+=5;
    j=i;

    while (aprs_request[++j] != '&');

    uint16_t diff = j-i;

    for(k = 0; k < 4; k++) {
        char *tmp = tabela[k];
        while(*tmp != '\0') {
            buff_len++;
            tmp++;
        }
    }

    buff_len += 3;  //broj zareza

    if(diff < buff_len) {
        for(k = strlen(aprs_request); k >= j; k--)
           aprs_request[k+buff_len-(j-i)] = aprs_request[k];
    }
    else if(diff > buff_len) {
           memmove(aprs_request+j-(diff-buff_len),aprs_request+j,strlen(aprs_request)-j+1);
    }

    for(k = 0; k < 4; k++) {
        for(j = 0; j < strlen(tabela[k]); j++) {
            aprs_request[i++] = tabela[k][j];
        }
        if(k != 3)
            aprs_request[i++] = ',';
    }
}

void setZoom(uint8_t zoom) {
    if(zoom > 20)
        return;

    char *zoom_str = strstr(map_request,"zoom");
    if(zoom_str == NULL)
        return;

    while(*zoom_str != '=')
        zoom_str++;
    zoom_str++;

    uint8_t trenutni_broj_cifara = 0;
    while(*zoom_str >= '0' && *zoom_str <= '9') {
        trenutni_broj_cifara++;
        zoom_str++;
    }

    zoom_str--;

    if((trenutni_broj_cifara == 1 && zoom >= 10)) {
        char *pomocni = zoom_str+1;
        while(*(pomocni++) != '\0');

        while(pomocni != zoom_str) {
            *(pomocni+1) = *(pomocni);
            pomocni--;
        }
        zoom_str++;
    } else if((trenutni_broj_cifara == 2 && zoom < 10)) {
        char *pomocni = zoom_str;
        while(*(pomocni) != '\0') {
            *(pomocni) = *(pomocni+1);
            pomocni++;
        }
        zoom_str--;
    }

    while(zoom != 0) {
            *zoom_str = (zoom%10) + '0';
            zoom_str--;
            zoom /= 10;
    }
}

void setCenter(double *lat, double *lng) {
    if(*lat > 85.05113 || *lat < -85.05133 || *lng > 180.0 || *lng < -180.0)
        return;

    char buff[32];
    snprintf(buff,31,"center=%.6f,%.6f",*lat,*lng);

    uint16_t map_request_len = strlen(map_request);
    uint16_t buff_len = strlen(buff);

    uint16_t i=0, j = 0, k = 0;
    while (map_request[++i] != '?');

    i++;
    j=i;

    while (map_request[++j] != '&');

    uint16_t diff = j-i;

    if(diff < buff_len) {
        for(k = strlen(map_request); k >= j; k--)
           map_request[k+buff_len-(j-i)] = map_request[k];
    }
    else if(diff > buff_len) {
        for(k = j-1; k < strlen(map_request); k++)
            map_request[k] = map_request[k+1];
    }
    memcpy(map_request+i,buff,buff_len);
}
