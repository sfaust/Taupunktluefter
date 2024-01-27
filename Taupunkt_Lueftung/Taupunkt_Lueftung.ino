// Dieser Code benötigt zwingend die folgenden Libraries:
#include "DHT.h"
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <esp_task_wdt.h>

#define WDT_TIMEOUT 8
#define RELAIPIN 6 // Anschluss des Lüfter-Relais
#define DHTPIN_1 5 // Datenleitung für den DHT-Sensor 1 (innen)
#define DHTPIN_2 4 // Datenleitung für den DHT-Sensor 2 (außen)

#define RELAIS_EIN LOW
#define RELAIS_AUS HIGH
bool rel;

#define DHTTYPE_1 DHT22 // DHT 22 
#define DHTTYPE_2 DHT22 // DHT 22  

// *******  Korrekturwerte der einzelnen Sensorwerte  *******
#define Korrektur_t_1  -3 // Korrekturwert Innensensor Temperatur
#define Korrektur_t_2  -4 // Korrekturwert Außensensor Temperatur
#define Korrektur_h_1  0  // Korrekturwert Innensensor Luftfeuchtigkeit
#define Korrektur_h_2  0  // Korrekturwert Außensensor Luftfeuchtigkeit
//***********************************************************

#define SCHALTmin   5.0 // minimaler Taupunktunterschied, bei dem das Relais schaltet
#define HYSTERESE   1.0 // Abstand von Ein- und Ausschaltpunkt
#define TEMP1_min  10.0 // Minimale Innentemperatur, bei der die Lüftung aktiviert wird
#define TEMP2_min -10.0 // Minimale Außentemperatur, bei der die Lüftung aktiviert wird

DHT dhti(DHTPIN_1, DHTTYPE_1); //Der Innensensor wird ab jetzt mit dhti angesprochen
DHT dhta(DHTPIN_2, DHTTYPE_2); //Der Außensensor wird ab jetzt mit dhta angesprochen

LiquidCrystal_I2C lcd(0x27,20,4); // LCD: I2C-Addresse und Displaygröße setzen

bool fehler = true;

void setup() {
  esp_task_wdt_init(WDT_TIMEOUT, true); //enable panic so ESP32 restarts
  esp_task_wdt_add(NULL); //add current thread to WDT watch
  
  pinMode(RELAIPIN, OUTPUT);          // Relaispin als Output definieren
  digitalWrite(RELAIPIN, RELAIS_AUS); // Relais ausschalten
  
  Serial.begin(9600);  // Serielle Ausgabe, falls noch kein LCD angeschlossen ist
  Serial.println(F("Teste Sensoren.."));

  lcd.init();
  lcd.backlight();                      
  lcd.setCursor(2,0);
  lcd.print(F("Teste Sensoren.."));
  
  byte Grad[8] = {B00111,B00101,B00111,B0000,B00000,B00000,B00000,B00000};      // Sonderzeichen ° definieren
  lcd.createChar(0, Grad);
  byte Strich[8] = {B00100,B00100,B00100,B00100,B00100,B00100,B00100,B00100};   // Sonderzeichen senkrechter Strich definieren
  lcd.createChar(1, Strich);
    
  dhti.begin(); // Sensoren starten
  dhta.begin();   
}

void loop() {

  float hi = dhti.readHumidity()+Korrektur_h_1;       // Innenluftfeuchtigkeit auslesen und unter „hi“ speichern
  float ti = dhti.readTemperature()+ Korrektur_t_1;   // Innentemperatur auslesen und unter ti speichern
  float ha = dhta.readHumidity()+Korrektur_h_2;       // Außenluftfeuchtigkeit auslesen und unter ha speichern
  float ta = dhta.readTemperature()+ Korrektur_t_2;   // Außentemperatur auslesen und unter ta speichern
  
  if (fehler == true)  // Prüfen, ob gültige Werte von den Sensoren kommen
  {
    fehler = false; 
    if (isnan(hi) || isnan(ti) || hi > 100 || hi < 1 || ti < -40 || ti > 80 )  {
      Serial.println(F("Fehler beim Auslesen vom 1. Sensor!"));
      lcd.setCursor(0,1);
      lcd.print(F("Fehler Sensor I"));
      fehler = true;
    }else {
     lcd.setCursor(0,1);
     lcd.print(F("Sensor I in Ordnung"));
   }
  
    delay(2000);  // Zeit um das Display zu lesen
  
      if (isnan(ha) || isnan(ta) || ha > 100 || ha < 1 || ta < -40 || ta  > 80)  {
        Serial.println(F("Fehler beim Auslesen vom 2. Sensor!"));
        lcd.setCursor(0,2);
        lcd.print(F("Fehler Sensor A"));
        fehler = true;
      } else {
        lcd.setCursor(0,2);
        lcd.print(F("Sensor A in Ordnung"));
     }

    delay(2000);  // Zeit um das Display zu lesen
  }
  if (isnan(hi) || isnan(ti) || isnan(ha) || isnan(ta)) fehler = true;
   
 if (fehler == true) {
    digitalWrite(RELAIPIN, RELAIS_AUS); // Relais ausschalten 
    lcd.setCursor(0,3);
    lcd.print(F("CPU Neustart....."));
    while (1);  // Endlosschleife um das Display zu lesen und die CPU durch den Watchdog neu zu starten
 }
 esp_task_wdt_reset();  // Watchdog zurücksetzen

//**** Taupunkte errechnen********
float Taupunkt_i = taupunkt(ti, hi);
float Taupunkt_a = taupunkt(ta, ha);

// Werteausgabe auf Serial Monitor
 Serial.print(F("Innen: " ));
  Serial.print(F("Luftfeuchtigkeit: "));
  Serial.print(hi);                     
  Serial.print(F("%  Temperatur: "));
  Serial.print(ti);
  Serial.print(F("°C  "));
  Serial.print(F("  Taupunkt: "));
  Serial.print(Taupunkt_i);
  Serial.println(F("°C  "));

  Serial.print("Aussen: " );
  Serial.print(F("Luftfeuchtigkeit: "));
  Serial.print(ha);
  Serial.print(F("%  Temperatur: "));
  Serial.print(ta);
  Serial.print(F("°C "));
  Serial.print(F("   Taupunkt: "));
  Serial.print(Taupunkt_a);
  Serial.println(F("°C  "));


 Serial.println();

  // Werteausgabe auf dem I2C-Display
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(F("SI: "));
  lcd.print(ti); 
  lcd.write((uint8_t)0); // Sonderzeichen °C
  lcd.write(('C'));
  lcd.write((uint8_t)1); // Sonderzeichen |
  lcd.print(hi);
  lcd.print(F(" %"));

  lcd.setCursor(0,1);
  lcd.print(F("SA: "));
  lcd.print(ta); 
  lcd.write((uint8_t)0); // Sonderzeichen °C
  lcd.write(('C'));
  lcd.write((uint8_t)1); // Sonderzeichen |
  lcd.print(ha);
  lcd.print(F(" %"));

  lcd.setCursor(0,2);
  lcd.print(F("Taupunkt I: "));
  lcd.print(Taupunkt_i); 
  lcd.write((uint8_t)0); // Sonderzeichen °C
  lcd.write(('C'));

  lcd.setCursor(0,3);
  lcd.print(F("Taupunkt A: "));
  lcd.print(Taupunkt_a); 
  lcd.write((uint8_t)0); // Sonderzeichen °C
  lcd.write(('C'));

delay(6000); // Zeit um das Display zu lesen
esp_task_wdt_reset(); // Watchdog zurücksetzen

  lcd.clear();
  lcd.setCursor(0,0);
  
float DeltaTP = Taupunkt_i - Taupunkt_a;

if (DeltaTP > (SCHALTmin + HYSTERESE))rel = true;
if (DeltaTP < (SCHALTmin))rel = false;
if (ti < TEMP1_min )rel = false;
if (ta < TEMP2_min )rel = false;

if (rel == true)
{
  digitalWrite(RELAIPIN, RELAIS_EIN); // Relais einschalten
  lcd.print(F("Lueftung AN"));  
} else {                             
  digitalWrite(RELAIPIN, RELAIS_AUS); // Relais ausschalten
  lcd.print(F("Lueftung AUS"));
}

 lcd.setCursor(0,1);
 lcd.print("Delta TP: ");
 lcd.print(DeltaTP);
 lcd.write((uint8_t)0); // Sonderzeichen °C
 lcd.write('C');

 delay(4000);   // Wartezeit zwischen zwei Messungen
 esp_task_wdt_reset();   // Watchdog zurücksetzen 
 
}

float taupunkt(float t, float r) {
  
float a = 0;
float b = 0;
  
  if (t >= 0) {
    a = 7.5;
    b = 237.3;
  } else if (t < 0) {
    a = 7.6;
    b = 240.7;
  }
  
  // Sättigungsdampfdruck in hPa
  float sdd = 6.1078 * pow(10, (a*t)/(b+t));
  
  // Dampfdruck in hPa
  float dd = sdd * (r/100);
  
  // v-Parameter
  float v = log10(dd/6.1078);
  
  // Taupunkttemperatur (°C)
  float tt = (b*v) / (a-v);
  return { tt };  
}


 void software_Reset() // Startet das Programm neu, nicht aber die Sensoren oder das LCD 
  {
    ESP.restart();
  }
