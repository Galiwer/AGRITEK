#include <Arduino.h>
#include <WiFi.h>
#include <FirebaseESP32.h>

#include "RTClib.h"  
#include <LiquidCrystal_I2C.h>
#include <DHT.h>

// Provide the token generation process info.
#include <addons/TokenHelper.h>

// Provide the RTDB payload printing info and other helper functions.
#include <addons/RTDBHelper.h>

/* 1. Define the Wi-Fi credentials */
#define WIFI_SSID "Pixel_3618"
#define WIFI_PASSWORD "12345678"

/* 2. Define the API Key */
#define API_KEY "AIzaSyBgbGOlp-zCR69h7F0cjwnxQAfME_bwPqM"

/* 3. Define the RTDB URL */
#define DATABASE_URL "agritek-2d80d-default-rtdb.firebaseio.com"

/* 4. Define the user Email and password that are already registered or added in your project */
#define USER_EMAIL "msaokumara@gmail.com"
#define USER_PASSWORD "Omindu2003"

// Define Firebase Data object
FirebaseData fbdo;

FirebaseAuth auth;
FirebaseConfig config;

unsigned long sendDataPrevMillis = 0;
unsigned long dataFetchPrevMillis = 0;

#define SENSOR_1_PIN 32 // Pin for sensor 1
#define SENSOR_2_PIN 33 // Pin for sensor 2
#define SENSOR_3_PIN 34 // Pin for sensor 3



#define RELAY_PIN1 12 // ESP32 pin GPIO12 connected to the IN pin of relay
#define RELAY_PIN2 13 // ESP32 pin GPIO13 connected to the IN pin of relay
#define RELAY_PIN3 14 // ESP32 pin GPIO14 connected to the IN pin of relay
#define RELAY_PIN4 15 // ESP32 pin GPIO14 connected to the IN pin of relay

#define led1 18
#define led2 19
#define led3 27

#define DHT_SENSOR_PIN  25 // ESP32 pin GPIO21 connected to DHT11 sensor
#define DHT_SENSOR_TYPE DHT11


LiquidCrystal_I2C lcd(0x27, 16, 2); // I2C address 0x27, 16 column and 2 rows
DHT dht_sensor(DHT_SENSOR_PIN, DHT_SENSOR_TYPE);
RTC_DS3231 rtc;
DateTime t;  


int sensorValue[10];

int cropwater[10];

int firtmode;
int triggerYear;
int triggerMonth;
int triggerDate;
int triggerHour;
int triggerMinute;
int firtneedAT; // in ml
int cropscount;
int firtilizingdone;
void moisturesensor();


bool switch1, switch2, switch3;

void setup() {
  Serial.begin(115200);

   WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("Connecting to Wi-Fi");
    while (WiFi.status() != WL_CONNECTED)
    {
        Serial.print(".");
        delay(300);
    }
    Serial.println();
    Serial.print("Connected with IP: ");
    Serial.println(WiFi.localIP());
    Serial.println();

    // Firebase configuration
    config.api_key = API_KEY;
    config.database_url = DATABASE_URL;
    auth.user.email = USER_EMAIL;
    auth.user.password = USER_PASSWORD;
    config.token_status_callback = tokenStatusCallback;

    Firebase.reconnectNetwork(true);

    Firebase.begin(&config, &auth);

  dht_sensor.begin(); // initialize the DHT sensor
  lcd.init(); // initialize the lcd
  lcd.backlight();
  rtc.begin(); 

   // initialize digital pin as an output.
  pinMode(RELAY_PIN1, OUTPUT);
  pinMode(RELAY_PIN2, OUTPUT);
  pinMode(RELAY_PIN3, OUTPUT);
  pinMode(RELAY_PIN4, OUTPUT);

  digitalWrite(RELAY_PIN1, HIGH);
  digitalWrite(RELAY_PIN2, HIGH);
  digitalWrite(RELAY_PIN3, HIGH);
  digitalWrite(RELAY_PIN4, HIGH);

          // Set GPIO modes for LEDs
    pinMode(led1, OUTPUT);
    pinMode(led2, OUTPUT);
    pinMode(led3, OUTPUT);

    // Set default states
    digitalWrite(led1, LOW);
    digitalWrite(led2, LOW);
    digitalWrite(led3, LOW);

//rtc.adjust(DateTime(2024, 4, 29, 7, 16, 0));
 rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
}

void loop() {

      if (Firebase.ready() && (millis() - dataFetchPrevMillis > 2000 || dataFetchPrevMillis == 0))
    {
        dataFetchPrevMillis = millis();

     if (Firebase.getInt(fbdo, "/control/growthmode")) {
    if (fbdo.dataType() == "int") {
      firtmode = fbdo.intData();
      Serial.print("firtmode");
      Serial.println(firtmode);
    } else {
      Serial.println("Failed to get growthmode data, data type mismatch");
    }
  } else {
    Serial.print("Failed to get growthmode data, ");
    Serial.println(fbdo.errorReason());
  }
    if (Firebase.getInt(fbdo, "/control/fertilized")) {
    if (fbdo.dataType() == "int") {
      firtilizingdone = fbdo.intData();
      Serial.print("fertilizing");
      Serial.println(firtilizingdone);
    } else {
      Serial.println("Failed to get fertilizing data, data type mismatch");
    }
  } else {
    Serial.print("Failed to get fertilizing data, ");
    Serial.println(fbdo.errorReason());
  }

     if (Firebase.getInt(fbdo, "/control/count")) {
    if (fbdo.dataType() == "int") {
      cropscount = fbdo.intData();
      Serial.print("count");
      Serial.println(cropscount);
    } else {
      Serial.println("Failed to get count, data type mismatch");
    }
  } else {
    Serial.print("Failed to get count, ");
    Serial.println(fbdo.errorReason());
  }

   if (Firebase.getInt(fbdo, "/control/irrigation1")) {
    if (fbdo.dataType() == "int") {
       triggerYear = fbdo.intData();
      Serial.print("Irrigation Time for Crop 1: ");
      Serial.println(triggerYear);
    } else {
      Serial.println("Failed to get irrigation time, data type mismatch");
    }
  } else {
    Serial.print("Failed to get irrigation time, ");
    Serial.println(fbdo.errorReason());
  } 

  if (Firebase.getInt(fbdo, "/control/irrigation2")) {
    if (fbdo.dataType() == "int") {
       triggerMonth = fbdo.intData();
      Serial.print("Irrigation Time for Crop 2: ");
      Serial.println(triggerMonth);
    } else {
      Serial.println("Failed to get irrigation time2, data type mismatch");
    }
  } else {
    Serial.print("Failed to get irrigation time2, ");
    Serial.println(fbdo.errorReason());
  } 

            if (Firebase.getInt(fbdo, "/control/irrigation3")) {
    if (fbdo.dataType() == "int") {
       triggerDate = fbdo.intData();
      Serial.print("Irrigation Time for Crop 3: ");
      Serial.println(triggerDate);
    } else {
      Serial.println("Failed to get irrigation time3, data type mismatch");
    }
  } else {
    Serial.print("Failed to get irrigation time3, ");
    Serial.println(fbdo.errorReason());
  } 
            if (Firebase.getInt(fbdo, "/control/irrigation4")) {
    if (fbdo.dataType() == "int") {
     triggerHour = fbdo.intData();
      Serial.print("Irrigation Time for Crop 4: ");
      Serial.println(triggerHour);
    } else {
      Serial.println("Failed to get irrigation time4, data type mismatch");
    }
  } else {
    Serial.print("Failed to get irrigation time4, ");
    Serial.println(fbdo.errorReason());
  } 

  if (Firebase.getInt(fbdo, "/control/irrigation5")) 
  {
    if (fbdo.dataType() == "int") 
    {
     triggerMinute = fbdo.intData();
      Serial.print("Irrigation Time for Crop 5: ");
      Serial.println(triggerMinute);
    } else {
      Serial.println("Failed to get irrigation time5, data type mismatch");
    }
  } else {
    Serial.print("Failed to get irrigation time5, ");
    Serial.println(fbdo.errorReason());
  } 

    }

      // Fetch switch states from Firebase to control LEDs
    if (Firebase.ready() && (millis() - dataFetchPrevMillis > 2000 || dataFetchPrevMillis == 0))
    {
        dataFetchPrevMillis = millis();


        // Fetch switch states for LEDs
        Firebase.getBool(fbdo, "/control/led1", &switch1);
        Firebase.getBool(fbdo, "/control/led2", &switch2);
        Firebase.getBool(fbdo, "/control/led3", &switch3);

        // Control LEDs based on Firebase data
        digitalWrite(led1, switch1 ? HIGH : LOW);
        digitalWrite(led2, switch2 ? HIGH : LOW);
        digitalWrite(led3, switch3 ? HIGH : LOW);

        Serial.println("LED states updated from Firebase");

    }

        readSensor(SENSOR_1_PIN,1);
        readSensor(SENSOR_2_PIN,2);
        readSensor(SENSOR_3_PIN,3);

        //omi end
  
  float humi  = dht_sensor.readHumidity(); // read humidity
  float tempC = dht_sensor.readTemperature(); // read temperature in Celsius
  float tempF = dht_sensor.readTemperature(true); // read temperature in Fahrenheit

  DateTime now = rtc.now();

  // check whether the reading is successful or not
  if ( isnan(tempC) || isnan(tempF) || isnan(humi)) {
    Serial.println("Failed to read from DHT sensor!");
    tempC=29;
    tempF=30;
    humi=60;
  } else {
    Serial.print("Humidity: ");
    Serial.print(humi);
    Serial.print("%");

    Serial.print("  |  ");

    Serial.print("Temperature: ");
    Serial.print(tempC);
    Serial.print("°C  ~  ");
    Serial.print(tempF);
    Serial.println("°F");  

    Firebase.setFloat(fbdo, "/sensors/temperature",tempC);
    Firebase.setFloat(fbdo, "/sensors/humidity",humi);
  }
  Firebase.setFloat(fbdo, "/sensors/temperature",tempC);
    Firebase.setFloat(fbdo, "/sensors/humidity",humi);
  

moisturesensor();

    for(int i=1;i<=cropscount;i++){
        Serial.print("Sensor value: ");
         Serial.print(i);
         Serial.print("  : ");
       Serial.println(sensorValue[i]);

    

  }
  lcd.clear();
    //digitalWrite(relay,LOW);
    lcd.setCursor(0,0);
    lcd.print("System Is Online");

if(cropscount>=3){
    lcd.setCursor(0,1);
    lcd.print("1:");
    lcd.setCursor(2, 1);    
    lcd.print(sensorValue[1]);
    lcd.setCursor(6, 1);
    lcd.print("2:");
    lcd.setCursor(8, 1);
    lcd.print(sensorValue[2]);
    lcd.setCursor(11, 1);
    lcd.print("3:");
    lcd.setCursor(13,1 );
    lcd.print(sensorValue[3]);
}
else if(cropscount>=2){
  lcd.setCursor(0,1);
    lcd.print("1:");
    lcd.setCursor(2, 1);    
    lcd.print(sensorValue[1]);
    lcd.setCursor(6, 1);
    lcd.print("2:");
    lcd.setCursor(8, 1);
    lcd.print(sensorValue[2]);
}
else if(cropscount>=1){
  lcd.setCursor(0,1);
    lcd.print("1:");
    lcd.setCursor(2, 1);    
    lcd.print(sensorValue[1]);
}

if(firtmode == 1){
  //lcd.clear();
  lcd.setCursor(0,1);
    lcd.print("Fertilize Mode");
}


  Serial.print("Current Time: ");
  Serial.print(now.year());
  Serial.print("-");
  Serial.print(now.month());
  Serial.print("-");
  Serial.print(now.day());
  Serial.print(" ");
  Serial.print(now.hour());
  Serial.print(":");
  Serial.print(now.minute());
  Serial.print(":");
  Serial.print(now.second());
  Serial.println();

//soilMoisture(mvalue1,mvalue2);
  // wait a 2 seconds between readings


if (/*cropscount == 0  &&*/ !(firtilizingdone) && firtmode ){
 if (now.year() == triggerYear && now.month() == triggerMonth && now.day() == triggerDate && now.hour() == triggerHour && now.minute() == triggerMinute && !(firtilizingdone)) {
    //firtneedAT ml
    firtneedAT=100;     
    controlMotor(firtneedAT ,RELAY_PIN4);
    Serial.println("Fertilizing according to time");
    firtilizingdone = 1;
    Firebase.setInt(fbdo, "/control/fertilized",1);
    Firebase.setInt(fbdo, "/control/growthmode",0);
     firtmode = 0;
  }
   else {
    Serial.println("Not Fertilizing according to time");
  }

}

for(int i=1;i<=cropscount;i++){
    cropwater[i] = calculateWaterNeeds(sensorValue[i], humi,tempC);
     
    Serial.print("Water needs : ");
    Serial.println(cropwater[i]);
   
   
    }
    //controlMotor(cropwater[1],14); 
    watering();
}





//loop ends

float calculateWaterNeeds(float fertilizerLevel, float humidity, float temperature) {
  // Example calculation logic based on sensor data

  float waterNeeds = 0.0;
  if( fertilizerLevel<=60){
  // Adjust water needs based on fertilizer level, humidity, temperature, and NPK level
  // Example logic:
  // Water needs increase with higher fertilizer level and lower humidity
  // Water needs increase with higher temperature
  // Water needs increase with lower NPK level (assuming higher NPK level indicates more nutrient uptake)
  waterNeeds += (100-fertilizerLevel)* 1; // Adjust this multiplier based on your sensor range and plant requirements
  waterNeeds += (100 - humidity) * 0.05; // Adjust this multiplier based on your sensor range and plant requirements
  waterNeeds += (temperature - 20) * 0.1; // Adjust this multiplier based on your sensor range and plant requirements
 
  }
  return waterNeeds;
}

/*int LEDcontrolvalue(){
  pin1
}*/

void watering(){
    int option = cropscount; // web value *omidu

    switch (option) {
      case 1:
        controlMotor(cropwater[1],12);  // Call function to control motor based on water level
        break;
      case 2:
        controlMotor(cropwater[1],12);  // Call function to control motor based on water level
        controlMotor(cropwater[2],13);  // Call function to control motor based on water level
        break;
      case 3:
        controlMotor(cropwater[1],12);  // Call function to control motor based on water level
        controlMotor(cropwater[2],13);  // Call function to control motor based on water level
        controlMotor(cropwater[3],14);  // Call function to control motor based on water level
        
        break;
      default:
        Serial.println("Cropscount 0 or Invalid option!");
        break;
    }
}





void controlMotor(int currentWaterLevel ,int motorPin) {

  float delaytimetomotor = (float)currentWaterLevel /10 *1000 ;
  Serial.println("Time : ");
  Serial.println(delaytimetomotor);
  if (currentWaterLevel > 0) {
    digitalWrite(motorPin, LOW);  // Turn motor on
    delay(delaytimetomotor);              // Wait for a moment
    digitalWrite(motorPin, HIGH);   // Turn motor off
  }
  // Add additional conditions or actions as needed
}




void moisturesensor(){
    int option = cropscount; // web value *omidu

    switch (option) {
      case 1:
        readSensor(SENSOR_1_PIN,1);
        break;
      case 2:
        readSensor(SENSOR_1_PIN,1);
        readSensor(SENSOR_2_PIN,2);
        break;
      case 3:
        readSensor(SENSOR_1_PIN,1);
        readSensor(SENSOR_2_PIN,2);
        readSensor(SENSOR_3_PIN,3);
        break;
      default:
        Serial.println("Invalid option!");
        break;
    }
}

void readSensor(int pin, int num) {
  //sensorValue[num] = 100-(analogRead(pin)/4095*100);
  sensorValue[num] = 100-((float)analogRead(pin)/4095*100);
  Firebase.setInt(fbdo, "/sensors/moisture1",sensorValue[1]);
  Firebase.setInt(fbdo, "/sensors/moisture2",sensorValue[2]);
  Firebase.setInt(fbdo, "/sensors/moisture3",sensorValue[3]);
}


