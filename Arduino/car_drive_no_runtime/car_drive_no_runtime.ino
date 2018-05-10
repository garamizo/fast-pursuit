#include <WiFi.h>

const char* ssid = "MichiganTechOpen";
//const char* password = "luckyskates590";

WiFiServer server(80);
WiFiClient client;

int in1 = 17;
int in2 = 5;
int in3 = 18;
int in4 = 23;
int ENA = 16;
int ENB = 19;
int runTime;
boolean motorState;
unsigned long startTime;

void setup() {
  // put your setup code here, to run once:
  ledcSetup(0, 500, 10);
  ledcAttachPin(17, 0);
  ledcSetup(1, 500, 10);
  ledcAttachPin(5, 1);

  ledcSetup(2, 500, 10);
  ledcAttachPin(18, 2);
  ledcSetup(3, 500, 10);
  ledcAttachPin(23, 3);

  ledcSetup(4, 500, 10);
  ledcAttachPin(16, 4);
  ledcSetup(5, 500, 10);
  ledcAttachPin(19, 5);

  pinMode(22, OUTPUT);
  digitalWrite(22, HIGH);

  Serial.begin(9600);
  WiFi.begin(ssid);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  server.begin();

  Serial.println("Waiting for client connection");
  while (!client) {
    client = server.available();
    Serial.println(WiFi.localIP());
    delay(1000);
  }

  Serial.println("Client connected");
}

void loop() {
  static long lastMsg = millis();
  if (millis() - lastMsg > 300)
    simple_drive(0, 0);
    
  if (client.available() > 0) {
    
    float vr = client.parseFloat();
    float vl = client.parseFloat();
    client.read();
    
    simple_drive(vr, vl);
    Serial.println(String(vr) + ' ' + vl);

    lastMsg = millis();
  }
  
  delay(10);
}

void simple_drive(float vr, float vl) {

  ledcWrite(0, vr > 0 ? 1023.0 : 0);
  ledcWrite(1, vr > 0 ? 0 : 1023.0);
  ledcWrite(4, constrain(abs(vr), 0, 100) * 1023.0 / 100.0);
//  Serial.println(abs(vr) * 1023.0 / 100.0);
  

  ledcWrite(2, vl > 0 ? 1023.0 : 0);
  ledcWrite(3, vl > 0 ? 0 : 1023.0);
  ledcWrite(5, constrain(abs(vl), 0, 100) * 1023.0 / 100.0);

}

