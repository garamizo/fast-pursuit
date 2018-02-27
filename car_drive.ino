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
  // put your main code here, to run repeatedly:

  //    if (Serial.available() > 0) {
  //      int vr = Serial.parseInt();
  //      int vl = Serial.parseInt();
  //      runTime = Serial.parseInt();
  //      startTime = millis();
  //
  //      Serial.println(String(vr) + ' ' + vl + ' ' + runTime);
  //
  //      simple_drive(vr, vl);
  //      motorState = 1;
  //    }

  if (client.connected()) {
    client.println("Connected");
    digitalWrite(22, LOW);
    delay(1000);
  } else {
    simple_drive(0, 0);
    digitalWrite(22, !digitalRead(22));
    delay(1000);
    Serial.println("Disconnected");
  }

  if (client.available() > 0) {
    String string = client.readStringUntil('r');
    String string2 = client.readStringUntil('r');
    String string3 = client.readStringUntil('r');
    float vr = string.toFloat();
    float vl = string2.toFloat();
    runTime = string3.toInt();
    startTime = millis();
    simple_drive(vr, vl);
    Serial.println(String(vr) + ' ' + vl + ' ' + runTime);

    motorState = 1;
  }

  if (motorState && millis() - startTime > runTime * 1000) {
    simple_drive(0, 0);
    motorState = 0;
  }
  delay(10);
}

void simple_drive(float vr, float vl) {

  ledcWrite(0, vr > 0 ? 1023.0 : 0);
  ledcWrite(1, vr > 0 ? 0 : 1023.0);
  ledcWrite(4, abs(vr) * 1023.0 / 100.0);
  Serial.println(abs(vr) * 1023.0 / 100.0);
  

  ledcWrite(2, vl > 0 ? 1023.0 : 0);
  ledcWrite(3, vl > 0 ? 0 : 1023.0);
  ledcWrite(5, abs(vl) * 1023.0 / 100.0);

}

