#include <WiFi.h>

const char* ssid = "Team1";
const char* password = "1q2w3e4r";

const int machine_id = 1;

WiFiServer server(80);

#define TCRT5000_SENSOR_PIN 34

int value = 1;

void setup()
{
  // Set the sensor pin mode
  pinMode(TCRT5000_SENSOR_PIN, INPUT);

  Serial.begin(115200);
  Serial.println("ESP32 TCP Server Start");
  Serial.println(ssid);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.print(".");
  }
  Serial.println();

  Serial.print("IP Address : ");
  Serial.println(WiFi.localIP());

  server.begin();
}

struct protocol
{
  int id = machine_id;
  int pin = TCRT5000_SENSOR_PIN;
  int status = 0;
};

void loop()
{
  WiFiClient client = server.available();
  if (client)
  {
    Serial.print("Client Connected : ");
    Serial.println(client.remoteIP());
    struct protocol p;
    while (client.connected())
    {
      char data[12];
      //int i = 0;
      while (client.available() > 0)
      {
        client.readBytes(data, 12);
        memcpy(&p, &data, sizeof(p));

        if (p.pin == TCRT5000_SENSOR_PIN)
        {
          value = analogRead(TCRT5000_SENSOR_PIN);
          p.id = machine_id;
          p.status = value;

          memcpy(&data, &p, sizeof(p));
        }
        Serial.println(p.id);
        Serial.println(p.pin);
        Serial.println(p.status);
        Serial.println(value);

        client.write(data, 12);
      }

      delay (10);
    }

    client.stop();
    Serial.println("Client Disconnedted!");
  }
}