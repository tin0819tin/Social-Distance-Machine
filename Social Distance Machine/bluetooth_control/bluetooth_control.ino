#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif#include "BluetoothSerial.h"

BluetoothSerial SerialBT;
int outPin = 12;
char data1;

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32test"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");
  
  pinMode(outPin, OUTPUT);
  digitalWrite(outPin, LOW);
}

void loop() {
  if (Serial.available()) {
    SerialBT.write(Serial.read());
  }
  if (SerialBT.available()) {
    data1 = SerialBT.read();
    Serial.write(data1);
    if (data1 == '1')
    {
      digitalWrite(outPin, HIGH);
    }
    else if (data1 == '0')
    {
      digitalWrite(outPin, LOW);
    }
  }
}
