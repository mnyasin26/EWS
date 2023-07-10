/*program relay PMU
 * author = willy.r48
 * version = v.1
 * last update = 02.05.2023
 * copyright makerindo 2023

 */
#include <Arduino.h>
#include <Wire.h>

#define RLY1 6
#define RLY2 7

void setup()
{
    Serial.begin(9600);
    Wire.begin();

    pinMode(RLY1, OUTPUT);
    pinMode(RLY2, OUTPUT);

    digitalWrite(RLY1, HIGH);
    digitalWrite(RLY2, HIGH);
    Serial.println("START");
}
void loop()
{
    relay_pmu();
}
// prosedure relay PMU
void relay_pmu()
{
    digitalWrite(RLY1, LOW);
    digitalWrite(RLY2, LOW);
    Serial.println("NYALA");
    delay(300000);
    digitalWrite(RLY1, HIGH);
    digitalWrite(RLY2, HIGH);
    Serial.println("MATI");
    delay(1800000);
}