void setup()
{
Serial.begin(9600);
}
void loop() {
  Serial.write("hello from ESP");
  delay(2000);
  
}
