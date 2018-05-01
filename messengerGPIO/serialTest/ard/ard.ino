String msg;

void setup() {
  Serial3.begin(115200);
  Serial.begin(9600);

}

void loop() {
  if(Serial.available() > 0)
  {
      msg = Serial3.readString(); 
      Serial3.println("Hello");
  }
}
