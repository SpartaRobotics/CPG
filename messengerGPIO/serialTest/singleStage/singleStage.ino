int start = 1;
int missionStatus = 1;
byte b;

void setup() {
  Serial3.begin(115200);
}

void loop() {
  if(Serial3.available())
  {
    Serial3.println("Hello"); 
  }
}
