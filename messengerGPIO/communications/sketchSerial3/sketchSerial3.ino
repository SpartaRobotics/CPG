int start = 1;
int missionStatus = 1;
char recv;

void setup() {
  Serial3.begin(115200);
  Serial.begin(9600);
}

void loop() {
  if(missionStatus)
  {
    while(0){} // while(!dataAcquired)
    // startController();
    while(0){} // while(pos != stationKeeping)
    rmCapture();
    // turnOffController();
    rmAssistedDocking();
    // turnOnDocking();

    //while(!docking){}

    rmRefueling();
    //turnOffDocking();
    //startController();
    //returnStartingPosition();
    Serial3.println("Mission Completed!");
    missionStatus = 0;
  }
}

void rmCapture()
{
  while(recv != '2')
  {
    Serial3.println("1");
    delay(100);
    if(Serial3.available() > 0)
      recv = '2';
    Serial.println(recv);
    delay(100);
  }
  Serial3.println("Successful Capture!");
  delay(1000);
}

void rmAssistedDocking()
{ 
  while(recv != '4')
  {
    Serial3.println("3");
    delay(100);
    recv = Serial3.read();
    Serial.println(recv);
    delay(100);
  }
  Serial3.println("Successful Docking!");
}

void rmRefueling()
{
  while(recv != '6')
  {
    Serial3.println("5");
    delay(100);
    recv = Serial3.read();
    delay(100);
  }
  Serial3.println("Successful Refueled!");
}

