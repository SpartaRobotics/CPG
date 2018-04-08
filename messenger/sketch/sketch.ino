int start = 1;
int missionStatus = 1;
char recv;

void setup() {
  Serial.begin(115200);
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
    Serial.println("Mission Completed!");
    missionStatus = 0;
  }
}

void rmCapture()
{
  Serial.println("1");
      
  while(recv != '2')
  {
    recv = Serial.read();
  }
  Serial.println("Successful Capture!");
}

void rmAssistedDocking()
{
  Serial.println("3");
      
  while(recv != '4')
  {
    recv = Serial.read();
  }
  Serial.println("Successful Docking!");
}

void rmRefueling()
{
  Serial.println("5");
      
  while(recv != '6')
  {
    recv = Serial.read();
  }
  Serial.println("Successful Refueled!");
}

