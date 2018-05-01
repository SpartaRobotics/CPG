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
  
  while(recv != '2')
  {
    Serial.println("1");
    delay(100);
    if(Serial.available()>0)
      recv = Serial.read();
    delay(100);
  }
  //Serial.println("Successful Capture!");
}

void rmAssistedDocking()
{ 
  while(recv != '4')
  {
    Serial.println("3");
    delay(100);
    if(Serial.available()>0)
      recv = Serial.read();
    delay(100);
  }
  //Serial.println("Successful Docking!");
}

void rmRefueling()
{
  while(recv != '6')
  {
    Serial.println("5");
    delay(100);
    if(Serial.available()>0)
      recv = Serial.read();
    delay(100);
  }
  //Serial.println("Successful Refueled!");
}

