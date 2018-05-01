int start = 1;
int missionStatus = 1;
char recv;

void setup() {
  Serial2.begin(115200);
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
    Serial2.println("Mission Completed!");
    missionStatus = 0;
  }
}

void rmCapture()
{
  while(recv != '2')
  {
    Serial2.println("1");
    delay(100);
    recv = Serial2.read();
    delay(100);
  }
  Serial2.println("Successful Capture!");
}

void rmAssistedDocking()
{ 
  while(recv != '4')
  {
    Serial2.println("3");
    delay(100);
    recv = Serial2.read();
    delay(100);
  }
  Serial2.println("Successful Docking!");
}

void rmRefueling()
{
  while(recv != '6')
  {
    Serial2.println("5");
    delay(100);
    recv = Serial2.read();
    delay(100);
  }
  Serial2.println("Successful Refueled!");
}

