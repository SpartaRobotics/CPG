

/* 

    AUTHOR: Bruce Nelson
    DATE: 4/5/2017

    CONTRIBUTIONS: Chrisheyd 
    www.chrisheydrick.com

    PURPOSE:
    * The purpose of this program to demostrate a simple example of reading
    and writing data from a linux system to an Arduino. The program setups
    the serial stream at a selected baud rate (matched with the Arduino's)
    and initializes communication. The program sends a single character string
    "2" to the Arduino. The Arduino will convert it to an integer add 4 to the
    value read, which should add up to 6. This value will then write from the
    Arduino to the host.

*/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <sys/ioctl.h>

int fd;

void serialSetup(); // Sets up serial communication to the Arduino
void clientStart(char *msg);
void receive();
void clientMessage(char* msg, int size); // prints a char array
void missionPhase(char start, char *end, char *startMsg, char*endMsg);

int main(int argc, char *argv[])
{
    int n;
    char buf[64] = "temp text";

    serialSetup();
   
    //Capture Maneuver
    missionPhase('1',"2","HOST: Start Capture!", "HOST: End Capture!");
     
    // Docking Maneuver
	missionPhase('3',"4","HOST: Start Docking!", "HOST: End Docking!"); 
	
    // Refueling
    missionPhase('5',"6","HOST: Start Refueling!", "HOST: End Refueling!");

	// Final Mission Message
    receive();

    return 0;
}

void missionPhase(char start, char *end, char *startMsg, char*endMsg)
{
	int n;
	char buf[64] = "";
		
    while( buf[0] != start )
    {
        n = read(fd, buf, 64);
        buf[n] = 0;
        sleep(1);
        printf("%s\n", buf);
    }

    printf("%s\n", startMsg);
    // Perform mission phase here
    printf("%s\n", endMsg);
    write(fd, "2", 1);
    receive();
}

void serialSetup()
{
    struct termios toptions;

    /* open serial port */
    fd = open("/dev/ttyTHS0", O_RDWR | O_NOCTTY);
    printf("fd opened as %i\n\n", fd);
    
    /* wait for the Arduino to reboot */
    usleep(3500000);
    
    /* get current serial port settings */
    tcgetattr(fd, &toptions);
    /* set 115200 baud both ways */
    cfsetispeed(&toptions, B115200);
    cfsetospeed(&toptions, B115200);
    /* 8 bits, no parity, no stop bits */
    toptions.c_cflag &= ~PARENB;
    toptions.c_cflag &= ~CSTOPB;
    toptions.c_cflag &= ~CSIZE;
    toptions.c_cflag |= CS8;
    /* Canonical mode */
    toptions.c_lflag |= ICANON;
    /* commit the serial port settings */
    tcsetattr(fd, TCSANOW, &toptions);
}

void receive()
{
    char buf[64];
    int length;

    length = read(fd, buf, 64); // read the message into buf and record the length
    buf[length] = 0;            // add null terminator to char array
    usleep(10000);
    clientMessage(buf, length); // print the char array message
}

void clientMessage(char *msg, int size)
{
	int i;
	
    printf("CLIENT: ");
    for(i = 0; i < size; i++)
    {
        printf("%c", msg[i]);
    }
    //printf("\n");
}
