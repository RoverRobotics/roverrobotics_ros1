// #include <iostream>
// #include <stdio.h>

// #include <string.h>
// #include <errno.h>
// #include <vector>

// using namespace std;

// int main(int argc, char ** argv)
// {
// 	char *levelSensorPort = "/dev/ttyUSB0"; //Serial Device Address

// 	int levelSensor = serialOpen(levelSensorPort, 115200);
// 	wiringPiSetup();
// 	serialPuts(levelSensor, "DP"); //Send command to the serial device

// 	while (1)
// 	{
// 		char buffer[100];
// 		ssize_t length = read(levelSensor, &buffer, sizeof(buffer));
// 		if (length == -1)
// 		{
// 			cerr << "Error reading from serial port" << endl;
// 			break;
// 		}
// 		else if (length == 0)
// 		{
// 			cerr << "No more data" << endl;
// 			break;
// 		}
// 		else
// 		{
// 			buffer[length] = '\0';
// 			cout << buffer; //Read serial data
// 		}
// 	}

// 	return 0;
// }