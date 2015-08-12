/*This file contains a sample program that excersises the Galil pci driver.
The function command() sends a command to the controller and retrieves the response.
main() uses command() to get the controller's model and serial numbers.

[root@localhost PciLinux]# gcc test.c
*/
char GALIL_FILE[100] = "/dev/galilpci"; //default device file is /dev/galilpci0.  The null character will be overwritten.
#include <stdio.h>      //printf
#include <fcntl.h>		// open
#include <string.h>   //strlen, strcat
//#include <sys/ioctl.h>

int file_desc;  //the file descriptor returned from open()


//command() sends an ASCII Command (e.g. "TPX") to the controller and retrieves a Response (e.g. "123\r\n:").
//The size of Response should be supplied as ResponseSize so that unallocated memory is not overwritten.
//If you statically allocate your response buffer (e.g. char buff[100]) use sizeof(buff).
int command(char* Command, char* Response, int ResponseSize) //returns the number of bytes read
{
   char acPartialResponse[512] = {0}; //buffer to contain partial responses (which will be concatenated together to form the final response)
   int iPartialBytesRead = 0; //number of bytes read each time through the loop
   int iTotalBytesRead = 0;   //the total number of bytes read.  Can't exceed ResponseSize.

   Response[0] = 0; //set response to null string 
   write(file_desc, Command, strlen(Command)); //write the command to the controller
   //write(file_desc, "\r", 1);

   //keep reading until we (a) get a colon (b) get a question mark (c) fill up the callers Response buffer
   while(1)
   {
      iPartialBytesRead = read(file_desc, acPartialResponse, sizeof(acPartialResponse)); //read some characters
      
      if(iPartialBytesRead <= 0)   //nothing read, keep reading until :
         continue;
      else if(iTotalBytesRead + iPartialBytesRead > ResponseSize) //get out of the loop if we will fill up the caller's buffer, iPartialBytesRead >= 1
         break;
      else 
      {
         strncat(Response, acPartialResponse, iPartialBytesRead); //add the partial response to the full response.  Response is null terminated
         iTotalBytesRead += iPartialBytesRead; //tally up the total number of bytes read
   //    printf("%s|%s|%i\n", Response, acPartialResponse, iPartialBytesRead); 
         if (acPartialResponse[iPartialBytesRead - 1] == ':' || acPartialResponse[iPartialBytesRead - 1] == '?') //got a colon, iPartialBytesRead >= 1
            break;
      }
   }
   
   return(iTotalBytesRead);
}


int main(int argc, char *argv[]) //./a.out X connects to /dev/galilpciX.  No arg connects to /dev/galilpci0
{
	char sBuffer[366];    //buffer to hold controller responses
   file_desc = open((argc == 2 ? strcat(GALIL_FILE, argv[1]) : strcat(GALIL_FILE, "0")), O_RDWR);  //open the Galil PCI driver for read and write
   if (file_desc < 0) 
   {
      printf("Can't open device file %s\n", GALIL_FILE);
      return 1; //fail
   }

	//get controller model number and firmware version
	command("\x12\x16\r", sBuffer, sizeof(sBuffer));
	printf("Connected to %s", sBuffer);

   //RESET
//   ioctl(file_desc, 'R');

/* //INTERRUPT
   int iTimeoutMs = 5000;
   printf("ioctl %i\n", ioctl(file_desc, 'i', &iTimeoutMs)); //0 success, -1 timeout
   printf("status byte %i\n", iTimeoutMs & 0xff); //least significant byte contains status byte (e.g. 0xF0 for UI0)
*/
         
   //RECORD
   command("CW,,0;", sBuffer, sizeof(sBuffer)); //turn interrupts off

   int iSize = 0, iTimeoutMs = 0;
   memcpy(sBuffer, &iSize, sizeof(iSize));
   memcpy(sBuffer + sizeof(iSize), &iTimeoutMs, sizeof(iTimeoutMs));
   while(0 == ioctl(file_desc, 'r', sBuffer)) {} //clear the record q in the driver.  wait for timeout

   command("CW,,1;", sBuffer, sizeof(sBuffer)); //turn interrupts on
   command("DR3;"  , sBuffer, sizeof(sBuffer)); //turn on records
   command("DU1;"  , sBuffer, sizeof(sBuffer)); //turn on dual port ram on 18x0 (no effect on 18x6)

   int errors = -1;  //first time will increment errors to 0
   unsigned short lasttime = 0;

   while(1)
   {
      iSize = sizeof(sBuffer); iTimeoutMs = 1000;
      memcpy(sBuffer, &iSize, sizeof(iSize));
      memcpy(sBuffer + sizeof(iSize), &iTimeoutMs, sizeof(iTimeoutMs));
      ioctl(file_desc, 'r', sBuffer); //get the next record

//         printf("iBytesRead %i\n", iBytesRead);
//         for(j = 0; j < sizeof(sBuffer); j++)
//            printf("%03i %02x\n", j, sBuffer[j] & 0xff); //print the data record in hex

      unsigned short time = *(unsigned short*) sBuffer; //pick off sample counter
      if(time != (lasttime + 8) % 65536)
         errors++;

      printf("%i %i\n", time, errors);
      lasttime = time;
   } //while(!_kbhit())

   command("DR0;"  , sBuffer, sizeof(sBuffer)); //turn off records 
   close(file_desc);  //close the Galil PCI driver
}
