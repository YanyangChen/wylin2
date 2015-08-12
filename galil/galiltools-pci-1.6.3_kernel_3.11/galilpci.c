//Galil PCI driver for DMC-18xx.  Requires kernel >= 2.6.19.  printk output shows up in /var/log/messages or dmesg
/*
#include <linux/fs.h>          //struct file, struct inode
#include <linux/pci.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>       //copy_to_user(), copy_from_user()
#include <linux/interrupt.h>   //request_irq()
#include <linux/sched.h> // TASK_INTERRUPTIBLE moved to different header
#include <linux/mutex.h> // replacing lock_kernel and unlock_kernel
#include <linux/module.h>
*/
#ifdef __OS_UBUNTU
#include <linux/init.h>
#include <linux/module.h>
#endif

#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/init.h>
#include <linux/file.h>
#include <linux/syscalls.h>
#include <linux/fcntl.h>
#include <linux/fs.h>          //struct file, struct inode
#include <linux/pci.h>
#include <linux/ioport.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>       //copy_to_user(), copy_from_user()
#include <linux/interrupt.h>   //request_irq()
#include <linux/version.h>
#include <linux/unistd.h>

MODULE_LICENSE("Dual BSD/GPL");

#ifdef __DRIVER_VER
static char * gVersion = __DRIVER_VER;	// internal version displayed in log
#else
static char * gVersion = "0.0.9-0";	// internal version displayed in log
#endif

#define GALILNAME    "galilpci"//name shows up in /dev and /sys/bus/pci/drivers
#define BUFFERSIZE   512       //size of command and response buffer
#define MAX_DEVICES  16        //max number of Galil PCI boards in a single PC

#define SIZE 255               //max records/status bytes + 1

#define DMC1417 0x1417
#define DMC1640 0x1640
#define DMC18x0 0x1800
#define DMC18x2 0x1842
#define DMC18x6 0x1806

#define REV1 1 // PLX based board
#define REV2 2 // LILINX based board

#define __devexit_p 
#define __devinit
#define __devinitdata
#define __devexit


typedef struct CPciInfo
{
	__u16 m_fSubVendor;	// 2 byte unsigned number
	__u16 m_fSubSystem;	// 2 byte unsigned number
	__u16 m_fRevsion;		// 2 byte unsigned number
} CPciInfo;

typedef struct                 //queue to hold status bytes (from isr to ioctl)
{
   unsigned char q[SIZE];      //queue (circular buffer)
   unsigned char in;           // in pointer into q
   unsigned char out;          //out pointer into q   
} ByteQ;                       //out == in means q is empty.  out == (in + 1) % SIZE means q is full

typedef struct                 //queue to hold records (up to 512 bytes) from isr to ioctl
{
   unsigned char q[SIZE][512]; //queue (circular buffer).  512 is DPRAM size (maximum data record size).  Currently 1886 uses 366 bytes of this
   unsigned char in;           // in pointer into q
   unsigned char out;          //out pointer into q   
} RecordQ;                     //out == in means q is empty.  out == (in + 1) % SIZE means q is full

typedef struct //contains data private to each Galil PCI device installed in the PC
{
   struct miscdevice galilpci_miscdev;   //contains the minor number.  misc devices have a major number of 10.  See http://www.linuxjournal.com/node/2920/print
   CPciInfo						pciInfo;
   unsigned long     baseAddress;        //base address N from BAR2.  This is 16 bytes of I/O ports.
   unsigned long     irqAddress;         //BAR1:  18x2 only.  This is 0 on 18x0/6.  All switching between 18x2 and 18x0/6 is done based on this variable
   char              buffer[BUFFERSIZE]; //kernel buffer for holding controller commands and responses
   int               bOpen;              //exclusive file open flag

//   long 					recordAddress;      //BAR0:  controller writes the data record here (in dual port RAM)
   void __iomem     *recordAddress;      //BAR0:  controller writes the data record here (in dual port RAM)
   RecordQ           recordQ;            //record q from isr to ioctl (circular buffer)
   wait_queue_head_t recordWaitQ;        //this is a list of processes waiting for a record interrupt from the controller

   unsigned int      interrupt;          //irq number (e.g. 11).  This is stored only to call free_irq upon rmmod
   ByteQ             interruptQ;         //status byte q from isr to ioctl (circular buffer).  Contains       Cx, Dx, Ex, Fx
   wait_queue_head_t interruptWaitQ;     //this is a list of processes waiting for an UI or EI interrupt from the controller
   wait_queue_head_t	sleepQ;
} DeviceContext;

static DeviceContext d[MAX_DEVICES];     //array of galil device (board) structure that hold the IO address
static int           device = -1;        //total galil boards found minus 1 (matching PLX vendor and device IDs)

static ssize_t GetReadSize ( DeviceContext *d )
{
	size_t result = 0;
	switch(d->pciInfo.m_fSubSystem)
	{
	case DMC18x2:
		if ( d->pciInfo.m_fRevsion == REV2 )
			result = inw(d->baseAddress + 6);
		else
		{
//			printk(KERN_INFO "Read size flag[%X]\n", inb(d->baseAddress+4));
			result = ((inb(d->baseAddress+1) & 0x20) == 0) ? 1 : 0;
		}
		break;
	case DMC1640:
	case DMC18x0:
	case DMC18x6:
		if ( d->pciInfo.m_fRevsion == REV2 )
			result = inw(d->baseAddress + 6);
		else
		{
//			printk(KERN_INFO "Read size flag[%X]\n", inb(d->baseAddress+4));
			result = ((inb(d->baseAddress+4) & 0x04) == 0) ? 1 : 0;
		}
		break;
	case DMC1417:
		result = ((inb(d->baseAddress+1) & 0x01) == 0) ? 1 : 0;
		break;
	default:
		;
	};
	return result;
}

static ssize_t GetWriteSize ( DeviceContext *d )
{
	size_t result = 0;
	u8 flag;
	switch(d->pciInfo.m_fSubSystem)
	{
	case DMC18x2:
		if ( d->pciInfo.m_fRevsion == REV2 )
			result = (size_t) 511 - inw(d->baseAddress + 10);
		else
			result = ((inb(d->baseAddress+1) & 0x40) == 0) ? 1 : 0;
		break;
	case DMC1640:
	case DMC18x0:
	case DMC18x6:
		if ( d->pciInfo.m_fRevsion == REV2 )
			result = (size_t) 511 - inw(d->baseAddress + 10);
		else
			result = ((inb(d->baseAddress+4) & 0x01) == 0) ? 1 : 0;
		break;
	case DMC1417:
		result = ((inb(d->baseAddress+1) & 0x02) == 0) ? 1 : 0;
		break;
	default:
		;
	};
	return result;
}

//___________________________________________________________________________________________
//called when the user process calls read().  Never fails.  Returns 0 if no data to read.  Otherwise, returns number of bytes read.
ssize_t EvtIoRead(struct file *filep, char *buf, size_t count, loff_t *ppos)
{
   unsigned char tempBuf[BUFFERSIZE];
//   unsigned char tempBuf[3];
   bool noData = false;
   size_t i, bytesToRead;
   size_t bufIdx = 0;
   size_t curTempIdx = 0;
   size_t bytesRead = 0;  //number of bytes actually read from the controller (may not reach count if there's only a few bytes to read)
   DeviceContext *d = filep->private_data;
   size_t bytesAvailable;

#if 0
   unsigned char dumpChar;
   char dumpBuf[512];
   
	printk(KERN_INFO "EvtIoRead TotalBuffer[%i]\n", count);
#endif
	
	bufIdx = 0;
	while ( bytesRead < count && !noData )
	{
		curTempIdx = 0;
		bytesToRead = min(sizeof(tempBuf), count-bytesRead);
		while ( curTempIdx < sizeof(tempBuf) && curTempIdx < count )
		{
			bytesAvailable = GetReadSize(d);
			if ( bytesAvailable <= 0 )
			{
				noData = true;
				break;
			}
			bytesToRead = min(bytesToRead, bytesAvailable);
			tempBuf[curTempIdx++] = inb(d->baseAddress);
		}
		copy_to_user(buf+bytesRead, tempBuf, curTempIdx);
		bytesRead += curTempIdx;
//		printk(KERN_INFO "EvtIoRead Copy to user Total[%i] Cycle[%i]\n", bytesRead, curTempIdx);
	}
#if 0
   if ( bytesRead > 0 )
   {
		memset(dumpBuf, 0, sizeof(dumpBuf));
		for ( i = 0; i < 16 && i < bytesRead; i++ )
		{
			dumpChar = *(buf+i);
//			sprintf(dumpBuf+strlen(dumpBuf), "%X ", dumpChar);
			dumpBuf[i] = *(buf+i);
		}
		printk(KERN_INFO "EvtIoRead Dump: size(%i) data[%s]\n", bytesRead, dumpBuf);
		printk(KERN_INFO "EvtIoRead Total[%i] Read[%i]\n", count, bytesRead);
   }
#endif

   return bytesRead;
}
//___________________________________________________________________________________________
//called when the user process calls write().
ssize_t EvtIoWrite(struct file *filep, const char *buf, size_t count, loff_t *ppos)
{
   size_t maxBytes;
   DeviceContext *d = filep->private_data;
	size_t curBytes;
   size_t bytesToWrite;
   size_t bytesInFifo;
   size_t bytesWritten = 0;
   int i;
   
#if 0
   unsigned char dumpChar;
   char dumpBuf[512];
   
   memset(dumpBuf, 0, sizeof(dumpBuf));
   for ( i = 0; i < 16 && i < count; i++ )
   {
   	dumpChar = *(buf+i);
//   	sprintf(dumpBuf+strlen(dumpBuf), "%X ", dumpChar);
		dumpBuf[i] = *(buf+i);
   }
   printk(KERN_INFO "EvtIoWrite Dump: size(%i) data[%s]\n", count, dumpBuf);
#endif
   
	while ( bytesWritten < count )
	{
//		printk(KERN_INFO "EvtIoWrite Total[%i] Written[%i]\n", count, bytesWritten);
		bytesInFifo = GetWriteSize(d); //(size_t) 511 - inw(d->m_pBaseAddr + 10);
		while ( bytesInFifo <= 0 )	// wait for available space
		{
			wait_event_interruptible_timeout(d->sleepQ, 1 == 0, 10*1000/HZ);
			bytesInFifo = GetWriteSize(d); // (size_t) 511 - inw(d->m_pBaseAddr + 10);
//			printk(KERN_INFO "EvtIoWrite Wait for flag[%i]\n", bytesInFifo);
		}

		maxBytes = min(count-bytesWritten, bytesInFifo);	// bytes to xfer this cycle
		while ( maxBytes > 0 )
		{
			curBytes = min(maxBytes, sizeof(d->buffer)); // sub cycle
			if ( copy_from_user(d->buffer, buf+bytesWritten, curBytes) )
				return -EFAULT;
			for ( i = 0; i < curBytes; i++ )
				outb(d->buffer[i], d->baseAddress);
			bytesWritten += curBytes;
			maxBytes -= curBytes;
//			printk(KERN_INFO "EvtIoWrite Total[%i] Written[%i] Cycle[%i] Max[%i]\n", count, bytesWritten, curBytes, maxBytes);
		}
	}
//	printk(KERN_INFO "EvtIoWrite Total[%i] Written[%i]\n", count, bytesWritten);
   return bytesWritten;
}
//___________________________________________________________________________________________
int EvtIoDeviceControl(struct inode*  inode, struct file*  filep, unsigned int cmd, unsigned long arg)
{
   int iTimeoutMs, iSize;  //used by interrupt and record
   DeviceContext *d = filep->private_data;

   switch(cmd)
   {
      case 'r': //get record, arg contains pointer to buffer containing timeout and size, and arg returns pointer to record
         if( copy_from_user(&iSize, (int*)arg, sizeof(iSize)) )	//get data from user space in arg (e.g. 1000) and copy to kernel buffer (iTimeoutMs)
            return -EFAULT;

         if( copy_from_user(&iTimeoutMs, (int*) (arg + sizeof(iSize)), sizeof(iTimeoutMs)) )	//get data from user space in arg (e.g. 1000) and copy to kernel buffer (iTimeoutMs)
            return -EFAULT;

         if     (iTimeoutMs  > 0) //positive timeout
            wait_event_interruptible_timeout(d->recordWaitQ, d->recordQ.in != d->recordQ.out, iTimeoutMs * HZ / 1000); //wait up to the timeout for the IRQ byte to be set by the IRQ handler (wait for the circular buffer to be non-empty)
         else if(iTimeoutMs < 0)  //negative timeout (timeout disabled)
            wait_event_interruptible(d->recordWaitQ, d->recordQ.in != d->recordQ.out); //wait forever

         if(d->recordQ.in != d->recordQ.out) //q non empty
         {
            if(copy_to_user((char*)arg, d->recordQ.q[ d->recordQ.out ], iSize))	//return the record retrieved in the interrupt handler to users space in arg
               return -EFAULT;

            d->recordQ.out = (d->recordQ.out + 1) % SIZE;  //increment the read pointer
         }
         else //wait_event was interrupted by a signal (e.g. ctrl-c) < 0, or timeout = 0
            return -ERESTARTSYS;  //user mode ioctl returns -1

         break;

      case 'i': //interrupt (get EI/UI status byte).  arg contains pointer to timeout, and arg returns pointer to status byte
         if( copy_from_user(&iTimeoutMs, (int*)arg, sizeof(iTimeoutMs)) )	//get data from user space in arg (e.g. 1000) and copy to kernel buffer (iTimeoutMs)
            return -EFAULT;

         if     (iTimeoutMs  > 0) //positive timeout
            wait_event_interruptible_timeout(d->interruptWaitQ, d->interruptQ.in != d->interruptQ.out, iTimeoutMs * HZ / 1000); //wait up to the timeout for the IRQ byte to be set by the IRQ handler (wait for the circular buffer to be non-empty)
         else if(iTimeoutMs < 0)  //negative timeout (timeout disabled)
            wait_event_interruptible(d->interruptWaitQ, d->interruptQ.in != d->interruptQ.out); //wait forever

         if(d->interruptQ.in != d->interruptQ.out) //q non empty
         {
            if(copy_to_user((int*)arg, &(d->interruptQ.q[ d->interruptQ.out ]), sizeof(unsigned char)))	//return the status byte retrieved in the interrupt handler to users space in arg
               return -EFAULT;

            d->interruptQ.out = (d->interruptQ.out + 1) % SIZE;  //increment the read pointer
         }
         else //wait_event was interrupted by a signal (e.g. ctrl-c) < 0, or timeout = 0
            return -ERESTARTSYS;  //user mode ioctl returns -1

         break;

      case 'R': //reset DMC-18x0/6
         if(d->pciInfo.m_fSubSystem == DMC18x6) //18x6 (do nothing on 18x2)
            outb(0x80, d->baseAddress + 8); //reset 18x6
      break;

			case 'v':
				printk(KERN_INFO "GalilIoctl IOCTL_GALIL_VERSION\n");
				if (copy_to_user((char*)arg, "galiltools", strlen("galiltools")))
					return -EACCES;
				break;

      default:
         return -ENOTTY; //invalid ioctl code (switch fell thru).  POSIX says we should return -ENOTTY instead of EINVAL      
   }

   return 0; //normal 
}

//___________________________________________________________________________________________
long EvtUnlockedIoDeviceControl(struct file*  filep, unsigned int cmd, unsigned long arg)
{
   int iTimeoutMs, iSize;  //used by interrupt and record
   DeviceContext *d = filep->private_data;

   switch(cmd)
   {
      case 'r': //get record, arg contains pointer to buffer containing timeout and size, and arg returns pointer to record
         if( copy_from_user(&iSize, (int*)arg, sizeof(iSize)) )	//get data from user space in arg (e.g. 1000) and copy to kernel buffer (iTimeoutMs)
            return -EFAULT;

         if( copy_from_user(&iTimeoutMs, (int*) (arg + sizeof(iSize)), sizeof(iTimeoutMs)) )	//get data from user space in arg (e.g. 1000) and copy to kernel buffer (iTimeoutMs)
            return -EFAULT;

         if     (iTimeoutMs  > 0) //positive timeout
            wait_event_interruptible_timeout(d->recordWaitQ, d->recordQ.in != d->recordQ.out, iTimeoutMs * HZ / 1000); //wait up to the timeout for the IRQ byte to be set by the IRQ handler (wait for the circular buffer to be non-empty)
         else if(iTimeoutMs < 0)  //negative timeout (timeout disabled)
            wait_event_interruptible(d->recordWaitQ, d->recordQ.in != d->recordQ.out); //wait forever

         if(d->recordQ.in != d->recordQ.out) //q non empty
         {
            if(copy_to_user((char*)arg, d->recordQ.q[ d->recordQ.out ], iSize))	//return the record retrieved in the interrupt handler to users space in arg
               return -EFAULT;

            d->recordQ.out = (d->recordQ.out + 1) % SIZE;  //increment the read pointer
         }
         else //wait_event was interrupted by a signal (e.g. ctrl-c) < 0, or timeout = 0
            return -ERESTARTSYS;  //user mode ioctl returns -1

         break;

      case 'i': //interrupt (get EI/UI status byte).  arg contains pointer to timeout, and arg returns pointer to status byte
         if( copy_from_user(&iTimeoutMs, (int*)arg, sizeof(iTimeoutMs)) )	//get data from user space in arg (e.g. 1000) and copy to kernel buffer (iTimeoutMs)
            return -EFAULT;

         if     (iTimeoutMs  > 0) //positive timeout
            wait_event_interruptible_timeout(d->interruptWaitQ, d->interruptQ.in != d->interruptQ.out, iTimeoutMs * HZ / 1000); //wait up to the timeout for the IRQ byte to be set by the IRQ handler (wait for the circular buffer to be non-empty)
         else if(iTimeoutMs < 0)  //negative timeout (timeout disabled)
            wait_event_interruptible(d->interruptWaitQ, d->interruptQ.in != d->interruptQ.out); //wait forever

         if(d->interruptQ.in != d->interruptQ.out) //q non empty
         {
            if(copy_to_user((int*)arg, &(d->interruptQ.q[ d->interruptQ.out ]), sizeof(unsigned char)))	//return the status byte retrieved in the interrupt handler to users space in arg
               return -EFAULT;

            d->interruptQ.out = (d->interruptQ.out + 1) % SIZE;  //increment the read pointer
         }
         else //wait_event was interrupted by a signal (e.g. ctrl-c) < 0, or timeout = 0
            return -ERESTARTSYS;  //user mode ioctl returns -1

         break;

      case 'R': //reset DMC-18x0/6
         if(d->pciInfo.m_fSubSystem == DMC18x6) //18x6 (do nothing on 18x2)
            outb(0x80, d->baseAddress + 8); //reset 18x6
      break;

			case 'v':
				printk(KERN_INFO "GalilIoctl IOCTL_GALIL_VERSION\n");
				if (copy_to_user((char*)arg, "galiltools", strlen("galiltools")))
					return -EACCES;
				break;

      default:
         return -ENOTTY; //invalid ioctl code (switch fell thru).  POSIX says we should return -ENOTTY instead of EINVAL      
   }

   return 0; //normal 
}

//___________________________________________________________________________________________
static irqreturn_t EvtInterruptIsr(int irq, void *dev_id) // struct pt_regs *regs)  //Kernel 2.6.19 removed regs argument http://lwn.net/Articles/202449/
{
	unsigned char flagByte;
   unsigned char statusByte;  DeviceContext *d = dev_id; //get a pointer to the device.  This was passed to request_irq()
	int dataRecordSize;

#if 1
	switch(d->pciInfo.m_fSubSystem)
	{
	case DMC18x2:
      if(0x45 == (inl(d->irqAddress) & 0x45)) //this is our interrupt (bit 0 local interrupts enabled, bit 2 status (our)s, bit 6 enable (set below))
      {
         unsigned char status = inb(d->baseAddress + 1); //read 4701 status register (clear 4701 IRQ flag)

         if(status & 0x04) //bit 2 means mailbox interrupt (controller sending status byte to PC)
         {
            outb(0x06, d->baseAddress + 1);   //write 6 to N+1 (set 4701 pointer register to point to other port's mailbox)
            statusByte = inb(d->baseAddress + 1); //read status byte (4701 other port's mailbox)
         } else //controller probably set MR bit (bit 7) of 4701 (by hard resetting controller)
         {
            outb(0x02, d->baseAddress + 1);  //set 4701 pointer register to point to the interrupt mask register (register 2)    
            outb(0x04, d->baseAddress + 1);  //interrupt when the controller writes to its mailbox (don't interrupt on 6 full, 5 empty, 4 almost full, 3 almost empty, 1 byte detect, nor 0 parity/frame error
            //below is necessary so 18x2 doesn't keep interrupting
            outb(0x06, d->baseAddress + 1);  //set 4701 pointer register to point to the other port's mailbox
            inb(d->baseAddress + 1);         //read other port's mailbox (clears any pending mailbox interrupt)
	         return IRQ_HANDLED; //Tell WDF, and hence Windows, that there WAS an interrupt outstanding for us to process
         }
      } else
      	return IRQ_NONE;
		break;
	case DMC18x6:
      if(0x60 == (inb(d->baseAddress + 4) & 0x60)) //if the Galil controller generated an interrupt and interrupts were enabled on the controller
      {
         statusByte = inb(d->baseAddress + 8); //read status byte (must happen before acknowledging the interrupt below)
         outb(0x20 | inb(d->baseAddress + 4), d->baseAddress + 4); //acknowledge the interrupt (else the controller will keep interrupting)       
      } else
      	return IRQ_NONE;
		break;
	case DMC1640:
	case DMC18x0:
		flagByte = inb(d->baseAddress + 4);
		if ( d->pciInfo.m_fRevsion == REV1 )
		{
	      if(0x60 == (flagByte & 0x60)) //if the Galil controller generated an interrupt and interrupts were enabled on the controller
	      {
	         statusByte = inb(d->baseAddress + 8); //read status byte (must happen before acknowledging the interrupt below)
	         outb(flagByte, d->baseAddress + 4); //acknowledge the interrupt (else the controller will keep interrupting)       
	      } else
	      	return IRQ_NONE;
		} else
		{
	      if(0x60 == (flagByte & 0x60)) //if the Galil controller generated an interrupt and interrupts were enabled on the controller
	      {
	         statusByte = inb(d->baseAddress + 8); //read status byte (must happen before acknowledging the interrupt below)
	         outb(0x20 | flagByte, d->baseAddress + 4); //acknowledge the interrupt (else the controller will keep interrupting)       
	      } else
	      	return IRQ_NONE;
		}
	break;
	case DMC1417:
		if(0x45 == (inb(d->irqAddress) & 0x45))
		{
			statusByte = 0xFF; // 1417 interrupt need to be read via IV command
			outb(0x05, d->irqAddress + 1); // stop Interrupt
		} else 
			return IRQ_NONE;
		break;
	default:
		;
	};
#else
   if(d->irqAddress) //18x2
   {
      if(0x45 == (inl(d->irqAddress) & 0x45)) //this is our interrupt (bit 0 local interrupts enabled, bit 2 status (our)s, bit 6 enable (set below))
      {
         unsigned char status = inb(d->baseAddress + 1); //read 4701 status register (clear 4701 IRQ flag)

         if(status & 0x04) //bit 2 means mailbox interrupt (controller sending status byte to PC)
         {
            outb(0x06, d->baseAddress + 1);   //write 6 to N+1 (set 4701 pointer register to point to other port's mailbox)
            statusByte = inb(d->baseAddress + 1); //read status byte (4701 other port's mailbox)
            goto DPC;
         }
         else //controller probably set MR bit (bit 7) of 4701 (by hard resetting controller)
         {
            outb(0x02, d->baseAddress + 1);  //set 4701 pointer register to point to the interrupt mask register (register 2)    
            outb(0x04, d->baseAddress + 1);  //interrupt when the controller writes to its mailbox (don't interrupt on 6 full, 5 empty, 4 almost full, 3 almost empty, 1 byte detect, nor 0 parity/frame error
            //below is necessary so 18x2 doesn't keep interrupting
            outb(0x06, d->baseAddress + 1);  //set 4701 pointer register to point to the other port's mailbox
            inb(d->baseAddress + 1);         //read other port's mailbox (clears any pending mailbox interrupt)
         }

         return IRQ_HANDLED; //Tell WDF, and hence Windows, that there WAS an interrupt outstanding for us to process
      }
   }
   else //18x6
   {
      if(0x60 == (inb(d->baseAddress + 4) & 0x60)) //if the Galil controller generated an interrupt and interrupts were enabled on the controller
      {
         statusByte = inb(d->baseAddress + 8); //read status byte (must happen before acknowledging the interrupt below)
         outb(0x20 | inb(d->baseAddress + 4), d->baseAddress + 4); //acknowledge the interrupt (else the controller will keep interrupting)       
         goto DPC;
      }
   }
   return IRQ_NONE; //not our interrupt, another device sharing the IRQ generated the interrupt    

DPC: 

#endif

//   printk("EvtInterruptIsr 0x%X\n", statusByte);
   switch(statusByte)
   {
      case 0xBA: break; //command done (colon sent)
      case 0xBB: break; //MG
      case 0xBC:        //record
      	
         if( (d->recordAddress != 0) && (d->recordQ.out != (d->recordQ.in + 1) % SIZE) ) //circular buffer NOT FULL (will lose new status bytes if full--old ones are retained)
         {
				if ( d->pciInfo.m_fRevsion == REV2 )
   	         memcpy_fromio(d->recordQ.q[ d->recordQ.in ], (void __iomem *)d->recordAddress, 512); //copy 512 bytes from the dual port RAM
				else
				{
					dataRecordSize = 0;
					memset(d->recordQ.q[ d->recordQ.in ], 0, sizeof(d->recordQ.q[ d->recordQ.in ]));
					while ( (0x00 == (inb(d->baseAddress+4) & 0x80)) && dataRecordSize < 512 )
					{
//						printk(KERN_INFO "DR read flag[%X]\n", inb(d->baseAddress+4));
						d->recordQ.q[ d->recordQ.in ][dataRecordSize++] = inb(d->baseAddress+0xC);
					}
					printk("PLX data record size = %i\n", dataRecordSize);
				}
            d->recordQ.in = (d->recordQ.in + 1) % SIZE;   //increment in (write) pointer
            wake_up_interruptible(&(d->recordWaitQ)); //wake up ioctl
         } //else FULL 
         break;

      default: //EI (0xCx, Dx, Ex)  and UI (0xf0 to ff) //CACHE the status byte for later retreival the next time ioctl is called from user mode                   
         if( d->interruptQ.out != (d->interruptQ.in + 1) % SIZE ) //circular buffer NOT FULL (will lose new status bytes if full--old ones are retained)
         {
            d->interruptQ.q[ d->interruptQ.in ] = statusByte;   //store status byte read above
            d->interruptQ.in = (d->interruptQ.in + 1) % SIZE;   //increment in (write) pointer
            wake_up_interruptible(&(d->interruptWaitQ)); //wake up ioctl
         } //else FULL 
                                       
         break;
   }//switch

   return IRQ_HANDLED;
}
//___________________________________________________________________________________________
//called when the open() system call is invoked from user space.  Implements exclusive open (Chapter 6 Single-Open Devices)
int EvtDeviceFileOpen(struct inode *inode, struct file *filep) //Create in WDF
{
   int i = 0;
//   printk("EvtDeviceFileOpen %i %i\n", imajor(inode), iminor(inode));
      
   for(i = 0; i <= device; i++) //search for the minor number
      if(d[i].galilpci_miscdev.minor == iminor(inode))
         break; //found it

   if(d[i].bOpen) //file already open
      return -EBUSY;      //fail the open
   else
   {
      filep->private_data = &d[i];	//initialize the private data for this file descriptor.  This will be accessed in read() and write()
      d[i].bOpen = 1;                //file is open now
      return 0;                              //successful open
   }
}
//___________________________________________________________________________________________
//called when the close() system call is invoked from user space
int EvtFileClose(struct inode *inode, struct file *filep)
{
   int i = 0;
//   printk("EvtFileClose %i %i\n", imajor(inode), iminor(inode));
   
   for(i = 0; i <= device; i++) //search for the minor number
      if(d[i].galilpci_miscdev.minor == iminor(inode))
         break; //found it
      
   d[i].bOpen = 0;                //file is closed now
   return 0;     
}
//___________________________________________________________________________________________
//file operations structure
static struct file_operations  galilpci_fops = {
   .owner      = THIS_MODULE,

   .read       = EvtIoRead,
   .write      = EvtIoWrite,
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35))
	.ioctl      = EvtIoDeviceControl, 
#else
   .unlocked_ioctl      = EvtUnlockedIoDeviceControl,
#endif

   .open       = EvtDeviceFileOpen,
   .release    = EvtFileClose,
};
//___________________________________________________________________________________________
//PCI device ID structure.  EvtDevicePrepareHardware will be called for each board that matches one the of the following
//sets of vendor ID, device ID, subsystem vendor ID, and subsystem device ID.  This is a filter.

static struct pci_device_id pcigalil_pci_tbl[] __devinitdata = {
   {.vendor = 0x10B5, .device = 0x9050, .subvendor = 0x1079, .subdevice = 0x1640},  //DMC-16x0  could set .subdevice to PCI_ANY_ID
   {.vendor = 0x10B5, .device = 0x9050, .subvendor = 0x1079, .subdevice = 0x1800},  //DMC-18x0
   {.vendor = 0x10B5, .device = 0x9050, .subvendor = 0x1079, .subdevice = 0x1842},  //DMC-18x2
   {.vendor = 0x10B5, .device = 0x9050, .subvendor = 0x1079, .subdevice = 0x1806},  //DMC-18x6
  { 0 },
};

MODULE_DEVICE_TABLE(pci, pcigalil_pci_tbl); //let module loading system know what module works with what hardware devices
//___________________________________________________________________________________________
//PCI "probe" function called when the kernel finds a Galil board (matching pcigalil_pci_tbl).
//0 is normal return.  Called once per board.  If the function doesn't want to control the device
//or an error occurs, returns a negative value.  This function stores the minor number 
//in galilpci_miscdev.minor so that open() can later figure out which board is being opened,
//and set the correct filep->private_data (so that read() and write() know which ioaddr to access).

static int __devinit EvtDevicePrepareHardware(struct pci_dev  *pdev, const struct pci_device_id  *ent)
{
   int ret = -EIO;
   u16 subVender;
   u16 uModel;    //will contain galil model number (e.g. 0x1802)
	long resourceStart, resourceEnd, resourceLen;
   char name[10]; //will contain name in /dev/ (e.g. galilpci0)
   device++;      //incremented each time be find a galil board
   
   if (device >= MAX_DEVICES) {
      printk("EvtDevicePrepareHardware This driver only supports %i devices\n", MAX_DEVICES);
      device--; //update the device count
      return -ENODEV;
   }
   
   if (pci_enable_device(pdev)) {  //wake up the device
      printk("EvtDevicePrepareHardware Not possible to enable PCI Device\n");
      device--; //update the device count
      return -ENODEV;
   }

   //Mark all PCI regions associated with PCI device pdev as being reserved by owner res_name.
   //Do not access any address inside the PCI regions unless this call returns successfully.
   //This will reverve both i/o ports and memory regions, and shows up in /proc/ioports and /proc/iomem
   if (pci_request_regions(pdev, GALILNAME)) {
      printk("EvtDevicePrepareHardware I/O address (0x%04x) already in use\n", (unsigned int) d[device].baseAddress);
      device--; //update the device count
      return -ENODEV;
   }
   
   //I/O ADDRESSES
   d[device].baseAddress   = pci_resource_start(pdev, /*BAR*/ 2); //get base address N from BAR2
   d[device].irqAddress    = pci_resource_start(pdev, /*BAR*/ 1); //get base address N from BAR2.  0 on 18x0/2.  Non zero on 18x2.
   resourceStart = pci_resource_start(pdev, /*BAR*/ 0); 
	if ( resourceStart == 0x0000 )
	{
		printk("No data record channel\n");
		resourceEnd = 0;
		resourceLen = 0;
	} else
	{
		resourceEnd = pci_resource_end(pdev, /*BAR*/ 0); 
		resourceLen = resourceEnd-resourceStart+1;
	}
	printk("Bar 0 address: 0x%X to 0x%X, len=%i\n", resourceStart, resourceEnd, resourceLen);

	if ( resourceLen == 128 )
	{
//	   d[device].recordAddress = pci_iomap(pdev, /*BAR*/ 0, 128 /*bytes*/);
		d[device].pciInfo.m_fRevsion = REV1; // PLX based device
		d[device].recordAddress = resourceStart;
		printk("PLX based controller - not supported\n");
      device--; //update the device count
      return -ENODEV;
	} else if ( resourceLen == 512 )
	{
		d[device].pciInfo.m_fRevsion = REV2; // XININX based device
		d[device].recordAddress = pci_iomap(pdev, 0, 512);
		printk("XINUNX based controller\n");
	}

   if (d[device].baseAddress == 0x0000) { //check base address.
      printk("EvtDevicePrepareHardware No Main I/O-Address for device detected\n");
      ret = -ENODEV;
      goto release_regions;
   }
   printk("Base Address: 0x%X\n", d[device].baseAddress);
   printk("Record Address: 0x%X\n", d[device].recordAddress);

   // Identify PCI device
   pci_read_config_word(pdev, PCI_SUBSYSTEM_VENDOR_ID, &subVender);    //read galil model (e.g. 0x1806)
   pci_read_config_word(pdev, PCI_SUBSYSTEM_ID, &uModel);    //read galil model (e.g. 0x1806)
   d[device].pciInfo.m_fSubVendor = subVender;
   d[device].pciInfo.m_fSubSystem = uModel;
   
  //register the device under major number 10.  Minor number will show up in /proc/misc
   d[device].galilpci_miscdev.minor  = MISC_DYNAMIC_MINOR, //assign the minor number dynamically (ask for a free one).  This field will eventually contain the actual minor number.
   sprintf(name, GALILNAME "%i", device);
   d[device].galilpci_miscdev.name   = name,               //the name for this device, meant for human consumption: users will find the name in the /proc/misc file.
   d[device].galilpci_miscdev.fops   = &galilpci_fops,     //the file operations
   d[device].bOpen = 0;                                    //file not open

   ret = misc_register(&(d[device].galilpci_miscdev));
   if (ret) {
      printk ("EvtDevicePrepareHardware cannot register miscdev (err=%d)\n", ret);
      goto release_regions;
   }
   
   init_waitqueue_head(&d[device].sleepQ);

   //INTERRUPTS
   d[device].interrupt = pdev->irq; //store the IRQ number so we can call free-irq in the cleanup function when the module us removed with rmmod
   if (request_irq(d[device].interrupt, EvtInterruptIsr, IRQF_SHARED, GALILNAME, &d[device])) //register the interrupt handler.  This should happen before we enable interrupts on the controller.  Kernel 2.6.18 2006Sep changed SA_SHIRQ to IRQF_SHARED http://lwn.net/Articles/2.6-kernel-api/
   {
      printk("EvtDevicePrepareHardware IRQ %x is not free\n", d[device].interrupt);
      goto misc_deregister;
   }

   init_waitqueue_head(&d[device].interruptWaitQ);         //initialize the list of processes waiting for EI/UI interrupts
   d[device].interruptQ.in = d[device].interruptQ.out = 0; //initialize circular buffer pointers

   init_waitqueue_head(&d[device].recordWaitQ);            //initialize the list of processes waiting for record interrupts
   d[device].recordQ.in    = d[device].recordQ.out    = 0; //initialize circular buffer pointers

   //ENABLE INTERRUPTS ON CONTROLLER
#if 1
	switch (d[device].pciInfo.m_fSubSystem)
	{
	case DMC18x2:
      inb(       d[device].baseAddress + 1);  //reset 4701 ("clear FIFOs").  Interrupts won't enable on 18x2 rev F without this (rev D1 doesn't require it)
      outb(0x01, d[device].baseAddress + 1);
      outb(0x80, d[device].baseAddress + 1);
      outb(0x01, d[device].baseAddress + 1);
      outb(0x80, d[device].baseAddress + 1);
      inb(       d[device].baseAddress + 1);

      outb(0x02, d[device].baseAddress + 1);  //set 4701 pointer register to point to the interrupt mask register (register 2)    
      outb(0x04, d[device].baseAddress + 1);  //interrupt when the controller writes to its mailbox (don't interrupt on 6 full, 5 empty, 4 almost full, 3 almost empty, 1 byte detect, nor 0 parity/frame error
      //below is necessary so 18x2 doesn't keep interrupting
      outb(0x06, d[device].baseAddress + 1);  //set 4701 pointer register to point to the other port's mailbox
      inb (      d[device].baseAddress + 1);  //read other port's mailbox (clears any pending mailbox interrupt)

      d[device].irqAddress += 0x4c;           //76 byte offset from the address in BAR1
      outl( 0x40 | inl(d[device].irqAddress), d[device].irqAddress); //enable interrupts on the controller.  They will stay enabled until the module is unloaded (or power is shut off)
		break;
	case DMC18x6:
	case DMC18x0:
      outb(0x40 | inb(d[device].baseAddress + 4), d[device].baseAddress + 4); //enable interrupts on the controller.  They will stay enabled until the module is unloaded (or power is shut off)
		break;
	case DMC1417:
		d[device].irqAddress += 0x4c;
		outb(0x40 | inb(d[device].irqAddress), d[device].irqAddress); //enable interrupts on the controller.  They will stay enabled until the module is unloaded (or power is shut off)
		break;
	case DMC1640:
	default:
		;
	};
#else
   if(d[device].irqAddress) //18x2
   {
      inb(       d[device].baseAddress + 1);  //reset 4701 ("clear FIFOs").  Interrupts won't enable on 18x2 rev F without this (rev D1 doesn't require it)
      outb(0x01, d[device].baseAddress + 1);
      outb(0x80, d[device].baseAddress + 1);
      outb(0x01, d[device].baseAddress + 1);
      outb(0x80, d[device].baseAddress + 1);
      inb(       d[device].baseAddress + 1);

      outb(0x02, d[device].baseAddress + 1);  //set 4701 pointer register to point to the interrupt mask register (register 2)    
      outb(0x04, d[device].baseAddress + 1);  //interrupt when the controller writes to its mailbox (don't interrupt on 6 full, 5 empty, 4 almost full, 3 almost empty, 1 byte detect, nor 0 parity/frame error
      //below is necessary so 18x2 doesn't keep interrupting
      outb(0x06, d[device].baseAddress + 1);  //set 4701 pointer register to point to the other port's mailbox
      inb (      d[device].baseAddress + 1);  //read other port's mailbox (clears any pending mailbox interrupt)

      d[device].irqAddress += 0x4c;           //76 byte offset from the address in BAR1
      outl( 0x40 | inl(d[device].irqAddress), d[device].irqAddress); //enable interrupts on the controller.  They will stay enabled until the module is unloaded (or power is shut off)
   }
   else //18x6   
      outb(0x40 | inb(d[device].baseAddress + 4), d[device].baseAddress + 4); //enable interrupts on the controller.  They will stay enabled until the module is unloaded (or power is shut off)
#endif

   printk("EvtDevicePrepareHardware I/O address (0x%lx), Model %x, minor number %d Rev(%i)\n", 
		d[device].baseAddress, uModel, d[device].galilpci_miscdev.minor, d[device].pciInfo.m_fRevsion);

   return 0;

misc_deregister:   misc_deregister(&(d[device].galilpci_miscdev));  //unregister the device with major number 10  
release_regions:   pci_release_regions(pdev);
disable_device:    pci_disable_device(pdev);
   device--; //update the device count
   return ret;
}
//___________________________________________________________________________________________
//Called once per board upon rmmod
static void __devexit EvtDeviceReleaseHardware(struct pci_dev *pdev)
{
//   printk("EvtDeviceReleaseHardware\n");
   misc_deregister(&(d[device].galilpci_miscdev));  //unregister the device with major number 10

#if 1
	switch (d[device].pciInfo.m_fSubSystem)
	{
	case DMC18x2:
      outl(~0x40 & inl(d[device].irqAddress), d[device].irqAddress);
		break;
	case DMC18x6:
	case DMC18x0:
      outb(~0x40 & inb(d[device].baseAddress + 4), d[device].baseAddress + 4);
		break;
	case DMC1417:
		outl(~0x40 & inl(d[device].irqAddress), d[device].irqAddress);
		break;
	case DMC1640:
	default:
		;
	};
#else
   if(d[device].irqAddress) //18x2
      outl(~0x40 & inl(d[device].irqAddress),      d[device].irqAddress     ); //disable interrupts on the controller.  They will stay disabled until the module is loaded again.
   else //18x6
      outb(~0x40 & inb(d[device].baseAddress + 4), d[device].baseAddress + 4); //disable interrupts on the controller.  They will stay disabled until the module is loaded again.
#endif

   free_irq(d[device].interrupt, &d[device]); //should happen after interrupts are disabled on the controller

   pci_release_regions(pdev);				 //relase the I/O ports and memory regions
   pci_disable_device(pdev);
   device--;
}
//___________________________________________________________________________________________
//PCI driver structure
static struct pci_driver   pcigalil_driver =
{
   .name     = GALILNAME,                       //name of driver (must be unique among all PCI drivers).  Shows up under /sys/bus/pci/drivers
   .id_table = pcigalil_pci_tbl,				     //list of PCI IDs this driver supports
   .probe    = EvtDevicePrepareHardware,				  //called when a board is found
   .remove   = __devexit_p(EvtDeviceReleaseHardware), //called on exit
};
//___________________________________________________________________________________________
//called when module is inserted into the kernel (insmod)
static int __init DriverEntry(void)
{
   printk("DriverEntry\n");
   return pci_register_driver(&pcigalil_driver); //calls EvtDevicePrepareHardware.  pci_module_init() is obselete
}
//___________________________________________________________________________________________
//called when module is removed from kernel (rmmod)
static void __exit DriverExit(void)
{
   printk("DriverExit\n");
   pci_unregister_driver(&pcigalil_driver); //calls EvtDeviceReleaseHardware
}
//___________________________________________________________________________________________
module_init(DriverEntry);
module_exit(DriverExit);


/*
[root@localhost PCIDriver]# ls
galilpci.c  Makefile
[root@localhost PCIDriver]# make #build the pci driver
make -C /lib/modules/2.6.11-1.1369_FC4/build SUBDIRS=/root/galilpci modules
make[1]: Entering directory `/usr/src/kernels/2.6.11-1.1369_FC4-i686'
  CC [M]  /root/galilpci/galilpci.o
  Building modules, stage 2.
  MODPOST
  CC      /root/galilpci/galilpci.mod.o
  LD [M]  /root/galilpci/galilpci.ko
make[1]: Leaving directory `/usr/src/kernels/2.6.11-1.1369_FC4-i686'
[root@localhost PCIDriver]# insmod galilpci.ko #load the module into the kernel
[root@localhost PCIDriver]# lsmod | grep galilpci #the galilpci kernel module should show up here
galilpci                5416  0
[root@localhost PCIDriver]# ls /dev/g* #the device file of the same name should show up here
/dev/galilpci0  /dev/gpmctl
[root@localhost PCIDriver]# g=/dev/galilpci0 #set an environment variable so we type less
[root@localhost PCIDriver]# echo -en ER0\\r > $g #send a command to the board.  The red LED should turn on
[root@localhost PCIDriver]# cat $g #get the colon response
:[root@localhost PCIDriver]#
[root@localhost PCIDriver]# echo -en ER-1\\r > $g && cat $g && echo  #send a command and get the response.  The red LED should turn off
:
[root@localhost PCIDriver]# cat > test.dmc #create a small program to download to the controller
#AUTO
ER0
WT1000
ER-1
WT1000
JP#AUTO


[root@localhost PCIDriver]# cat test.dmc
#AUTO
ER0
WT1000
ER-1
WT1000
JP#AUTO

[root@localhost PCIDriver]# #now download the file we just created
[root@localhost PCIDriver]# echo -en DL\\r > $g && cat test.dmc | tr \\n \\r > $g && echo -en \\ > $g && cat $g
[root@localhost PCIDriver]# echo -en LS\\r > $g && cat $g && echo  #list the program
0 #AUTO
1 ER0
2 WT1000
3 ER-1
4 WT1000
5 JP#AUTO
6
7
:
[root@localhost PCIDriver]# echo -en XQ\\r > $g && cat $g && echo  #run the program (the red LED should blink every second)
:
[root@localhost PCIDriver]# #now upload the program to a file
[root@localhost PCIDriver]# echo -en UL\\r > $g && cat $g | tr -d \\r:\\032 > ul.dmc
[root@localhost PCIDriver]# cat ul.dmc
#AUTO
ER0
WT1000
ER-1
WT1000
JP#AUTO
*/

/*
#0.0.0.2  2008-02-25 Add interrupt-driven DR (need DR fw for 18x2), Add EI/UI support, Add register-level reset for 18x6 OS
#1.0.0.0  2008-05-16 Same as 0.0.0.2
*/
