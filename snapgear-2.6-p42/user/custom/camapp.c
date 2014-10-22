#include <stdio.h>
#include <stdlib.h>	/*exit()*/
#include <string.h>	/*memset()*/
#include <fcntl.h>	/*open()*/
#include <sys/ioctl.h>	/*ioctl()*/
#include <sys/time.h>	/*timing functions*/
#include <sys/mman.h>	/*mmap()*/
#include <unistd.h>	/*read(),close()*/
#include <arpa/inet.h>	/*struct sockaddr_in */
#include <sys/socket.h>	/*socket()*/ 
#include "ceid_camera.h"

unsigned int mem_address;

void read_i2c_reg(int fd, unsigned int addr)
{
	reg_struct q;
	q.addr=addr;	
	if (ioctl(fd, CEIDCAM_RD_REG, &q) == -1)
		perror("ioctl read");
	else
		printf("Reading register 0x%02x:0x%04x\n",addr,q.val);
}

void write_i2c_reg(int fd, unsigned int addr, unsigned int val)
{
	reg_struct q;
	
	q.addr=addr;
	q.val =val;	
	if (ioctl(fd, CEIDCAM_WR_REG, &q) == -1)
		perror("ioctl read");
	else
		read_i2c_reg(fd,addr);
}

void read_sif_addr(int fd)
{
	reg_struct q;

	if (ioctl(fd, CEIDCAM_RD_SIFADDR, &q) == -1)
		perror("ioctl sif address");
	else
		mem_address = q.addr;
}


int    s;
struct sockaddr_in target_host_address;

int create_udp_socket(int port, char *dest_ip)
{
	s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (s < 0) {
		perror("socket()");
		return -1;
	}
	memset((char *)&target_host_address, 0, sizeof(target_host_address));
	target_host_address.sin_family=AF_INET;
	target_host_address.sin_addr.s_addr=INADDR_ANY;
	target_host_address.sin_port=htons(port);
	inet_pton(AF_INET,dest_ip,&target_host_address.sin_addr);
	
	return 0;
}

void send_frame(unsigned int *virt_addr)
{
	int j,i;
	register unsigned int w;
	unsigned char msg[800];// = flash;
	int ipos, slen = sizeof(target_host_address);
	unsigned int pos = (unsigned int)virt_addr;
	struct timeval st,et;

	for (j=0;j<256;j=j+1){
		msg[0] = 'I';
		msg[1] = ( 256 & 0x03FF ) >> 2 ;
		msg[2] = ( 256 & 0x03FF ) >> 2 ;
		msg[3] =  (j&0xFF00)>>8;
		msg[4] =  j&0x00FF;
		msg[5] =  1;
		ipos = 6;

		for (i=0;i<256;i++){
			w=*(unsigned int *)pos;
			/*
			msg[ipos++] = (w&(0x00FF0000))>>16;  // red
			msg[ipos++] = (w&(0xFF000000))>>24;  // green
			msg[ipos++] = (w&(0x0000FF00))>>8;   // blue
			*/
			msg[ipos++] = w>>16;  // red
			msg[ipos++] = w>>24;  // green
			msg[ipos++] = w>>8;   // blue

			pos=pos+4;
		}
		sendto(s, msg, 798, 0, &target_host_address, slen); //798 = ipos+24
	}
}

const unsigned int mem_size = 0x40000;
unsigned int  *mem_pointer, *virt_addr;

int main()
{
        int fd;
	FILE *output;
        char ch, write_buf[11];
	unsigned long uaddr,waddr,wval,count;
	char loop;
	unsigned long mem[BUFFER_SIZE * 1024];
	struct timeval st,et;
	int i,k,mem_dev,mem2;
	char dest_ip[16] = "10.21.30.49";
	int port = 1902;
	//clock_t start, end;
	//double time;
	//unsigned int image[65536];
	//reg_struct *t=(reg_struct *)malloc(sizeof(reg_struct));
	//reg_struct t;


	fd = open("/dev/ceidCam", O_RDWR);
        if (fd == -1){
                printf("Error in opening file \n");
                exit(-1);
        }
	
	//printf("R=%x",0x0 | (0x1<<15));
	printf ("'r' --> read I2C register\n"
		"'w' --> write I2C register\n"
		"'c' --> capture image and save to file\n"
		"'v' --> send video stream\n"
		"'x' --> exit\n"
		"\nSelect: ");
        scanf ("%c", &ch);
	//ch='m';
        switch (ch) {
                case 'w':
			printf ("Address to write: ");
                        scanf (" %x", &waddr);
			printf ("Value to write to 0x%02x: ",waddr);
                        scanf (" %x", &wval);
                        write_i2c_reg(fd,waddr,wval);
			//read_reg(fd,waddr);
			//write(fd, write_buf, sizeof(write_buf));
                        break;			
                case 'r':
                        //ret_addr = read(fd, read_buf, sizeof(read_buf));
                        //printf ("USER Received: %*.*s\n",8,8, read_buf);
			printf ("I2C Register to read: ");
                        scanf (" %x", &uaddr);
			//printf("USER:uaddr: 0x%04x\n",uaddr);
			read_i2c_reg(fd,uaddr);
			break;
                case 'c':
			/* IOCTL STUFF
			printf ("Start reading from memory address: ");
                        scanf (" %x", &uaddr);
			printf("USER:saddr: 0x%08x\n",uaddr);
			printf ("Count: ");
                        scanf (" %x", &count);
			printf("USER:count: %d\n",count);
			
			//dummy data
			//iaddr = 0x00000000;
			//count = 1;
			//read_mem(fd,uaddr,count);
			*/

			// Start time measurement
			gettimeofday(&st,NULL);
			//while(1){
			read(fd,mem,sizeof(mem));
			// Print debug info	
			//printf("USR: Start Buffer Value\t= 0x%lx\n",mem[0]);
			//printf("USR: End Buffer Value\t= 0x%lx\n",mem[65535]);
			
			//Save to file
			if ( (output = fopen("output", "w")) == NULL ){
				printf ("USR: Cannot create file! \n");
			}
			else{
				fwrite(mem, sizeof(unsigned long), 65536, output);
			}
			fclose(output);
			//}
			gettimeofday(&et, NULL);
			printf("\nTotal time taken is : %lu microseconds\n",((et.tv_sec - st.tv_sec)*1000000+(et.tv_usec - st.tv_usec)));
			break;
		case 'v':
			printf("Video will be streamed to IP: %s\n"
				"Do you want to change it(y/n)? ",dest_ip);
			scanf("%c",&ch); //ignore \n
			scanf("%c",&ch);
			if(ch=='y'){
				printf("Destination's IPv4 address: ");
				scanf("%s", dest_ip);
			}
			printf("Streaming to %s...\n",dest_ip);
			//udp_init(dest_ip); 
			create_udp_socket(port,dest_ip);
			read_sif_addr(fd);
			//printf("Memory Start: 0x%08x\n",mem_address);
			//break;
			mem_dev = open("/dev/mem", O_RDWR | O_SYNC);
			if(mem_dev == -1)
			{
				printf("Error opening /dev/mem\n");
			}
			
			unsigned int alloc_mem_size, page_mask, page_size;

			page_size = 4096;
			alloc_mem_size = (((mem_size / page_size) + 1) * page_size);
			page_mask = (page_size - 1);

			mem_pointer = mmap(NULL,
					   alloc_mem_size,
					   PROT_READ | PROT_WRITE,
					   MAP_SHARED,
					   mem_dev,
					   (mem_address & ~page_mask)
					   );

			if(mem_pointer == MAP_FAILED)
			{  
			      printf("MAP FAILED\n");
			}
			virt_addr = (mem_pointer + (mem_address & page_mask)); 
			//Save to file
			/*
			if ( (output = fopen("output", "w")) == NULL ){
				printf ("USR: Cannot create file! \n");
			}
			else{
			*/
			int k=0;
			while(1){
				//gettimeofday(&st,NULL);
				read(fd,mem,sizeof(mem));
				//gettimeofday(&et, NULL);
				//gettimeofday(&st,NULL);
				//if ((k % 10) == 0)
				send_frame(virt_addr);
				//fwrite(virt_addr, sizeof(unsigned long), 65536, output);
				//printf("\nTotal time taken is : %lu msec\n",((et.tv_sec - st.tv_sec)*1000000+(et.tv_usec - st.tv_usec))/1000);
				//k++;
			//	sleep(2);
			}
			//fclose(output);
			close(mem_dev);
			munmap(mem_pointer,mem_size);
			//close(mem2);
			close(s);
			
			break;
		case 'x':
			printf("Exiting...\n");
			break;	
		default:
                        printf("Wrong choice\n");
                        //break;
	}
        close(fd);
	return 0;
}
