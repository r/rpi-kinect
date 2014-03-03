#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>

#define DEFAULT_DEVICE "/dev/kinect0"

int main(int argc, char *argv[])
{
  char *dev = DEFAULT_DEVICE;
  printf("opening...\n");
  int fd = open(dev, O_RDWR);
  if (fd == -1) {
    perror("open");
    exit(1);
  }

  char buffer[10];

  signed char cmds[] = { 0, 45, 0, -45 };
  int c1, c2;
  int retval;
  for (c1=0;c1<=3;c1++) {
    for (c2=0;c2<=3;c2++) {
      signed char value = cmds[c2] * ((3.0-c1)/3.0);
      printf("writing %d\n", value);
      retval = write(fd, &value, 1);
      if (retval < 0)
	fprintf(stderr, "could not send command, %d\n", retval);
      printf("reading\n");

      retval = read(fd, buffer, 10);
      if (retval < 0)
	fprintf(stderr, "could not read sensor data, %d\n", retval);
      printf("positive_angle_degrees = %d\n", (int)buffer[8] / 2);
      printf("accelerometer values. ux = %d, uy = %d, uz = %d\n",
	     ((uint16_t)buffer[2] << 8) | buffer[3],
	     ((uint16_t)buffer[4] << 8) | buffer[5],
	     ((uint16_t)buffer[6] << 8) | buffer[7]);

      sleep(1);
    }
  }
  printf("closing...\n");
  close(fd);
}
