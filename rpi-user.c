#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#define DEFAULT_DEVICE "/dev/kinect-motor0"

int main(int argc, char *argv[])
{
  char *dev = DEFAULT_DEVICE;
  int fd = open(dev, O_RDWR);
  if (fd == -1) {
    perror("open");
    exit(1);
  }

  signed char cmds[] = { 0, 45, 0, -45 };
  int c1, c2;
  int retval;
  for (c1=0;c1<=3;c1++) {
    for (c2=0;c2<=3;c2++) {
      signed char value = cmds[c2] * ((3.0-c1)/3.0);
      retval = write(fd, &value, 1);
      if (retval < 0)
	fprintf(stderr, "could not send command, %d\n", retval);
      sleep(1*((3.0-c1)/3.0)+0.25);
    }
  }
  close(fd);
}
