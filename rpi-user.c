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

  signed char cmd = 0;
  int direction = 1;
  int retval;
  while (1) {
    retval = write(fd, &cmd, 1);
    if (retval < 0)
      fprintf(stderr, "could not send command, %d\n", retval);
    if ((cmd == 0) && (direction == 1))
      cmd = 45;
    else if (cmd == 45) {
      cmd = 0;
      direction = -1;
    } else if ((cmd == 0) && (direction == -1)) 
      cmd = -45;
    else {
      cmd = 0;
      direction = 1;
    }
    sleep(1);
    printf("cmd = %d\n", cmd);
  }


  /* int cmd = 45; */
  /* int retval = write(fd, &cmd, 1); */
  /* if (retval < 0) */
  /*   fprintf(stderr, "could not send command, %d\n", retval); */
  close(fd);
}
