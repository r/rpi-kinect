#include <linux/module.h>
#include <linux/init.h>

#include <linux/slab.h>         /* kmalloc() */
#include <linux/usb.h>          /* USB stuff */
#include <linux/mutex.h>        /* mutexes */
#include <linux/ioctl.h>

#include <asm/uaccess.h>        /* copy_*_user */

#include "debug.h" 
#include "semaphore.h"

/* 
 * kinect motor driver
 */
#define KINECT_MOTOR_VENDOR_ID    0x045e
#define KINECT_MOTOR_PRODUCT_ID   0x02b0

#define ML_CTRL_BUFFER_SIZE     8
#define ML_CTRL_REQEUST_TYPE    0x21
#define ML_CTRL_REQUEST         0x09
#define ML_CTRL_VALUE           0x0 
#define ML_CTRL_INDEX           0x0

#define ML_STOP         0x00
#define ML_UP           0x01
#define ML_DOWN         0x02
#define ML_LEFT         0x04
#define ML_RIGHT        0x08
#define ML_UP_LEFT      (ML_UP | ML_LEFT)
#define ML_DOWN_LEFT    (ML_DOWN | ML_LEFT)
#define ML_UP_RIGHT     (ML_UP | ML_RIGHT)
#define ML_DOWN_RIGHT   (ML_DOWN | ML_RIGHT)
#define ML_FIRE         0x10

#define ML_MAX_UP       0x80        /* 80 00 00 00 00 00 00 00 */
#define ML_MAX_DOWN     0x40        /* 40 00 00 00 00 00 00 00 */
#define ML_MAX_LEFT     0x04        /* 00 04 00 00 00 00 00 00 */
#define ML_MAX_RIGHT    0x08        /* 00 08 00 00 00 00 00 00 */

#ifdef CONFIG_USB_DYNAMIC_MINORS
#define ML_MINOR_BASE   0
#else
#define ML_MINOR_BASE   96
#endif

struct usb_kinect_motor {
    struct usb_device   *udev;
    struct usb_interface *interface;
    unsigned char       minor;
    char                serial_number[8];

    int                 open_count;     /* Open count for this port */
    struct              semaphore sem;  /* Locks this structure */
    spinlock_t          cmd_spinlock;   /* locks dev->command */

    char                *int_in_buffer;
    struct usb_endpoint_descriptor *int_in_endpoint;
    struct urb          *int_in_urb;
    int                 int_in_running;

    char                *ctrl_buffer;   /* 8 byte buffer for the control msg */
    struct urb          *ctrl_urb;
    struct usb_ctrlrequest *ctrl_dr;    /* Setup packet information */
    int                 correction_required;

    __u8                command;        /* Last issued command */
};

static struct usb_device_id kinect_motor_table [] = {
    { USB_DEVICE(KINECT_MOTOR_VENDOR_ID, KINECT_MOTOR_PRODUCT_ID) },
    { }
};
MODULE_DEVICE_TABLE (usb, kinect_motor_table);

static int debug_level = DEBUG_LEVEL_INFO;
static int debug_trace = 0;
module_param(debug_level, int, S_IRUGO | S_IWUSR);
module_param(debug_trace, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(debug_level, "debug level (bitmask)");
MODULE_PARM_DESC(debug_trace, "enable function tracing");

/* Prevent races between open() and disconnect */
static DEFINE_MUTEX(disconnect_mutex);
static struct usb_driver kinect_motor_driver;

static inline void ml_debug_data(const char *function, int size,
        const unsigned char *data)
{
    int i;

    if ((debug_level & DEBUG_LEVEL_DEBUG) == DEBUG_LEVEL_DEBUG) {
        printk(KERN_DEBUG "[debug] %s: length = %d, data = ", function, size);
        for (i = 0; i < size; ++i)
            printk("%.2x ", data[i]);
        printk("\n");
    }
}

static void ml_abort_transfers(struct usb_kinect_motor *dev)
{
    if (! dev) { 
        DBG_ERR("dev is NULL");
        return;
    }

    if (! dev->udev) {
        DBG_ERR("udev is NULL");
        return;
    }

    if (dev->udev->state == USB_STATE_NOTATTACHED) {
        DBG_ERR("udev not attached");
        return;
    }

    /* Shutdown transfer */
    if (dev->int_in_running) {
        dev->int_in_running = 0;
        mb();
        if (dev->int_in_urb)
            usb_kill_urb(dev->int_in_urb);
    }

    if (dev->ctrl_urb)
        usb_kill_urb(dev->ctrl_urb);
}

static inline void ml_delete(struct usb_kinect_motor *dev)
{
    ml_abort_transfers(dev);

    /* Free data structures. */
    if (dev->int_in_urb)
        usb_free_urb(dev->int_in_urb);
    if (dev->ctrl_urb)
        usb_free_urb(dev->ctrl_urb);

    kfree(dev->int_in_buffer);
    kfree(dev->ctrl_buffer);
    kfree(dev->ctrl_dr);
    kfree(dev);
}

static void ml_ctrl_callback(struct urb *urb)
{
    struct usb_kinect_motor *dev = urb->context;
    dev->correction_required = 0;   /* TODO: do we need race protection? */
}

static void ml_int_in_callback(struct urb *urb)
{
    struct usb_kinect_motor *dev = urb->context;
    int retval;

    ml_debug_data(__FUNCTION__, urb->actual_length, urb->transfer_buffer);

    if (urb->status) {
        if (urb->status == -ENOENT ||
                urb->status == -ECONNRESET ||
                urb->status == -ESHUTDOWN) {
            return;
        } else {
            DBG_ERR("non-zero urb status (%d)", urb->status);
            goto resubmit; /* Maybe we can recover. */
        }
    }

    if (urb->actual_length > 0) {
        spin_lock(&dev->cmd_spinlock);

        if (dev->int_in_buffer[0] & ML_MAX_UP && dev->command & ML_UP) {
            dev->command &= ~ML_UP;
            dev->correction_required = 1;
        } else if (dev->int_in_buffer[0] & ML_MAX_DOWN && 
                dev->command & ML_DOWN) {
            dev->command &= ~ML_DOWN;
            dev->correction_required = 1;
        }

        if (dev->int_in_buffer[1] & ML_MAX_LEFT && dev->command & ML_LEFT) {
            dev->command &= ~ML_LEFT;
            dev->correction_required = 1;
        } else if (dev->int_in_buffer[1] & ML_MAX_RIGHT && 
                dev->command & ML_RIGHT) {
            dev->command &= ~ML_RIGHT;
            dev->correction_required = 1;
        }


        if (dev->correction_required) {
            dev->ctrl_buffer[0] = dev->command;
            spin_unlock(&dev->cmd_spinlock);
            retval = usb_submit_urb(dev->ctrl_urb, GFP_ATOMIC);
            if (retval) {
                DBG_ERR("submitting correction control URB failed (%d)",
                        retval);
            } 
        } else {
            spin_unlock(&dev->cmd_spinlock);
        }
    }

resubmit:
    /* Resubmit if we're still running. */
    if (dev->int_in_running && dev->udev) {
        retval = usb_submit_urb(dev->int_in_urb, GFP_ATOMIC);
        if (retval) {
            DBG_ERR("resubmitting urb failed (%d)", retval);
            dev->int_in_running = 0;
        }
    }
}


static int kinect_motor_open(struct inode *inode, struct file *file)
{
    struct usb_kinect_motor *dev = NULL;
    struct usb_interface *interface;
    int subminor;
    int retval = 0;

    subminor = iminor(inode);

    mutex_lock(&disconnect_mutex);

    interface = usb_find_interface(&kinect_motor_driver, subminor);
    if (! interface) {
        DBG_ERR("can't find device for minor %d", subminor);
        retval = -ENODEV;
        goto exit;
    }

    dev = usb_get_intfdata(interface);
    if (! dev) {
        retval = -ENODEV;
        goto exit;
    }

    /* lock this device */
    if (down_interruptible (&dev->sem)) {
        DBG_ERR("sem down failed");
        retval = -ERESTARTSYS;
        goto exit;
    }

    /* Increment our usage count for the device. */
    ++dev->open_count;
    if (dev->open_count > 1)
        DBG_DEBUG("open_count = %d", dev->open_count);

    /* /\* Initialize interrupt URB. *\/ */
    /* usb_fill_int_urb(dev->int_in_urb, dev->udev, */
    /*         usb_rcvintpipe(dev->udev, dev->int_in_endpoint->bEndpointAddress), */
    /*         dev->int_in_buffer, */
    /*         le16_to_cpu(dev->int_in_endpoint->wMaxPacketSize), */
    /*         ml_int_in_callback, */
    /*         dev, */
    /*         dev->int_in_endpoint->bInterval); */

    /* dev->int_in_running = 1; */
    /* mb(); */

    /* retval = usb_submit_urb(dev->int_in_urb, GFP_KERNEL); */
    /* if (retval) { */
    /*     DBG_ERR("submitting int urb failed (%d)", retval); */
    /*     dev->int_in_running = 0; */
    /*     --dev->open_count; */
    /*     goto unlock_exit; */
    /* } */

    /* Save our object in the file's private structure. */
    file->private_data = dev;
    
unlock_exit:
    up(&dev->sem);

exit:
    mutex_unlock(&disconnect_mutex);
    return retval;
}

static int ml_release(struct inode *inode, struct file *file)
{
    struct usb_kinect_motor *dev = NULL;
    int retval = 0;

    dev = file->private_data;

    if (! dev) {
        DBG_ERR("dev is NULL");
        retval =  -ENODEV;
        goto exit;
    }

    /* Lock our device */
    if (down_interruptible(&dev->sem)) {
        retval = -ERESTARTSYS;
        goto exit;
    }

    if (dev->open_count <= 0) {
        DBG_ERR("device not opened");
        retval = -ENODEV;
        goto unlock_exit;
    }

    if (! dev->udev) {
        DBG_DEBUG("device unplugged before the file was released");
        up (&dev->sem); /* Unlock here as ml_delete frees dev. */
        ml_delete(dev);
        goto exit;
    }

    if (dev->open_count > 1)
        DBG_DEBUG("open_count = %d", dev->open_count);

    ml_abort_transfers(dev);
    --dev->open_count;

unlock_exit:
    up(&dev->sem);

exit:
    return retval;
}

static ssize_t kinect_motor_write(struct file *file, const char __user *user_buf, size_t count, loff_t *ppos)
{
    struct usb_kinect_motor *dev;
    int retval = 0;
    u8 buf[8];
    __u8 cmd = ML_STOP;

    dev = file->private_data;

    /* Lock this object. */
    if (down_interruptible(&dev->sem)) {
        retval = -ERESTARTSYS;
        goto exit;
    }

    /* Verify that the device wasn't unplugged. */
    if (! dev->udev) {
        retval = -ENODEV;
        DBG_ERR("No device or device unplugged (%d)", retval);
        goto unlock_exit;
    }

    /* Verify that we actually have some data to write. */
    if (count == 0)
        goto unlock_exit;

    /* We only accept one-byte writes. */
    if (count != 1)
        count = 1;

    if (copy_from_user(&cmd, user_buf, count)) {
        retval = -EFAULT;
        goto unlock_exit;
    }

    signed char command = (signed char)cmd;
    DBG_INFO("received %d", command);
    if ((command < -128) || (command > 128)) {
      DBG_ERR("illegal range for motor movement");
      retval = -0x2a; // not sure what this means
      goto unlock_exit;
    }

    /* /\* FIXME: does this impose too much policy restrictions? *\/ */
    /* if (! (cmd == ML_STOP || cmd == ML_UP || cmd == ML_DOWN || cmd == ML_LEFT */
    /*             || cmd == ML_RIGHT || cmd == ML_UP_LEFT || cmd == ML_DOWN_LEFT */
    /*             || cmd == ML_UP_RIGHT || cmd == ML_DOWN_RIGHT  */
    /*             || cmd == ML_FIRE)) { */
    /*     DBG_ERR("illegal command issued"); */
    /*     retval = -0x2a;     /\* scnr *\/ */
    /*     goto unlock_exit; */
    /* } */

    /* memset(&buf, 0, sizeof(buf)); */
    /* buf[0] = cmd; */

    /* /\* The interrupt-in-endpoint handler also modifies dev->command. *\/ */
    /* spin_lock(&dev->cmd_spinlock); */
    /* dev->command = cmd; */
    /* spin_unlock(&dev->cmd_spinlock); */

    /* retval = usb_control_msg(dev->udev, */
    /*         usb_sndctrlpipe(dev->udev, 0), */
    /*         ML_CTRL_REQUEST, */
    /*         ML_CTRL_REQEUST_TYPE, */
    /*         ML_CTRL_VALUE, */
    /*         ML_CTRL_INDEX, */
    /*         &buf, */
    /*         sizeof(buf), */
    /*         HZ*5);   */

    /* if (retval < 0) { */
    /*     DBG_ERR("usb_control_msg failed (%d)", retval); */
    /*     goto unlock_exit; */
    /* } */

    /* /\* We should have written only one byte. *\/ */
    /* retval = count;  */

    retval = usb_control_msg(dev->udev,
			     usb_sndctrlpipe(dev->udev, 0),
			     0x31,
			     0x40,
			     cpu_to_le16(command),
			     cpu_to_le16(0x0000),
			     dev->ctrl_buffer,
			     ML_CTRL_BUFFER_SIZE,
			     0);
    if (retval < 0) {
      DBG_ERR("usb_control_msg failed (%d)", retval);
      goto unlock_exit;
    }

unlock_exit:
    up(&dev->sem);

exit: 
    return retval;
}
    

static struct file_operations kinect_motor_fops = {
    .owner =    THIS_MODULE,
    .write =    kinect_motor_write,
    .open =     kinect_motor_open,
    .release =  ml_release,
};

static struct usb_class_driver kinect_motor_class = {
    .name = "kinect-motor%d",
    .fops = &kinect_motor_fops,
    .minor_base = ML_MINOR_BASE,
};

static int kinect_motor_probe(struct usb_interface *interface, const struct usb_device_id *id)
{

  DBG_INFO("kinect_motor_probe");

  struct usb_device *udev = interface_to_usbdev(interface);
  struct usb_kinect_motor *dev = NULL;
  struct usb_host_interface *iface_desc;
  //  struct usb_endpoint_descriptor *endpoint;
  //  int i, int_end_size;
  int retval = -ENODEV;
  
  if (! udev) {
    DBG_ERR("udev is NULL");
    goto exit;
  }
  
  dev = kzalloc(sizeof(struct usb_kinect_motor), GFP_KERNEL);
  if (! dev) {
    DBG_ERR("cannot allocate memory for struct usb_kinect_motor");
    retval = -ENOMEM;
    goto exit;
  }
  
  //  dev->command = ML_STOP;
  
  
  init_MUTEX(&dev->sem);
  spin_lock_init(&dev->cmd_spinlock);
  
  dev->udev = udev;
  dev->interface = interface;
  iface_desc = interface->cur_altsetting;

  /*
  
    we're not dealing with interrupts for the motor control

  // Set up interrupt endpoint information.
  for (i = 0; i < iface_desc->desc.bNumEndpoints; ++i) {
    endpoint = &iface_desc->endpoint[i].desc;
    
    if (((endpoint->bEndpointAddress & USB_ENDPOINT_DIR_MASK) == USB_DIR_IN)
	&& ((endpoint->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) ==
	    USB_ENDPOINT_XFER_INT)) 
      dev->int_in_endpoint = endpoint;
    
  }
  if (! dev->int_in_endpoint) {
    DBG_ERR("could not find interrupt in endpoint");
    goto error;
  }
  
  // the buffer that receives data during the interrupt?
  int_end_size = le16_to_cpu(dev->int_in_endpoint->wMaxPacketSize);
  dev->int_in_buffer = kmalloc(int_end_size, GFP_KERNEL);
  if (! dev->int_in_buffer) {
    DBG_ERR("could not allocate int_in_buffer");
    retval = -ENOMEM;
    goto error;
  }
  
  // creates a new urb for a USB driver to use for interrupts???
  dev->int_in_urb = usb_alloc_urb(0, GFP_KERNEL);
  if (! dev->int_in_urb) {
    DBG_ERR("could not allocate int_in_urb");
    retval = -ENOMEM;
    goto error;
  }
  
  */

  /* Set up the control URB. */
  DBG_INFO("setting up the control URB");
  dev->ctrl_urb = usb_alloc_urb(0, GFP_KERNEL);
  if (! dev->ctrl_urb) {
    DBG_ERR("could not allocate ctrl_urb");
    retval = -ENOMEM;
    goto error;
  }
  
  // the buffer that we would use when we want to send commands? -- 8 bytes
  DBG_INFO("setting up the control buffer");
  dev->ctrl_buffer = kzalloc(ML_CTRL_BUFFER_SIZE, GFP_KERNEL);
  if (! dev->ctrl_buffer) {
    DBG_ERR("could not allocate ctrl_buffer");
    retval = -ENOMEM;
    goto error;
  }
  
  // another buffer, but, not exactly sure what we want to use it for
  /* Setup packet information?????? */
  /* DBG_INFO("creating the packet that we are going to send"); */
  /* dev->ctrl_dr = kmalloc(sizeof(struct usb_ctrlrequest), GFP_KERNEL); */
  /* if (! dev->ctrl_dr) { */
  /*   DBG_ERR("could not allocate usb_ctrlrequest"); */
  /*   retval = -ENOMEM; */
  /*   goto error; */
  /* } */
  /* dev->ctrl_dr->bRequestType = 0xC0; */
  /* dev->ctrl_dr->bRequest = 0x10; */
  /* dev->ctrl_dr->wValue = cpu_to_le16(0x0000); */
  /* dev->ctrl_dr->wIndex = cpu_to_le16(0x0000); */
  /* dev->ctrl_dr->wLength = cpu_to_le16(0x0001); */
  
  /* usb_fill_control_urb(dev->ctrl_urb, dev->udev, */
  /* 		       usb_sndctrlpipe(dev->udev, 0), */
  /* 		       (unsigned char *)dev->ctrl_dr, */
  /* 		       dev->ctrl_buffer, */
  /* 		       ML_CTRL_BUFFER_SIZE, */
  /* 		       ml_ctrl_callback, */
  /* 		       dev); */

  // do a synchronous call to transmit to the driver
  DBG_INFO("calling usb_control_msg");
  int response = usb_control_msg(dev->udev,
				 usb_rcvctrlpipe(dev->udev, 0),
				 0x10,
				 0xC0,
				 cpu_to_le16(0x0000),
				 cpu_to_le16(0x0000),
				 dev->ctrl_buffer,
				 ML_CTRL_BUFFER_SIZE,
				 0);
  if (response < 0) {
    DBG_ERR("calling usb_control_msg = %d", response);
    goto exit;
  }
  DBG_INFO("received %d bytes from usb_control_msg", response);
  DBG_INFO("received %d", dev->ctrl_buffer[0]);
  if (dev->ctrl_buffer[0] != 0x22) {
    DBG_ERR("did not receive the correct response from the kinect");
    goto exit;
  }

  DBG_INFO("moving motor to zero position");
  response = usb_control_msg(dev->udev,
			     usb_sndctrlpipe(dev->udev, 0),
			     0x31,
			     0x40,
			     cpu_to_le16(0x0000),
			     cpu_to_le16(0x0000),
			     dev->ctrl_buffer,
			     ML_CTRL_BUFFER_SIZE,
			     0);
  if (response < 0) {
    DBG_ERR("calling usb_control_msg = %d", response);
    goto exit;
  }
			     
  /* /\* Retrieve a serial. *\/ */
  /* if (! usb_string(udev, udev->descriptor.iSerialNumber, dev->serial_number, */
  /* 		   sizeof(dev->serial_number))) { */
  /*   DBG_ERR("could not retrieve serial number"); */
  /*   goto error; */
  /* } */
  /* DBG_INFO("kinect motor serial number = %s", dev->serial_number); */
  
  /* Save our data pointer in this interface device. */
  usb_set_intfdata(interface, dev);
  
  /* We can register the device now, as it is ready. */
  retval = usb_register_dev(interface, &kinect_motor_class);
  if (retval) {
    DBG_ERR("not able to get a minor for this device.");
    usb_set_intfdata(interface, NULL);
    goto error;
  }
  
  dev->minor = interface->minor;
  
  DBG_INFO("kinect motor now attached to /dev/kinect-motor%d",
 	   interface->minor - ML_MINOR_BASE);
  
 exit:
  return retval;
  
 error:
  return -1;
 /*  ml_delete(dev); */
 /*  return retval; */
}

static void ml_disconnect(struct usb_interface *interface)
{
    struct usb_kinect_motor *dev;
    int minor;

    mutex_lock(&disconnect_mutex);  /* Not interruptible */

    dev = usb_get_intfdata(interface);
    usb_set_intfdata(interface, NULL);

    down(&dev->sem); /* Not interruptible */

    minor = dev->minor;

    /* Give back our minor. */
    usb_deregister_dev(interface, &kinect_motor_class);

    /* If the device is not opened, then we clean up right now. */
    if (! dev->open_count) {
        up(&dev->sem);
        ml_delete(dev);
    } else {
        dev->udev = NULL;
        up(&dev->sem);
    }

    mutex_unlock(&disconnect_mutex);

    DBG_INFO("USB missile launcher /dev/ml%d now disconnected", 
            minor - ML_MINOR_BASE);
}

static struct usb_driver kinect_motor_driver = {
  .name = "kinect_motor",
  .id_table = kinect_motor_table,
  .probe = kinect_motor_probe,
  .disconnect = ml_disconnect,
};

static int __init usb_kinect_init(void)
{
  int result = usb_register(&kinect_motor_driver);
  if (result) {
    DBG_ERR("registering kinect motor driver failed");
  } else {
    DBG_INFO("kinect driver motor registered successfully");
  }
  
  return result;
}


static void __exit usb_kinect_exit(void)
{
  usb_deregister(&kinect_motor_driver);
  DBG_INFO("kinect motor module deregistered");
}

module_init(usb_kinect_init);
module_exit(usb_kinect_exit);

MODULE_AUTHOR("Raffi Krikorian");
MODULE_LICENSE("GPL");
