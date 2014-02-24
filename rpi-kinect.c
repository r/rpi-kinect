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

#define KINECT_MOTOR_CTRL_BUFFER_SIZE     8

#ifdef CONFIG_USB_DYNAMIC_MINORS
#define ML_MINOR_BASE   0
#else
#define ML_MINOR_BASE   96
#endif

struct usb_kinect_motor {
    struct usb_device   *udev;
    struct usb_interface *interface;
    unsigned char       minor;

    int                 open_count;     /* Open count for this port */
    struct              semaphore sem;  /* Locks this structure */
    spinlock_t          cmd_spinlock;   /* locks dev->command */

    char                *ctrl_buffer;   /* 8 byte buffer for the control msg */
    struct urb          *ctrl_urb;
    struct usb_ctrlrequest *ctrl_dr;    /* Setup packet information */

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

static void kinect_motor_abort_transfers(struct usb_kinect_motor *dev)
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

    if (dev->ctrl_urb)
        usb_kill_urb(dev->ctrl_urb);
}

static inline void kinect_motor_delete(struct usb_kinect_motor *dev)
{
    kinect_motor_abort_transfers(dev);

    kfree(dev->ctrl_buffer);
    kfree(dev->ctrl_dr);
    kfree(dev);
}

static void kinect_motor_ctrl_callback(struct urb *urb)
{
    struct usb_kinect_motor *dev = urb->context;
}

/*! \brief called when the /dev/kinect-motor%d is opened.
 *
 * \param inode the inode of the file 
 * \param file the file object
 * \return 0 if all went well
 */
static int kinect_motor_open(struct inode *inode, struct file *file)
{
    struct usb_kinect_motor *dev = NULL;
    struct usb_interface *interface;
    int subminor;
    int retval = 0;

    subminor = iminor(inode);

    // synchronize calls between kinect_motor_open and kinect_motor_disconnect
    mutex_lock(&disconnect_mutex);

    // find the interface that is registered to this subminor
    // number. error out if we can't find one.
    interface = usb_find_interface(&kinect_motor_driver, subminor);
    if (! interface) {
        DBG_ERR("can't find device for minor %d", subminor);
        retval = -ENODEV;
        goto exit;
    }

    // find the usb_kinect_motor structure that we have associated
    // with this interface. error out if we can't find one.
    dev = usb_get_intfdata(interface);
    if (! dev) {
        retval = -ENODEV;
        goto exit;
    }

    /* lock this device */
    if (down_interruptible(&dev->sem)) {
        DBG_ERR("sem down failed");
        retval = -ERESTARTSYS;
        goto unlock_exit;
    }

    /* Increment our usage count for the device. */
    ++dev->open_count;
    if (dev->open_count > 1)
        DBG_DEBUG("open_count = %d", dev->open_count);

    file->private_data = dev;
    
unlock_exit:
    up(&dev->sem);

exit:
    mutex_unlock(&disconnect_mutex);
    return retval;
}

static int kinect_motor_release(struct inode *inode, struct file *file)
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
        up (&dev->sem); /* Unlock here as kinect_motor_delete frees dev. */
        kinect_motor_delete(dev);
        goto exit;
    }

    if (dev->open_count > 1)
        DBG_DEBUG("open_count = %d", dev->open_count);

    kinect_motor_abort_transfers(dev);
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
    __u8 cmd;

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
    DBG_DEBUG("received %d", command);
    if ((command < -128) || (command > 128)) {
      DBG_ERR("illegal range for motor movement");
      retval = -0x2a; // not sure what this means
      goto unlock_exit;
    }

    dev->ctrl_dr->bRequestType = 0x40;
    dev->ctrl_dr->bRequest = 0x31;
    dev->ctrl_dr->wValue = cpu_to_le16(command);
    dev->ctrl_dr->wIndex = cpu_to_le16(0x0000);
    dev->ctrl_dr->wLength = cpu_to_le16(KINECT_MOTOR_CTRL_BUFFER_SIZE);
    usb_fill_control_urb(dev->ctrl_urb, dev->udev,
			 usb_sndctrlpipe(dev->udev, 0),
			 (unsigned char *)dev->ctrl_dr,
			 dev->ctrl_buffer,
			 KINECT_MOTOR_CTRL_BUFFER_SIZE,
			 kinect_motor_ctrl_callback,
			 dev);

    retval = usb_submit_urb(dev->ctrl_urb, GFP_ATOMIC);
    if (retval) {
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
    .release =  kinect_motor_release,
};

static struct usb_class_driver kinect_motor_class = {
    .name = "kinect-motor%d",
    .fops = &kinect_motor_fops,
    .minor_base = ML_MINOR_BASE,
};

static int kinect_motor_probe(struct usb_interface *interface, const struct usb_device_id *id)
{

  struct usb_device *udev;
  struct usb_kinect_motor *dev = NULL;
  int retval, response;

  udev = interface_to_usbdev(interface);
  retval = -ENODEV;

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
  
  init_MUTEX(&dev->sem);
  spin_lock_init(&dev->cmd_spinlock);
  
  dev->udev = udev;
  dev->interface = interface;

  /* Set up the control URB. */
  DBG_DEBUG("setting up the control URB");
  dev->ctrl_urb = usb_alloc_urb(0, GFP_KERNEL);
  if (! dev->ctrl_urb) {
    DBG_ERR("could not allocate ctrl_urb");
    retval = -ENOMEM;
    goto error;
  }
  
  // the buffer that we would use when we want to send commands? -- 8 bytes
  DBG_DEBUG("setting up the control buffer");
  dev->ctrl_buffer = kzalloc(KINECT_MOTOR_CTRL_BUFFER_SIZE, GFP_KERNEL);
  if (! dev->ctrl_buffer) {
    DBG_ERR("could not allocate ctrl_buffer");
    retval = -ENOMEM;
    goto error;
  }
  
  dev->ctrl_dr = kmalloc(sizeof(struct usb_ctrlrequest), GFP_KERNEL);
  if (! dev->ctrl_dr) {
    DBG_ERR("could not allocate usb_ctrlrequest");
    retval = -ENOMEM;
    goto error;
  }

// do a synchronous call to transmit to the driver
  DBG_DEBUG("calling usb_control_msg");
  response = usb_control_msg(dev->udev,
			     usb_rcvctrlpipe(dev->udev, 0),
			     0x10,
			     0xC0,
			     cpu_to_le16(0x0000),
			     cpu_to_le16(0x0000),
			     dev->ctrl_buffer,
			     KINECT_MOTOR_CTRL_BUFFER_SIZE,
			     0);
  if (response < 0) {
    DBG_ERR("calling usb_control_msg = %d", response);
    goto exit;
  }
  DBG_DEBUG("received %d bytes from usb_control_msg", response);
  DBG_DEBUG("received %d", dev->ctrl_buffer[0]);
  if (dev->ctrl_buffer[0] != 0x22) {
    DBG_ERR("did not receive the correct response from the kinect");
    goto error;
  }

  DBG_DEBUG("moving motor to zero position");
  response = usb_control_msg(dev->udev,
			     usb_sndctrlpipe(dev->udev, 0),
			     0x31,
			     0x40,
			     cpu_to_le16(0x0000),
			     cpu_to_le16(0x0000),
			     dev->ctrl_buffer,
			     KINECT_MOTOR_CTRL_BUFFER_SIZE,
			     0);
  if (response < 0) {
    DBG_ERR("calling usb_control_msg = %d", response);
    goto exit;
  }
			     
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
  
  DBG_INFO("kinect motor now attached to /dev/kinect-motor%d", interface->minor - ML_MINOR_BASE);
  
 exit:
  return retval;
  
 error:
  return -1;
 /*  kinect_motor_delete(dev); */
 /*  return retval; */
}

static void kinect_motor_disconnect(struct usb_interface *interface)
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
        kinect_motor_delete(dev);
    } else {
        dev->udev = NULL;
        up(&dev->sem);
    }

    mutex_unlock(&disconnect_mutex);

    DBG_INFO("kinect motor /dev/kinect-motor%d now disconnected", minor - ML_MINOR_BASE);
}

static struct usb_driver kinect_motor_driver = {
  .name = "kinect_motor",
  .id_table = kinect_motor_table,
  .probe = kinect_motor_probe,
  .disconnect = kinect_motor_disconnect,
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
