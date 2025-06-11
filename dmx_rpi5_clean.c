/*
 * DMX512 Kernel Driver for Raspberry Pi 5 - CLEAN BSD VERSION
 * Optimized for fast frame rates using BSD manual break control
 * 
 * Copyright (C) 2025 DMX-RPI5 Project
 * Licensed under GPL v2
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/ktime.h>
#include <linux/hrtimer.h>
#include <linux/workqueue.h>
#include <uapi/asm-generic/ioctls.h>

/* BSD UART break commands for precise timing */
#ifndef TIOCSBRK
#define TIOCSBRK        0x5427  /* BSD start break */
#endif
#ifndef TIOCCBRK  
#define TIOCCBRK        0x5428  /* BSD stop break */
#endif

#define DRIVER_NAME "dmx_rpi5"
#define DEVICE_NAME "dmx"
#define DMX_UNIVERSE_SIZE 512
#define DMX_FRAME_SIZE (DMX_UNIVERSE_SIZE + 1)

/* DMX timing constants */
#define DMX_BREAK_TIME_US 88        /* DMX standard break - increased for compatibility */
#define DMX_MAB_TIME_US 8           /* Mark After Break - increased for compatibility */
#define DMX_REFRESH_RATE_HZ 40       /* Standard 44 Hz for optimal performance */
#define CHUNK_SIZE 512               /* NO CHUNKING - send entire frame at once */
#define INTER_CHUNK_DELAY_US 0       /* NO DELAYS between chunks */
#define MAX_RETRIES 0                /* Single retry for speed */

/* GPIO pins */
#define DMX_OUTPUT_MODE_PIN 18       /* BitWizard mode control */

/* IOCTL definitions */
#define DMX_IOC_MAGIC 'd'
#define DMX_IOC_SET_CHANNEL _IOW(DMX_IOC_MAGIC, 3, struct dmx_channel)
#define DMX_IOC_GET_CHANNEL _IOR(DMX_IOC_MAGIC, 4, struct dmx_channel)
#define DMX_IOC_START_TX _IO(DMX_IOC_MAGIC, 5)
#define DMX_IOC_STOP_TX _IO(DMX_IOC_MAGIC, 6)
#define DMX_IOC_GET_STATS _IOR(DMX_IOC_MAGIC, 7, struct dmx_stats)
#define DMX_IOC_RESET _IO(DMX_IOC_MAGIC, 8)
#define DMX_IOC_SET_ACTIVE_CHANNELS _IOW(DMX_IOC_MAGIC, 9, __u16)
#define DMX_IOC_GET_ACTIVE_CHANNELS _IOR(DMX_IOC_MAGIC, 10, __u16)

struct dmx_channel {
    __u16 channel;  /* 1-512 */
    __u8 value;     /* 0-255 */
};

struct dmx_stats {
    __u64 frames_sent;
    __u64 errors;
    __u64 last_frame_duration_ns;
    __u8 tx_running;
    __u32 frame_rate;
};

/* Clean device structure */
struct dmx_device {
    struct cdev cdev;
    struct device *dev;
    struct class *class;
    dev_t devt;
    int major;
    
    /* DMX data buffer */
    __u8 dmx_data[DMX_UNIVERSE_SIZE];
    __u16 active_channels;  /* Number of channels to transmit */
    struct mutex data_mutex;
    
    /* UART communication */
    struct file *uart_filp;
    
    /* Transmission control */
    bool tx_enabled;
    struct hrtimer tx_timer;
    struct work_struct tx_work;
    struct workqueue_struct *tx_workqueue;
    
    /* Statistics */
    atomic64_t frames_sent;
    atomic64_t errors;
    ktime_t last_frame_time;
};

static struct dmx_device *dmx_dev = NULL;

/* Module parameters */
static int use_hardware_breaks = 1;
module_param(use_hardware_breaks, int, 0644);
MODULE_PARM_DESC(use_hardware_breaks, "Use hardware breaks: 0=software, 1=BSD manual breaks (default: 1)");

static int frame_rate = DMX_REFRESH_RATE_HZ;
module_param(frame_rate, int, 0644);
MODULE_PARM_DESC(frame_rate, "DMX frame rate in Hz (1-44, default: 44)");

static int chunk_size = CHUNK_SIZE;
module_param(chunk_size, int, 0644);
MODULE_PARM_DESC(chunk_size, "Data chunk size for transmission (8-64, default: 16)");

static int inter_chunk_delay = INTER_CHUNK_DELAY_US;
module_param(inter_chunk_delay, int, 0644);
MODULE_PARM_DESC(inter_chunk_delay, "Delay between chunks in microseconds (5-100, default: 10)");

static int active_channels = 512;
module_param(active_channels, int, 0644);
MODULE_PARM_DESC(active_channels, "Number of active DMX channels to transmit (1-512, default: 30)");

/* UART device path */
static char *uart_device = "/dev/ttyAMA0";
module_param(uart_device, charp, 0644);
MODULE_PARM_DESC(uart_device, "UART device for DMX (default: /dev/ttyAMA0)");

/* GPIO pin for TX enable (optional) */
static int tx_enable_gpio = -1;
module_param(tx_enable_gpio, int, 0644);
MODULE_PARM_DESC(tx_enable_gpio, "GPIO pin for TX enable (-1 to disable)");

/* Function prototypes */
static int dmx_open(struct inode *inode, struct file *file);
static int dmx_release(struct inode *inode, struct file *file);
static long dmx_ioctl(struct file *file, unsigned int cmd, unsigned long arg);
static ssize_t dmx_read(struct file *file, char __user *buf, size_t count, loff_t *ppos);
static ssize_t dmx_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos);
static int dmx_send_break_bsd_manual(struct dmx_device *dev);
static int dmx_uart_open(struct dmx_device *dev);
static void dmx_uart_close(struct dmx_device *dev);

/* BSD MANUAL BREAK: Precise 88μs break control */
static int dmx_send_break_bsd_manual(struct dmx_device *dev)
{
    int ret;
    ktime_t start_time, end_time;
    
    if (!dev->uart_filp || !dev->uart_filp->f_op || !dev->uart_filp->f_op->unlocked_ioctl) {
        return -ENODEV;
    }
    
    start_time = ktime_get();
    
    /* Step 1: Start break - pull TX line LOW */
    ret = dev->uart_filp->f_op->unlocked_ioctl(dev->uart_filp, TIOCSBRK, 0);
    if (ret < 0) {
        return ret;
    }
    
    /* Step 2: Hold break for precise DMX break time */
    udelay(DMX_BREAK_TIME_US);  /* Precise 88μs break */
    
    /* Step 3: Stop break - TX line returns to HIGH */
    ret = dev->uart_filp->f_op->unlocked_ioctl(dev->uart_filp, TIOCCBRK, 0);
    if (ret < 0) {
        return ret;
    }
    
    /* Step 4: Mark After Break */
    udelay(DMX_MAB_TIME_US);    /* Precise 8μs MAB */
    
    end_time = ktime_get();
    
    /* Log timing for first few calls to verify precision */
    if (atomic64_read(&dev->frames_sent) < 3) {
        s64 duration_ns = ktime_to_ns(ktime_sub(end_time, start_time));
        dev_info(dev->dev, "BSD break timing: %lld ns (%lld.%03lld ms)\n", 
                 duration_ns, duration_ns / 1000000, (duration_ns / 1000) % 1000);
    }
    
    return 0;
}

/* DMX Break Generation - Clean and Simple */
static int dmx_send_break(struct dmx_device *dev)
{
    int ret;
    
    if (!dev->uart_filp) {
        return -ENODEV;
    }
    
    /* Try BSD Manual Break Control for optimal performance */
    if (use_hardware_breaks) {
        ret = dmx_send_break_bsd_manual(dev);
        if (ret == 0) {
            return 0;  /* Success with BSD manual break! */
        }
        dev_warn_once(dev->dev, "BSD manual break failed (%d), using fallback\n", ret);
    }
    
    /* Fallback: Standard UART break */
    if (dev->uart_filp->f_op && dev->uart_filp->f_op->unlocked_ioctl) {
        ret = dev->uart_filp->f_op->unlocked_ioctl(dev->uart_filp, TCSBRKP, 0);
        if (ret == 0) {
            udelay(DMX_MAB_TIME_US);
            return 0;
        }
    }
    
    /* Ultimate fallback: Software timing */
    udelay(DMX_BREAK_TIME_US);
    udelay(DMX_MAB_TIME_US);
    return 0;
}

/* DMX512 frame transmission with chunking and retry logic (from working backup) */
static int dmx_send_data(struct dmx_device *dev)
{
    __u8 frame_data[DMX_FRAME_SIZE];
    ssize_t written;
    loff_t pos = 0;
    int remaining;
    int chunk_size = 64;  /* Optimal chunk size from working version */
    int retry_count = 0;
    const int max_retries = 5;
    int frame_size;
    
    if (!dev->uart_filp) {
        atomic64_inc(&dev->errors);
        return -ENODEV;
    }
    
    /* Prepare frame: start code (0x00) + DMX data */
    frame_data[0] = 0x00;  /* DMX start code */
    
    mutex_lock(&dev->data_mutex);
    memcpy(&frame_data[1], dev->dmx_data, dev->active_channels);
    frame_size = dev->active_channels + 1;  /* +1 for start code */
    mutex_unlock(&dev->data_mutex);
    
    /* Send frame in optimized chunks with retry logic */
    remaining = frame_size;  /* Send only active channels + start code */
    pos = 0;
    
    while (remaining > 0) {
        int write_size = min(remaining, chunk_size);
        
        written = kernel_write(dev->uart_filp, &frame_data[pos], write_size, &pos);
        
        if (written > 0) {
            remaining -= written;
            retry_count = 0;  /* Reset retry count on success */
            
            /* Add minimal delays between chunks if needed */
            if (remaining > 0) {
                if (remaining > chunk_size * 2) {
                    usleep_range(80, 120);  /* Longer delay for big remaining data */
                } else {
                    udelay(25);  /* 25μs precise delay for final chunks */
                }
            }
        } else if (written == -EAGAIN || written == -EWOULDBLOCK) {
            retry_count++;
            if (retry_count > max_retries) {
                dev_err(dev->dev, "UART write timeout after %d retries\n", max_retries);
                atomic64_inc(&dev->errors);
                return -EIO;
            }
            
            /* Adaptive delay based on retry count */
            if (retry_count <= 2) {
                udelay(50);  /* 50μs precise delay for first retries */
            } else {
                usleep_range(150, 200);  /* Longer delay for persistent issues */
            }
        } else {
            dev_err(dev->dev, "UART write error: %zd\n", written);
            atomic64_inc(&dev->errors);
            return written;
        }
    }
    
    return 0;
}

/* Transmission work function */
static void dmx_tx_work_func(struct work_struct *work)
{
    struct dmx_device *dev = container_of(work, struct dmx_device, tx_work);
    ktime_t start_time, end_time;
    
    if (!dev->tx_enabled) {
        return;
    }
    
    start_time = ktime_get();
    
    /* Send DMX break with precise timing */
    if (dmx_send_break(dev) == 0) {
        /* Send frame data efficiently */
        if (dmx_send_data(dev) == 0) {
            atomic64_inc(&dev->frames_sent);
        }
    }
    
    end_time = ktime_get();
    dev->last_frame_time = ktime_sub(end_time, start_time);
}

/* High-resolution timer callback */
static enum hrtimer_restart dmx_tx_timer_callback(struct hrtimer *timer)
{
    struct dmx_device *dev = container_of(timer, struct dmx_device, tx_timer);
    
    if (dev->tx_enabled) {
        /* Queue work - no delays here */
        queue_work(dev->tx_workqueue, &dev->tx_work);
        
        /* Schedule next frame using configurable frame rate */
        hrtimer_forward_now(timer, ms_to_ktime(1000 / frame_rate));
        return HRTIMER_RESTART;
    }
    
    return HRTIMER_NORESTART;
}

/* UART File Operations */
static int dmx_uart_open(struct dmx_device *dev)
{
    struct file *filp;
    int ret;
    
    if (dev->uart_filp) {
        return 0;  /* Already open */
    }
    
    /* Open UART device - use O_RDWR and O_NONBLOCK like working version */
    filp = filp_open(uart_device, O_RDWR | O_NOCTTY | O_NONBLOCK, 0);
    if (IS_ERR(filp)) {
        ret = PTR_ERR(filp);
        dev_err(dev->dev, "Failed to open UART device %s: %d\n", uart_device, ret);
        return ret;
    }
    
    dev->uart_filp = filp;
    dev_info(dev->dev, "UART device %s opened successfully (O_RDWR | O_NONBLOCK)\n", uart_device);
    
    return 0;
}

static void dmx_uart_close(struct dmx_device *dev)
{
    if (dev->uart_filp) {
        filp_close(dev->uart_filp, NULL);
        dev->uart_filp = NULL;
        dev_info(dev->dev, "UART device closed\n");
    }
}

/* File operations structure */
static const struct file_operations dmx_fops = {
    .owner = THIS_MODULE,
    .open = dmx_open,
    .release = dmx_release,
    .read = dmx_read,
    .write = dmx_write,
    .unlocked_ioctl = dmx_ioctl,
};

/* File operations implementation */
static int dmx_open(struct inode *inode, struct file *file)
{
    if (!dmx_dev) {
        return -ENODEV;
    }
    
    file->private_data = dmx_dev;
    
    /* Open UART if not already open */
    if (!dmx_dev->uart_filp) {
        int ret = dmx_uart_open(dmx_dev);
        if (ret < 0) {
            return ret;
        }
    }
    
    return 0;
}

static int dmx_release(struct inode *inode, struct file *file)
{
    return 0;
}

static ssize_t dmx_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
    struct dmx_device *dev = file->private_data;
    size_t to_copy;
    
    if (!dev) {
        return -ENODEV;
    }
    
    if (*ppos >= DMX_UNIVERSE_SIZE) {
        return 0;
    }
    
    to_copy = min(count, (size_t)(DMX_UNIVERSE_SIZE - *ppos));
    
    mutex_lock(&dev->data_mutex);
    if (copy_to_user(buf, &dev->dmx_data[*ppos], to_copy)) {
        mutex_unlock(&dev->data_mutex);
        return -EFAULT;
    }
    mutex_unlock(&dev->data_mutex);
    
    *ppos += to_copy;
    return to_copy;
}

static ssize_t dmx_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
    struct dmx_device *dev = file->private_data;
    size_t to_copy;
    
    if (!dev) {
        return -ENODEV;
    }
    
    if (*ppos >= DMX_UNIVERSE_SIZE) {
        return -ENOSPC;
    }
    
    to_copy = min(count, (size_t)(DMX_UNIVERSE_SIZE - *ppos));
    
    mutex_lock(&dev->data_mutex);
    if (copy_from_user(&dev->dmx_data[*ppos], buf, to_copy)) {
        mutex_unlock(&dev->data_mutex);
        return -EFAULT;
    }
    mutex_unlock(&dev->data_mutex);
    
    *ppos += to_copy;
    return to_copy;
}

static long dmx_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    struct dmx_device *dev = file->private_data;
    struct dmx_channel channel;
    struct dmx_stats stats;
    
    if (!dev) {
        return -ENODEV;
    }
    
    switch (cmd) {
    case DMX_IOC_SET_CHANNEL:
        if (copy_from_user(&channel, (void __user *)arg, sizeof(channel))) {
            return -EFAULT;
        }
        
        if (channel.channel < 1 || channel.channel > DMX_UNIVERSE_SIZE) {
            return -EINVAL;
        }
        
        mutex_lock(&dev->data_mutex);
        dev->dmx_data[channel.channel - 1] = channel.value;
        mutex_unlock(&dev->data_mutex);
        
        break;
        
    case DMX_IOC_GET_CHANNEL:
        if (copy_from_user(&channel, (void __user *)arg, sizeof(channel))) {
            return -EFAULT;
        }
        
        if (channel.channel < 1 || channel.channel > DMX_UNIVERSE_SIZE) {
            return -EINVAL;
        }
        
        mutex_lock(&dev->data_mutex);
        channel.value = dev->dmx_data[channel.channel - 1];
        mutex_unlock(&dev->data_mutex);
        
        if (copy_to_user((void __user *)arg, &channel, sizeof(channel))) {
            return -EFAULT;
        }
        
        break;
        
    case DMX_IOC_START_TX:
        if (!dev->tx_enabled) {
            dev->tx_enabled = true;
            hrtimer_start(&dev->tx_timer, ms_to_ktime(1000 / frame_rate), 
                         HRTIMER_MODE_REL);
            dev_info(dev->dev, "DMX transmission started (BSD Manual Break, %d Hz, %d-byte chunks)\n", 
                    frame_rate, chunk_size);
        }
        break;
        
    case DMX_IOC_STOP_TX:
        if (dev->tx_enabled) {
            dev->tx_enabled = false;
            hrtimer_cancel(&dev->tx_timer);
            cancel_work_sync(&dev->tx_work);
            dev_info(dev->dev, "DMX transmission stopped\n");
        }
        break;
        
    case DMX_IOC_GET_STATS:
        stats.frames_sent = atomic64_read(&dev->frames_sent);
        stats.errors = atomic64_read(&dev->errors);
        stats.last_frame_duration_ns = ktime_to_ns(dev->last_frame_time);
        stats.tx_running = dev->tx_enabled;
        stats.frame_rate = frame_rate;
        
        if (copy_to_user((void __user *)arg, &stats, sizeof(stats))) {
            return -EFAULT;
        }
        break;
        
    case DMX_IOC_RESET:
        mutex_lock(&dev->data_mutex);
        memset(dev->dmx_data, 0, DMX_UNIVERSE_SIZE);
        mutex_unlock(&dev->data_mutex);
        
        atomic64_set(&dev->frames_sent, 0);
        atomic64_set(&dev->errors, 0);
        
        dev_info(dev->dev, "DMX data reset\n");
        break;
        
    case DMX_IOC_SET_ACTIVE_CHANNELS:
        {
            __u16 channels;
            if (copy_from_user(&channels, (void __user *)arg, sizeof(channels))) {
                return -EFAULT;
            }
            
            if (channels < 1 || channels > DMX_UNIVERSE_SIZE) {
                return -EINVAL;
            }
            
            mutex_lock(&dev->data_mutex);
            dev->active_channels = channels;
            mutex_unlock(&dev->data_mutex);
            
            dev_info(dev->dev, "Active channels set to %d\n", channels);
            break;
        }
        
    case DMX_IOC_GET_ACTIVE_CHANNELS:
        {
            __u16 channels;
            mutex_lock(&dev->data_mutex);
            channels = dev->active_channels;
            mutex_unlock(&dev->data_mutex);
            
            if (copy_to_user((void __user *)arg, &channels, sizeof(channels))) {
                return -EFAULT;
            }
            break;
        }
        
    default:
        return -ENOTTY;
    }
    
    return 0;
}

/* Module initialization */
static int __init dmx_init(void)
{
    int ret;
    
    /* Allocate device structure */
    dmx_dev = kzalloc(sizeof(struct dmx_device), GFP_KERNEL);
    if (!dmx_dev) {
        return -ENOMEM;
    }
    
    /* Allocate device number */
    ret = alloc_chrdev_region(&dmx_dev->devt, 0, 1, DEVICE_NAME);
    if (ret < 0) {
        pr_err("Failed to allocate device number\n");
        goto err_free_dev;
    }
    
    dmx_dev->major = MAJOR(dmx_dev->devt);
    
    /* Initialize character device */
    cdev_init(&dmx_dev->cdev, &dmx_fops);
    dmx_dev->cdev.owner = THIS_MODULE;
    
    ret = cdev_add(&dmx_dev->cdev, dmx_dev->devt, 1);
    if (ret < 0) {
        pr_err("Failed to add character device\n");
        goto err_unreg_chrdev;
    }
    
    /* Create device class */
    dmx_dev->class = class_create(DEVICE_NAME);
    if (IS_ERR(dmx_dev->class)) {
        ret = PTR_ERR(dmx_dev->class);
        pr_err("Failed to create device class\n");
        goto err_del_cdev;
    }
    
    /* Create device */
    dmx_dev->dev = device_create(dmx_dev->class, NULL, dmx_dev->devt, NULL, 
                                DEVICE_NAME "0");
    if (IS_ERR(dmx_dev->dev)) {
        ret = PTR_ERR(dmx_dev->dev);
        pr_err("Failed to create device\n");
        goto err_destroy_class;
    }
    
    /* Initialize mutex and data */
    mutex_init(&dmx_dev->data_mutex);
    memset(dmx_dev->dmx_data, 0, DMX_UNIVERSE_SIZE);
    dmx_dev->active_channels = active_channels;  /* Initialize active channels count */
    
    /* Initialize timer and work queue */
    hrtimer_init(&dmx_dev->tx_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    dmx_dev->tx_timer.function = dmx_tx_timer_callback;
    
    dmx_dev->tx_workqueue = create_singlethread_workqueue("dmx_tx");
    if (!dmx_dev->tx_workqueue) {
        ret = -ENOMEM;
        pr_err("Failed to create work queue\n");
        goto err_destroy_device;
    }
    
    INIT_WORK(&dmx_dev->tx_work, dmx_tx_work_func);
    
    /* Initialize statistics */
    atomic64_set(&dmx_dev->frames_sent, 0);
    atomic64_set(&dmx_dev->errors, 0);
    dmx_dev->tx_enabled = false;
    
    /* Initialize TX enable GPIO if specified */
    if (tx_enable_gpio >= 0) {
        ret = gpio_request_one(tx_enable_gpio, GPIOF_OUT_INIT_HIGH, "dmx_tx_enable");
        if (ret < 0) {
            pr_warn("Failed to request TX enable GPIO %d: %d\n", tx_enable_gpio, ret);
            tx_enable_gpio = -1;  /* Disable GPIO control */
        } else {
            pr_info("Using GPIO %d for TX enable\n", tx_enable_gpio);
        }
    }
    
    /* Set up GPIO 18 for DMX output mode (BitWizard requirement) */
    ret = gpio_request_one(DMX_OUTPUT_MODE_PIN, GPIOF_OUT_INIT_HIGH, "dmx_output_mode");
    if (ret < 0) {
        pr_warn("Failed to setup GPIO %d for DMX output mode: %d\n", DMX_OUTPUT_MODE_PIN, ret);
    } else {
        pr_info("GPIO %d configured for DMX output mode\n", DMX_OUTPUT_MODE_PIN);
    }
    
    /* Validate and clamp parameters for DMX512 compliance */
    if (frame_rate < 1 || frame_rate > 44) {
        pr_warn("Invalid frame rate %d, clamping to range 1-44 Hz\n", frame_rate);
        frame_rate = clamp(frame_rate, 1, 44);
    }
    
    /* Allow large chunk sizes for no-chunking operation */
    if (chunk_size < 8 || chunk_size > 512) {
        pr_warn("Invalid chunk size %d, clamping to range 8-512 bytes\n", chunk_size);
        chunk_size = clamp(chunk_size, 8, 512);
    }
    
    /* Allow zero delay for protocol compliance */
    if (inter_chunk_delay < 0 || inter_chunk_delay > 100) {
        pr_warn("Invalid inter-chunk delay %d, clamping to range 0-100 μs\n", inter_chunk_delay);
        inter_chunk_delay = clamp(inter_chunk_delay, 0, 100);
    }
    
    if (active_channels < 1 || active_channels > DMX_UNIVERSE_SIZE) {
        pr_warn("Invalid active channels %d, clamping to range 1-512\n", active_channels);
        active_channels = clamp(active_channels, 1, DMX_UNIVERSE_SIZE);
        dmx_dev->active_channels = active_channels;  /* Update device structure */
    }
    
    pr_info("DMX driver loaded successfully (DMX512 COMPLIANT, device: /dev/dmx0, major: %d)\n", 
            dmx_dev->major);
    pr_info("UART device: %s\n", uart_device);
    pr_info("Break method: %s\n", 
            use_hardware_breaks ? "BSD Manual Break (TIOCSBRK/TIOCCBRK)" : "Software timing");
    pr_info("Performance: %d Hz, %d μs break, %d μs MAB, %d active channels\n", 
            frame_rate, DMX_BREAK_TIME_US, DMX_MAB_TIME_US, dmx_dev->active_channels);
    
    /* Log transmission mode */
    if (chunk_size >= 512 && inter_chunk_delay == 0) {
        pr_info("Transmission mode: NO CHUNKING (DMX512 compliant)\n");
    } else {
        pr_info("Transmission mode: %d-byte chunks, %d μs delays\n", chunk_size, inter_chunk_delay);
    }
    
    return 0;
    
err_destroy_device:
    device_destroy(dmx_dev->class, dmx_dev->devt);
err_destroy_class:
    class_destroy(dmx_dev->class);
err_del_cdev:
    cdev_del(&dmx_dev->cdev);
err_unreg_chrdev:
    unregister_chrdev_region(dmx_dev->devt, 1);
err_free_dev:
    kfree(dmx_dev);
    dmx_dev = NULL;
    
    return ret;
}

/* Module cleanup */
static void __exit dmx_exit(void)
{
    if (!dmx_dev) {
        return;
    }
    
    /* Stop transmission */
    dmx_dev->tx_enabled = false;
    hrtimer_cancel(&dmx_dev->tx_timer);
    
    if (dmx_dev->tx_workqueue) {
        cancel_work_sync(&dmx_dev->tx_work);
        destroy_workqueue(dmx_dev->tx_workqueue);
    }
    
    /* Close UART */
    dmx_uart_close(dmx_dev);
    
    /* Release TX enable GPIO */
    if (tx_enable_gpio >= 0) {
        gpio_free(tx_enable_gpio);
    }
    
    /* Release GPIO 18 (DMX output mode) */
    gpio_free(DMX_OUTPUT_MODE_PIN);
    
    /* Cleanup device */
    device_destroy(dmx_dev->class, dmx_dev->devt);
    class_destroy(dmx_dev->class);
    cdev_del(&dmx_dev->cdev);
    unregister_chrdev_region(dmx_dev->devt, 1);
    
    kfree(dmx_dev);
    dmx_dev = NULL;
    
    pr_info("DMX driver unloaded (DMX512 COMPLIANT)\n");
}

module_init(dmx_init);
module_exit(dmx_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("DMX-RPI5 Project");
MODULE_DESCRIPTION("DMX512 UART Driver for Raspberry Pi 5 - DMX512 COMPLIANT");
MODULE_VERSION("3.0-dmx512-clean"); 
