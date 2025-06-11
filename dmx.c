/*
 * DMX Command Line Tool
 * Simple interface for controlling DMX channels
 * Usage: dmx [channel] [value]
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <errno.h>

/* IOCTL definitions - must match kernel driver */
#define DMX_IOC_MAGIC 'd'
#define DMX_IOC_SET_CHANNEL _IOW(DMX_IOC_MAGIC, 3, struct dmx_channel)
#define DMX_IOC_GET_CHANNEL _IOR(DMX_IOC_MAGIC, 4, struct dmx_channel)
#define DMX_IOC_START_TX _IO(DMX_IOC_MAGIC, 5)
#define DMX_IOC_STOP_TX _IO(DMX_IOC_MAGIC, 6)
#define DMX_IOC_GET_STATS _IOR(DMX_IOC_MAGIC, 7, struct dmx_stats)
#define DMX_IOC_RESET _IO(DMX_IOC_MAGIC, 8)

struct dmx_channel {
    unsigned short channel;  /* 1-512 */
    unsigned char value;     /* 0-255 */
};

struct dmx_stats {
    unsigned long long frames_sent;
    unsigned long long errors;
    unsigned long long last_frame_duration_ns;
    unsigned char tx_running;
    unsigned int frame_rate;
};

void print_usage(const char *prog_name) {
    printf("DMX512 Control Tool v2.0\n\n");
    printf("USAGE:\n");
    printf("  %s CHANNEL VALUE     Set channel (1-512) to value (0-255)\n", prog_name);
    printf("  %s get CHANNEL       Get channel value\n", prog_name);
    printf("  %s start             Start DMX transmission\n", prog_name);
    printf("  %s stop              Stop DMX transmission\n", prog_name);
    printf("  %s stats             Show transmission statistics\n", prog_name);
    printf("  %s reset             Reset all channels to 0\n", prog_name);
    printf("  %s off               SAFETY: Turn off all channels (same as reset)\n", prog_name);
    printf("  %s help              Show this help\n\n", prog_name);
    printf("EXAMPLES:\n");
    printf("  %s 1 255             # Set channel 1 to full brightness\n", prog_name);
    printf("  %s 10 128            # Set channel 10 to half brightness\n", prog_name);
    printf("  %s start             # Start transmission\n", prog_name);
    printf("  %s get 1             # Get channel 1 value\n", prog_name);
    printf("  %s stats             # Show transmission stats\n", prog_name);
    printf("  %s off               # EMERGENCY: Turn off all lighting\n", prog_name);
}

int main(int argc, char *argv[]) {
    int fd;
    struct dmx_channel channel;
    struct dmx_stats stats;
    int retval = 0;
    
    if (argc < 2) {
        print_usage(argv[0]);
        return 1;
    }
    
    /* Open DMX device */
    fd = open("/dev/dmx0", O_RDWR);
    if (fd < 0) {
        perror("Error opening /dev/dmx0");
        printf("Make sure the kernel module is loaded: sudo make install\n");
        return 1;
    }
    
    /* Parse command */
    if (strcmp(argv[1], "help") == 0) {
        print_usage(argv[0]);
    } 
    else if (strcmp(argv[1], "start") == 0) {
        if (ioctl(fd, DMX_IOC_START_TX) < 0) {
            perror("Error starting transmission");
            retval = 1;
        } else {
            printf("DMX transmission started\n");
        }
    } 
    else if (strcmp(argv[1], "stop") == 0) {
        if (ioctl(fd, DMX_IOC_STOP_TX) < 0) {
            perror("Error stopping transmission");
            retval = 1;
        } else {
            printf("DMX transmission stopped\n");
        }
    } 
    else if (strcmp(argv[1], "reset") == 0) {
        if (ioctl(fd, DMX_IOC_RESET) < 0) {
            perror("Error resetting channels");
            retval = 1;
        } else {
            printf("All channels reset to 0\n");
        }
    } 
    else if (strcmp(argv[1], "off") == 0) {
        if (ioctl(fd, DMX_IOC_RESET) < 0) {
            perror("Error turning off channels");
            retval = 1;
        } else {
            printf("SAFETY: All channels turned off (set to 0)\n");
        }
    } 
    else if (strcmp(argv[1], "stats") == 0) {
        if (ioctl(fd, DMX_IOC_GET_STATS, &stats) < 0) {
            perror("Error getting statistics");
            retval = 1;
        } else {
            printf("DMX Statistics:\n");
            printf("  Frames sent: %llu\n", stats.frames_sent);
            printf("  Errors: %llu\n", stats.errors);
            printf("  Transmission: %s\n", stats.tx_running ? "Running" : "Stopped");
            printf("  Frame rate: %u Hz\n", stats.frame_rate);
            printf("  Last frame time: %llu ns\n", stats.last_frame_duration_ns);
        }
    } 
    else if (strcmp(argv[1], "get") == 0) {
        if (argc < 3) {
            printf("Usage: %s get CHANNEL\n", argv[0]);
            retval = 1;
        } else {
            channel.channel = atoi(argv[2]);
            if (channel.channel < 1 || channel.channel > 512) {
                printf("Error: Channel must be 1-512\n");
                retval = 1;
            } else {
                if (ioctl(fd, DMX_IOC_GET_CHANNEL, &channel) < 0) {
                    perror("Error getting channel");
                    retval = 1;
                } else {
                    printf("Channel %d = %d\n", channel.channel, channel.value);
                }
            }
        }
    } 
    else if (argc >= 3) {
        /* Set channel value: dmx CHANNEL VALUE */
        channel.channel = atoi(argv[1]);
        channel.value = atoi(argv[2]);
        
        if (channel.channel < 1 || channel.channel > 512) {
            printf("Error: Channel must be 1-512\n");
            retval = 1;
        } else if (channel.value > 255) {
            printf("Error: Value must be 0-255\n");
            retval = 1;
        } else {
            if (ioctl(fd, DMX_IOC_SET_CHANNEL, &channel) < 0) {
                perror("Error setting channel");
                retval = 1;
            } else {
                printf("Set channel %d to %d\n", channel.channel, channel.value);
            }
        }
    } 
    else {
        printf("Unknown command: %s\n", argv[1]);
        print_usage(argv[0]);
        retval = 1;
    }
    
    close(fd);
    return retval;
} 
