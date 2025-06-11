/*
 * DMX Frame Player
 * Plays back DMX frame sequences from files with precise real-time timing
 * 
 * Usage: dmx_player [options] framefile
 * Options:
 *   -duration MICROSEC    Frame duration in microseconds
 *   -fps FPS             Frames per second (alternative to -duration)
 *   -start EPOCH_USEC    Start at specific epoch time (microseconds)
 *   -delay DELAY_USEC    Fine-tune delay (positive/negative microseconds)
 *   -loop                Loop playback continuously
 *   -verbose             Show detailed timing information
 * 
 * File format:
 *   Each line represents one frame with channel/value pairs:
 *   1 10 2 10 5 10 3 0
 *   1 0 2 0 5 0 3 1
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <time.h>
#include <signal.h>
#include <sys/stat.h>

/* IOCTL definitions - must match kernel driver */
#define DMX_IOC_MAGIC 'd'
#define DMX_IOC_SET_CHANNEL _IOW(DMX_IOC_MAGIC, 3, struct dmx_channel)
#define DMX_IOC_START_TX _IO(DMX_IOC_MAGIC, 5)
#define DMX_IOC_STOP_TX _IO(DMX_IOC_MAGIC, 6)

struct dmx_channel {
    unsigned short channel;  /* 1-512 */
    unsigned char value;     /* 0-255 */
};

/* Frame structure */
struct dmx_frame {
    int num_channels;
    struct dmx_channel channels[512];  /* Max channels per frame */
};

/* Global variables */
static int dmx_fd = -1;
static int verbose = 0;
static volatile int running = 1;

/* Signal handler for clean shutdown */
void signal_handler(int sig) {
    printf("\nReceived signal %d, stopping playback...\n", sig);
    running = 0;
}

void print_usage(const char *prog_name) {
    printf("DMX Frame Player v1.0\n\n");
    printf("USAGE:\n");
    printf("  %s [options] framefile\n\n", prog_name);
    printf("OPTIONS:\n");
    printf("  -duration MICROSEC    Frame duration in microseconds\n");
    printf("  -fps FPS             Frames per second (alternative to -duration)\n");
    printf("  -start EPOCH_USEC    Start at specific epoch time (microseconds)\n");
    printf("  -delay DELAY_USEC    Fine-tune delay (positive/negative microseconds)\n");
    printf("  -loop                Loop playback continuously\n");
    printf("  -verbose             Show detailed timing information\n");
    printf("  -help                Show this help\n\n");
    printf("FILE FORMAT:\n");
    printf("  Each line represents one frame with channel/value pairs:\n");
    printf("  1 10 2 10 5 10 3 0\n");
    printf("  1 0 2 0 5 0 3 1\n\n");
    printf("EXAMPLES:\n");
    printf("  %s -fps 30 sequence.txt                    # Play at 30 FPS\n", prog_name);
    printf("  %s -duration 33333 sequence.txt            # Play with 33.333ms intervals\n", prog_name);
    printf("  %s -start 1640995200000000 -delay -1000 sequence.txt  # Start at specific time with -1ms adjustment\n", prog_name);
    printf("  %s -loop -fps 25 sequence.txt              # Loop at 25 FPS\n", prog_name);
}

/* Get current time in microseconds */
long long get_time_usec() {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return (long long)ts.tv_sec * 1000000LL + ts.tv_nsec / 1000;
}

/* Sleep until specific time */
void sleep_until_usec(long long target_time_usec) {
    struct timespec ts;
    ts.tv_sec = target_time_usec / 1000000;
    ts.tv_nsec = (target_time_usec % 1000000) * 1000;
    
    while (clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &ts, NULL) == EINTR) {
        /* Retry if interrupted by signal */
    }
}

/* Set a single DMX channel */
int set_dmx_channel(int channel, int value) {
    struct dmx_channel ch;
    ch.channel = channel;
    ch.value = value;
    
    if (ioctl(dmx_fd, DMX_IOC_SET_CHANNEL, &ch) < 0) {
        if (verbose) {
            printf("Error setting channel %d to %d: %s\n", channel, value, strerror(errno));
        }
        return -1;
    }
    
    return 0;
}

/* Apply a frame to the DMX controller */
int apply_frame(struct dmx_frame *frame) {
    int errors = 0;
    
    for (int i = 0; i < frame->num_channels; i++) {
        if (set_dmx_channel(frame->channels[i].channel, frame->channels[i].value) < 0) {
            errors++;
        }
    }
    
    return errors;
}

/* Parse a frame line from the file */
int parse_frame_line(const char *line, struct dmx_frame *frame) {
    char *line_copy = strdup(line);
    char *token;
    int channel, value;
    int pair_count = 0;
    
    frame->num_channels = 0;
    
    token = strtok(line_copy, " \t\n\r");
    while (token != NULL && frame->num_channels < 512) {
        if (pair_count % 2 == 0) {
            /* Channel number */
            channel = atoi(token);
            if (channel < 1 || channel > 512) {
                printf("Error: Invalid channel %d in line: %s\n", channel, line);
                free(line_copy);
                return -1;
            }
        } else {
            /* Channel value */
            value = atoi(token);
            if (value < 0 || value > 255) {
                printf("Error: Invalid value %d in line: %s\n", value, line);
                free(line_copy);
                return -1;
            }
            
            /* Store the channel/value pair */
            frame->channels[frame->num_channels].channel = channel;
            frame->channels[frame->num_channels].value = value;
            frame->num_channels++;
        }
        
        pair_count++;
        token = strtok(NULL, " \t\n\r");
    }
    
    if (pair_count % 2 != 0) {
        printf("Error: Incomplete channel/value pairs in line: %s\n", line);
        free(line_copy);
        return -1;
    }
    
    free(line_copy);
    return 0;
}

/* Load frames from file */
int load_frames(const char *filename, struct dmx_frame **frames, int *frame_count) {
    FILE *file;
    char line[1024];
    int capacity = 100;
    int count = 0;
    
    file = fopen(filename, "r");
    if (!file) {
        printf("Error opening file %s: %s\n", filename, strerror(errno));
        return -1;
    }
    
    *frames = malloc(capacity * sizeof(struct dmx_frame));
    if (!*frames) {
        printf("Error allocating memory for frames\n");
        fclose(file);
        return -1;
    }
    
    while (fgets(line, sizeof(line), file) && running) {
        /* Skip empty lines and comments */
        if (line[0] == '\n' || line[0] == '#' || line[0] == '\0') {
            continue;
        }
        
        /* Expand array if needed */
        if (count >= capacity) {
            capacity *= 2;
            *frames = realloc(*frames, capacity * sizeof(struct dmx_frame));
            if (!*frames) {
                printf("Error reallocating memory for frames\n");
                fclose(file);
                return -1;
            }
        }
        
        /* Parse the frame */
        if (parse_frame_line(line, &(*frames)[count]) == 0) {
            count++;
        } else {
            printf("Skipping invalid line: %s", line);
        }
    }
    
    fclose(file);
    *frame_count = count;
    
    printf("Loaded %d frames from %s\n", count, filename);
    return 0;
}

int main(int argc, char *argv[]) {
    const char *filename = NULL;
    long long frame_duration_usec = 0;
    long long start_time_usec = 0;
    long long delay_usec = 0;
    int loop_playback = 0;
    int fps = 0;
    
    struct dmx_frame *frames = NULL;
    int frame_count = 0;
    
    /* Parse command line arguments */
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-duration") == 0 && i + 1 < argc) {
            frame_duration_usec = atoll(argv[++i]);
        } else if (strcmp(argv[i], "-fps") == 0 && i + 1 < argc) {
            fps = atoi(argv[++i]);
            frame_duration_usec = 1000000LL / fps;
        } else if (strcmp(argv[i], "-start") == 0 && i + 1 < argc) {
            start_time_usec = atoll(argv[++i]);
        } else if (strcmp(argv[i], "-delay") == 0 && i + 1 < argc) {
            delay_usec = atoll(argv[++i]);
        } else if (strcmp(argv[i], "-loop") == 0) {
            loop_playback = 1;
        } else if (strcmp(argv[i], "-verbose") == 0) {
            verbose = 1;
        } else if (strcmp(argv[i], "-help") == 0) {
            print_usage(argv[0]);
            return 0;
        } else if (argv[i][0] != '-') {
            filename = argv[i];
        } else {
            printf("Unknown option: %s\n", argv[i]);
            print_usage(argv[0]);
            return 1;
        }
    }
    
    /* Validate arguments */
    if (!filename) {
        printf("Error: No frame file specified\n");
        print_usage(argv[0]);
        return 1;
    }
    
    if (frame_duration_usec <= 0) {
        printf("Error: Must specify either -duration or -fps\n");
        print_usage(argv[0]);
        return 1;
    }
    
    /* Set up signal handlers */
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    /* Open DMX device */
    dmx_fd = open("/dev/dmx0", O_RDWR);
    if (dmx_fd < 0) {
        printf("Error opening /dev/dmx0: %s\n", strerror(errno));
        printf("Make sure the kernel module is loaded: sudo make install\n");
        return 1;
    }
    
    /* Start DMX transmission */
    if (ioctl(dmx_fd, DMX_IOC_START_TX) < 0) {
        printf("Warning: Could not start DMX transmission: %s\n", strerror(errno));
    }
    
    /* Load frames from file */
    if (load_frames(filename, &frames, &frame_count) < 0) {
        close(dmx_fd);
        return 1;
    }
    
    if (frame_count == 0) {
        printf("Error: No valid frames found in file\n");
        free(frames);
        close(dmx_fd);
        return 1;
    }
    
    /* Calculate start time */
    if (start_time_usec == 0) {
        /* Start at next full second */
        long long current_time = get_time_usec();
        start_time_usec = ((current_time / 1000000) + 1) * 1000000;
    }
    start_time_usec += delay_usec;
    
    printf("Frame playback configuration:\n");
    printf("  Duration: %lld μs", frame_duration_usec);
    if (fps > 0) {
        printf(" (%d FPS)", fps);
    }
    printf("\n");
    printf("  Start time: %lld μs (in %lld ms)\n", start_time_usec, 
           (start_time_usec - get_time_usec()) / 1000);
    printf("  Frame count: %d\n", frame_count);
    printf("  Loop playback: %s\n", loop_playback ? "yes" : "no");
    if (delay_usec != 0) {
        printf("  Timing adjustment: %+lld μs\n", delay_usec);
    }
    printf("\nStarting playback in ");
    
    /* Countdown */
    long long current_time = get_time_usec();
    long long wait_time = start_time_usec - current_time;
    if (wait_time > 3000000) {  /* More than 3 seconds */
        int countdown = (int)(wait_time / 1000000);
        for (int i = countdown; i > 0 && running; i--) {
            printf("%d... ", i);
            fflush(stdout);
            sleep(1);
        }
    }
    printf("NOW!\n");
    
    /* Main playback loop */
    long long frame_time = start_time_usec;
    int current_frame = 0;
    int total_frames_played = 0;
    
    do {
        for (current_frame = 0; current_frame < frame_count && running; current_frame++) {
            /* Wait for frame time */
            sleep_until_usec(frame_time);
            
            /* Apply the frame */
            int errors = apply_frame(&frames[current_frame]);
            total_frames_played++;
            
            if (verbose) {
                long long actual_time = get_time_usec();
                long long timing_error = actual_time - frame_time;
                printf("Frame %d/%d: %d channels, timing error: %+lld μs\n", 
                       current_frame + 1, frame_count, frames[current_frame].num_channels, timing_error);
                
                if (errors > 0) {
                    printf("  Warning: %d channel errors\n", errors);
                }
            } else if (total_frames_played % 100 == 0) {
                printf("Played %d frames...\n", total_frames_played);
            }
            
            /* Schedule next frame */
            frame_time += frame_duration_usec;
        }
        
        if (loop_playback && running) {
            printf("Looping playback (total frames played: %d)\n", total_frames_played);
        }
        
    } while (loop_playback && running);
    
    printf("\nPlayback completed. Total frames played: %d\n", total_frames_played);
    
    /* Stop DMX transmission */
    if (ioctl(dmx_fd, DMX_IOC_STOP_TX) < 0) {
        printf("Warning: Could not stop DMX transmission: %s\n", strerror(errno));
    }
    
    /* Cleanup */
    free(frames);
    close(dmx_fd);
    
    return 0;
} 
