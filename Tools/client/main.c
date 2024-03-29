#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
/* #include <stdint.h> */
#include <stdbool.h>
#include <errno.h>
#include <pthread.h>
#include <editline.h>

/* #include <editline/history.h> */

#include "uart.h"

static bool measuring = false;
static pthread_t th;
static pthread_mutex_t fp_lock;
static int fd;

static void sig_handler(int signo)
{
    if (signo == SIGINT) {
        printf("\ninterrupted by signal\n");
        measuring = false;
    }
}

static void *receive_thread(void *arg)
{
    char readbuffer[256] = { 0 };
    int size;

    while (1) {

        if (measuring) {
            pthread_mutex_lock(&fp_lock);
            size = uart_read(fd, readbuffer, 256);
            printf("[read] %s", readbuffer);
            pthread_mutex_unlock(&fp_lock);
        }
    }

    return NULL;
}

int main(int argc, char *argv[])
{
    struct sigaction actions;
    const char *port;
    int baudrate;

    port = argv[1];
    baudrate = atoi(argv[2]);

    memset(&actions, 0, sizeof(actions));
    sigemptyset(&actions.sa_mask);
    actions.sa_flags = 0;
    actions.sa_handler = sig_handler;
    sigaction(SIGINT, &actions, NULL);
    sigaction(SIGUSR1, &actions, NULL);
    sigaction(SIGPIPE, &actions, NULL);

    fd = uart_open(NULL, 0);
    if (fd < 0) {
        fprintf(stderr, "uart_open error\n");
        return -1;
    }

    pthread_mutex_init(&fp_lock, NULL);
    pthread_create(&th, NULL, receive_thread, NULL);

    while (1) {
        char *input = readline(">> ");

        if (!strcmp(input, "s") || !strcmp(input, "start")) {
            measuring = true;
            pthread_mutex_lock(&fp_lock);
            uart_write(fd, "s", 1);
            pthread_mutex_unlock(&fp_lock);
        }
        else if (!strcmp(input, "t") || !strcmp(input, "stop")) {
            uart_write(fd, "s", 1);
        }

        free(input);
    }

    measuring = false;
    pthread_kill(th, SIGUSR1);
    pthread_join(th, NULL);
    pthread_mutex_destroy(&fp_lock);

    uart_close(fd);

    return 0;
}
