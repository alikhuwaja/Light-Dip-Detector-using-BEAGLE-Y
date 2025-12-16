#define _POSIX_C_SOURCE 200809L

#include "sampler.h"
#include "dip_detector.h"
#include "periodTimer.h"
#include "udp.h"
#include "hal/light_sensor.h"
#include "hal/pwm_led.h"
#include "hal/encoder.h"


#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <signal.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <math.h>
#include <arpa/inet.h>
#include <errno.h>
#include <pthread.h>
#include <sys/socket.h>
#include <netinet/in.h> 
#include <stdatomic.h>


static pthread_t  thr;
static int  sock= -1;
static  _Atomic bool *stop=  NULL;
static _Atomic bool running =  false;

static struct sockaddr_storage client; // client address
static socklen_t  client_length=0; // client's address size
static _Atomic bool have_client =  false;

static char command[16] =  "";

#ifndef MAXIMUM_SEND
#define MAXIMUM_SEND 1500 
#endif
//maximum packet to send


static int last_dips= 0;


//functoin to analyse the dips last second
//helper functions



static void analyse_last_second_dips(void)
{

    int n= 0;
    double* h=Sampler_getHistory(&n);
    if (!h || n <=0)
    {
        last_dips=0;
        free(h);
        return;
    }

    double average = Sampler_getAverageReading();
    DipConfig config= Dip_default(); //initializinf  the dip detector
    last_dips =  Dip_count(h,n, average, &config);
    free(h);
}


static int get_last_second_dips(void)
{
    return last_dips;

}

//repalcing th eh /r or /n from teh string and pad with 0;

static void trim(char *s)
{
    size_t n =  strlen(s);
    if(n==0)
    {
        return;
    }
    while (n > 0 && ((s[n-1] == '\r') ||( s[n-1] == '\n'))) 
    {
        --n;
        s[n] = '\0';
    }

}

static int is_blank(const char *s)
{
    while ((*s==' ')||(*s=='\t')||(*s=='\r')||(*s=='\n')) 
    {
        s++;
    }
    return *s=='\0';
}

static int send_to_client(const void *buf, size_t len, const struct sockaddr *p, socklen_t pl)
{
    return sendto(sock, buf, len, 0, p, pl) == (ssize_t)len;
}

//API

bool udp_send(const void *data, size_t len)
 {
    if (!have_client) 
    {
        return false;
    }

    return send_to_client(data, len, (struct sockaddr*)&client, client_length);
}

//END

static void help(const struct sockaddr *p, socklen_t pl)
{
    const char *m =
        "Accepted command examples:\n"
        "count -- get the total number of samples taken.\n"
        "length -- get the number of samples taken in the previously completed\n"
        "second.\n"
        "dips -- get the number of dips in the previously completed second.\n"
        "history -- get all the samples in the previously completed second.\n"
        "stop -- cause the server program to end.\n"
        "<enter> -- repeat last command.\n";
    send_to_client(m, strlen(m), p, pl);
}

static void count(const struct sockaddr *p, socklen_t pl) 
{
    char out[64];
    int n = snprintf(out, sizeof(out), "# samples taken total: %lld\n", Sampler_getNumSamplesTaken());
    send_to_client(out, (size_t)n, p, pl);
}

static void length(const struct sockaddr *p, socklen_t pl)
 {
    char out[64];
    int n = snprintf(out, sizeof(out), "# samples taken last second: %d\n", Sampler_getHistorySize());
    send_to_client(out, (size_t)n, p, pl);
}

static void dips(const struct sockaddr *p, socklen_t pl)
 {
    analyse_last_second_dips();
    int d = get_last_second_dips();
    char out[64];
    
    int n = snprintf(out, sizeof(out), "# Dips: %d\n", d);
    send_to_client(out, (size_t)n, p, pl);
}

static void send_history(const struct sockaddr *addr, socklen_t addr_len)
{
    int count = 0;
    double *samples = Sampler_getHistory(&count);
    char out[MAXIMUM_SEND];
    size_t used = 0;
    int on_line = 0;

    for (int i = 0; i < count; i++)
    {
       
        char tok[32];
        int tok_len;
        if (on_line == 0)
        {
            tok_len = snprintf(tok, sizeof(tok), "%.3f", samples[i]);
        }
        else
        {
            tok_len = snprintf(tok, sizeof(tok), ", %.3f", samples[i]);
        }

        if (used + (size_t)tok_len > sizeof(out))
         {
            send_to_client(out, used, addr, addr_len);
            used = 0;
        }

        memcpy(out + used, tok, (size_t)tok_len);
        used += (size_t)tok_len;
        on_line++;

        if (on_line == 10)
         {
            if (used + 1 > sizeof(out)) 
            {
                send_to_client(out, used, addr, addr_len);
                used = 0;
            }
            out[used++] = '\n';
            on_line = 0;
        }
    }

    
    if (on_line != 0)
     {
        if (used + 1 > sizeof(out)) 
        {
            send_to_client(out, used, addr, addr_len);
            used = 0;
        }
        out[used++] = '\n';
    }

    if (used)
     {
        send_to_client(out, used, addr, addr_len);
    }

    free(samples);
}




static void *worker(void *unused)
{


    (void)unused;
    running= true;

    while(true)
    {
        if(stop && atomic_load(stop))
        {
            break;
        }

        char buf[1024];
        struct sockaddr_storage from;
        socklen_t from_len= sizeof(from);

        ssize_t n = recvfrom(sock, buf, sizeof(buf) - 1, 0, (struct sockaddr *)&from, &from_len);
        if (n < 0)
        {
            if (errno == EINTR)
            {
                continue;
            }
            if (!atomic_load(&running))
            {
                break;

            }
            
            continue;
        }

        buf[n]= '\0';
        trim(buf);

        memcpy(&client, &from, from_len);
        client_length= from_len;
        atomic_store(&running, true);

        const char *cmd = buf;
        char repeat[16];

        if (is_blank(buf))
        {
            if (!command[0])
            {
                const char *msg = "(no last command)\n";
                send_to_client(msg, strlen(msg),(struct sockaddr *)&from, from_len);
                continue;
            }
            snprintf(repeat, sizeof(repeat), "%s", command);
            cmd = repeat;
        }



        if (!strcmp(cmd, "help") || !strcmp(cmd, "?"))
        {
            help((struct sockaddr *)&from, from_len);
            snprintf(command, sizeof(command), "%s", cmd);
        }
         else if (!strcmp(cmd, "count"))
        {
            count((struct sockaddr *)&from, from_len);
            snprintf(command, sizeof(command), "count");
        }

         else if (!strcmp(cmd, "length"))
        {
            length((struct sockaddr *)&from, from_len);
            snprintf(command, sizeof(command), "length");
        }

         else if (!strcmp(cmd, "dips"))
        {
            dips((struct sockaddr *)&from, from_len);
            snprintf(command, sizeof(command), "dips");
        }

        else if (!strcmp(cmd, "history"))
        {
            send_history((struct sockaddr *)&from, from_len);
            snprintf(command, sizeof(command), "history");
        }

        else if (!strcmp(cmd, "stop"))
        {
            const char *msg = "Program terminating.\n";
            send_to_client(msg, strlen(msg), (struct sockaddr *)&from, from_len);
            if (stop) atomic_store(stop, true);   
            break; 
        }


        else
        {
            char msg[96];
            int m = snprintf(msg, sizeof(msg), "Unknown: \"%s\". Try 'help'.\n", cmd);
            send_to_client(msg, (size_t)m,(struct sockaddr *)&from, from_len);
        }
    }

    atomic_store(&running, false);
    return NULL;
}


bool udp_start(uint16_t port, _Atomic bool *request_exit)
{

    if(atomic_load(&running))
    {
        return true;
    }

    stop=  request_exit;

    sock= socket(AF_INET, SOCK_DGRAM,0);
    if(sock <0)
    {
        return false;
    }

    int yes = 1;
    (void)setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

    struct sockaddr_in addr = {0};
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port        = htons(port);

    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        close(sock);
        sock = -1;
        return false;
    }

    if (pthread_create(&thr, NULL, worker, NULL) != 0)
    {
        close(sock);
        sock = -1;
        return false;
    }
    atomic_store(&running, true);
    return true;
}


void udp_stop(void)
{
    if (stop) atomic_store(stop, true);

    if (sock >= 0)
    {
        int s = sock;
        sock = -1;
        close(s);
    }

    if (thr)
    {
        (void)pthread_join(thr, NULL);
        thr = 0;
    }

    atomic_store(&running, false);
    have_client = false;
    command[0] = '\0';
}












