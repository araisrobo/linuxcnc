/*
  vfdb_vfd.c

  userspace HAL program to control a Delta VFD-B VFD

  Yishin Li, adapted from Michael Haberler's _vfd/.

  Copyright (C) 2007, 2008 Stephen Wille Padnos, Thoth Systems, Inc.
  Copyright (C) 2009,2010,2011,2012 Michael Haberler
  Copyright (C) 2013 Yishin Li

  Based on a work (test-modbus program, part of libmodbus) which is
  Copyright (C) 2001-2005 Stéphane Raimbault <stephane.raimbault@free.fr>

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation, version 2.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this program; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307  USA.

 */

#ifndef ULAPI
#error This is intended as a userspace component only.
#endif

#ifdef DEBUG
#define DBG(fmt, ...)					\
        do {						\
            if (param.debug) printf(fmt,  ## __VA_ARGS__);	\
        } while(0)
#else
#define DBG(fmt, ...)
#endif

#include "stdio.h"
#include <unistd.h>
#include <time.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include <signal.h>
#include <errno.h>
#include <getopt.h>
#include <math.h>
#include <signal.h>
#include <stdarg.h>

#include "rtapi.h"
#include "hal.h"
#include <modbus.h>
#include <modbus-tcp.h>
#include "inifile.h"

// registers for CTEC 713P Power Meter
#define REG_PRODUCT             0x0000  // Product number and Release number
#define REG_MODEL               0x0001  // Model number

// status registers
#define SR_CH1_PWR_FACTOR       0x0092          // CH1   Power Factor, 0~+/-1000
#define SR_CH2_PWR_FACTOR       0x0093          // CH2   Power Factor, 0~+/-1000
#define SR_CH3_PWR_FACTOR       0x0094          // CH3   Power Factor, 0~+/-1000
#define SR_SIGMA_PWR_FACTOR     0x0095          // Sigma Power Factor, 0~+/-1000
#define SR_CH12_V               0x0096          // 1-2   Voltage, 32-bit
#define SR_CH23_V               0x0098          // 1-2   Voltage, 32-bit
#define SR_CH13_V               0x009A          // 1-2   Voltage, 32-bit
#define SR_CH1_V                0x009C          // CH1   Voltage, 32-bit
#define SR_CH2_V                0x009E          // CH2   Voltage, 32-bit
#define SR_CH3_V                0x00A0          // CH3   Voltage, 32-bit
#define SR_SIGMA_V              0x00A2          // SIGMA Voltage, 32-bit
#define SR_CH1_I                0x00A4          // CH1   Current, 32-bit
#define SR_CH2_I                0x00A6          // CH2   Current, 32-bit
#define SR_CH3_I                0x00A8          // CH3   Current, 32-bit
#define SR_SIGMA_I              0x00AA          // SIGMA Current, 32-bit
#define SR_CH1_W                0x00AC          // CH1   WATT, 32-bit
#define SR_CH2_W                0x00AE          // CH2   WATT, 32-bit
#define SR_CH3_W                0x00B0          // CH3   WATT, 32-bit
#define SR_SIGMA_W              0x00B2          // SIGMA WATT, 32-bit
#define SR_CH1_VA               0x00B4          // CH1   VA, 32-bit
#define SR_CH2_VA               0x00B6          // CH2   VA, 32-bit
#define SR_CH3_VA               0x00B8          // CH3   VA, 32-bit
#define SR_SIGMA_VA             0x00BA          // SIGMA VA, 32-bit
#define SR_CH1_VAR              0x00BC          // CH1   VAR, 32-bit
#define SR_CH2_VAR              0x00BE          // CH2   VAR, 32-bit
#define SR_CH3_VAR              0x00C0          // CH3   VAR, 32-bit
#define SR_SIGMA_VAR            0x00C2          // SIGMA VAR, 32-bit
#define SR_WATT_HOUR            0x00C4          // WATT-HOUR, 32-bit
#define SR_POS_WATT_HOUR        0x00C6          // Positive WATT-HOUR, 32-bit
#define SR_NEG_WATT_HOUR        0x00C8          // Negitive WATT-HOUR, 32-bit

/* HAL data struct */
typedef struct {
    hal_float_t *ch1_pwr_factor;        // output
    hal_float_t *ch2_pwr_factor;        // output
    hal_float_t *ch3_pwr_factor;        // output
    hal_s32_t *sigma_pwr_factor;      // output
    hal_float_t *ch12_v;                // output
    hal_float_t *ch23_v;                // output
    hal_float_t *ch13_v;                // output
    hal_float_t *ch1_v;                 // output
    hal_float_t *ch2_v;                 // output
    hal_float_t *ch3_v;                 // output
    hal_float_t *sigma_v;               // output
    hal_float_t *ch1_i;                 // output
    hal_float_t *ch2_i;                 // output
    hal_float_t *ch3_i;                 // output
    hal_float_t *sigma_i;               // output
    hal_float_t *ch1_w;                 // output
    hal_float_t *ch2_w;                 // output
    hal_float_t *ch3_w;                 // output
    hal_s32_t *sigma_w;               // output
    hal_float_t *ch1_va;                // output
    hal_float_t *ch2_va;                // output
    hal_float_t *ch3_va;                // output
    hal_float_t *sigma_va;              // output
    hal_float_t *ch1_var;               // output
    hal_float_t *ch2_var;               // output
    hal_float_t *ch3_var;               // output
    hal_float_t *sigma_var;             // output
    hal_float_t *watt_hour;             // output
    hal_float_t *pos_watt_hour;         // output
    hal_float_t *neg_watt_hour;         // output

    hal_bit_t   *modbus_ok;             // the last MODBUS_OK transactions returned successfully
    hal_s32_t   *errorcount;            // number of failed Modbus transactions - hints at logical errors
    hal_bit_t   *max_speed;             // 1: run as fast as possible, ignore unimportant registers
    hal_float_t looptime;
} haldata_t;

// configuration and execution state
typedef struct params {
    int type;
    char *modname;
    int modbus_debug;
    int debug;
    int slave;
    int pollcycles; 
    char *device;
    int baud;
    int bits;
    char parity;
    int stopbits;
    int rts_mode;
    int serial_mode;
    struct timeval response_timeout;
    struct timeval byte_timeout;
    int tcp_portno;
    char *progname;
    char *section;
    FILE *fp;
    char *inifile;
    int reconnect_delay;
    modbus_t *ctx;
    haldata_t *haldata;
    int hal_comp_id;
    int read_initial_done;
    int old_err_reset; 
    uint16_t old_cmd1_reg;		// copy of last write to FA00 */
    int modbus_ok;
    uint16_t failed_reg;		// remember register for failed modbus transaction for debugging
    int	last_errno;
    char *tcp_destip;
    int report_device;
} params_type, *param_pointer;

#define TYPE_RTU 0
#define TYPE_TCP_SERVER 1
#define TYPE_TCP_CLIENT 2

// param: default options; read from inifile or command line
static params_type param = {
        .type = -1,
        .modname = NULL,
        .modbus_debug = 0,
        .debug = 0,
        .slave = 1,
        .pollcycles = POLLCYCLES,
        .device = "/dev/ttyS0",
        .baud = 19200,
        .bits = 8,
        .parity = 'E',
        .stopbits = 1,
        .serial_mode = -1,
        .rts_mode = -1,
        .response_timeout = { .tv_sec = 0, .tv_usec = 500000 },
        .byte_timeout = {.tv_sec = 0, .tv_usec = 500000},
        .tcp_portno = 1502, // MODBUS_TCP_DEFAULT_PORT (502) would require root privileges
        .progname = "_vfd",
        .section = "VFS11",
        .fp = NULL,
        .inifile = NULL,
        .reconnect_delay = 1,
        .ctx = NULL,
        .haldata = NULL,
        .hal_comp_id = -1,
        .read_initial_done = 0,
        .old_err_reset = 0,
        .old_cmd1_reg = 0,
        .modbus_ok = 0,    // set modbus-ok bit if last MODBUS_OK transactions went well
        .failed_reg =0,
        .last_errno = 0,
        .tcp_destip = "127.0.0.1",
        .report_device = 0,
};

static int connection_state;
enum connstate {NOT_CONNECTED, OPENING, CONNECTING, CONNECTED, RECOVER, DONE};

static char *option_string = "dhrmn:S:I:";
static struct option long_options[] = {
        {"debug", no_argument, 0, 'd'},
        {"help", no_argument, 0, 'h'},
        {"modbus-debug", no_argument, 0, 'm'},
        {"report-device", no_argument, 0, 'r'},
        {"ini", required_argument, 0, 'I'},     // default: getenv(INI_FILE_NAME)
        {"section", required_argument, 0, 'S'}, // default section = LIBMODBUS
        {"name", required_argument, 0, 'n'},    // _vfd
        {0,0,0,0}
};


static void  windup(param_pointer p)
{
    if (p->haldata && *(p->haldata->errorcount)) {
        fprintf(stderr,"%s: %d modbus errors\n",p->progname, *(p->haldata->errorcount));
        fprintf(stderr,"%s: last command register: 0x%.4x\n",p->progname, p->failed_reg);
        fprintf(stderr,"%s: last error: %s\n",p->progname, modbus_strerror(p->last_errno));
    }
    if (p->hal_comp_id >= 0)
        hal_exit(p->hal_comp_id);
    if (p->ctx)
        modbus_close(p->ctx);
}

static void toggle_modbus_debug(int sig)
{
    param.modbus_debug = !param.modbus_debug;
    modbus_set_debug(param.ctx, param.modbus_debug);
}

static void toggle_debug(int sig)
{
    param.debug = !param.debug;
}

static void quit(int sig) 
{
    if (param.debug)
        fprintf(stderr,"quit(connection_state=%d)\n",connection_state);

    switch (connection_state) {

    case CONNECTING:  
        // modbus_tcp_accept() or TCP modbus_connect()  were interrupted
        // these wont return to the main loop, so exit here
        windup(&param);
        exit(0);
        break;

    default:
        connection_state = DONE;
        break;
    }
}

enum kwdresult {NAME_NOT_FOUND, KEYWORD_INVALID, KEYWORD_FOUND};
#define MAX_KWD 10

/**
 * findkwd -- find keyword
 * @param p
 * @param name
 * @param result
 * @param keyword
 * @param value
 * @return
 */
int findkwd(param_pointer p, const char *name, int *result, const char *keyword, int value, ...)
{
    const char *word;
    va_list ap;
    const char *kwds[MAX_KWD], **s;
    int nargs = 0;

    if ((word = iniFind(p->fp, name, p->section)) == NULL)
        return NAME_NOT_FOUND;

    kwds[nargs++] = keyword;
    va_start(ap, value);

    while (keyword != NULL) {
        if (!strcasecmp(word, keyword)) {
            *result = value;
            va_end(ap);
            return KEYWORD_FOUND;
        }
        keyword = va_arg(ap, const char *);
        kwds[nargs++] = keyword;
        if (keyword)
            value = va_arg(ap, int);
    }  
    fprintf(stderr, "%s: %s:[%s]%s: found '%s' - not one of: ", 
            p->progname, p->inifile, p->section, name, word);
    for (s = kwds; *s; s++) 
        fprintf(stderr, "%s ", *s);
    fprintf(stderr, "\n");
    va_end(ap);
    return KEYWORD_INVALID;
}

static int read_ini(param_pointer p)
{
    const char *s;
    double f;
    int value;

    if ((p->fp = fopen(p->inifile,"r")) != NULL) {
        if (!p->debug)
            iniFindInt(p->fp, "DEBUG", p->section, &p->debug);
        if (!p->modbus_debug)
            iniFindInt(p->fp, "MODBUS_DEBUG", p->section, &p->modbus_debug);
        iniFindInt(p->fp, "BITS", p->section, &p->bits);
        iniFindInt(p->fp, "BAUD", p->section, &p->baud);
        iniFindInt(p->fp, "STOPBITS", p->section, &p->stopbits);
        iniFindInt(p->fp, "TARGET", p->section, &p->slave);
        iniFindInt(p->fp, "POLLCYCLES", p->section, &p->pollcycles);
        iniFindInt(p->fp, "PORT", p->section, &p->tcp_portno);
        iniFindInt(p->fp, "RECONNECT_DELAY", p->section, &p->reconnect_delay);

        if ((s = iniFind(p->fp, "TCPDEST", p->section))) {
            p->tcp_destip = strdup(s);
        }
        if ((s = iniFind(p->fp, "DEVICE", p->section))) {
            p->device = strdup(s);
        }
        if (iniFindDouble(p->fp, "RESPONSE_TIMEOUT", p->section, &f)) {
            p->response_timeout.tv_sec = (int) f;
            p->response_timeout.tv_usec = (f-p->response_timeout.tv_sec) * 1000000;
        }
        if (iniFindDouble(p->fp, "BYTE_TIMEOUT", p->section, &f)) {
            p->byte_timeout.tv_sec = (int) f;
            p->byte_timeout.tv_usec = (f-p->byte_timeout.tv_sec) * 1000000;
        }
        value = p->parity;
        if (findkwd(p, "PARITY", &value,
                "even",'E',
                "odd", 'O',
                "none", 'N',
                NULL) == KEYWORD_INVALID)
            return -1;
        p->parity = value;

#ifdef MODBUS_RTU_RTS_UP	
        if (findkwd(p, "RTS_MODE", &p->rts_mode,
                "up", MODBUS_RTU_RTS_UP,
                "down", MODBUS_RTU_RTS_DOWN,
                "none", MODBUS_RTU_RTS_NONE,
                NULL) == KEYWORD_INVALID)
            return -1;
#else
        if (iniFind(p->fp, "RTS_MODE", p->section) != NULL) {
            fprintf(stderr,"%s: warning - the RTS_MODE feature is not available with the installed libmodbus version (%s).\n"
                    "to enable it, uninstall libmodbus-dev and rebuild with "
                    "libmodbus built http://github.com/stephane/libmodbus:master .\n",
                    LIBMODBUS_VERSION_STRING, p->progname);
        }
#endif
        if (findkwd(p,"SERIAL_MODE", &p->serial_mode,
                "rs232", MODBUS_RTU_RS232,
                "rs485", MODBUS_RTU_RS485,
                NULL) == KEYWORD_INVALID)
            return -1;

        if (findkwd(p, "TYPE", &p->type,
                "rtu", TYPE_RTU,
                "tcpserver", TYPE_TCP_SERVER,
                "tcpclient", TYPE_TCP_CLIENT,
                NULL) == NAME_NOT_FOUND) {
            fprintf(stderr, "%s: missing required TYPE in section %s\n",
                    p->progname, p->section);
            return -1;
        }
    } else {
        fprintf(stderr, "%s:cant open inifile '%s'\n",
                p->progname, p->inifile);
        return -1;
    }
    return 0;
}

static void usage(int argc, char **argv) {
    printf("Usage:  %s [options]\n", argv[0]);
    printf("This is a userspace HAL program, typically loaded using the halcmd \"loadusr\" command:\n"
            "    loadusr _vfd [options]\n"
            "Options are:\n"
            "-I or --ini <inifile>\n"
            "    Use <inifile> (default: take ini filename from environment variable INI_FILE_NAME)\n"
            "-S or --section <section-name> (default 8)\n"
            "    Read parameters from <section_name> (default 'VFS11')\n"
            "-d or --debug\n"
            "    Turn on debugging messages. Toggled by USR1 signal.\n"
            "-m or --modbus-debug\n"
            "    Turn on modbus debugging.  This will cause all modbus messages\n"
            "    to be printed in hex on the terminal. Toggled by USR2 signal.\n"
            "-r or --report-device\n"
            "    Report device properties on console at startup\n");
}

#define GETREG(reg,into)					\
        do {							\
            curr_reg = reg;						\
            if (modbus_read_registers(ctx, reg, 1, into) != 1)	\
            goto failed;					\
        } while (0)

#define GETREGS(reg,n,into)					\
        do {							\
            curr_reg = reg;						\
            if (modbus_read_registers(ctx, reg, n, into) != 1)	\
            goto failed;					\
        } while (0)


static int read_initial(modbus_t *ctx, haldata_t *haldata, param_pointer p)
{
    uint16_t curr_reg, product, model;

    if (p->report_device) {
        GETREG(REG_PRODUCT, &product);
        GETREG(REG_MODEL, &model);

        printf("%s: product: %d/0x%4.4x\n",
                p->progname, product, product);
        printf("%s: model: %d/0x%4.4x\n",
                p->progname, model, model);
    }
    return 0;

    failed:
    p->failed_reg = curr_reg;
    p->last_errno = errno;
    (*haldata->errorcount)++;
    DBG("modbus_read_registers() fail, errno(%d)\n", errno);
    if (p->debug)
        fprintf(stderr, "%s: read_initial: modbus_read_registers(0x%4.4x): %s\n",
                p->progname, curr_reg, modbus_strerror(errno));
    return p->last_errno;
}

static int read_data(modbus_t *ctx, haldata_t *haldata, param_pointer p)
{
    int retval;
    uint16_t curr_reg, val, modbus_reg[2];
    static int pollcount = 0;

    if (!p->read_initial_done) {
        if ((retval = read_initial(ctx, haldata, p)))
            return retval;
        else
            p->read_initial_done = 1;
    }

    GETREGS(SR_CH1_W, 2, modbus_reg);
    *(haldata->ch1_w) = (float *) modbus_reg;
    printf ("debug: ch1_w(0x%4.4x)\n", (int32_t *) modbus_reg);
    printf ("debug: ch1_w(%f)\n", (float *) modbus_reg);

    GETREGS(SR_SIGMA_W, 2, modbus_reg);
    *(haldata->sigma_w) = (float *) modbus_reg;
    printf ("debug: sigma_w(0x%4.4x)\n", (int32_t *) modbus_reg);
    printf ("debug: sigma_w(%d)\n", (int32_t *) modbus_reg);

    //    // we always at least read the main status register SR_INV_OPSTATUS
    //    // and current operating frequency SR_OP_FREQUENCY
    //    GETREG(SR_INV_OPSTATUS, &status_reg);
    //    *(haldata->status) = status_reg;
    //
    //    GETREG(SR_OUTPUT_FREQ, &freq_reg);
    //    *(haldata->freq_out) = freq_reg * 0.01;
    //
    //    DBG("read_data: status_reg=%4.4x freq_reg=%4.4x\n", status_reg, freq_reg);
    //
    //    // JET if freq out is 0 then the drive is stopped
    //    *(haldata->is_stopped) = (freq_reg == 0);

    p->last_errno = (retval = 0);
    return 0;

    failed:
    p->failed_reg = curr_reg;
    p->last_errno = errno;
    (*haldata->errorcount)++;
    DBG("modbus_read_registers() fail, errno(%d)\n", errno);
    if (p->debug)
        fprintf(stderr, "%s: read_data: modbus_read_registers(0x%4.4x): %s\n",
                p->progname, curr_reg, modbus_strerror(errno));
    return p->last_errno;
}

#undef GETREG

#define PIN(x)					\
        do {						\
            status = (x);					\
            if ((status) != 0)				\
            return status;				\
        } while (0)

int hal_setup(int id, haldata_t *h, const char *name)
{
    int status;
    PIN(hal_pin_bit_newf(HAL_IN, &(h->ch1_pwr_factor), id, "%s.ch1-pwr-factor", name));
    PIN(hal_pin_bit_newf(HAL_IN, &(h->ch1_pwr_factor), id, "%s.ch1-pwr-factor", name));
    PIN(hal_pin_bit_newf(HAL_IN, &(h->ch1_pwr_factor), id, "%s.ch1-pwr-factor", name));
    PIN(hal_pin_s32_newf(HAL_IN, &(h->sigma_pwr_factor), id, "%s.sigma-pwr-factor", name));
    PIN(hal_pin_bit_newf(HAL_IN, &(h->ch12_v), id, "%s.ch12-v", name));
    PIN(hal_pin_bit_newf(HAL_IN, &(h->ch23_v), id, "%s.ch23-v", name));
    PIN(hal_pin_bit_newf(HAL_IN, &(h->ch13_v), id, "%s.ch13-v", name));
    PIN(hal_pin_bit_newf(HAL_IN, &(h->ch1_v), id, "%s.ch1-v", name));
    PIN(hal_pin_bit_newf(HAL_IN, &(h->ch2_v), id, "%s.ch2-v", name));
    PIN(hal_pin_bit_newf(HAL_IN, &(h->ch3_v), id, "%s.ch3-v", name));
    PIN(hal_pin_bit_newf(HAL_IN, &(h->sigma_v), id, "%s.sigma-v", name));
    PIN(hal_pin_bit_newf(HAL_IN, &(h->ch1_i), id, "%s.ch1-i", name));
    PIN(hal_pin_bit_newf(HAL_IN, &(h->ch2_i), id, "%s.ch2-i", name));
    PIN(hal_pin_bit_newf(HAL_IN, &(h->ch3_i), id, "%s.ch3-i", name));
    PIN(hal_pin_bit_newf(HAL_IN, &(h->sigma_i), id, "%s.sigma-i", name));
    PIN(hal_pin_bit_newf(HAL_IN, &(h->ch1_w), id, "%s.ch1-w", name));
    PIN(hal_pin_bit_newf(HAL_IN, &(h->ch2_w), id, "%s.ch2-w", name));
    PIN(hal_pin_bit_newf(HAL_IN, &(h->ch3_w), id, "%s.ch3-w", name));
    PIN(hal_pin_s32_newf(HAL_IN, &(h->sigma_w), id, "%s.sigma-w", name));
    PIN(hal_pin_bit_newf(HAL_IN, &(h->ch1_va), id, "%s.ch1-va", name));
    PIN(hal_pin_bit_newf(HAL_IN, &(h->ch2_va), id, "%s.ch2-va", name));
    PIN(hal_pin_bit_newf(HAL_IN, &(h->ch3_va), id, "%s.ch3-va", name));
    PIN(hal_pin_bit_newf(HAL_IN, &(h->sigma_va), id, "%s.sigma-va", name));
    PIN(hal_pin_bit_newf(HAL_IN, &(h->ch1_var), id, "%s.ch1-var", name));
    PIN(hal_pin_bit_newf(HAL_IN, &(h->ch2_var), id, "%s.ch2-var", name));
    PIN(hal_pin_bit_newf(HAL_IN, &(h->ch3_var), id, "%s.ch3-var", name));
    PIN(hal_pin_bit_newf(HAL_IN, &(h->sigma_var), id, "%s.sigma-var", name));
    PIN(hal_pin_bit_newf(HAL_IN, &(h->watt_hour), id, "%s.watt-hour", name));
    PIN(hal_pin_bit_newf(HAL_IN, &(h->pos_watt_hour), id, "%s.pow-watt-hour", name));
    PIN(hal_pin_bit_newf(HAL_IN, &(h->net_watt_hour), id, "%s.net-watt-hour", name));


    PIN(hal_pin_bit_newf(HAL_IN, &(h->max_speed), id, "%s.max-speed", name));
    PIN(hal_pin_bit_newf(HAL_OUT, &(h->modbus_ok), id, "%s.modbus-ok", name));
    PIN(hal_pin_s32_newf(HAL_OUT, &(h->errorcount), id, "%s.error-count", name));

    PIN(hal_param_float_newf(HAL_RW, &(h->looptime), id, "%s.loop-time", name));

    return 0;
}
#undef PIN

int set_defaults(param_pointer p)
{
    haldata_t *h = p->haldata;

    *(h->ch1_pwr_factor) = 0;
    *(h->ch2_pwr_factor) = 0;
    *(h->ch3_pwr_factor) = 0;
    *(h->sigma_pwr_factor) = 0;

    *(h->ch12_v) = 0;
    *(h->ch23_v) = 0;
    *(h->ch13_v) = 0;
    *(h->ch1_v) = 0;
    *(h->ch2_v) = 0;
    *(h->ch3_v) = 0;
    *(h->sigma_v) = 0;

    *(h->ch1_i) = 0;
    *(h->ch2_i) = 0;
    *(h->ch3_i) = 0;
    *(h->sigma_i) = 0;

    *(h->ch1_w) = 0;
    *(h->ch2_w) = 0;
    *(h->ch3_w) = 0;
    *(h->sigma_w) = 0;

    *(h->ch1_va) = 0;
    *(h->ch2_va) = 0;
    *(h->ch3_va) = 0;
    *(h->sigma_va) = 0;

    *(h->ch1_var) = 0;
    *(h->ch2_var) = 0;
    *(h->ch3_var) = 0;
    *(h->sigma_var) = 0;

    *(h->watt_hour) = 0;
    *(h->pos_watt_hour) = 0;
    *(h->neg_watt_hour) = 0;

    *(h->modbus_ok) = 0;
    *(h->errorcount) = 0;
    *(h->max_speed) = 0;
    h->looptime = 0.1;

    p->failed_reg = 0;
    return 0;
}

int main(int argc, char **argv)
{
    struct timespec loop_timespec, remaining;
    int opt;
    param_pointer p = &param;
    int retval = 0;
    retval = -1;
    p->progname = argv[0];
    connection_state = NOT_CONNECTED;
    p->inifile = getenv("INI_FILE_NAME");

    while ((opt = getopt_long(argc, argv, option_string, long_options, NULL)) != -1) {
        switch(opt) {
        case 'n':
            p->modname = strdup(optarg);
            break;
        case 'm':
            p->modbus_debug = 1;
            break;
        case 'd':
            p->debug = 1;
            break;
        case 'S':
            p->section = optarg;
            break;
        case 'I':
            p->inifile = optarg;
            break;
        case 'r':
            p->report_device = 1;
            break;
        case 'h':
        default:
            usage(argc, argv);
            exit(0);
        }
    }

    if (p->inifile) {
        if (read_ini(p))
            goto finish;
        if (!p->modname)
            p->modname = "_vfd";
    } else {
        fprintf(stderr, "%s: ERROR: no inifile - either use '--ini inifile' or set INI_FILE_NAME environment variable\n", p->progname);
        goto finish;
    }

    signal(SIGINT, quit);
    signal(SIGTERM, quit);
    signal(SIGUSR1, toggle_debug);
    signal(SIGUSR2, toggle_modbus_debug);

    // create HAL component 
    p->hal_comp_id = hal_init(p->modname);
    if ((p->hal_comp_id < 0) || (connection_state == DONE)) {
        fprintf(stderr, "%s: ERROR: hal_init(%s) failed: HAL error code=%d\n",
                p->progname, p->modname, p->hal_comp_id);
        retval = p->hal_comp_id;
        goto finish;
    }

    // grab some shmem to store the HAL data in
    p->haldata = (haldata_t *)hal_malloc(sizeof(haldata_t));
    if ((p->haldata == 0) || (connection_state == DONE)) {
        fprintf(stderr, "%s: ERROR: unable to allocate shared memory\n", p->modname);
        retval = -1;
        goto finish;
    }
    if (hal_setup(p->hal_comp_id,p->haldata, p->modname))
        goto finish;

    set_defaults(p);
    hal_ready(p->hal_comp_id);

    DBG("using libmodbus version %s\n", LIBMODBUS_VERSION_STRING);

    switch (p->type) {

    case TYPE_RTU:
        connection_state = OPENING;
        if ((p->ctx = modbus_new_rtu(p->device, p->baud, p->parity, p->bits, p->stopbits)) == NULL) {
            fprintf(stderr, "%s: ERROR: modbus_new_rtu(%s): %s\n",
                    p->progname, p->device, modbus_strerror(errno));
            goto finish;
        }
        DBG("device(%s) baud(%d) parity(%s) bits(%d) stopbits(%d)\n", p->device, p->baud, &(p->parity), p->bits, p->stopbits);
        if (modbus_set_slave(p->ctx, p->slave) < 0) {
            fprintf(stderr, "%s: ERROR: invalid slave number: %d\n", p->modname, p->slave);
            goto finish;
        }
        if ((retval = modbus_connect(p->ctx)) != 0) {
            fprintf(stderr, "%s: ERROR: couldn't open serial device: %s\n", p->modname, modbus_strerror(errno));
            goto finish;
        }
        DBG("%s: serial port %s connected\n", p->progname, p->device);
        break;

    default:
        fprintf(stderr, "%s: ERROR: invalid connection type %d\n",
                p->progname, p->type);
        goto finish;
    }

    modbus_set_debug(p->ctx, p->modbus_debug);
    if (modbus_set_slave(p->ctx, p->slave) < 0) {
        fprintf(stderr, "%s: ERROR: invalid slave number: %d\n", p->modname, p->slave);
        goto finish;
    }

    connection_state = CONNECTED;
    while (connection_state != DONE) {

        while (connection_state == CONNECTED) {
            if ((retval = read_data(p->ctx, p->haldata, p))) {
                p->modbus_ok = 0;
            } else {
                p->modbus_ok++;
            }
            if (p->modbus_ok > MODBUS_MIN_OK) {
                *(p->haldata->modbus_ok) = 1;
            } else {
                *(p->haldata->modbus_ok) = 0;
            }

            /* don't want to scan too fast, and shouldn't delay more than a few seconds */
            if (p->haldata->looptime < 0.05) p->haldata->looptime = 0.05;
            if (p->haldata->looptime > 5.0) p->haldata->looptime = 5.0;
            loop_timespec.tv_sec = (time_t)(p->haldata->looptime);
            loop_timespec.tv_nsec = (long)((p->haldata->looptime - loop_timespec.tv_sec) * 1000000000l);
            if (!p->haldata->max_speed)
                nanosleep(&loop_timespec, &remaining);
        }

        switch (connection_state)
        {
        case DONE:
            // cleanup actions before exiting.
            modbus_flush(p->ctx);
            // clear the command register (control and frequency override) so panel operation gets reactivated
            if ((retval = modbus_write_register(p->ctx, REG_COMMAND1, 0)) != 1) {
                // not much we can do about it here if it goes wrong, so complain
                fprintf(stderr, "%s: failed to release VFD from bus control (write to register 0x%x): %s\n",
                        p->progname, REG_COMMAND1, modbus_strerror(errno));
            } else {
                DBG("%s: VFD released from bus control.\n", p->progname);
            }
            break;

        default:
            break;
        }
    }
    retval = 0;

    finish:
    windup(p);
    return retval;
}

