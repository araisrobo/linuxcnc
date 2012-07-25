/********************************************************************
* Description: emcrsh.cc
*   Extended telnet based EMC interface
*
*   Derived from a work by Fred Proctor & Will Shackleford
*   Further derived from work by jmkasunich
*
* Author: Eric H. Johnson
* License: GPL Version 2
* System: Linux
*
* Copyright (c) 2006-2008 All rights reserved.
*
* Last change:
********************************************************************/

#define _REENTRANT

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <ctype.h>
#include <math.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/uio.h>
#include <netinet/in.h>
#include <pthread.h>
#include <errno.h>
#include <assert.h>
#include <time.h>

#include <getopt.h>
#include <modbus.h>

#include "rcs.hh"
#include "posemath.h"		// PM_POSE, TO_RAD
#include "emc.hh"		// EMC NML
#include "canon.hh"		// CANON_UNITS, CANON_UNITS_INCHES,MM,CM
#include "emcglb.h"		// EMC_NMLFILE, TRAJ_MAX_VELOCITY, etc.
#include "emccfg.h"		// DEFAULT_TRAJ_MAX_VELOCITY
#include "inifile.hh"		// INIFILE
#include "rcs_print.hh"
#include "timer.hh"             // etime()
#include "shcom.hh"             // NML Messaging functions

#define MAX_JOINT_NUM           5   // support up to 5 joints

#define DEBUG			0

/*
  Using emcrsh:

  emcrsh {-- --port <port number> --name <server name> --connectpw <password>
             --enablepw <password> --sessions <max sessions> --path <path>
             -ini<inifile>}
*/

// global vars for modbus connection:
static modbus_t            *mb_ctx;
static modbus_mapping_t    *mb_mapping;
static int status_bits[32];
static int com_ready;

struct option longopts[] = {
  {"help", 0, NULL, 'h'},
  {"port", 1, NULL, 'p'},
  {"name", 1, NULL, 'n'},
  {"sessions", 1, NULL, 's'},
  {"connectpw", 1, NULL, 'w'},
  {"enablepw", 1, NULL, 'e'},
  {"path", 1, NULL, 'd'},
  {0,0,0,0}};

static void thisQuit()
{
    EMC_NULL emc_null_msg;

    if (emcStatusBuffer != 0) {
	// wait until current message has been received
	emcCommandWaitReceived(emcCommandSerialNumber);
    }

    if (emcCommandBuffer != 0) {
	// send null message to reset serial number to original
	emc_null_msg.serial_number = saveEmcCommandSerialNumber;
	emcCommandBuffer->write(emc_null_msg);
    }
    // clean up NML buffers

    if (emcErrorBuffer != 0) {
	delete emcErrorBuffer;
	emcErrorBuffer = 0;
    }

    if (emcStatusBuffer != 0) {
	delete emcStatusBuffer;
	emcStatusBuffer = 0;
	emcStatus = 0;
    }

    if (emcCommandBuffer != 0) {
	delete emcCommandBuffer;
	emcCommandBuffer = 0;
    }
    
    modbus_mapping_free(mb_mapping);
    modbus_close(mb_ctx);
    modbus_free(mb_ctx);

    printf("finish quit process\n");
//    Tcl_Exit(0);
    exit(0);
}

static int initModbus()
{
    int server_id;
    int baud, bits, stopbits, debug;
    const char *device, *parity;
    int rc;
    int i;
    
    // baud = 115200;
    baud = 19200;
    // baud = 57600;
    // baud = 9600;
    bits = 8;
    stopbits = 1;
    debug = DEBUG;
    device = "/dev/ttyO1";
    // device = "/dev/ttyUSB0";
    // device = "/dev/ttyUSB1";
    parity = "O";   // O: odd, E: even, N: none
    // parity = "N";   // O: odd, E: even, N: none
    server_id = 1;

    printf("modbus_rtu: device='%s', baud=%d, bits=%d, parity='%s', stopbits=%d, verbose=%d\n", device, baud, bits, parity, stopbits, debug);

    /* Initialize Modbus */
    // serial: 
    mb_ctx = modbus_new_rtu(device, baud, *parity, bits, stopbits);
    modbus_set_slave(mb_ctx, server_id);
    for (i = 0; i < 32; i++) {
	status_bits[i] = 0;
    }
            
    // ethernet:
    // ctx = modbus_new_tcp("127.0.0.1", 1502);
    // ctx = modbus_new_tcp("10.1.1.138", 1502);
    
    if (mb_ctx == NULL) {
        fprintf(stderr, "Unable to create the libmodbus context\n");
        return -1;
    }

    modbus_set_debug(mb_ctx, debug);

    mb_mapping = modbus_mapping_new(500, 500, 500, 500);
    if (mb_mapping == NULL) {
        fprintf(stderr, "Failed to allocate the mapping: %s\n",
                modbus_strerror(errno));
        modbus_free(mb_ctx);
        return -1;
    }

    // TCP:
    // socket = modbus_tcp_listen(ctx, 1);
    // modbus_tcp_accept(ctx, &socket);

    // RTU:
    rc = modbus_connect(mb_ctx);
    if (rc == -1) {
        fprintf(stderr, "Unable to connect %s\n", modbus_strerror(errno));
        modbus_free(mb_ctx);
        return -1;
    }

    return 0;
}

static void sigQuit(int sig)
{
    (void) signal(SIGINT, SIG_IGN); // ignore CTRL-C signal
    thisQuit();
}


/* Modbus Function codes */
#define _FC_READ_COILS                0x01
#define _FC_READ_DISCRETE_INPUTS      0x02
#define _FC_READ_HOLDING_REGISTERS    0x03
#define _FC_READ_INPUT_REGISTERS      0x04
#define _FC_WRITE_SINGLE_COIL         0x05
#define _FC_WRITE_SINGLE_REGISTER     0x06
#define _FC_READ_EXCEPTION_STATUS     0x07
#define _FC_WRITE_MULTIPLE_COILS      0x0F
#define _FC_WRITE_MULTIPLE_REGISTERS  0x10
#define _FC_REPORT_SLAVE_ID           0x11
#define _FC_WRITE_AND_READ_REGISTERS  0x17

/* Set a float to 4 bytes in Modbus format; was modbus_set_float() */
static inline void com_set_float(uint16_t *dest, float f)
{
    uint32_t *ip;
    ip = (uint32_t *) &f;

    dest[0] = (uint16_t) *ip;
    dest[1] = (uint16_t)(*ip >> 16);
}

/* Set a uint32 to 4 bytes in Modbus format */
static inline void com_set_uint32(uint16_t *dest, uint32_t u)
{
    dest[0] = (uint16_t) u;
    dest[1] = (uint16_t)(u >> 16);
}
			
/* Set array of 32 bool values to 2 bytes in Modbus format */
static inline void com_set_intArray32_to_uint32 (uint16_t *dest, int *ia)
{
    int i;
    uint32_t tmp = 0;
    tmp |= (ia[31] & 1);
    for (i = 30; i > -1; i--) {
	tmp <<= 1;
	tmp |= (ia[i] & 1);
    }
    com_set_uint32 (dest, tmp);
}

static void parseModbusCommand(const uint8_t *req, int req_length)
{
    int offset = modbus_get_header_length(mb_ctx);
    int slave = req[offset - 1];
    int function = req[offset];
    uint16_t address = (req[offset + 1] << 8) + req[offset + 2];
    int nb;
    static uint32_t sn = 0;   // serial number
    float *fp;
    
    // TODO: read slave address through INI file
    assert (slave == 1);

    updateStatus();

    switch (function) {
    case _FC_READ_INPUT_REGISTERS:
        switch (address) {
            case    0:  
#if DEBUG
		printf("RD REGS\n");
		printf("queueFull: %d\n", emcStatus->motion.traj.queueFull);
		printf("inpos: %d\n", emcStatus->motion.traj.inpos);

		printf("queue: %d\n", emcStatus->motion.traj.queue);
		printf("echo_serial_number: %d\n", emcStatus->echo_serial_number);
		// EmcPose position;		// current commanded position
		printf("pos_x: %f\n", emcStatus->motion.traj.position.tran.x);
		printf("pos_y: %f\n", emcStatus->motion.traj.position.tran.y);
		printf("pos_z: %f\n", emcStatus->motion.traj.position.tran.z);
		printf("pos_a: %f\n", emcStatus->motion.traj.position.a);
		printf("pos_b: %f\n", emcStatus->motion.traj.position.b);
		// EmcPose actualPosition;	// current actual position, from forward kins
		printf("actual_pos_x: %f\n", emcStatus->motion.traj.actualPosition.tran.x);
		printf("actual_pos_y: %f\n", emcStatus->motion.traj.actualPosition.tran.y);
		printf("actual_pos_z: %f\n", emcStatus->motion.traj.actualPosition.tran.z);
		printf("actual_pos_a: %f\n", emcStatus->motion.traj.actualPosition.a);
		printf("actual_pos_b: %f\n", emcStatus->motion.traj.actualPosition.b);
		printf("sync_di[0]: %d\n", emcStatus->motion.synch_di[0]);
		printf("sync_di[1]: %d\n", emcStatus->motion.synch_di[1]);
#endif
		status_bits[0] = (int) emcStatus->motion.traj.queueFull;
		status_bits[1] = (int) emcStatus->motion.traj.inpos;
		status_bits[2] = (int) com_ready;

		com_set_uint32 (mb_mapping->tab_input_registers +  0, (uint32_t) emcStatus->motion.traj.queue);
		com_set_uint32 (mb_mapping->tab_input_registers +  2, (uint32_t) emcStatus->echo_serial_number);
		// EmcPose position;		// current commanded position
		com_set_float (mb_mapping->tab_input_registers +  4, (float) emcStatus->motion.traj.position.tran.x);
		com_set_float (mb_mapping->tab_input_registers +  6, (float) emcStatus->motion.traj.position.tran.y);
		com_set_float (mb_mapping->tab_input_registers +  8, (float) emcStatus->motion.traj.position.tran.z);
		com_set_float (mb_mapping->tab_input_registers + 10, (float) emcStatus->motion.traj.position.a);
		com_set_float (mb_mapping->tab_input_registers + 12, (float) emcStatus->motion.traj.position.b);
		// EmcPose actualPosition;	// current actual position, from forward kins
		com_set_float (mb_mapping->tab_input_registers + 14, (float) emcStatus->motion.traj.actualPosition.tran.x);
		com_set_float (mb_mapping->tab_input_registers + 16, (float) emcStatus->motion.traj.actualPosition.tran.y);
		com_set_float (mb_mapping->tab_input_registers + 18, (float) emcStatus->motion.traj.actualPosition.tran.z);
		com_set_float (mb_mapping->tab_input_registers + 20, (float) emcStatus->motion.traj.actualPosition.a);
		com_set_float (mb_mapping->tab_input_registers + 22, (float) emcStatus->motion.traj.actualPosition.b);
		com_set_intArray32_to_uint32 (mb_mapping->tab_input_registers + 24, emcStatus->motion.synch_di);
		com_set_intArray32_to_uint32 (mb_mapping->tab_input_registers + 26, emcStatus->motion.synch_di + 32);
		com_set_intArray32_to_uint32 (mb_mapping->tab_input_registers + 28, emcStatus->motion.synch_do);
		com_set_intArray32_to_uint32 (mb_mapping->tab_input_registers + 30, status_bits);
		break;
            default:    
	        printf("TODO ");
                break;
        }
        break;

    case _FC_WRITE_MULTIPLE_REGISTERS: 
        nb = (req[offset + 3] << 8) + req[offset + 4];

        // printf("debug: _FC_WRITE_MULTIPLE_REGISTERS(%d)\n", function);
        // printf("debug: slave(%d) address(%u)\n", slave, address);
        if ((address + nb) > mb_mapping->nb_registers) {
            fprintf(stderr, "Illegal data address %0X in write_registers\n",
                    address + nb);
        } else if (com_ready == 1) {
            int i, j, k, val;
            char mdi_buf[256];
            char buf[32];
            const char cmd[8] = "FXYZAB";
            
            /** 
             * FC(16) Preset Multiple Registers, P.55 of PI_MBUS_300.pdf
             * Field Name           <Hex>
             * SLAVE                <16> 
             * FC                   <10>
             * START                <00><0A>
             * NU_OF_REGS           <00><0C>
             * BYTE_COUNT           <18>
             * FEED_RATE            <E4><00><45><AB>: 0x45ABE400 => 5000.5
             * X                    <99><9A><41><A5>
             * Y                    <99><9A><41><F1>
             * Z                    <CC><CD><41><24>
             * A                    <99><9A><41><F5>
             * B                    <99><9A><3E><99>
             * CRC                  <74><FE>
             **/
            /**
             * IEEE-754 Floating point to HEX converter:
             * http://www.h-schmidt.net/FloatConverter/IEEE754.html
             * http://babbage.cs.qc.cuny.edu/IEEE-754/
             **/

            sprintf(mdi_buf, "N%d ", sn);
            sn++;
            for (i = address, j = 6, k=0; 
                 i < address + nb; 
                 i+=2, j += 4, k+=1) {
                /* 6 and 7 = first value */
                val = (req[offset + j + 2] << 24) + 
                      (req[offset + j + 3] << 16) + 
                      (req[offset + j]     << 8 ) + 
                       req[offset + j + 1];
                fp = (float *) &val;
                // printf("\tj[%d](0x%08X), %9.3f\n", i, val, *fp);
                sprintf(buf, "%c%f ", cmd[k], *fp);
                strcat(mdi_buf, buf);
                if (i == address) {
                    switch (address) {
                        case    0:  strcat(mdi_buf, "G0 ");
                                    break;
                        case    1:  strcat(mdi_buf, "G1 ");
                                    break;
                        default:    printf("TODO ");
                                    break;
                    }
                }
            }
#if DEBUG
            printf("%s\n", mdi_buf);
#endif

            // execState: usually at EMC_TASK_EXEC_DONE(2)
            // printf("execState(%d)\n", emcStatus->task.execState);
            // printf("status(%d)\n", emcStatus->status);
	    /**
	     * emcStatus->status:
	     *	    RCS_DONE = 1
	     * 	    RCS_EXEC = 2
	     * 	    RCS_ERROR = 3
	     **/
	    // if (emcStatus->status != RCS_EXEC)
	    if (0 == emcStatus->motion.traj.queueFull) {
		if (sendMdiCmd(mdi_buf) != 0) {
		    printf("sendMdiCmd ERROR or emcTimeout(%.2f)\n", emcTimeout); 
		    // this may happen when system loading is high
		    // assert(0);
		}
	    } else {
		printf("queue: %d\n", emcStatus->motion.traj.queue);
		printf("queueFull: %d\n", emcStatus->motion.traj.queueFull);
	    }
        }
        break;

    default:
        printf("debug: unknown Modbus Function Code:0x%02X\n", function);
        assert(0);
        break;
    }

    return;
}

#if 0
static void print_status()
{
    printf("task: %10ld %10d %10d\n", 
            emcStatus->task.heartbeat,
            emcStatus->echo_serial_number,
            emcStatus->status);
    printf("io: %10ld %10d\n", 
            emcStatus->io.heartbeat,
            emcStatus->io.echo_serial_number);
    printf("motion: %10ld %10d\n", 
            emcStatus->motion.heartbeat,
            emcStatus->motion.echo_serial_number);
    return;
}
#endif 

static void diff_time(struct timespec *start, struct timespec *end,
		      struct timespec *diff)
{
    if ((end->tv_nsec - start->tv_nsec) < 0) {
	diff->tv_sec = end->tv_sec - start->tv_sec - 1;
	diff->tv_nsec = 1000000000 + end->tv_nsec - start->tv_nsec;
    } else {
	diff->tv_sec = end->tv_sec - start->tv_sec;
	diff->tv_nsec = end->tv_nsec - start->tv_nsec;
    }
    return;
}


/**
 * check_estop() - check if the machine is at ESTOP state
 **/
static int check_estop()
{
    static int updateMsg = -1;
    if (emcStatus->task.state != EMC_TASK_STATE_ON) {
	if (emcStatus->task.state == EMC_TASK_STATE_ESTOP) {
	    if (updateMsg != 1) {
		printf("ESTOP-ON,    MACHINE-OFF\n");
		updateMsg = 1;
	    }
	    return (-1);
	} else {
	    if (updateMsg != 0) {
		printf("ESTOP-RESET, MACHINE-OFF\n");
		updateMsg = 0;
	    }
	}
    } else {
	updateMsg = -1;
    }
    return (0);
}

/**
 * check_machine_on() - check the machine-on state
 **/
static int check_machine_on ()
{
    static int check_power_switch = 0;
    static timespec time_begin;
    timespec time_cur;
    timespec time_delta;

    if (emcStatus->motion.synch_di[1] == 0) {
	if (emcStatus->task.state == EMC_TASK_STATE_ON) {
	    printf("SET MACHINE OFF\n");
	    sendMachineOff();
            check_power_switch = 1;
            clock_gettime(CLOCK_REALTIME, &time_begin);
	} 
        if (check_power_switch) {
            clock_gettime(CLOCK_REALTIME, &time_cur);
            diff_time(&time_begin, &time_cur, &time_delta);
            if (time_delta.tv_sec > 30) {
                printf ("about to power off the system\n");
		thisQuit();
            }
        }
	return (-1);
    } else {
	if (emcStatus->task.state != EMC_TASK_STATE_ON) {
	    printf("SET MACHINE ON\n");
	    sendMachineOn();
            check_power_switch = 0;
	    return (-1);
	}
    }
    return (0);
}


/**
 * check_homing() - check the home status and do homing if necessary
 **/
static int check_home_stat()
{
    int i;
    int to_home_all = MAX_JOINT_NUM;
    
    // support J0 ~ J4 so far
    for (i=0; i<MAX_JOINT_NUM; i++) {
        if (emcStatus->motion.joint[i].homed == 0) {
            if (emcStatus->motion.joint[i].homing == 0) {
                to_home_all -= 1;
            }
        }
    }
    
    // printf("check_home_stat: to_home_all(%d)\n", to_home_all);
        
    if (to_home_all != MAX_JOINT_NUM) {
        if (to_home_all == 0) {
            printf("lcom: issue HOME_ALL command\n");
            sendHome(-1);   // -1: home all
        }
        return (-1);        // either homing or about to home all
    }
    return (0);
}

/**
 * check and set as MDI_MODE
 **/
static int check_mdi_mode()
{
    if (emcStatus->task.mode != EMC_TASK_MODE_MDI) {
        sendMdi();  // set as MDI mode
        return (-1);
    }
    return (0); 
}

static int check_machine_stat()
{
    // return 0 if machine is ready
    if (check_estop() == 0) {
        // at ESTOP-RESET(READY) state
	if (check_machine_on() == 0) {
	    // at MACHINE_ON state
	    if (check_home_stat() == 0) {
		// all joints are at HOMED state
		if (check_mdi_mode() == 0) {
		    // at MDI mode
		    return (0);
		}
	    }
	}
    }
    return (-1);
}

static void modbus_main()
{
    int rc;
    uint8_t query[MODBUS_TCP_MAX_ADU_LENGTH];
    
    while(1) {
	struct timespec	req= {0,10500000};   // 10.5ms

        updateStatus();
        if (check_machine_stat()) {
            // bypass receiving modbus MDI messages if the machine is not READY
            com_ready = 0;
        } else {
	    com_ready = 1;
	}
	
        rc = modbus_receive(mb_ctx, query);
	nanosleep(&req, NULL);  // sleep for 10.5ms to avoid blocking by UART
        if (rc > 0) {
            /* rc is the query size */
            parseModbusCommand(query, rc);
            modbus_reply(mb_ctx, query, rc, mb_mapping);
	    nanosleep(&req, NULL);  // sleep for 10.5ms to avoid blocking by UART
        } else if (rc == -1) {
            /* modbus related error */
	    rcs_print_error("ERROR: %s\n", modbus_strerror(errno));
        } else {
            rcs_print_error("ERROR: modbus query size: %d\n", rc);
        }

    }

    return;
}

static void initMain()
{
    emcWaitType = EMC_WAIT_RECEIVED;
    // emcWaitType = EMC_WAIT_DONE;
    emcCommandSerialNumber = 0;
    saveEmcCommandSerialNumber = 0;
    emcTimeout = 3.0;
    emcUpdateType = EMC_UPDATE_AUTO;
    linearUnitConversion = LINEAR_UNITS_AUTO;
    angularUnitConversion = ANGULAR_UNITS_AUTO;
    emcCommandBuffer = 0;
    emcStatusBuffer = 0;
    emcStatus = 0;

    emcErrorBuffer = 0;
    error_string[LINELEN-1] = 0;
    operator_text_string[LINELEN-1] = 0;
    operator_display_string[LINELEN-1] = 0;
    programStartLine = 0;
}

static void usage(char* pname) {
    printf("Usage: TODO\n");
    //obsolete: printf("         %s [Options] [-- emcOptions]\n"
    //obsolete:        "Options:\n"
    //obsolete:        "         --help       this help\n"
    //obsolete:        "         --port       <port number>  (default=%d)\n"
    //obsolete:        "         --name       <server name>  (default=%s)\n"
    //obsolete:        "         --connectpw  <password>     (default=%s)\n"
    //obsolete:        "         --enablepw   <password>     (default=%s)\n"
    //obsolete:        "         --sessions   <max sessions> (default=%d) (-1 ==> no limit) \n"
    //obsolete:        "         --path       <path>         (default=%s)\n"
    //obsolete:        "emcOptions:\n"
    //obsolete:        "          -ini        <inifile>      (default=%s)\n"
    //obsolete:       ,pname,port,serverName,pwd,enablePWD,maxSessions,defaultPath,emc_inifile
    //obsolete:       );
}

int main(int argc, char *argv[])
{
    int opt;

    initMain();
    // process local command line args
    while((opt = getopt_long(argc, argv, "he:n:p:s:w:d:", longopts, NULL)) != - 1) {
      switch(opt) {
        case 'h': usage(argv[0]); exit(1);
        //obsolete: case 'e': strncpy(enablePWD, optarg, strlen(optarg) + 1); break;
        //obsolete: case 'n': strncpy(serverName, optarg, strlen(optarg) + 1); break;
        //obsolete: case 'p': sscanf(optarg, "%d", &port); break;
        //obsolete: case 's': sscanf(optarg, "%d", &maxSessions); break;
        //obsolete: case 'w': strncpy(pwd, optarg, strlen(optarg) + 1); break;
        //obsolete: case 'd': strncpy(defaultPath, optarg, strlen(optarg) + 1);
        }
      }

    // process emc command line args
    if (emcGetArgs(argc, argv) != 0) {
	rcs_print_error("error in argument list\n");
	exit(1);
    }

    // get configuration information
    iniLoad(emc_inifile);

    // init NML, set retry_time as 120 seconds, retry_interval as 1 second
    if (tryNml(120.0, 1.0) != 0) {
	rcs_print_error("can't connect to emc\n");
	thisQuit();
	exit(1);
    }
    
    if (initModbus() != 0) {
	rcs_print_error("can't initialize Modbus\n");
	thisQuit();
	exit(1);
    }

    // get current serial number, and save it for restoring when we quit
    // so as not to interfere with real operator interface
    updateStatus();
    emcCommandSerialNumber = emcStatus->echo_serial_number;
    saveEmcCommandSerialNumber = emcStatus->echo_serial_number;

    // TODO: learn SIGINT
    // attach our quit function to SIGINT
    signal(SIGINT, sigQuit);

    modbus_main(); 

    return 0;
}
