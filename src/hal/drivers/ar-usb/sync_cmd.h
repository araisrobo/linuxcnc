#ifndef __sync_cmd_h__
#define __sync_cmd_h__
/*
 *******************************************************************************
 * SYNC Command Format:
 *    NAME        OP_CODE[15:14]  OPERAND[13:0]   Description
 *    SYNC_JNT           2'b00    {DIR_W, POS_W}  DIR_W[13]:    Direction, (positive(1), negative(0))
 *                                                POS_W[12:0]:  Relative Angle Distance (0 ~ 8191)
 *    NAME        OP_CODE[15:12]  OPERAND[11:0]   Description
 *    SYNC_DOUT          4'b0100  {ID, VAL}       ID[11:6]: Output PIN ID
 *                                                VAL[0]:   ON(1), OFF(0)
 *    SYNC_DIN           4'b0101  {ID, TYPE}      ID[11:6]: Input PIN ID
 *                                                TYPE[2:0]: LOW(000), HIGH(001), FALL(010), RISE(011)
 *                                                           TIMEOUT(100)
 *    SYNC_MOT_POS_CMD   4'b0110  {JOINT}         Synchronize motor_pos_cmd/rawcount from HOST.
 *                                                Take 64-bit data from immediate data buffer.
 *    SYNC_MOT_PARM      4'b0111  {ADDR}{ID}      ADDR[11:4]
 *                                                ID[3:0]:
 *                                                VAL: from immediate data
 *    SYNC_AHC           4'b1000  ... RESERVED    auto height control
 *    SYNC_VEL           4'b1001  {VEL, VAL}      VEL: velocity in mm/s
 *                                                VAL[0]: 1: velocity sync'd
 *                                                        0: velocity not sync'd
 *    SYNC_USB_CMD       4'b1010  
 *    SYNC_MACH_PARAM    4'b1011  ... TODO      
 *    SYNC_DATA          4'b1100  ... TODO:       {VAL} Send immediate data
 *                                                VAL[7:0]: one byte data
 *    SYNC_EOF           4'b1101                  End of frame                                            
 *    SYNC_DAC           4'b1110  {ID, ADDR}      write into DAC register with {ID[11:8], ADDR[7:0]}
 *                                                ADDR: 0x01 ... Data register
 *                                                ADDR: 0x55 ... Control register
 *    Write 2nd byte of SYNC_CMD[] will push it into SFIFO.
 *    The WB_WRITE got stalled if SFIFO is full.
 */

//      SFIFO COMMANDS
#define SYNC_JNT            0x0000      // 0x0000 ~ 0x3FFF
#define SYNC_DOUT           0x4000
#define SYNC_DIN            0x5000
#define SYNC_MOT_POS_CMD    0x6000
#define SYNC_MOT_PARAM      0x7000
// RESERVED #define SYNC_AHC            0x8000         // auto height control
#define SYNC_VEL            0x9000
#define SYNC_USB_CMD        0xA000
#define SYNC_MACH_PARAM     0xB000
#define SYNC_DATA           0xC000
#define SYNC_EOF            0xD000
#define SYNC_DAC            0xE000      // 1st 16-bit for ID and ADDR, 2nd 32-bit for VALUE
// RESERVED  0xf000

//  timeout type
#define WAIT_LEVEL_LOWER    0x0   // wait analog input to be lower than specified value
#define WAIT_LEVEL_HIGHER   0x1   // wait analog input to be higher than specified value
// 0x2 reserved
// 0x3 reserved
#define WAIT_LOW            0x4
#define WAIT_HIGH           0x5
#define WAIT_FALL           0x6
#define WAIT_RISE           0x7
#define NO_WAIT             0xF

//      SFIFO COMMAND MASK
#define SFIFO_SYNC_JNT_MASK             0xC000
#define DIR_P                           0x2000
#define DIR_N                           0x0000
#define POS_MASK                        0x1FFF

#define SYNC_OP_CODE_MASK               0xF000
#define SYNC_DI_DO_PIN_MASK             0x0FC0
#define SYNC_DOUT_VAL_MASK              0x0001
#define SYNC_DIN_TYPE_MASK              0x0007
#define SYNC_DATA_MASK                  0x00FF
#define SYNC_MOT_PARAM_ADDR_MASK        0x0FF0
#define SYNC_MOT_PARAM_ID_MASK          0x000F
#define SYNC_MACH_PARAM_ADDR_MASK       0x0FFF
#define SYNC_USB_CMD_TYPE_MASK 		0x0FFF

// SYNC VEL CMD masks
#define VEL_MASK                        0x0FFE
#define VEL_SYNC_MASK                   0x0001

// PROBE mask
#define SYNC_PROBE_MASK                 0x0FFF

// SYNC_DAC masks
#define SYNC_DAC_ID_MASK                0x0F00
#define SYNC_DAC_ADDR_MASK              0x00FF
#define SYNC_DAC_VAL_MASK               0xFFFF

//      SFIFO DATA MACROS
#define GET_IO_ID(i)                    (((i) & SYNC_DI_DO_PIN_MASK) >> 6)
#define GET_DO_VAL(v)                   (((v) & SYNC_DOUT_VAL_MASK))
#define GET_DI_TYPE(t)                  (((t) & SYNC_DIN_TYPE_MASK))
#define GET_DATA_VAL(t)                 (((t) & SYNC_DATA_MASK))
#define GET_MOT_PARAM_ADDR(t)           (((t) & SYNC_MOT_PARAM_ADDR_MASK) >> 4)
#define GET_MOT_PARAM_ID(t)             (((t) & SYNC_MOT_PARAM_ID_MASK))
#define GET_MACH_PARAM_ADDR(t)          ((t) & SYNC_MACH_PARAM_ADDR_MASK)
#define GET_USB_CMD_TYPE(t)             ((t) & SYNC_USB_CMD_TYPE_MASK)
#define GET_DAC_ID(i)                   (((i) & SYNC_DAC_ID_MASK) >> 8)
#define GET_DAC_ADDR(a)                 ((a) & SYNC_DAC_ADDR_MASK)
#define GET_DAC_VAL(v)                  ((v) & SYNC_DAC_VAL_MASK)

#define PACK_SYNC_DATA(t)               ((t & 0xFF))
#define PACK_IO_ID(i)                   (((i) & 0x3F) << 6)
#define PACK_DO_VAL(v)                  (((v) & 0x01))
#define PACK_DI_TYPE(t)                 (((t) & 0x07))
#define PACK_MOT_PARAM_ID(t)            ((t))
#define PACK_MOT_PARAM_ADDR(t)          ((t) << 4)
#define PACK_MACH_PARAM_ADDR(t)         ((t) & SYNC_MACH_PARAM_ADDR_MASK)
#define PACK_USB_CMD_TYPE(t)            ((t) & SYNC_USB_CMD_TYPE_MASK)

#define RISC_CMD_TYPE                   0x0004  // for SYNC_USB_CMD

/* bit index for machine_status[31:0] */
#define FERROR_MASK                     0x000000FF  // machine_status[7:0]
#define ALARM_MASK                      0x00000100  // machine_status[8]
#define SFIFO_IS_EMPTY_MASK             0x00000200  // machine_status[9]
#define SFIFO_IS_EMPTY_BIT              9           // set to 1 if SFIFO is EMPTY
#define TP_RUNNING_BIT                  17
#define AHC_DOING_BIT                   18

/**
 *  MACHINE_CTRL,   // [31:28]  JOG_VEL_SCALE
 *                  // [27:24]  SPINDLE_JOINT_ID
 *                  // [23:16]  NUM_JOINTS
 *                  // [l5: 8]  JOG_SEL
 *                                  [15]: MPG(1), CONT(0)
 *                                  [14]: RESERVED
 *                                  [13:8]: J[5:0], EN(1), DISABLE(0)
 *                  // [ 7: 4]  ACCEL_STATE
 *                  // [ 3: 1]  MOTION_MODE: 
 *                                  FREE    (0) 
 *                                  TELEOP  (1) 
 *                                  COORD   (2)
 *                                  HOMING  (4)
 *                  // [    0]  MACHINE_ON
 **/
#define MCTRL_MACHINE_ON_MASK           0x00000001  // MACHINE_ON mask for MACHINE_CTRL
#define MCTRL_MOTION_TYPE_MASK          0x0000000E  // MOTION_TYPE mask for MACHINE_CTRL
#define MCTRL_HOMING_MASK               0x00000008  // HOMING_MASK for MACHINE_CTRL
#define MCTRL_ACCEL_STATE_MASK          0x000000F0  // ACCEL_STATE mask for MACHINE_CTRL
//obsolete: #define MCTRL_TAPPING_MASK              0x0000FF00  // TAPPING mask for MACHINE_CTRL
#define MCTRL_NUM_JOINTS_MASK           0x00FF0000  // NUM_JOINTS mask for MACHINE_CTRL
#define MCTRL_SPINDLE_ID_MASK           0x0F000000  // SPINDLE_JOINT_ID mask for MACHINE_CTRL
#define MCTRL_JOG_SEL_MASK              0x0000FF00  // JOG_SEL mask for MACHINE_CTRL
#define MCTRL_JOG_VEL_SCALE_VALUE_MASK  0x70000000  // JOG_VEL_SCALE mask for MACHINE_CTRL
#define MCTRL_JOG_VEL_SCALE_SIGN_MASK   0x80000000  // JOG_VEL_SCALE mask for MACHINE_CTRL

/**
 *  GANTRY_CTRL,    // [31]     GANTRY_EN
 **/                  
#define GCTRL_EN_MASK                   0x80000000  // Gantry Enable Bit
// TODO: add GANTRY_MASTER_ID and GANTRY_SLAVE_ID to GANTRY_CTRL register

typedef enum {
    // naming: RISC_...CMD..._REQ
    RCMD_IDLE = 0,              // RCMD_FSM, set by RISC
    RCMD_ALIGNING,              // RCMD_FSM, joint is aligning
    RCMD_UPDATE_POS_REQ,        // RCMD_FSM, request HOST to update position
    RCMD_UPDATE_POS_ACK,        // RCMD set by HOST 
    RCMD_RISC_PROBE,            // Do risc probe 
    RCMD_HOST_PROBE,            // Do host probe
    RCMD_PSO,                   // PSO -- progress synced output
    RCMD_REMOTE_JOG,            // remote control 
} rsic_cmd_t;

typedef enum {
    RISC_PROBE_LOW = 0,
    RISC_PROBE_HIGH,
    RISC_PROBE_INDEX,
} rsic_probe_type_t;

typedef enum {
    OR = 0,
    AONLY,
    DONLY,
    AND,
} host_probe_type_t;

// memory map for machine config
enum machine_parameter_addr {
    AHC_JNT,
    AHC_POLARITY,
    GANTRY_POLARITY,
    TEST_PATTERN_TYPE,
    TEST_PATTERN,
    ANALOG_REF_LEVEL,       // wait analog signal: M110
    AHC_MAX_OFFSET,
    AHC_ANALOG_CH,
    WAIT_TIMEOUT,
    AHC_STATE,
    AHC_LEVEL,
    GANTRY_CTRL,            // [31]     GANTRY_EN
                            
    JOINT_LSP_LSN,          // format: {JOINT[31:16], LSP_ID[15:8], LSN_ID[7:0]}
    JOINT_JSP_JSN,          // format jog: {JOINT[31:16], LSP_ID[15:8], LSN_ID[7:0]}
    ALR_OUTPUT_0,           // DOUT_0 value, dout[31:0], when ESTOP is pressed
    ALR_OUTPUT_1,           // DOUT_1 value, dout[63:32], when ESTOP is pressed
    ALR_EN_BITS,            // the enable bitmap of ALARM input bits (

    SSIF_MODE,              // [7:0]    bitwise mapping of mode for SSIF_PULSE_TYPE
                            //          0: POSITION MODE (STEP-DIR or AB-PHASE)
                            //          1: PWM MODE (velocity or torque)

    MACHINE_CTRL,           // [31:28]  JOG_VEL_SCALE
                            // [27:24]  SPINDLE_JOINT_ID
                            // [23:16]  NUM_JOINTS
                            // [15: 8]  JOG_SEL
                            // [ 7: 4]  ACCEL_STATE
                            // [ 3: 1]  MOTION_MODE: 
                            // [    0]  MACHINE_ON
    MACHINE_PARAM_ITEM
};

// accel_state enum for MACHINE_CTRL judgement
enum accel_state_type {
  MACH_ACCEL_S0 = (0 << 4),     // 0
  MACH_ACCEL_S1 = (1 << 4),     // 1
  MACH_ACCEL_S2 = (2 << 4),     // 2
  MACH_ACCEL_S3 = (3 << 4),     // 3
  MACH_ACCEL_S4 = (4 << 4),     // 4
  MACH_ACCEL_S5 = (5 << 4),     // 5
  MACH_ACCEL_S6 = (6 << 4)      // 6
};

enum test_pattern_type_enum {
    NO_TEST,
    DIGITAL_IN,
    ANALOG_IN,
};

enum ahc_state_enum {
    AHC_DISABLE,
    AHC_ENABLE,
};

// memory map for motion parameter for each joint
enum motion_parameter_addr {
    MAX_VELOCITY      ,
    MAX_ACCEL         ,
    MAX_JERK	      ,
    MAXFOLLWING_ERR   ,
    // PID section: begin
    P_GAIN            ,     // (unit: 1/65536)
    I_GAIN            ,     // (unit: 1/65536)
    D_GAIN            ,     // (unit: 1/65536)
    FF0               ,     // useless for position mode servo
    FF1               ,     // (unit: 1/65536)    
    FF2               ,     // (unit: 1/65536)
    DEAD_BAND         ,     // unit: 1 pulse
    BIAS              ,     // useless for position mode servo
    MAXERROR          ,
    MAXERROR_I        ,
    MAXERROR_D        ,
    MAXCMD_D          ,
    MAXCMD_DD         ,
    MAXOUTPUT         ,     
    // PID section: end
    SCALE             ,     // unit_pulses/servo_period : 16.16 format, 
    ENC_SCALE         ,     // encoder scale: 16.16 format
    SSYNC_SCALE       ,     // spindle sync compensation scale: 16.16 format
    MAX_PARAM_ITEM
};
#define NUM_PID_PARAMS 14   // pid params: from P_GAIN to MAXOUTPUT

#endif // __sync_cmd_h__
