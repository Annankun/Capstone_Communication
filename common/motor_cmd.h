#ifndef MOTOR_CMD_H
#define MOTOR_CMD_H

#include <stdint.h>

/*
 * Motor Command — Global Data Structure
 *
 * Written by students' decision code (background).
 * Read by communication code every 100ms and sent to Motor Board.
 *
 * All fields are 0/1 binary values only.
 *
 * Payload layout (8 bytes):
 *   [0] enable      — 1 = motors active, 0 = stop all
 *   [1] left_dir    — left motor direction:  0 = reverse, 1 = forward
 *   [2] right_dir   — right motor direction: 0 = reverse, 1 = forward
 *   [3..7] reserved — 0x00 (future expansion)
 *
 * Usage (students' code):
 *   g_motor_cmd.enable    = 1;
 *   g_motor_cmd.left_dir  = 1;   // forward
 *   g_motor_cmd.right_dir = 0;   // reverse (turn)
 *
 * The communication layer reads g_motor_cmd every 100ms automatically.
 * Students do NOT need to call any send function.
 */

#define MOTOR_CMD_SIZE  8u   /* serialised byte count, matches SNAPSHOT_SIZE */

typedef struct {
    uint8_t enable;       /* 0 = stop all motors, 1 = motors active       */
    uint8_t left_dir;     /* left  motor: 0 = reverse,   1 = forward       */
    uint8_t right_dir;    /* right motor: 0 = reverse,   1 = forward       */
    uint8_t reserved[5];  /* set to 0x00, reserved for future motor fields  */
} motor_cmd_t;

/*
 * THE global motor command — shared between students' logic and the comms layer.
 *
 * Definition lives in control_board/main.c.
 * All other files reference it through this extern declaration.
 */
extern volatile motor_cmd_t g_motor_cmd;

/* ---- Helpers ---- */

/*
 * Initialise all fields to a safe stopped state.
 * Call once at startup before enabling the comms loop.
 */
static inline void motor_cmd_init(volatile motor_cmd_t *cmd)
{
    cmd->enable       = 0;
    cmd->left_dir     = 0;
    cmd->right_dir    = 0;
    cmd->reserved[0]  = 0;
    cmd->reserved[1]  = 0;
    cmd->reserved[2]  = 0;
    cmd->reserved[3]  = 0;
    cmd->reserved[4]  = 0;
}

/*
 * Serialize motor_cmd_t into an 8-byte buffer for framing.
 * buf must be at least MOTOR_CMD_SIZE bytes.
 */
static inline void motor_cmd_pack(volatile const motor_cmd_t *cmd, uint8_t *buf)
{
    buf[0] = cmd->enable    & 0x01u;
    buf[1] = cmd->left_dir  & 0x01u;
    buf[2] = cmd->right_dir & 0x01u;
    buf[3] = 0x00;
    buf[4] = 0x00;
    buf[5] = 0x00;
    buf[6] = 0x00;
    buf[7] = 0x00;
}

/*
 * Deserialize an 8-byte buffer back into a motor_cmd_t (Motor Board side).
 */
static inline void motor_cmd_unpack(motor_cmd_t *cmd, const uint8_t *buf)
{
    cmd->enable       = buf[0] & 0x01u;
    cmd->left_dir     = buf[1] & 0x01u;
    cmd->right_dir    = buf[2] & 0x01u;
    cmd->reserved[0]  = 0;
    cmd->reserved[1]  = 0;
    cmd->reserved[2]  = 0;
    cmd->reserved[3]  = 0;
    cmd->reserved[4]  = 0;
}

#endif /* MOTOR_CMD_H */
