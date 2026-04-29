#pragma once

typedef enum {
    CAPSTAN_CMD_15IPS,
    CAPSTAN_CMD_30IPS,
    CAPSTAN_CMD_7P5IPS,
    CAPSTAN_CMD_OFF,
    CAPSTAN_CMD_OTHER,
} CapstanSetting;

typedef enum {
    CMD_ENABLE,
    CMD_DISABLE,
    CMD_CALIB_ENCODER,
    CMD_CAPSTAN_SETTING,
    CMD_TORQUE_UPDATE,
} CommandCenterCmd;

typedef void (*CommandCenterCb)(CommandCenterCmd cmd);

void command_center_init();
void command_center_register_cb(CommandCenterCb cb);

