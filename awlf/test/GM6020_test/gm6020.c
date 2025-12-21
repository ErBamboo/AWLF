#include "gm6020.h"
#include "algorithm/controller/pid.h"
#include "core/awlf_def.h"
#include <math.h>

int16_t speedOut;
__dbg_param_def(float, dbgNowSpeed);
__dbg_param_def(float, dbgSetSpeed);

__dbg_param_def(float, anglePidParam)[3] = {32.0f, 1.3f, 0.00f};
__dbg_param_def(float, speedPidParam)[3] = {50.0, 500.0f, 0.0f};

void gm6020_set_speed_feedback(GM6020_t gm6020, float speedFb, float speedFbCoef)
{
    gm6020->speedFb = speedFb;
    gm6020->speedFbCoef = speedFbCoef;
}

void gm6020_set_acc_feedback(GM6020_t gm6020, float accFb, float accFbCoef)
{
    gm6020->accFb = accFb;
    gm6020->accFbCoef = accFbCoef;
}

void gm6020_update(GM6020_t gm6020)
{
    // 反馈值更新
    if (gm6020->status == GM6020_STATUS_SHUTDOWN)
        return;
    if (gm6020->status == GM6020_STATUS_INIT)
    {
        gm6020->initSampleCnt++;
        if (gm6020->initSampleCnt < 50)
        {
            gm6020->rawData.angleEcd = ((gm6020->feedbackValue[0] << 8) & 0xFF00) | (gm6020->feedbackValue[1] & 0x00FF);
            gm6020->statusValue_s.offsetEcd = gm6020->rawData.angleEcd;
            return;
        }
        else
        {
            gm6020->status = GM6020_STATUS_RUNNING;
        }
    }

    float alpha = 0.8f;

    gm6020->rawData.lastAngleEcd = gm6020->rawData.angleEcd; // 上一周期的编码器角度, 单位机械角度
    gm6020->rawData.angleEcd = ((gm6020->feedbackValue[0] << 8) & 0xFF00) | (gm6020->feedbackValue[1] & 0x00FF);
    gm6020->rawData.speed = ((gm6020->feedbackValue[2] << 8) & 0xFF00) | (gm6020->feedbackValue[3] & 0x00FF);

    gm6020->statusValue_s.filterSpeed =
        alpha * gm6020->rawData.speed + (1.0f - alpha) * gm6020->statusValue_s.filterSpeed; // 滤波后速度

    // 状态值更新
    gm6020->statusValue_s.currunt = ((gm6020->feedbackValue[4] << 8) & 0xFF00) | (gm6020->feedbackValue[5] & 0x00FF);
    gm6020->statusValue_s.velocity = gm6020->rawData.speed;       // 速度
    gm6020->statusValue_s.temperature = gm6020->feedbackValue[6]; // 温度

    // 角度更新
    // 过0处理
    if (gm6020->rawData.angleEcd - gm6020->rawData.lastAngleEcd > 4096)
        gm6020->statusValue_s.totalRound--;
    else if (gm6020->rawData.angleEcd - gm6020->rawData.lastAngleEcd < -4096)
        gm6020->statusValue_s.totalRound++;
    gm6020->statusValue_s.singleAngle = ECD_2_ANGLE(gm6020->rawData.angleEcd - gm6020->statusValue_s.offsetEcd);
    if (gm6020->statusValue_s.singleAngle > 180.0f)
        gm6020->statusValue_s.singleAngle -= 360.0f;
    if (gm6020->statusValue_s.singleAngle < -180.0f)
        gm6020->statusValue_s.singleAngle += 360.0f;

    gm6020->statusValue_s.totalAngle =
        ECD_2_ANGLE(gm6020->rawData.angleEcd + gm6020->statusValue_s.totalRound * 8192.0f - gm6020->statusValue_s.offsetEcd);
}

void gm6020_set_speed(GM6020_t gm6020, float speed_rpm)
{
    if (gm6020->status != GM6020_STATUS_RUNNING)
        return;

    float nowSpeed = gm6020->statusValue_s.filterSpeed;
    float setSpeed = speed_rpm;
    dbgNowSpeed = nowSpeed;
    dbgSetSpeed = setSpeed;

    float currentTick = xTaskGetTickCount() / 1000.0f;

    speedOut = (int16_t)pid_compute(&gm6020->speedPid, setSpeed, nowSpeed, currentTick);

    gm6020->settingCurrunt[0] = (speedOut >> 8) & 0xFF;
    gm6020->settingCurrunt[1] = speedOut & 0xFF;
    device_write(gm6020->canHandler, NULL, &gm6020->sendMsg, 1);
}

/**
 * @brief 设置单圈角度
 *
 * @param gm6020
 * @param angleDeg 角度，角度制
 */

__dbg_param_def(float, dbgSetAngle);
__dbg_param_def(float, dbgNowAngle);
__dbg_param_def(float, dbgErrAngle);
void gm6020_set_single_angle(GM6020_t gm6020, float setAngleDeg)
{
    if (gm6020->status != GM6020_STATUS_RUNNING)
        return;
    float nowAngle = gm6020->statusValue_s.singleAngle;
    setAngleDeg = fmod(setAngleDeg, 360.0f);
    float errAngle = setAngleDeg - nowAngle;
    if (errAngle >= 180.0f)
        setAngleDeg -= 360.0f;
    else if (errAngle <= -180.0f)
        setAngleDeg += 360.0f;

    dbgNowAngle = nowAngle;
    dbgSetAngle = setAngleDeg;

    float currentTick = xTaskGetTickCount() / 1000.0f;
    float angleOut = pid_compute(&gm6020->anglePid, setAngleDeg, nowAngle, currentTick);
    gm6020_set_speed(gm6020, angleOut + gm6020->speedFbCoef * gm6020->speedFb + gm6020->accFbCoef * gm6020->accFb);
}

void gm6020_init(GM6020_t gm6020, Device_t canHandler)
{
    gm6020->canHandler = canHandler;
    gm6020->settings.motorID = GM6020_ID;
    gm6020->settings.sendID = GM6020_SEND_ID;
    gm6020->settings.FeedbackID = GM6020_FEEDBACK_ID;
    gm6020->settings.constVelPerVolt = 13.33f;
    gm6020->settings.maxVoltageValue = 25000.0f;

    gm6020->status = GM6020_STATUS_INIT;
    gm6020->initSampleCnt = 0;

    gm6020->feedbackMsg.bank = 0;
    gm6020->feedbackMsg.userBuf = (uint8_t*)&gm6020->feedbackValue;
    gm6020->sendMsg.userBuf = (uint8_t*)gm6020->settingCurrunt;
    gm6020->sendMsg.dsc = CAN_DATA_MSG_DSC_INIT(GM6020_SEND_ID, CAN_IDE_STD, 8);

    pid_init(&gm6020->anglePid, PID_POSITIONAL_MODE, anglePidParam[0], anglePidParam[1], anglePidParam[2]);
    pid_set_dead_band(&gm6020->anglePid, 0.15f);
    pid_set_output_limit(&gm6020->anglePid, -320.0f, 320.0f);
    pid_set_variable_integral_thresholds(&gm6020->anglePid, 8.75f, 2.5f);

    pid_init(&gm6020->speedPid, PID_POSITIONAL_MODE, speedPidParam[0], speedPidParam[1], speedPidParam[2]);
    pid_set_dead_band(&gm6020->speedPid, 1.0f);
    pid_set_output_limit(&gm6020->speedPid, -25000, 25000);
}
