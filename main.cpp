#include "mbed.h"
#include "math.h"
#include <chrono>

<<<<<<< HEAD
#include "AnalogIn.h"
#include "DigitalOut.h"
#include "mbed_wait_api.h"
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "ACAN2517FD.h"
#include "MbedHardwareSPI.h"
#include "CANSerialBridge.hpp"

#include "MbedHardwareSerial.hpp"
#include "SerialBridge.hpp"
#include "Controller.hpp"

#include "mdc_client/MDCClient.hpp"

#include "Mecanum.hpp"

Timer timer;
double pre_timer = 0.01;

#define ENCODER_REVOLUTION 1296

using namespace acan2517fd;

const double PI = 3.141592653589;

uint32_t getMillisecond() {
    return (uint32_t) duration_cast<std::chrono::milliseconds>(timer.elapsed_time()).count();
}

SPI spi(PC_1, PB_14, PC_7);
MbedHardwareSPI hardware_dev0(spi, PB_9);
ACAN2517FD dev0_can(hardware_dev0, getMillisecond);
CANSerialBridge serial(&dev0_can);

MDCClient mdc_client(&serial, 0);

DigitalOut acknowledge_0(PC_8);

SerialDev *dev = new MbedHardwareSerial(new BufferedSerial(PC_10, PC_11, 115200));
SerialBridge serial_control(dev, 1024);
=======
#include "MbedHardwareSerial.hpp"
#include "SerialBridge.hpp"
#include "Controller.hpp"

#include "Encoder.hpp"
#include "Mecanum.hpp"
#include "md.hpp"
#include "pid.hpp"

const double PI = 3.141592653589;

SerialDev *dev = new MbedHardwareSerial(new BufferedSerial(PA_15, PB_7, 115200));
SerialBridge serial(dev, 1024);
>>>>>>> upstream/master
Controller msc;

MecanumWheel mw;

<<<<<<< HEAD


=======
Timer timer;
double pre_timer = 0.01;

>>>>>>> upstream/master
/*
    [0] --→ 左前　 (FrontLeft)  [FL]
    [1] --→ 右前　 (FrontRight) [FR]
    [2] --→ 左後ろ (RearLeft)   [RL]
    [3] --→ 右後ろ (RearRight)  [RR]
*/

<<<<<<< HEAD
static uint32_t gUpdateDate = 0;
static uint32_t gSentDate = 0;


int main() {

    serial_control.add_frame(0, &msc);

    timer.start();

    //  set up
    ACAN2517FDSettings settings (ACAN2517FDSettings::OSC_4MHz, 125UL * 1000UL, DataBitRateFactor::x8);

    settings.mRequestedMode = ACAN2517FDSettings::NormalFD;
    
    settings.mDriverTransmitFIFOSize = 5;
    settings.mDriverReceiveFIFOSize = 5;

    settings.mBitRatePrescaler = 1;
    //  Arbitation Bit Rate
    settings.mArbitrationPhaseSegment1 = 255;
    settings.mArbitrationPhaseSegment2 = 64;
    settings.mArbitrationSJW = 64;
    //  Data Bit Rate
    settings.mDataPhaseSegment1 = 31;
    settings.mDataPhaseSegment2 = 8;
    settings.mDataSJW = 8;

    //--- RAM Usage
    printf("MCP2517FD RAM Usage: %d [bytes]\n\r", settings.ramUsage());

    printf("initializing device 0...\n\r");
    const uint32_t errorCode0 = dev0_can.begin (settings) ;
    if (errorCode0 == 0) {
        printf("initialized device 0!\n\r");
    }else{
        printf("Configuration error 0x%x\n\r", errorCode0);
    }

    printf("all configuration completed!\n\r");

        setting_struct_t mdc_settings_0 = {
        OperatorMode::MD_OPERATOR,
        EncoderType::VELOCITY,
        ENCODER_REVOLUTION,
        false,
        0.40,
        0.8,
        0,
        0,
        0,
        0,
        0
    };

    setting_struct_t mdc_settings_1 = {
        OperatorMode::MD_OPERATOR,
        EncoderType::VELOCITY,
        ENCODER_REVOLUTION,
        true,
        0.40,
        0.9,
        0,
        0,
        0,
        0,
        0
    };

    setting_struct_t mdc_settings_2 = {
        OperatorMode::MD_OPERATOR,
        EncoderType::VELOCITY,
        ENCODER_REVOLUTION,
        false,
        0.40,
        0.8,
        0,
        0,
        0,
        0,
        0
    };

    setting_struct_t mdc_settings_3 = {
        OperatorMode::MD_OPERATOR,
        EncoderType::VELOCITY,
        ENCODER_REVOLUTION,
        true,
        0.45,
        0.8,
        0,
        0,
        0,
        0,
        0
    };

    mdc_client.update_setting(0, mdc_settings_0);
    mdc_client.update_setting(1, mdc_settings_1);
    mdc_client.update_setting(2, mdc_settings_2);
    mdc_client.update_setting(3, mdc_settings_3);
=======
// 呼び出し
Encoder *encoder[4];
PID *pid[4];
MD *md[4];




void initialize_module();

int main() {
    initialize_module();

    serial.add_frame(0, &msc);
>>>>>>> upstream/master

    timer.start();

    while (1) {

        serial_control.update();

<<<<<<< HEAD
        dev0_can.poll();



        // if(msc.was_updated()){

            


            // PID用周期調整
            double DELTA_T = timer.read() - pre_timer;

            // Joystickの値を取得(値域を±0.5から±1にする)
            double joyXValue = (msc.data.x - 0.5) * 2;
            double joyYValue = (msc.data.y - 0.5) * 2;

            if(joyXValue < 0.1 && joyXValue > -0.1){
                joyXValue = 0;
            }

            if(joyYValue < 0.1 && joyYValue > -0.1){
                joyYValue = 0;
            }

            // ボタンの状態を取得(Lならマイナス,Rならプラス)
            double turn = (msc.data.r * 0.3) - (msc.data.l * 0.3);

            if(msc.data.n == 1){
                turn = msc.data.r - msc.data.l;
            }

            // Joystickのベクトル化
            double targetSpeed    = sqrt(joyXValue * joyXValue + joyYValue * joyYValue);
            double targetRotation = atan2(joyYValue, joyXValue) - (PI /4);

            // targetSpeedが1,-1を超えないようにする
            if(targetSpeed > 1){
                targetSpeed = 1;
            }else if (targetSpeed < -1) {
                targetSpeed = -1;
            }

            // targetSpeedが0.1以下の時に起動しないようにする

            if(targetSpeed < 0.1 && targetSpeed > -0.1){
                targetSpeed = 0;
            }

            // targetRotationがマイナスにならないように2πたす
            if(targetRotation < 0){
                targetRotation += (2 * PI);
            }

            // 目標速度, 回転速度, 回転方向を設定
            mw.control(targetSpeed, targetRotation, turn);

            if (getMillisecond() - gUpdateDate > 40) {
                serial.update();
                if(mdc_client.update()) {
                    for(int i = 0; i < 4; i++) { 
                        printf("u%d:%4.2f ", i, mdc_client.feedback.data.node[i].velocity);
                    }
                }

                gUpdateDate = getMillisecond();
            }

            if(getMillisecond() - gSentDate > 100){
                mdc_client.set_target(0, mw.getSpeed(0));
                mdc_client.set_target(1, mw.getSpeed(1));
                mdc_client.set_target(2, mw.getSpeed(2));
                mdc_client.set_target(3, mw.getSpeed(3));

                mdc_client.send_target();

                gSentDate = getMillisecond();
            }
            pre_timer = (double)timer.read();

            // 周期調整用 (ここを変えるならDELTA_Tも変える)
            ThisThread::sleep_for(10ms);





        
=======


        if(msc.was_updated()){


            // PID用周期調整
            double DELTA_T = timer.read() - pre_timer;

            // Joystickの値を取得(値域を±0.5から±1にする)
            double joyXValue = (msc.data.x - 0.5) * 2;
            double joyYValue = (msc.data.y - 0.5) * 2;

            if(joyXValue < 0.1 && joyXValue > -0.1){
                joyXValue = 0;
            }

            if(joyYValue < 0.1 && joyYValue > -0.1){
                joyYValue = 0;
            }

            // ボタンの状態を取得(Lならマイナス,Rならプラス)
            double turn = (msc.data.r * 0.3) - (msc.data.l * 0.3);

            if(msc.data.n == 1){
                turn = msc.data.r - msc.data.l;
            }

            // Joystickのベクトル化
            double targetSpeed    = sqrt(joyXValue * joyXValue + joyYValue * joyYValue);
            double targetRotation = atan2(joyYValue, joyXValue) - (PI /4);

            // targetSpeedが1,-1を超えないようにする
            if(targetSpeed > 1){
                targetSpeed = 1;
            }else if (targetSpeed < -1) {
                targetSpeed = -1;
            }

            // targetSpeedが0.1以下の時に起動しないようにする

            if(targetSpeed < 0.1 && targetSpeed > -0.1){
                targetSpeed = 0;
            }

            // targetRotationがマイナスにならないように2πたす
            if(targetRotation < 0){
                targetRotation += (2 * PI);
            }

            // 目標速度, 回転速度, 回転方向を設定
            mw.control(targetSpeed, targetRotation, turn);


            // PID制御
            pid[0]->control(encoder[0]->get_rps(), mw.getSpeed(0), DELTA_T);
            pid[1]->control(encoder[1]->get_rps(), mw.getSpeed(1), DELTA_T);
            pid[2]->control(encoder[2]->get_rps(), mw.getSpeed(2), DELTA_T);
            pid[3]->control(encoder[3]->get_rps(), mw.getSpeed(3), DELTA_T);

            // printf("1 = %.4lf 2 = %.4lf 3 = %.4lf 4 = %.4lf",encoder[0]->get_rps(),encoder[1]->get_rps(),encoder[2]->get_rps(),encoder[3]->get_rps());

            // MD出力
            md[0]->drive(pid[0]->get_pid());
            md[1]->drive(pid[1]->get_pid());
            md[2]->drive(pid[2]->get_pid());
            md[3]->drive(pid[3]->get_pid());

            double current = encoder[0]->get_rotation();

//            printf("current = %.4lf  get_pid = %.4lf target = %.4lf\n\r", current, pid[0]->get_pid() ,mw.getSpeed(0));

            printf("behind = %.4lf\n\r",pid[0]->get_error_behind());


            /*
            md[0]->drive(mw.getSpeed(0));
            md[1]->drive(mw.getSpeed(1));
            md[2]->drive(mw.getSpeed(2));
            md[3]->drive(mw.getSpeed(3));
            */

            pre_timer = (double)timer.read();

            // 周期調整用 (ここを変えるならDELTA_Tも変える)
            ThisThread::sleep_for(10ms);



        }
>>>>>>> upstream/master
    }
}

void initialize_module()
{
// PIDゲイン調整 {kp(比例), ki(積分), kd(微分)}
    pid[0] = new PID(1.1, 0, 0, 0);
    pid[1] = new PID(1.1, 0, 0, 1);
    pid[2] = new PID(1.1, 0, 0, 0);
    pid[3] = new PID(1.1, 0, 0, 1);

// エンコーダーの制御ピン (a, b)
    encoder[0] = new Encoder(PB_2, PA_11);
    encoder[1] = new Encoder(PC_5, PA_12);
    encoder[2] = new Encoder(PB_1, PC_8);
    encoder[3] = new Encoder(PA_6, PC_9);

// MDの制御ピン (pwmピン, dirピン, 逆転モード)
    md[0] = new MD(PA_10, PD_2,  0);
    md[1] = new MD(PB_3 , PC_11, 1);
    md[2] = new MD(PB_5 , PC_10, 0);
    md[3] = new MD(PB_4 , PC_12, 1);

}

