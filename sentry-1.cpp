#include <xprintf.h>
#include <System/SafetyLock.hpp>
#include "Master.hpp"

//---Initialize static member variables---//
MotorsUpdater Master::mu;
DJICOM Master::djicom[2] = { DJICOM(&hcan1), DJICOM(&hcan2) };
UART_COM Master::uartcom(&huart7);

std::vector<M3508> Master::rail_motors;
std::vector<M3508> Master::ammo_booster_motors;
std::vector<M2006> Master::ammo_loader_motors;
std::vector<M3508> Master::yaw_motor;
std::vector<M3508> Master::pitch_motor;

std::vector<DJICanMotor *> Master::can1_p_motors;
std::vector<DJICanMotor *> Master::can2_p_motors;
std::vector<Motor *> Master::p_motors;

bool Master::hal_update_start_flag = false;
bool Master::button_flag;
bool Master::can_error_flag;
bool Master::fire_flag;
bool Master::ammo_loader_arm_pos_init_flag;
bool Master::system_reset_flag;


int Master::led_counter;
int Master::can_error_recovery_counter;
int Master::ammo_loader_spin_counter;
int Master::xprint_counter;
int Master::rail_counter;

int Master::rail_direction;

float Master::pitch_angle = 0.0;
float Master::yaw_angle = 0.0;
int random_rail_index = 0;
int pitch_countor = 0;
float Master::pdy = 0;
float Master::pdx = 0;

WaitTimer Master::ammo_loader_reverse;


Master::OperationMode Master::mode;

//---Initialize static member variables end---//

void Master::Init() {
    //Initialize static class
    RemoteController::Init();
    Common::Init();
    Gyro::Init();
    Common::ResetAllLEDs();
    SafetyLock::Init();

    std::vector<uint8_t> tx_header;
    tx_header.push_back(0xfa);
    std::vector<uint8_t> rx_header;
    rx_header.push_back(0xab);
    rx_header.push_back(0xcd);
    uint8_t rx_length=8;
    uartcom.Init(tx_header,rx_header,rx_length);


    Master::led_counter = 0;
    Master::button_flag = false;
    Master::can_error_flag = false;
    Master::can_error_recovery_counter = 0;
    Master::xprint_counter = 0;
    Master::rail_counter = 0;

    Master::system_reset_flag = false;

    Master::ammo_loader_spin_counter = 1;
    Master::fire_flag = false;
    Master::ammo_loader_arm_pos_init_flag = false;

    Master::rail_direction = 1;

    float Ts = 0.001;
    float omega_c = 24.0 * (2.0 * M_PI);
    float omega_v = 15.0 * (2.0 * M_PI);
    float omega_p = 2.0 * (2.0 * M_PI);


    //足回りモーター
    rail_motors.push_back(M3508(1 - 1, Motor::ControlType::VELOCITY, CONST::M3508_NO_PAYLOAD_D,
                                CONST::M3508_NO_PAYLOAD_J, CONST::M3508_GEAR_RATIO, Ts, omega_v, omega_p, 0));
    rail_motors.push_back(M3508(2 - 1, Motor::ControlType::VELOCITY, CONST::M3508_NO_PAYLOAD_D,
                                CONST::M3508_NO_PAYLOAD_J, CONST::M3508_GEAR_RATIO, Ts, omega_v, omega_p, 0));
    rail_motors[0].GetPositionPID()->SetParams(PID::MODE_PI,Ts,4,-4,omega_p/2,0);
    rail_motors[1].GetPositionPID()->SetParams(PID::MODE_PI,Ts,4,-4,omega_p/2,0);

    can1_p_motors.push_back(&rail_motors[0]);
    can1_p_motors.push_back(&rail_motors[1]);

    //Yaw
    yaw_motor.push_back(M3508(3 - 1, Motor::ControlType::POSITION, CONST::M3508_NO_PAYLOAD_D, CONST::M3508_NO_PAYLOAD_J*2,CONST::M3508_GEAR_RATIO * 72/16, Ts, omega_v, omega_p, 0));
//    yaw_motor[0].SetAbsoluteOffset(0.0);  //零点補正
    yaw_motor[0].SetInitializeOffsetFlag(0.0); //初期位置を０と設定
    yaw_motor[0].GetPositionPID()->SetParams(PID::MODE_PI,Ts,1.2,-1.2,omega_p/2,0);
    can1_p_motors.push_back(&yaw_motor[0]);

    //Pitch
    pitch_motor.push_back(M3508(1 - 1, Motor::ControlType::POSITION, CONST::M3508_NO_PAYLOAD_D,CONST::M3508_NO_PAYLOAD_J*2.5, CONST::M3508_GEAR_RATIO * 25/16, Ts, omega_v, omega_p, 0));
//    pitch_motor[0].SetAbsoluteOffset(0.0); //零点補正
    pitch_motor[0].SetInitializeOffsetFlag(0.0); //初期位置を０と設定
    pitch_motor[0].GetPositionPID()->SetParams(PID::MODE_PI,Ts,4,-4,omega_p/4,0);
//    pitch_motor[0].GetPositionPID()->SetParams(PID::MODE_PI,Ts,1e7,-1e7,omega_p/2,0);
    can2_p_motors.push_back(&pitch_motor[0]);

    //Ammo loader
    ammo_loader_motors.push_back(M2006(2 - 1, Motor::ControlType::CURRENT, CONST::M3508_NO_PAYLOAD_D / 5, CONST::M3508_NO_PAYLOAD_J / 2,CONST::M2006_GEAR_RATIO, Ts, omega_v / 10, omega_p / 3));
    ammo_loader_motors.push_back(M2006(3 - 1, Motor::ControlType::CURRENT, CONST::M3508_NO_PAYLOAD_D / 5, CONST::M3508_NO_PAYLOAD_J / 2,CONST::M2006_GEAR_RATIO, Ts, omega_v / 10, omega_p / 3));
    ammo_loader_motors[0].SetInitializeOffsetFlag(0.0);
    ammo_loader_motors[1].SetInitializeOffsetFlag(0.0);
    //電流上限（リロード中に一定電流を越したら動作を止める）
    ammo_loader_motors[0].SetCurrentLimit(12,-18);
    ammo_loader_motors[1].SetCurrentLimit(12,-18);
    for (int i = 0; i < ammo_loader_motors.size(); i++) {
        can2_p_motors.push_back(&ammo_loader_motors[i]);
    }

    //Ammo Booster right motors
    ammo_booster_motors.push_back(M3508(4 - 1, Motor::ControlType::CURRENT, CONST::M3508_NO_PAYLOAD_D / 10,CONST::M3508_NO_PAYLOAD_J / 6, 1.0, Ts, omega_v / 2, omega_p, 0));
    ammo_booster_motors.push_back(M3508(5 - 1, Motor::ControlType::CURRENT, CONST::M3508_NO_PAYLOAD_D / 10,CONST::M3508_NO_PAYLOAD_J / 6, 1.0, Ts, omega_v / 2, omega_p, 0));

    //Ammo Booster left motors
    ammo_booster_motors.push_back(M3508(6 - 1, Motor::ControlType::CURRENT, CONST::M3508_NO_PAYLOAD_D / 10,CONST::M3508_NO_PAYLOAD_J / 6, 1.0, Ts, omega_v / 2, omega_p, 0));
    ammo_booster_motors.push_back(M3508(7 - 1, Motor::ControlType::CURRENT, CONST::M3508_NO_PAYLOAD_D / 10,CONST::M3508_NO_PAYLOAD_J / 6, 1.0, Ts, omega_v / 2, omega_p, 0));

    for (int i = 0; i < ammo_booster_motors.size(); i++) {
        can2_p_motors.push_back(&ammo_booster_motors[i]);
    }

    //モーターのインデックス設定
    djicom[CONST::CAN1_INDEX].SetMotors(can1_p_motors);
    djicom[CONST::CAN2_INDEX].SetMotors(can2_p_motors);
    for (int i = 0; i < can1_p_motors.size(); i++)
        p_motors.push_back(can1_p_motors[i]);
    for (int i = 0; i < can2_p_motors.size(); i++)
        p_motors.push_back(can2_p_motors[i]);
    mu.SetMotors(p_motors);
}

void Master::Start() {
    SafetyLock::Unlock();

    Common::CheckCommonLED();
    Gyro::StartBiasMeasurement(1000);

//  ammo_loader_arm_push.SetTimer(1.0); //[s]

    //初期指令値

    //-----------------------Start-----------------------//
    mu.Start();
    djicom[CONST::CAN1_INDEX].Start();
    djicom[CONST::CAN2_INDEX].Start();
    //===================================================//
    hal_update_start_flag = true;
}

void Master::MainUpdate() {
    if (Common::GetUserButton()) {
        if (!button_flag) {
        }
        button_flag = true;
    }
    else {
        button_flag = false;
    }

    if (Master::led_counter >= 500) {
        Common::ToggleGreenLED();
        Master::led_counter = 0;
    }

    if (RemoteController::GetLeftToggleSwitch() == RemoteController::ToggleSwitchState::DOWN) {
        Master::system_reset_flag = true;
        Common::SetBuzzer(1000);
        HAL_Delay(200);
        Common::SetBuzzer(-1);
        HAL_Delay(100);
        Common::SetBuzzer(1000);
        HAL_Delay(100);
        Common::SetBuzzer(-1);
        HAL_Delay(100);
        Common::SetBuzzer(1000);
        HAL_Delay(100);
        Common::SetBuzzer(-1);
        HAL_Delay(100);
        NVIC_SystemReset();
    } else if (RemoteController::GetLeftToggleSwitch() == RemoteController::ToggleSwitchState::NEUTRAL) {
        Master::SetOperationMode(OperationMode::MANUAL);
    } else {
        Master::SetOperationMode(OperationMode::AUTO);
    }



    //-----------------MainUpdate-----------------//
    mu.MainUpdate();
    djicom[CONST::CAN1_INDEX].MainUpdate();
    djicom[CONST::CAN2_INDEX].MainUpdate();
    //============================================//

    //-----------センサー出力をUDPで送信-----------//
//    nuccom.SendSensorValue();
    //============================================//

}

void Master::HALUpdate(TIM_HandleTypeDef *htim) {
    //基本ループ
    //スタートフラグが立った場合ループ開始
    if (!hal_update_start_flag) {
        return;
    }
    //1kHzタイマーで動作するループ
    if (htim->Instance == TIM6) {
        //<<Gyroの定常値の測定
        if (!Gyro::GetEndOfBiasMeasurement()) {
            Gyro::HALUpdate();
            return;
        }

        //<<測定終了までこれより下が実行されない
        Gyro::SetActive(true);

        //Button
        if (Common::GetUserButton()) {
            Common::Beep(2, 600);
        }

        //UART通信、表示
        uartcom.Update();
//        xprint_counter += 1;
        float detect_x = (float)(int32_t)(
                (((int32_t)uartcom.GetIndexedValue(0) << 8) & 0x0000ff00)
                | (((int32_t)uartcom.GetIndexedValue(1) << 0) & 0x000000ff)
        );
        float detect_y = (float)(int32_t)(
                (((int32_t)uartcom.GetIndexedValue(2) << 8) & 0x0000ff00)
                | (((int32_t)uartcom.GetIndexedValue(3) << 0) & 0x000000ff)
        );
        float detect_h = (float)(int32_t)(
                (((int32_t)uartcom.GetIndexedValue(4) << 8) & 0x0000ff00)
                | (((int32_t)uartcom.GetIndexedValue(5) << 0) & 0x000000ff)
        );
        float detect_w = (int)(int32_t)(
                (((int32_t)uartcom.GetIndexedValue(6) << 8) & 0x0000ff00)
                | (((int32_t)uartcom.GetIndexedValue(7) << 0) & 0x000000ff)
        );
        // UARTからの値を画像の最大値でリミット
//        float gravity_grad =  0.003 * CONST::DETECT_IMG_MAX_Y * pow((1 - (detect_h/3)/CONST::DETECT_IMG_MAX_Y),1.0);
        float gravity_grad = 0;
        if (detect_y > CONST::DETECT_IMG_MAX_Y) detect_y = CONST::DETECT_IMG_MAX_Y;
        if (detect_x > CONST::DETECT_IMG_MAX_X) detect_x = CONST::DETECT_IMG_MAX_X;
        if (detect_y < 0) detect_y = 0;
        if (detect_x < 0) detect_x = 0;

        //獲得値を正規化し-1~1の範囲に
        float dy = (2*(detect_y/CONST::DETECT_IMG_MAX_Y) - 1);
        dy = ((dy>0) - (dy<0)) * pow(abs(dy),1.2)*24;
        float dx = (2*((detect_x)/CONST::DETECT_IMG_MAX_X) - 1);
        dx = ((dx>0) - (dx<0)) * pow(abs(dx),1.2)*16;
//        float vr_y = yaw_motor[0].GetResVelocity() - ((pdy - dy) / 0.001);
//        float sr_y = yaw_motor[0].GetResPositionAsMultiRotationRange() - dy;
//        float tc_y = 0;
//        if(vr_y > 0.001){
//          tc_y = abs(vr_y) / abs(sr_y);
//        }
//        float vr_x = pitch_motor[0].GetResVelocity() - ((pdx - dx) / 0.001);
//        float sr_x = pitch_motor[0].GetResPositionAsMultiRotationRange() - dx;
//        float tc_x = 0;
//        if (vr_x > 0.001){
//          tc_x = abs(vr_x) / abs(sr_x);
//        }
//        float point_x = pitch_motor[0].GetResVelocity() * tc_x + pitch_motor[0].GetResPositionAsMultiRotationRange();
//        float point_y = yaw_motor[0].GetResVelocity() * tc_y + yaw_motor[0].GetResPositionAsMultiRotationRange();
//        pdy = dy;
//        pdx = dx;
//        float dx = detect_x - CONST::DETECT_IMG_MAX_X/2;
        if (detect_h == 0){
            dy = 0;
            dx = 0;
        }
////        float dy = detect_y - CONST::DETECT_IMG_MAX_Y/2;
//        float yawRate = -dx;
////        float pitchRate = dy;
//        if (abs(yawRate) < 3/(CONST::K_X)){
//            yawRate = 0;
//        }
//        else if (abs(yawRate) > 3500/(CONST::K_X)){
//            yawRate = ((yawRate>0)?1:-1)*3500/CONST::K_X;
//        }
////        if (abs(pitchRate) < 3/(CONST::K_Y)){
////            pitchRate = 0;
////        }
////        else if (abs(pitchRate) > 3500/(CONST::K_Y)){
////            pitchRate = ((pitchRate>0)?1:-1)*3500/CONST::K_Y;
////        }
////        float pitch_angle_step = pitchRate/180.*M_PI/CONST::Y_RESOLUTION*CONST::K_Y;
//        float yaw_angle_step = yawRate/180.*M_PI/CONST::X_RESOLUTION*CONST::K_X;
//        //pitch角は獲得情報の500点平均によって算出(0.5[sec]における総データの平均)
//        //正規化、相対位置で計算してるから角度値を倍数にしてもいいかもしれない
//        //またn点平均と現在の値との差分を射出フラグの条件式に加えることで相手の動きが緩やかなときに射出するといった処理も可
//        if (detect_w == 0){
//            //            pitch_angle_step = pitch_angle;
//            yaw_angle_step = yaw_angle;
//        }

        float pitch_ref = dy;

        float rp = (56 + 56 * RemoteController::GetLeftV());
        float ry = (-yaw_motor[0].GetResPositionAsMultiRotationRange()*180/M_PI - 30 * RemoteController::GetRightH());
        if (detect_h != 0){
            rp = (-pitch_motor[0].GetResPositionAsMultiRotationRange()*180/M_PI + dy + gravity_grad);
            ry = (-yaw_motor[0].GetResPositionAsMultiRotationRange()*180/M_PI + dx);
        }
//        float rp = 66 + 66 * RemoteController::GetLeftV() + dy;
//        pitch_angle = pitch_angle_step;
//        yaw_angle = yaw_angle_step;

//        if(xprint_counter == 1000){
//            for(int i =0; i < (int)(uartcom.GetRxLength());i++){
//                xprintf(" %d ,",uartcom.GetIndexedValue(i));
//                if(i==(int)(uartcom.GetRxLength()/2)-1) xprintf("|");
//            }
//            xprintf("\r\n");
//            xprintf("data x=%d , y=%d",(int)(detect_x),(int)(detect_y));
//            xprintf("data w=%d",(int)(detect_w));
//            xprintf("data gravity=%d", (int)(gravity_grad * 10000));
//            xprintf("ref dy=%d , dx=%d",(int)(dy * 10000),(int)(dx * 10000));
//            xprintf("now deg = %d",(int)(pitch_motor[0].GetResPositionAsMultiRotationRange()*180/M_PI));
//            xprint_counter = 0;
//        }

        if (mu.GetEnableAllMotorsControl()) {
            //---指令値生成 START---//
            if (GetOperationMode() == Master::OperationMode::MANUAL) {
                //足回り
                rail_motors[0].SetControlType(Motor::ControlType::VELOCITY);
                rail_motors[1].SetControlType(Motor::ControlType::VELOCITY);
                rail_motors[0].SetRefValue(
                        RemoteController::GetLeftH() * M_PI * MECHANISM_PARAM::MAX_RAIL_ROLLER_RPM);
                rail_motors[1].SetRefValue(
                        RemoteController::GetLeftH() * M_PI * MECHANISM_PARAM::MAX_RAIL_ROLLER_RPM);
                //モーター動作
                if (RemoteController::GetRightToggleSwitch() == RemoteController::ToggleSwitchState::DOWN) {
                    //yaw,pitchの動力なくす
                    //yaw軸
                    yaw_motor[0].SetControlType(Motor::ControlType::VELOCITY);
                    yaw_motor[0].SetRefValue(0);
                    //pitch軸
                    pitch_motor[0].SetControlType(Motor::ControlType::VELOCITY);
                    pitch_motor[0].SetRefValue(0);
                } else {
                    //通常角度を45°として0~90°でピッチを動かす
                    //yaw軸
                    yaw_motor[0].SetControlType(Motor::ControlType::POSITION);
                    yaw_motor[0].SetRefValue(-(ry)*M_PI/180.0);

//                    yaw_motor[0].SetControlType(Motor::ControlType::VELOCITY);
//
//                    yaw_motor[0].SetRefValue((RemoteController::GetRightH() * M_PI) + yaw_angle_step);
                    //pitch軸
                    pitch_motor[0].SetControlType(Motor::ControlType::POSITION);
//                    pitch_motor[0].SetRefValue(-((35.0*M_PI)/180.0 ) - ((RemoteController::GetLeftV())*40)*M_PI/180.0 - pitch_angle );
                    if (rp > 95){
                        rp = 95;
                    }
                    else if (rp < -10){
                        rp = -10;
                    }
                    pitch_motor[0].SetRefValue(-(rp)*M_PI/180.0);
                }
            }else if(GetOperationMode() == Master::OperationMode::AUTO){
                //足回り
                rail_counter += 1;
                rail_motors[0].SetControlType(Motor::ControlType::POSITION);
                rail_motors[1].SetControlType(Motor::ControlType::POSITION);
                int spin = rail_direction;
                float rail_speed_extinction; // 速度を減衰させる係数 1でMAX 0で消える
                float rail_move_counter_thresh = 4.0*1000; // 移動時間のカウントの最大値
                if (detect_h > 0){
                    spin = 0;
                }else{
                    spin = rail_direction;
                }
                if (rail_counter < rail_move_counter_thresh) { // ここでrail_counterが一定時間以内なら速度指令を与える．
                    // 0から緩やかに上昇して中心で1になりthresh付近めがけてまた緩やかに0まで減衰してほしい
                    // これ速度制御だから線形に変更してもいんじゃね．．．？
                    rail_motors[0].SetRefValue(rail_speed_extinction * rail_direction* M_PI * MECHANISM_PARAM::MAX_RAIL_ROLLER_RPM*3);
                    rail_motors[1].SetRefValue(rail_speed_extinction * rail_direction* M_PI * MECHANISM_PARAM::MAX_RAIL_ROLLER_RPM*3);
                }else{ // 一定を超えたら逆方向に更新
                    rail_counter = 0;
                    rail_direction *= -1;
                }
                yaw_motor[0].SetControlType(Motor::ControlType::VELOCITY);
                yaw_motor[0].SetRefValue((spin * M_PI*0.5) + yaw_angle);
                //pitch軸
                pitch_motor[0].SetControlType(Motor::ControlType::POSITION);
                //                    pitch_motor[0].SetRefValue(-((35.0*M_PI)/180.0 ) - ((RemoteController::GetLeftV())*40)*M_PI/180.0 - pitch_angle );
                if (rp > 95){
                    rp = 95;
                }
                else if (rp < -10){
                    rp = -10;
                }
                pitch_motor[0].SetRefValue(-(rp)*M_PI/180.0);
            }
            // fire algorithm
            if (RemoteController::GetRightToggleSwitch() == RemoteController::ToggleSwitchState::UP){
                float AB_velocity = (2* M_PI) * 120; //99 is best?
                //right blaster
                ammo_booster_motors[0].SetControlType(Motor::ControlType::VELOCITY);
                ammo_booster_motors[1].SetControlType(Motor::ControlType::VELOCITY);
                ammo_booster_motors[0].SetRefValue(-AB_velocity);
                ammo_booster_motors[1].SetRefValue(AB_velocity);

                //left blaster
                ammo_booster_motors[2].SetControlType(Motor::ControlType::VELOCITY);
                ammo_booster_motors[3].SetControlType(Motor::ControlType::VELOCITY);
                ammo_booster_motors[2].SetRefValue(AB_velocity);
                ammo_booster_motors[3].SetRefValue(-AB_velocity);

//                ammo_loader_motors[0].SetControlType(Motor::ControlType::VELOCITY);
//                ammo_loader_motors[0].SetRefValue(2*M_PI*6/12);
//                //left reload
//                ammo_loader_motors[1].SetControlType(Motor::ControlType::VELOCITY);
//                ammo_loader_motors[1].SetRefValue(2*M_PI*6/12);

                if (abs(detect_y - CONST::DETECT_IMG_MAX_Y/2) < detect_h){
                    //right reload
                    ammo_loader_motors[0].SetControlType(Motor::ControlType::VELOCITY);
                    ammo_loader_motors[0].SetRefValue(2*M_PI*6/12);
                    //left reload
                    ammo_loader_motors[1].SetControlType(Motor::ControlType::VELOCITY);
                    ammo_loader_motors[1].SetRefValue(2*M_PI*6/12);
                }
                else{
                    //right reload
                    ammo_loader_motors[0].SetControlType(Motor::ControlType::VELOCITY);
                    ammo_loader_motors[0].SetRefValue(0);
                    //left reload
                    ammo_loader_motors[1].SetControlType(Motor::ControlType::VELOCITY);
                    ammo_loader_motors[1].SetRefValue(0);
                }


                if (ammo_loader_motors[0].GetCurrentLimitOverFlag() || ammo_loader_motors[1].GetCurrentLimitOverFlag()) {
                    ammo_loader_reverse.SetTimer(0.2);
                    ammo_loader_motors[1].ResetPID();
                    ammo_loader_motors[0].ResetPID();
                }
                if (ammo_loader_reverse.GetRunning()){
                    ammo_loader_motors[0].SetRefValue(-4 * M_PI / 12.0);
                    ammo_loader_motors[1].SetRefValue(-4 * M_PI / 12.0);
                    Common::Beep(1,770);
                }
            }
            else {
                float AB_velocity = 0;
                ammo_booster_motors[0].SetControlType(Motor::ControlType::VELOCITY);
                ammo_booster_motors[1].SetControlType(Motor::ControlType::VELOCITY);
                ammo_booster_motors[0].SetRefValue(0);
                ammo_booster_motors[1].SetRefValue(0);
                ammo_booster_motors[2].SetControlType(Motor::ControlType::VELOCITY);
                ammo_booster_motors[3].SetControlType(Motor::ControlType::VELOCITY);
                ammo_booster_motors[2].SetRefValue(0);
                ammo_booster_motors[3].SetRefValue(0);
                ammo_loader_motors[0].SetControlType(Motor::ControlType::VELOCITY);
                ammo_loader_motors[0].SetRefValue(0);
                ammo_loader_motors[1].SetControlType(Motor::ControlType::VELOCITY);
                ammo_loader_motors[1].SetRefValue(0);
            }
            ammo_loader_reverse.Update(1.0e-3);

            //射出

//            }
            //---指令値生成 END---//
        }
        //---CAN ERROR 処理 START---//
        if (djicom[CONST::CAN1_INDEX].GetCanTransmitHalError() || djicom[CONST::CAN2_INDEX].GetCanTransmitHalError()) {
            Common::Beep(3, 600);
        }
        //===CAN ERROR 処理 END===//

        //---Motor Control Enable START---//
        if (RemoteController::GetPowerOn() && !system_reset_flag) {
            Common::ResetRedLED();
            mu.EnableAllMotorsControl();
        } else {
            Common::SetRedLED();
            mu.DisableAllMotorsControl();
        }
        //===Motor Control Enable END===//

        Master::led_counter++;

        //----------------------HALUpdate----------------------//
        mu.HALUpdate();
        djicom[CONST::CAN1_INDEX].HALUpdate();
        djicom[CONST::CAN2_INDEX].HALUpdate();
        RemoteController::HALUpdate();

        Common::HALUpdate();
        Gyro::HALUpdate();

        //=====================================================//
    }
}

void Master::InterruptRxCAN(CAN_HandleTypeDef *p_hcan) {
    djicom[CONST::CAN1_INDEX].InterruptCan(p_hcan);
    djicom[CONST::CAN2_INDEX].InterruptCan(p_hcan);
}

void Master::InterruptRxUART(UART_HandleTypeDef *p_huart) {
    Common::SetCommonLED(Common::LedNum::LED_E);
    uartcom.InterruptRxUART(p_huart);
}

void Master::InterruptRxUDP(std::string str) {
    if (GetOperationMode() == Master::OperationMode::AUTO) {
//        nuccom.UDPUpdate(str);
    }
}

void Master::SetOperationMode(Master::OperationMode mode_) {
    Master::mode = mode_;
}

Master::OperationMode Master::GetOperationMode() {
    return Master::mode;
}
