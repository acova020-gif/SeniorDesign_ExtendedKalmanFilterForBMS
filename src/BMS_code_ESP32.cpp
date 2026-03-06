#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "BQ_Commands.h"
#include "EKF_Functions.h"

#define buttonA_pin A0

/*-------------------------------------------*/
/*------ Global Variables -------------------*/
/*-------------------------------------------*/

bool SysON = false;
bool CHARGE = false;

/*-------------------------------------------*/
/*------ Task Periods -----------------------*/
/*-------------------------------------------*/

const unsigned long EKF_Period = 500;
const unsigned long Button_Period = 500;

/*-------------------------------------------*/
/*--------- Button State Machine ------------*/
/*-------------------------------------------*/

enum ButtonStates {ButtonINIT, OFF, ButtonPressed_ON, ON, ButtonPressed_OFF};

int Button_TickFun(int state){
    bool button = digitalRead(buttonA_pin);

    switch(state){

        case ButtonINIT:
            SysON = 0;
            state = OFF;
        break;

        case OFF:
            if(button && !CHARGE){
                SysON = 1;
                state = ButtonPressed_ON;
            }
        break;

        case ButtonPressed_ON:
            if(!button){
                state = ON;
            }
        break;

        case ON:
            if(button || CHARGE){
                SysON = 0;
                state = ButtonPressed_OFF;
            }
        break;

        case ButtonPressed_OFF:
            if(!button){
                state = OFF;
            }
        break;

        default:
            state = ButtonINIT;
        break;
    }

    return state;
}

/*-------------------------------------------*/
/*---------- EKF State Machine --------------*/
/*-------------------------------------------*/

enum EKF_State { EKF_init, EKF_RUN };

int TickFun_ExtendedKalmanFilter(int state){
    static float current = 0.031f;
    static float voltage = 3.65f;
    const float min_SOC = 0.3;
    const float max_SOC = 0.8;

    /*----------- State Transitions ----------*/
    switch(state){
        case EKF_init:
            cells_INIT(1);
            state = EKF_RUN;
        break;

        case EKF_RUN:
            state = EKF_RUN;
        break;

        default:
            state = EKF_init;
        break;
    }

    /*------------- State Actions ------------*/
    switch(state){
        case EKF_init:
            break;

        case EKF_RUN:
            if(SysON && ekf[1].SoC > min_SOC){
                Prediction_TimeUpdate(1, current);
                Correction_MeasUpdate(1, voltage, current);

                Serial.print("Measured Voltage: ");
                Serial.println(voltage);

                Serial.print("Soc: ");
                Serial.println(ekf[1].SoC);
            }
            else if(SysON && ekf[1].SoC <= min_SOC){
                Serial.println("EKF AT THRESHOLD - STOPPED");
            }
            else if(!SysON){
                Serial.println("Sys off");
            }
            break;

        default:
            break;
    }

    return state;
}

/*-------------------------------------------*/
/*----------- FreeRTOS Task Wrappers --------*/
/*-------------------------------------------*/

void ButtonTask(void *pvParameters){
    int state = ButtonINIT;
    while(true){

        state = Button_TickFun(state);
        vTaskDelay(Button_Period / portTICK_PERIOD_MS);
    }
}


void EKFTask(void *pvParameters){
    int state = EKF_init;
    while(true){

        state = TickFun_ExtendedKalmanFilter(state);
        vTaskDelay(EKF_Period / portTICK_PERIOD_MS);
    }
}

/*-------------------------------------------*/
/*--------------- Setup ---------------------*/
/*-------------------------------------------*/

void setup(){

    Wire.begin();
    Serial.begin(9600);
    delay(10);

    pinMode(buttonA_pin, INPUT);

    /*------- Create FreeRTOS Tasks ----------*/

    xTaskCreatePinnedToCore(
        ButtonTask,
        "ButtonTask",
        4096,
        NULL,
        1,
        NULL,
        1
    );

    xTaskCreatePinnedToCore(
        EKFTask,
        "EKFTask",
        8192,
        NULL,
        1,
        NULL,
        1
    );
}

/*-------------------------------------------*/
/*--------------- Loop ----------------------*/
/*-------------------------------------------*/
void loop(){
    // Nothing needed here when using FreeRTOS
}