#define _FIRMWARE_VERSION_ "HW_V3_2020_04_15_00"

//PID
//resistenza 5- 20
//core_config.P=12.8;
//core_config.I=16;
//core_config.D=0;

//resistenza 50- 200
//core_config.P=70;
//core_config.I=4;
//core_config.D=0;

#include <Ticker.h> //Ticker Library
#include <Wire.h>
#include <math.h>
#include <SimpleCLI.h>
#include "sfm3019_all.h"

#define TCAADDR 0x70

#define IIC_MUX_FLOW2 3
#define IIC_MUX_FLOW1 2
#define IIC_MUX_P_FASTLOOP 4
#define IIC_MUX_P_2 5
#define IIC_MUX_P_3 6

#define VERBOSE_LEVEL 1
#define LISTEN_PORT 80

#define N_PRESSURE_SENSORS 4

#define TIMERCORE_INTERVAL_MS 10
//100

#define VALVE_CLOSE 0
#define VALVE_OPEN 100

//#define VALVE_IN_PIN  A0 //15//14  (25)

#define DAC1 A0 //25
#define VALVE_OUT_PIN 32 //2 //12   (34)

#define BUZZER A11
#define ALARM_LED A12

#define SIMULATE_SENSORS 0

#define __ERROR_INPUT_PRESSURE_LOW 0
#define __ERROR_INPUT_PRESSURE_HIGH 1
#define __ERROR_INSIDE_PRESSURE_LOW 2
#define __ERROR_INSIDE_PRESSURE_HIGH 3
#define __ERROR_BATTERY_LOW 4
#define __ERROR_LEAKAGE 5
#define __ERROR_FULL_OCCLUSION 6
#define __ERROR_PARTIAL_OCCLUSION 7
#define __ERROR_ALARM_PI 29
#define __ERROR_WDOG_PI 30
#define __ERROR_SYSTEM_FALIURE 31

uint32_t ALARM_FLAG = 0x0;
uint32_t ALARM_FLAG_SNOOZE = 0x0;
uint32_t ALARM_FLAG_SNOOZE_millis = 0;

typedef enum {
    FR_OPEN_INVALVE,
    FR_WAIT_INHALE_TIME,
    FR_WAIT_EXHALE_TIME,
    AST_WAIT_MIN_INHALE_TIME,
    AST_WAIT_FLUX_DROP,
    AST_WAIT_FLUX_DROP_b,
    AST_DEADTIME,
    AST_PAUSE_EXHALE
} t_core__force_sm;

typedef enum {
    PRESSURE_DROP_INHALE,
    UNABLE_TO_READ_SENSOR_PRESSURE,
    UNABLE_TO_READ_SENSOR_FLUX,
    UNABLE_TO_READ_SENSOR_VENTURI,
    ALARM_COMPLETE_OCCLUSION,
    ALARM_PARTIAL_OCCLUSION,
    ALARM_PRESSURE_INSIDE_TOO_HIGH,
    ALARM_PRESSURE_INSIDE_TOO_LOW,
    ALARM_LEAKAGE,
    BATTERY_LOW,
    ALARM_PRESSURE_INPUT_TOO_LOW,
    ALARM_PRESSURE_INPUT_TOO_HIGH,
    ALARM_GUI_ALARM,
    ALARM_GUI_WDOG,
    UNPREDICTABLE_CODE_EXECUTION

} t_ALARM;
typedef enum { VALVE_IN,
    VALVE_OUT } valves;

typedef enum { M_BREATH_FORCED,
    M_BREATH_ASSISTED } t_assist_mode;
typedef enum { DS_01,
    GS_05 } t_ps_sensor;

// The port to listen for incoming TCP connections

Ticker CoreTask;

// Create CLI Object
SimpleCLI cli;
// Commands
Command param_set;
Command param_get;

SfmConfig sfm3019_inhale;

bool in_pressure_alarm = false;

struct
    {
    bool run;
    bool constant_rate_mode;
    uint16_t inhale_ms;
    uint16_t exhale_ms;
    uint16_t inhale_ms_extra;
    float pressure_forced_inhale_max;
    float pressure_forced_exhale_min;
    float pressure_drop;

    float assist_pressure_delta_trigger;
    uint16_t inhale_critical_alarm_ms;
    uint16_t exhale_critical_alarm_ms;
    t_assist_mode BreathMode;

    float pressure_alarm;
    float pressure_alarm_off;

    float flux_close;
    float target_pressure;
    float target_pressure_auto;
    float target_pressure_assist;
    float respiratory_rate;
    float respiratory_ratio;

    struct
        {
        float rate_inhale_pressure;
        float rate_exhale_pressure;
    } sim;
    float P;
    float I;
    float D;

    float P2;
    float I2;
    float D2;

    bool pause_inhale;
    bool pause_exhale;

    float pid_limit;

    bool backup_enable;
    float backup_min_rate;

    float pause_lg_timer;
    bool pause_lg;

} core_config;

typedef struct
    {
    float last_pressure;
    long read_millis;
} t_pressure;
t_pressure pressure[N_PRESSURE_SENSORS];

typedef struct
    {
    float last_flux;
    long read_millis;
} t_flux;
t_flux gasflux[1];

struct
    {
    t_core__force_sm force_sm = FR_OPEN_INVALVE;
    unsigned long timer1;
    unsigned long timer2;

} core_sm_context;

typedef struct
    {
    int32_t C[6];
    int32_t Q[6];
    float ZERO;
} t_5525DSO_calibration_table;

typedef struct
    {
    float FLUX;
    float TidalVolume;
    float InspVolumeSensirion;
    float InspVolumeVenturi;
    float TidalCorrection;
    int TidalStatus;
    float ExpVolumeVenturi;
    float AutoZero;
} t_tidal_volume_c;

t_tidal_volume_c tidal_volume_c;

typedef struct
    {
    float overshoot_avg;
    float overshoot_length_avg;
    float time_to_peak_avg;
    float final_error_avg;
    float t1050_avg;
    float t1090_avg;
    float tpeak_avg;
    float t9010_avg;
    float t9050_avg;
    float peep_avg;
    float t10_avg;
    float flux_peak_avg;
    float flux_t1090_avg;
    float flux_t9010_avg;

    float time_to_peak;
    float overshoot;
    float final_error;
    float t10a;
    float t50a;
    float t90a;
    float t10b;
    float t50b;
    float t90b;
    float overshoot_length;
    float flux_peak;
    float flux_t10a;
    float flux_t90a;
    float flux_t10b;
    float flux_t90b;

    int mean_cnt;
    uint32_t start_time;
    int phase;
} t_stat_param;
t_stat_param __stat_param;

t_ps_sensor pressure_sensor_type[] = { DS_01, DS_01, DS_01, DS_01 };

uint8_t pressure_sensor_i2c_address[] = { 0x76, 0x77, 0x76, 0x77 };
uint8_t pressure_sensor_i2c_mux[] = { IIC_MUX_P_FASTLOOP, IIC_MUX_P_FASTLOOP, IIC_MUX_P_2, IIC_MUX_P_2 };

uint8_t flow_sensor_i2c_address[] = { 0x40 };
t_5525DSO_calibration_table PRES_SENS_CT[N_PRESSURE_SENSORS];

int valve1_status = 0;
int valve2_status = 0;

float Pset = 0;

float PIDMonitor = 0;
float PIDMonitor2 = 0;

bool peep_look = false;

float last_peep = 0;
float last_bpm = 0;
float averaged_bpm = 0;
float temperature = 0;

bool batteryPowered = false;
float currentBatteryCharge = 100;

float currentP_Peak = 0;
float currentTvIsnp = 0;
float currentTvEsp = 0;
float currentVM = 0;

int dbg_state_machine = 0;
float dgb_delta;
int dbg_trigger;
float dgb_peaktime;
float fluxpeak = 0;
float pres_peak = 0;
unsigned long peaktime = 0;

float pplow = 0;

static float delta = 0;
static float delta2 = 0;

bool __WDENABLE = false;
bool __CONSOLE_MODE = false;
bool __ADDTimeStamp = true;

uint32_t watchdog_time = 0;
float last_O2 = 21.7;

bool use_Sensirion_Backup = false;

int read_pressure_sensor(int idx);
int valve_contol(valves valve, int level);
void CoreSM_FORCE_ChangeState(t_core__force_sm* sm, t_core__force_sm NEW_STATE);
void DBG_print(int level, String str);
void TriggerAlarm(t_ALARM Alarm);

void CalibrateDate_5525DSO(t_5525DSO_calibration_table CT, int32_t raw_temp, int32_t raw_pressure, float* T, float* P);
bool Convert_5525DSO(int address, int32_t* temp, int32_t* pressure, bool read_temp);
bool Reset_5525DSO(int address);
bool ReadCalibration_5525DSO(int address, t_5525DSO_calibration_table* ct);
bool FirstConversion_5525DSO(int address);

bool MeasureFlux(float* Flow);
void MeasureFluxInit();
void PressureControlLoop_PRESSIN();
void SetCommandCallback(cmd* c);
void GetCommandCallback(cmd* c);
void CliErrorCallback(cmd_error* e);

void MeasureFluxInit3019(uint8_t address);
bool MeasureFluxReadOffsetScale3019(uint8_t address, int16_t* flow_scale, int16_t* flow_offset, uint16_t* unit);

bool MeasureFlux_SFM3019(SfmConfig* sfm3019, float* Flow, float* T);
bool InitFlowMeter_SFM3019(SfmConfig* sfm3019);
void i2c_MuxSelect(uint8_t i);

void TidalReset();
void TidalInhale();
void TidalExhale();

void ResetStats();
void ResetStatsBegin();
void StatPhaseExpire();
void StatEndCycle();
void StatsUpdate();

void DBG_print(int level, String str)
{
    if (level <= VERBOSE_LEVEL) {
        Serial.println(str);
    }
}

void CoreSM_FORCE_ChangeState(t_core__force_sm* sm, t_core__force_sm NEW_STATE)
{
    *sm = NEW_STATE;
}

uint32_t GenerateFlag(int alarm_code)
{
    return (1 << alarm_code);
}

void TriggerAlarm(t_ALARM Alarm)
{

    switch (Alarm) {
    case PRESSURE_DROP_INHALE:
        DBG_print(2, "ALARM @ " + String(millis()) + " PRESSURE_DROP_INHALE");
        ALARM_FLAG = ALARM_FLAG | GenerateFlag(__ERROR_LEAKAGE);
        break;

    case UNABLE_TO_READ_SENSOR_PRESSURE:
        DBG_print(2, "ALARM @ " + String(millis()) + " UNABLE_TO_READ_SENSOR_PRESSURE");
        ALARM_FLAG = ALARM_FLAG | GenerateFlag(__ERROR_SYSTEM_FALIURE);
        break;

    case UNABLE_TO_READ_SENSOR_FLUX:
        DBG_print(2, "ALARM @ " + String(millis()) + " UNABLE_TO_READ_SENSOR_FLUX");
        ALARM_FLAG = ALARM_FLAG | GenerateFlag(__ERROR_SYSTEM_FALIURE);
        break;

    case UNABLE_TO_READ_SENSOR_VENTURI:
        DBG_print(2, "ALARM @ " + String(millis()) + " UNABLE_TO_READ_SENSOR_VENTURI");
        ALARM_FLAG = ALARM_FLAG | GenerateFlag(__ERROR_SYSTEM_FALIURE);
        break;

    case ALARM_COMPLETE_OCCLUSION:
        DBG_print(2, "ALARM @ " + String(millis()) + " ALARM_COMPLETE_OCCLUSION");
        ALARM_FLAG = ALARM_FLAG | GenerateFlag(__ERROR_FULL_OCCLUSION);
        break;

    case ALARM_PARTIAL_OCCLUSION:
        DBG_print(2, "ALARM @ " + String(millis()) + " ALARM_PARTIAL_OCCLUSION");
        ALARM_FLAG = ALARM_FLAG | GenerateFlag(__ERROR_PARTIAL_OCCLUSION);
        break;

    case ALARM_PRESSURE_INSIDE_TOO_HIGH:
        DBG_print(2, "ALARM @ " + String(millis()) + " ALARM_PRESSURE_INSIDE_TOO_HIGH");
        ALARM_FLAG = ALARM_FLAG | GenerateFlag(__ERROR_INSIDE_PRESSURE_HIGH);
        break;

    case ALARM_PRESSURE_INSIDE_TOO_LOW:
        DBG_print(2, "ALARM @ " + String(millis()) + " ALARM_PRESSURE_INSIDE_TOO_LOW");
        ALARM_FLAG = ALARM_FLAG | GenerateFlag(__ERROR_INSIDE_PRESSURE_LOW);
        break;

    case ALARM_LEAKAGE:
        DBG_print(2, "ALARM @ " + String(millis()) + " ALARM_LEAKAGE");
        ALARM_FLAG = ALARM_FLAG | GenerateFlag(__ERROR_LEAKAGE);
        break;

    case BATTERY_LOW:
        DBG_print(2, "ALARM @ " + String(millis()) + " BATTERY_LOW");
        ALARM_FLAG = ALARM_FLAG | GenerateFlag(__ERROR_BATTERY_LOW);
        break;

    case ALARM_PRESSURE_INPUT_TOO_LOW:
        DBG_print(2, "ALARM @ " + String(millis()) + " ALARM_PRESSURE_INPUT_TOO_LOW");
        ALARM_FLAG = ALARM_FLAG | GenerateFlag(__ERROR_INPUT_PRESSURE_LOW);
        break;

    case ALARM_PRESSURE_INPUT_TOO_HIGH:
        DBG_print(2, "ALARM @ " + String(millis()) + " ALARM_PRESSURE_INPUT_TOO_HIGH");
        ALARM_FLAG = ALARM_FLAG | GenerateFlag(__ERROR_INPUT_PRESSURE_HIGH);
        break;

    case ALARM_GUI_ALARM:
        DBG_print(2, "ALARM @ " + String(millis()) + " ALARM_GUI_ALARM");
        ALARM_FLAG = ALARM_FLAG | GenerateFlag(__ERROR_ALARM_PI);
        break;

    case ALARM_GUI_WDOG:
        DBG_print(2, "ALARM @ " + String(millis()) + " ALARM_WDOG");
        ALARM_FLAG = ALARM_FLAG | GenerateFlag(__ERROR_WDOG_PI);
        break;

    case UNPREDICTABLE_CODE_EXECUTION:
        DBG_print(2, "ALARM @ " + String(millis()) + " UNPREDICTABLE_CODE_EXECUTION");
        ALARM_FLAG = ALARM_FLAG | GenerateFlag(__ERROR_SYSTEM_FALIURE);
        break;

    default:

        break;
    }
}

void ResetAlarm(int alarm_code)
{
    //ALARM_FLAG = 0;// ALARM_FLAG & (~GenerateFlag(alarm_code));
    ALARM_FLAG_SNOOZE = ALARM_FLAG & (~GenerateFlag(__ERROR_ALARM_PI));
    ALARM_FLAG_SNOOZE_millis = millis();
    ALARM_FLAG = 0;
}

void CheckAlarmConditions(t_core__force_sm sm)
{
    static t_core__force_sm last_sm;
    static uint32_t buzzer_time = 0;
    static bool buzzer_beep = false;
    static uint8_t alarm_state = 0;
    static uint32_t led_time = 0;
    static bool led_on = false;

    //Over Pressure Input

    //Under Pressure Input

    //Over Pressure Inside
    if (pressure[0].last_pressure > 50)
        TriggerAlarm(ALARM_PRESSURE_INSIDE_TOO_HIGH);

    //Under Pressure Inside
    if ((last_sm == FR_WAIT_INHALE_TIME) && (sm != last_sm)) {
        if (pressure[0].last_pressure < 0.5 * core_config.target_pressure)
            TriggerAlarm(ALARM_PRESSURE_INSIDE_TOO_LOW);
    }
    //leakage
    if ((last_sm == FR_WAIT_INHALE_TIME) && (sm != last_sm)) {
        if (pressure[1].last_pressure < 0.8 * core_config.target_pressure)
            TriggerAlarm(ALARM_LEAKAGE);
    }

    //Occlusion complete
    /*if ((last_sm == FR_WAIT_INHALE_TIME) && (sm != last_sm))
  {
    if (fabs(pressure[1].last_pressure-pressure[0].last_pressure)< 0.2 * core_config.target_pressure )
      TriggerAlarm(ALARM_COMPLETE_OCCLUSION);
  }
  
  //Occlusion partial
  if ((last_sm == FR_WAIT_INHALE_TIME) && (sm != last_sm))
  {
    if (fabs(pressure[1].last_pressure-pressure[0].last_pressure)< 0.8 * core_config.target_pressure )
      TriggerAlarm(ALARM_PARTIAL_OCCLUSION);
  }
*/
    if (currentBatteryCharge < 20)
        TriggerAlarm(BATTERY_LOW);

    last_sm = sm;

    ALARM_FLAG = ALARM_FLAG & (~ALARM_FLAG_SNOOZE);

    if (ALARM_FLAG != 0) {
        //ledcWrite(1, 128);

        if (millis() - led_time > 250) {
            led_time = millis();
            led_on = led_on ? false : true;
            digitalWrite(ALARM_LED, led_on);
        }

        switch (alarm_state) {
        case 0:
            buzzer_time = millis();
            alarm_state = 1;
            digitalWrite(BUZZER, HIGH); //#1
            break;

        case 1:
            if (millis() - buzzer_time > 150) {
                digitalWrite(BUZZER, LOW);
                buzzer_time = millis();
                alarm_state = 2;
            }
            break;

        case 2:
            if (millis() - buzzer_time > 100) {
                digitalWrite(BUZZER, HIGH); //#2
                buzzer_time = millis();
                alarm_state = 3;
            }
            break;

        case 3:
            if (millis() - buzzer_time > 150) {
                digitalWrite(BUZZER, LOW);
                buzzer_time = millis();
                alarm_state = 4;
            }
            break;

        case 4:
            if (millis() - buzzer_time > 100) {
                digitalWrite(BUZZER, HIGH); //#3
                buzzer_time = millis();
                alarm_state = 5;
            }
            break;

        case 5:
            if (millis() - buzzer_time > 150) {
                digitalWrite(BUZZER, LOW);
                buzzer_time = millis();
                alarm_state = 6;
            }
            break;

        case 6:
            if (millis() - buzzer_time > 300) {
                digitalWrite(BUZZER, HIGH); //#4
                buzzer_time = millis();
                alarm_state = 7;
            }
            break;

        case 7:
            if (millis() - buzzer_time > 150) {
                digitalWrite(BUZZER, LOW);
                buzzer_time = millis();
                alarm_state = 8;
            }
            break;

        case 8:
            if (millis() - buzzer_time > 100) {
                digitalWrite(BUZZER, HIGH); //#5
                buzzer_time = millis();
                alarm_state = 9;
            }
            break;

        case 9:
            if (millis() - buzzer_time > 150) {
                digitalWrite(BUZZER, LOW);
                buzzer_time = millis();
                alarm_state = 10;
            }
            break;

        case 10:
            if (millis() - buzzer_time > 800) {
                digitalWrite(BUZZER, HIGH); //#5
                buzzer_time = millis();
                alarm_state = 11;
            }
            break;

        case 11:
            if (millis() - buzzer_time > 150) {
                digitalWrite(BUZZER, LOW);
                buzzer_time = millis();
                alarm_state = 12;
            }
            break;

        case 12:
            if (millis() - buzzer_time > 100) {
                digitalWrite(BUZZER, HIGH); //#2
                buzzer_time = millis();
                alarm_state = 13;
            }
            break;

        case 13:
            if (millis() - buzzer_time > 150) {
                digitalWrite(BUZZER, LOW);
                buzzer_time = millis();
                alarm_state = 14;
            }
            break;

        case 14:
            if (millis() - buzzer_time > 100) {
                digitalWrite(BUZZER, HIGH); //#3
                buzzer_time = millis();
                alarm_state = 15;
            }
            break;

        case 15:
            if (millis() - buzzer_time > 150) {
                digitalWrite(BUZZER, LOW);
                buzzer_time = millis();
                alarm_state = 16;
            }
            break;

        case 16:
            if (millis() - buzzer_time > 300) {
                digitalWrite(BUZZER, HIGH); //#4
                buzzer_time = millis();
                alarm_state = 17;
            }
            break;

        case 17:
            if (millis() - buzzer_time > 150) {
                digitalWrite(BUZZER, LOW);
                buzzer_time = millis();
                alarm_state = 18;
            }
            break;

        case 18:
            if (millis() - buzzer_time > 100) {
                digitalWrite(BUZZER, HIGH); //#5
                buzzer_time = millis();
                alarm_state = 19;
            }
            break;

        case 19:
            if (millis() - buzzer_time > 150) {
                digitalWrite(BUZZER, LOW);
                buzzer_time = millis();
                alarm_state = 20;
            }
            break;

        case 20:
            if (millis() - buzzer_time > 10000) {
                alarm_state = 0;
            }
            break;
        }
    }
    else {
        //ledcWrite(1, 0);
        digitalWrite(ALARM_LED, LOW);
        digitalWrite(BUZZER, LOW);
        alarm_state = 0;
        if (millis() > (ALARM_FLAG_SNOOZE_millis + 120000))
            ALARM_FLAG_SNOOZE = 0;
    }
}

void onTimerCoreTask()
{
    static float old_pressure[256];
    static float last_isp_time;

    static int timer_divider = 0;

    static uint32_t last_start = 0;

    DBG_print(10, "ITimer0: millis() = " + String(millis()));

    for (int q = 0; q < 255; q++) {
        old_pressure[255 - q] = old_pressure[254 - q];
    }
    old_pressure[0] = pressure[1].last_pressure;

    float mean_peep = (old_pressure[5] + old_pressure[6] + old_pressure[7] + old_pressure[8] + old_pressure[9]) / 5.0;
    float mean_peep_older = (old_pressure[25] + old_pressure[26] + old_pressure[27] + old_pressure[28] + old_pressure[29]) / 5.0;

    core_sm_context.timer1++;
    core_sm_context.timer2++;

    switch (core_sm_context.force_sm) {
    case FR_OPEN_INVALVE:
        dbg_state_machine = 0;
        if (core_config.run) {
            //RUN RESPIRATORY
            if (core_config.BreathMode == M_BREATH_FORCED) {
                //AUTOMATIC
                if (tidal_volume_c.TidalCorrection > 0) {
                    currentTvEsp = -1.0 * tidal_volume_c.ExpVolumeVenturi / tidal_volume_c.TidalCorrection;
                }

                ResetStats();
                TidalReset();
                last_peep = mean_peep;
                pres_peak = 0;

                float last_delta_time = millis() - last_start;
                last_start = millis();
                if (last_delta_time > 0) {
                    last_bpm = 60000.0 / last_delta_time;
                    averaged_bpm = averaged_bpm * 0.6 + last_bpm;
                }

                TidalInhale();
                fluxpeak = 0;
                peaktime = millis();

                valve_contol(VALVE_IN, VALVE_OPEN);

                core_sm_context.timer1 = 1;
                CoreSM_FORCE_ChangeState(&core_sm_context.force_sm,
                    FR_WAIT_INHALE_TIME);
                DBG_print(3, "FR_OPEN_INVALVE");
            }
            else {
                //ASSISTED
                if (core_config.BreathMode == M_BREATH_ASSISTED) {
                    pres_peak = 0;
                    valve_contol(VALVE_OUT, VALVE_OPEN);
                    dbg_trigger = 0;
                    //Trigger for assist breathing
                    if (((-1.0 * delta2) > core_config.assist_pressure_delta_trigger) && (delta < 0)) {
                        dbg_trigger = 1;
                        if (tidal_volume_c.TidalCorrection > 0) {
                            currentTvEsp = -1.0 * tidal_volume_c.ExpVolumeVenturi / tidal_volume_c.TidalCorrection;
                        }
                        StatEndCycle();
                        ResetStats();
                        TidalReset();
                        last_peep = mean_peep;

                        float last_delta_time = millis() - last_start;
                        last_start = millis();

                        if (last_delta_time > 0) {
                            last_bpm = 60000.0 / last_delta_time;
                            averaged_bpm = averaged_bpm * 0.6 + last_bpm;
                        }

                        TidalInhale();
                        fluxpeak = 0;
                        peaktime = 0;
                        valve_contol(VALVE_IN, VALVE_OPEN);
                        valve_contol(VALVE_OUT, VALVE_CLOSE);
                        core_sm_context.timer1 = 1;

                        CoreSM_FORCE_ChangeState(&core_sm_context.force_sm,
                            AST_WAIT_MIN_INHALE_TIME);

                        DBG_print(3, "FR_OPEN_INVALVE");
                    }
                }
                else {
                    //WE SHOULD NEVER GO HERE
                    TriggerAlarm(UNPREDICTABLE_CODE_EXECUTION);
                }
            }
        }
        else {
            //WE ARE NOT RUNNING
            valve_contol(VALVE_IN, VALVE_CLOSE);
            valve_contol(VALVE_OUT, VALVE_OPEN);
        }
        break;

    case FR_WAIT_INHALE_TIME:
        dbg_state_machine = 1;
        if (core_sm_context.timer1 >= (core_config.inhale_ms / TIMERCORE_INTERVAL_MS)) {
            if ((core_config.pause_inhale == false) && (core_config.pause_lg == false)) {
                core_sm_context.timer1 = 0;
                TidalExhale();
                StatPhaseExpire();

                currentP_Peak = pres_peak;
                currentTvIsnp = tidal_volume_c.InspVolumeSensirion;
                currentVM = fluxpeak;

                valve_contol(VALVE_IN, VALVE_CLOSE);
                valve_contol(VALVE_OUT, VALVE_OPEN);

                CoreSM_FORCE_ChangeState(&core_sm_context.force_sm,
                    FR_WAIT_EXHALE_TIME);
                DBG_print(3, "FR_WAIT_INHALE_TIME");
            }
            else {
                if (core_config.pause_lg == false)
                    valve_contol(VALVE_IN, VALVE_CLOSE);
                else {
                    core_config.pause_lg_timer -= TIMERCORE_INTERVAL_MS;
                    if (core_config.pause_lg_timer <= 0)
                        core_config.pause_lg = false;
                }
            }
        }
        break;

    case FR_WAIT_EXHALE_TIME:
        dbg_state_machine = 2;

        if (core_sm_context.timer1 >= (core_config.exhale_ms / TIMERCORE_INTERVAL_MS)) {
            if (core_config.pause_exhale == false) {
                StatEndCycle();
                peep_look = false;

                valve_contol(VALVE_OUT, VALVE_CLOSE);
                CoreSM_FORCE_ChangeState(&core_sm_context.force_sm,
                    FR_OPEN_INVALVE);

                DBG_print(3, "FR_WAIT_EXHALE_TIME");
            }
            else {
                valve_contol(VALVE_OUT, VALVE_CLOSE);
            }
        }
        break;

    case AST_WAIT_MIN_INHALE_TIME:
        dbg_state_machine = 3;
        if (core_sm_context.timer1 > 300 / TIMERCORE_INTERVAL_MS) {
            if (pressure[1].last_pressure >= core_config.target_pressure * 0.5) {
                CoreSM_FORCE_ChangeState(&core_sm_context.force_sm,
                    AST_WAIT_FLUX_DROP);
                DBG_print(3, "AST_WAIT_MIN_INHALE_TIME");
            }
        }
        break;

    case AST_WAIT_FLUX_DROP:
        dbg_state_machine = 4;

        if (gasflux[0].last_flux <= (core_config.flux_close * fluxpeak) / 100.0) {
            last_isp_time = core_sm_context.timer1;
            CoreSM_FORCE_ChangeState(&core_sm_context.force_sm,
                AST_WAIT_FLUX_DROP_b);
            DBG_print(3, "FR_WAIT_FLUX_DROP");
        }
        break;

    case AST_WAIT_FLUX_DROP_b:
        dbg_state_machine = 5;
        if ((core_config.pause_inhale == false) && (core_config.pause_lg == false)) {
            core_sm_context.timer1 = 1;
            TidalExhale();
            StatPhaseExpire();
            currentP_Peak = pres_peak;
            currentTvIsnp = tidal_volume_c.InspVolumeSensirion;
            currentVM = fluxpeak;
            valve_contol(VALVE_IN, VALVE_CLOSE);
            valve_contol(VALVE_OUT, VALVE_OPEN);
            CoreSM_FORCE_ChangeState(&core_sm_context.force_sm,
                AST_DEADTIME);
            DBG_print(3, "FR_WAIT_FLUX_DROP_b");
        }
        else {
            if (core_config.pause_lg == false)
                valve_contol(VALVE_IN, VALVE_CLOSE);
            else {
                core_config.pause_lg_timer -= TIMERCORE_INTERVAL_MS;
                if (core_config.pause_lg_timer <= 0)
                    core_config.pause_lg = false;
            }
        }
        break;

    case AST_DEADTIME:
        dbg_state_machine = 6;
        if (core_sm_context.timer1 >= last_isp_time) {
            if (core_config.pause_exhale == false) {
                CoreSM_FORCE_ChangeState(&core_sm_context.force_sm,
                    FR_OPEN_INVALVE);
            }
            else {
                CoreSM_FORCE_ChangeState(&core_sm_context.force_sm,
                    FR_OPEN_INVALVE);
            }
            DBG_print(3, "AST_DEADTIME");
        }
        break;

    case AST_PAUSE_EXHALE:
        dbg_state_machine = 7;
        if (core_config.pause_exhale == false) {
            CoreSM_FORCE_ChangeState(&core_sm_context.force_sm,
                FR_OPEN_INVALVE);
        }
        else {
            if (((-1.0 * delta2) > core_config.assist_pressure_delta_trigger) && (delta < 0)) {
                valve_contol(VALVE_IN, VALVE_CLOSE);
                valve_contol(VALVE_OUT, VALVE_CLOSE);
            }
        }
        break;

    default:
        dbg_state_machine = 1000;
        TriggerAlarm(UNPREDICTABLE_CODE_EXECUTION);
        CoreSM_FORCE_ChangeState(&core_sm_context.force_sm, FR_OPEN_INVALVE);
        break;
    }
    CheckAlarmConditions(core_sm_context.force_sm);
}

void InitParameters()
{
    core_config.run = false;
    core_config.constant_rate_mode = true;
    core_config.inhale_ms = 750;
    core_config.inhale_ms_extra = 00;
    core_config.exhale_ms = 1250;
    core_config.pressure_alarm = 100;
    core_config.pressure_alarm_off = 10;
    core_config.pressure_forced_inhale_max = 0;
    core_config.pressure_forced_exhale_min = 0;
    core_config.pressure_drop = 8;
    core_config.inhale_critical_alarm_ms = 16000;
    core_config.exhale_critical_alarm_ms = 16000;
    core_config.BreathMode = M_BREATH_FORCED; //M_BREATH_ASSISTED;//;
    core_config.sim.rate_inhale_pressure = 5;
    core_config.sim.rate_exhale_pressure = 10;
    core_config.flux_close = 5;
    core_config.assist_pressure_delta_trigger = 15;
    core_config.target_pressure = 20;
    core_config.target_pressure_auto = core_config.target_pressure;
    core_config.target_pressure_assist = core_config.target_pressure;

    core_config.respiratory_rate = 15;
    core_config.respiratory_ratio = 0.66;
    core_config.inhale_ms = 60000.0 / core_config.respiratory_rate * (1 - core_config.respiratory_ratio);
    core_config.exhale_ms = 60000.0 / core_config.respiratory_rate * (core_config.respiratory_ratio);

    core_config.P = 70; //70;//12.8;
    core_config.I = 10; //10;//16;
    core_config.D = 0;

    core_config.P2 = 1.4;
    core_config.I2 = 0.4;
    core_config.D2 = 0;

    core_config.pid_limit = 0.65;
    tidal_volume_c.AutoZero = 1;

    core_config.pause_lg = false;
    core_config.pause_lg_timer = 0;
}

void setup(void)
{
    //pinMode (VALVE_IN_PIN, OUTPUT);

    ledcSetup(0, 10000, 12);
    //ledcSetup(1, 4000, 8);

    // attach the channel to the GPIO to be controlled
    ledcAttachPin(DAC1, 0);
    //ledcAttachPin(BUZZER, 1);

    ledcWrite(0, 0);

    //ledcWrite(1, 0);
    pinMode(VALVE_OUT_PIN, OUTPUT);
    pinMode(ALARM_LED, OUTPUT);
    pinMode(BUZZER, OUTPUT);

    digitalWrite(ALARM_LED, LOW);
    digitalWrite(BUZZER, LOW);

    // Start Serial
    Serial.begin(115200);
    Wire.begin();

    delay(100);

    valve_contol(VALVE_IN, VALVE_CLOSE);
    valve_contol(VALVE_OUT, VALVE_OPEN);
    delay(3000);

    InitParameters();

    CoreTask.attach(TIMERCORE_INTERVAL_MS / 1000.0, onTimerCoreTask);

    for (int i = 0; i < 8; i++) {
        i2c_MuxSelect(i);
        Serial.println("SCAN I2C BUS: " + String(i));
        __service_i2c_detect();
    }

    for (int j = 0; j < N_PRESSURE_SENSORS; j++) {
        i2c_MuxSelect(pressure_sensor_i2c_mux[j]);
        Reset_5525DSO((int)pressure_sensor_i2c_address[j]);
    }

    delay(100);
    for (int j = 0; j < N_PRESSURE_SENSORS; j++) {
        i2c_MuxSelect(pressure_sensor_i2c_mux[j]);
        FirstConversion_5525DSO((int)pressure_sensor_i2c_address[j]);
    }

    for (int j = 0; j < N_PRESSURE_SENSORS; j++) {
        i2c_MuxSelect(pressure_sensor_i2c_mux[j]);
        ReadCalibration_5525DSO((int)pressure_sensor_i2c_address[j], &PRES_SENS_CT[j]);
        Serial.print("SENSOR:           ");
        Serial.println(j);
        Serial.print("SENS_T1:          ");
        Serial.println(PRES_SENS_CT[j].C[0]);
        Serial.print("OFF_T1:           ");
        Serial.println(PRES_SENS_CT[j].C[1]);
        Serial.print("TCS:              ");
        Serial.println(PRES_SENS_CT[j].C[2]);
        Serial.print("TCO:              ");
        Serial.println(PRES_SENS_CT[j].C[3]);
        Serial.print("TREF:             ");
        Serial.println(PRES_SENS_CT[j].C[4]);
        Serial.print("TEMPSENS:         ");
        Serial.println(PRES_SENS_CT[j].C[5]);

        switch (pressure_sensor_type[j]) {
        case DS_01:
            PRES_SENS_CT[j].Q[0] = 15;
            PRES_SENS_CT[j].Q[1] = 17;
            PRES_SENS_CT[j].Q[2] = 7;
            PRES_SENS_CT[j].Q[3] = 5;
            PRES_SENS_CT[j].Q[4] = 7;
            PRES_SENS_CT[j].Q[5] = 21;
            break;

        case GS_05:
            PRES_SENS_CT[j].Q[0] = 16;
            PRES_SENS_CT[j].Q[1] = 17;
            PRES_SENS_CT[j].Q[2] = 6;
            PRES_SENS_CT[j].Q[3] = 5;
            PRES_SENS_CT[j].Q[4] = 7;
            PRES_SENS_CT[j].Q[5] = 21;
            break;
        }

        float mean = 0;
        PRES_SENS_CT[j].ZERO = 0;
        for (int q = 0; q < 100; q++) {
            read_pressure_sensor(j);
            mean += pressure[j].last_pressure;
        }
        PRES_SENS_CT[j].ZERO = mean / 100;

        Serial.print("OFFSET:            ");
        Serial.println(PRES_SENS_CT[j].ZERO);
    }

    //MeasureFluxInit();
    i2c_MuxSelect(IIC_MUX_FLOW1);
    if (InitFlowMeter_SFM3019(&sfm3019_inhale) == false) {
        Serial.println(" Try using Sensirion Backup!");
        use_Sensirion_Backup = true;
        MeasureFluxInit();
    }

    cli.setOnError(CliErrorCallback); // Set error Callback
    param_set = cli.addCommand("set", SetCommandCallback);
    param_set.addPositionalArgument("param", "null");
    param_set.addPositionalArgument("value", "0");

    param_get = cli.addCommand("get", GetCommandCallback);
    param_get.addPositionalArgument("param", "null");

    valve_contol(VALVE_IN, VALVE_CLOSE);
    valve_contol(VALVE_OUT, VALVE_CLOSE);

    i2c_MuxSelect(IIC_MUX_P_FASTLOOP);

    watchdog_time = millis();

    ResetStatsBegin();
}

void CliErrorCallback(cmd_error* e)
{
    CommandError cmdError(e); // Create wrapper object
    Serial.print("valore=ERROR: ");
    Serial.println(cmdError.toString());
}

void SetCommandCallback(cmd* c)
{
    Command cmd(c); // Create wrapper object

    // Get arguments
    Argument param = cmd.getArgument("param");
    Argument value = cmd.getArgument("value");

    String strPatam = param.getValue();

    //Serial.println("CMD: " +  param.getValue() + " " +  value.getValue());

    if (strPatam == "run") {
        int numberValue = value.getValue().toInt();

        if (numberValue < 1)
            core_config.run = false;
        else
            core_config.run = true;

        Serial.println("valore=OK");
    }

    if (strPatam == "mode") {
        int numberValue = value.getValue().toInt();
        if (numberValue == 0) {
            //Forced Mode
            core_config.constant_rate_mode = true;
            core_config.pressure_alarm = 100;
            core_config.pressure_alarm_off = 50;
            core_config.inhale_critical_alarm_ms = 16000;
            core_config.exhale_critical_alarm_ms = 16000;
            core_config.BreathMode = M_BREATH_FORCED;
        }
        else {
            //Assisted Mode
            core_config.constant_rate_mode = false;
            core_config.pressure_alarm = 100;
            core_config.pressure_alarm_off = 50;
            core_config.inhale_critical_alarm_ms = 16000;
            core_config.exhale_critical_alarm_ms = 16000;
            core_config.BreathMode = M_BREATH_ASSISTED;
        }
        Serial.println("valore=OK");
    }

    if (strPatam == "rate") {
        float numberValue = value.getValue().toFloat();
        core_config.respiratory_rate = numberValue;
        core_config.inhale_ms = 60000.0 / core_config.respiratory_rate * (1 - core_config.respiratory_ratio);
        core_config.exhale_ms = 60000.0 / core_config.respiratory_rate * (core_config.respiratory_ratio);
        Serial.println("valore=OK");
    }

    if (strPatam == "ratio") {
        float numberValue = value.getValue().toFloat();
        core_config.respiratory_ratio = numberValue;
        core_config.inhale_ms = 60000.0 / core_config.respiratory_rate * (1 - core_config.respiratory_ratio);
        core_config.exhale_ms = 60000.0 / core_config.respiratory_rate * (core_config.respiratory_ratio);
        Serial.println("valore=OK");
    }

    if (strPatam == "assist_ptrigger") {
        float numberValue = value.getValue().toFloat();
        core_config.assist_pressure_delta_trigger = numberValue;
        Serial.println("valore=OK");
    }

    if (strPatam == "assist_flow_min") {
        float numberValue = value.getValue().toFloat();
        core_config.flux_close = numberValue;
        Serial.println("valore=OK");
    }

    if (strPatam == "ptarget") {
        float numberValue = value.getValue().toFloat();
        core_config.target_pressure_auto = numberValue;
        Serial.println("valore=OK");
    }

    if (strPatam == "pressure_support") {
        float numberValue = value.getValue().toFloat();
        core_config.target_pressure_assist = numberValue;
        Serial.println("valore=OK");
    }

    if (strPatam == "peep") {
        float numberValue = value.getValue().toFloat();
        core_config.pressure_forced_exhale_min = numberValue;
        Serial.println("valore=OK");
    }

    if (strPatam == "pid_p") {
        float numberValue = value.getValue().toFloat();
        core_config.P = numberValue;
        Serial.println("valore=OK");
    }

    if (strPatam == "pid_i") {
        float numberValue = value.getValue().toFloat();
        core_config.I = numberValue;
        Serial.println("valore=OK");
    }

    if (strPatam == "pid_d") {
        float numberValue = value.getValue().toFloat();
        core_config.D = numberValue;
        Serial.println("valore=OK");
    }

    if (strPatam == "pid_p2") {
        float numberValue = value.getValue().toFloat();
        core_config.P2 = numberValue;
        Serial.println("valore=OK");
    }

    if (strPatam == "pid_i2") {
        float numberValue = value.getValue().toFloat();
        core_config.I2 = numberValue;
        Serial.println("valore=OK");
    }

    if (strPatam == "pid_d2") {
        float numberValue = value.getValue().toFloat();
        core_config.D2 = numberValue;
        Serial.println("valore=OK");
    }

    if (strPatam == "pause_inhale") {
        int numberValue = value.getValue().toInt();
        core_config.pause_inhale = numberValue;
        Serial.println("valore=OK");
    }

    if (strPatam == "pause_lg") {
        int numberValue = value.getValue().toInt();
        core_config.pause_lg = numberValue ? true : false;
        Serial.println("valore=OK");
    }

    if (strPatam == "pause_lg_time") {
        int numberValue = value.getValue().toFloat();
        core_config.pause_lg_timer = numberValue * 1000.0;
        Serial.println("valore=OK");
    }

    if (strPatam == "pause_exhale") {
        int numberValue = value.getValue().toInt();
        core_config.pause_exhale = numberValue;
        Serial.println("valore=OK");
    }

    if (strPatam == "pid_limit") {
        float numberValue = value.getValue().toFloat();
        core_config.pid_limit = numberValue;
        Serial.println("valore=OK");
    }

    if (strPatam == "alarm_snooze") {
        int numberValue = value.getValue().toInt();
        ResetAlarm(numberValue);
        Serial.println("valore=OK");
    }

    if (strPatam == "alarm") {
        int numberValue = value.getValue().toInt();
        TriggerAlarm(ALARM_GUI_ALARM);
        Serial.println("valore=OK");
    }

    if (strPatam == "watchdog_reset") {
        int numberValue = value.getValue().toInt();
        watchdog_time = millis();
        ALARM_FLAG = ALARM_FLAG & (~GenerateFlag(__ERROR_WDOG_PI));
        Serial.println("valore=OK");
    }

    if (strPatam == "console") {
        int numberValue = value.getValue().toInt();
        __CONSOLE_MODE = numberValue != 0 ? true : false;
        Serial.println("valore=OK");
    }

    if (strPatam == "timestamp") {
        int numberValue = value.getValue().toInt();
        __ADDTimeStamp = numberValue != 0 ? true : false;
        Serial.println("valore=OK");
    }

    if (strPatam == "wdenable") {
        int numberValue = value.getValue().toInt();
        __WDENABLE = numberValue != 0 ? true : false;
        ALARM_FLAG = ALARM_FLAG & (~GenerateFlag(__ERROR_ALARM_PI));
        Serial.println("valore=OK");
    }

    if (strPatam == "backup_enable") {
        int numberValue = value.getValue().toInt();
        core_config.backup_enable = numberValue ? true : false;
        Serial.println("valore=OK");
    }

    if (strPatam == "backup_min_rate") {
        float numberValue = value.getValue().toFloat();
        core_config.backup_min_rate = numberValue;
        Serial.println("valore=OK");
    }

    if (strPatam == "stats_clear") {
        ResetStatsBegin();
    }
}

void GetCommandCallback(cmd* c)
{
    Command cmd(c); // Create wrapper object

    // Get arguments
    Argument param = cmd.getArgument("param");

    String strPatam = param.getValue();

    //Serial.println("CMD: " +  param.getValue() + " " +  value.getValue());

    if (strPatam == "pressure") {
        Serial.println("valore=" + String(pressure[0].last_pressure));
    }

    if (strPatam == "flow") {
        Serial.println("valore=" + String(gasflux[0].last_flux));
    }

    if (strPatam == "o2") {
        Serial.println("valore=" + String(last_O2));
    }

    if (strPatam == "bpm") {
        Serial.println("valore=" + String(last_bpm));
    }

    if (strPatam == "backup") {
        Serial.println("valore=" + String(0));
    }

    if (strPatam == "tidal") {
        Serial.println("valore=" + String(tidal_volume_c.TidalVolume));
    }

    if (strPatam == "peep") {
        Serial.println("valore=" + String(last_peep));
    }

    if (strPatam == "temperature") {
        Serial.println("valore=" + String(temperature));
    }

    if (strPatam == "power_mode") {
        Serial.println("valore=" + String(batteryPowered ? 1 : 0));
    }

    if (strPatam == "battery") {

        Serial.println("valore=" + String(currentBatteryCharge));
    }

    if (strPatam == "version") {
        Serial.println("valore=" + String(_FIRMWARE_VERSION_));
    }

    if (strPatam == "alarm") {
        Serial.println("valore=" + String(ALARM_FLAG));
    }

    if (strPatam == "warning") {
        Serial.println("valore=" + String(0));
    }

    if (strPatam == "run") {
        Serial.println("valore=" + String(core_config.run ? 1 : 0));
    }

    if (strPatam == "mode") {
        Serial.println("valore=" + String(core_config.BreathMode == M_BREATH_ASSISTED ? 1 : 0));
    }
    if (strPatam == "rate") {
        Serial.println("valore=" + String(core_config.respiratory_rate));
    }
    if (strPatam == "ratio") {
        Serial.println("valore=" + String(core_config.respiratory_ratio));
    }
    if (strPatam == "assist_ptrigger") {
        Serial.println("valore=" + String(core_config.assist_pressure_delta_trigger));
    }
    if (strPatam == "assist_flow_min") {
        Serial.println("valore=" + String(core_config.flux_close));
    }
    if (strPatam == "ptarget") {
        Serial.println("valore=" + String(core_config.target_pressure_auto));
    }
    if (strPatam == "pressure_support") {
        Serial.println("valore=" + String(core_config.target_pressure_assist));
    }
    if (strPatam == "backup_enable") {
        Serial.println("valore=" + String(core_config.backup_enable ? 1 : 0));
    }
    if (strPatam == "backup_min_rate") {
        Serial.println("valore=" + String(core_config.backup_min_rate));
    }

    if (strPatam == "all") {
        Serial.println("valore=" + String(pressure[1].last_pressure) + "," + String(tidal_volume_c.FLUX) + "," + String(last_O2) + "," + String(last_bpm)
            + "," + String(tidal_volume_c.TidalVolume) + "," + String(last_peep)
            + "," + String(temperature) + "," + String(batteryPowered ? 1 : 0) + "," + String(currentBatteryCharge)
            + "," + String(currentP_Peak)
            + "," + String(currentTvIsnp)
            + "," + String(currentTvEsp)
            + "," + String(currentVM));
    }

    if (strPatam == "calib") {
        Serial.print("Valore=");
        for (int j = 0; j < N_PRESSURE_SENSORS; j++) {
            i2c_MuxSelect(pressure_sensor_i2c_mux[j]);
            float mean = 0;
            PRES_SENS_CT[j].ZERO = 0;
            for (int q = 0; q < 100; q++) {
                read_pressure_sensor(j);
                mean += pressure[j].last_pressure;
            }
            PRES_SENS_CT[j].ZERO = mean / 100;

            Serial.print(String(PRES_SENS_CT[j].ZERO) + ",");
        }
        Serial.println(" ");
    }

    if (strPatam == "calibv") {
        if (fabs(tidal_volume_c.ExpVolumeVenturi) > 0)
            tidal_volume_c.AutoZero = fabs(tidal_volume_c.InspVolumeVenturi) / fabs(tidal_volume_c.ExpVolumeVenturi);

        Serial.println("valore=" + String(tidal_volume_c.InspVolumeVenturi) + "," + String(tidal_volume_c.ExpVolumeVenturi) + "," + String(tidal_volume_c.AutoZero));
    }

    if (strPatam == "stats") {
        if (__stat_param.mean_cnt > 0) {
            float overshoot_avg = __stat_param.overshoot_avg / __stat_param.mean_cnt;
            float overshoot_length_avg = __stat_param.overshoot_length_avg / __stat_param.mean_cnt;
            float final_error_avg = __stat_param.final_error_avg / __stat_param.mean_cnt;
            float t1050_avg = __stat_param.t1050_avg / __stat_param.mean_cnt;
            float t1090_avg = __stat_param.t1090_avg / __stat_param.mean_cnt;
            float tpeak_avg = __stat_param.tpeak_avg / __stat_param.mean_cnt;
            float t9010_avg = __stat_param.t9010_avg / __stat_param.mean_cnt;
            float t9050_avg = __stat_param.t9050_avg / __stat_param.mean_cnt;
            float peep_avg = __stat_param.peep_avg / __stat_param.mean_cnt;
            float t10_avg = __stat_param.t10_avg / __stat_param.mean_cnt;
            float time_to_peak_avg = __stat_param.time_to_peak_avg / __stat_param.mean_cnt;
            float flux_peak_avg = __stat_param.flux_peak_avg / __stat_param.mean_cnt;
            float flux_t1090_avg = __stat_param.flux_t1090_avg / __stat_param.mean_cnt;
            float flux_t9010_avg = __stat_param.flux_t9010_avg / __stat_param.mean_cnt;

            Serial.println("valore=overshoot_avg:" + String(overshoot_avg)
                + ",overshoot_length_avg:" + String(overshoot_length_avg)
                + ",final_error:" + String(final_error_avg)
                + ",t1050_avg:" + String(t1050_avg)
                + ",t1090_avg:" + String(t1090_avg)
                + ",tpeak_avg:" + String(tpeak_avg)
                + ",t9010_avg:" + String(t9010_avg)
                + ",t9050_avg:" + String(t9050_avg)
                + ",peep_avg:" + String(peep_avg)
                + ",t10_avg:" + String(t10_avg)
                + ",time_to_peak_avg:" + String(time_to_peak_avg)
                + ",flux_peak_avg:" + String(flux_peak_avg)
                + ",flux_t1090_avg:" + String(flux_t1090_avg)
                + ",flux_t9010_avg:" + String(flux_t9010_avg));
        }
        else {
            Serial.println("valore=no_data");
        }
    }
}

/*float P = 0.8;
  float I = 1;
  float D =0;*/

float fast_pid_set = 0;

void PressureControlLoop_PRESSIN_SLOW()
{

    static float pid_error = 0;
    static float pid_integral = 0;
    static float pid_prec = 0;
    static float pid_out = 0;

    float PID_P = core_config.P2;
    float PID_I = core_config.I2; //0.2 over resistance 50
    float PID_D = core_config.D2;
    static float Pset2 = 0;

    static float pid_outb = 0;

    float Pmeas = 0;

    Pmeas = pressure[1].last_pressure;

    if (Pset == 0) {
        Pset2 = Pmeas;
        pid_integral = 0;
        pid_prec = 0;
        pid_outb = 0;
    }
    else {
        Pset2 = (Pset2 * 0.7) + (0.3 * Pset);

        pid_error = Pset2 - Pmeas;
        pid_integral += pid_error;
        // if ((pid_integral*PID_I) > 60 ) pid_integral = (60/PID_I);
        if (pid_integral < 0)
            pid_integral = 0;

        pid_out = PID_P * pid_error + PID_I * pid_integral + PID_D * (pid_error - pid_prec);

        pid_outb = pid_out;

        if (pid_outb < Pset2 * core_config.pid_limit)
            pid_outb = Pset2 * core_config.pid_limit;
        if (pid_outb > 50)
            pid_outb = 50;

        pid_prec = pid_error;

        fast_pid_set = pid_outb;
    }

    PIDMonitor2 = pid_outb;
}

void PressureControlLoop_PRESSIN()
{

    static float pid_error = 0;
    static float pid_integral = 0;
    static float pid_prec = 0;
    static float pid_out = 0;

    //valvola parker
    //P=0.8
    //I=1;  //0.2 over 50 resistance
    //D=0;
    float PID_P = core_config.P;
    float PID_I = core_config.I; //0.2 over resistance 50
    float PID_D = core_config.D;
    static float Pset2 = 0;

    static float pid_outb = 0;

    float Pmeas = 0;

    Pmeas = pressure[0].last_pressure;

    if (Pset == 0) {
        Pset2 = Pmeas;
        //if (pid_integral == (4095/PID_I))
        pid_integral = 0;
        pid_prec = 0;
        ledcWrite(0, 0);
    }
    else {
        Pset2 = (Pset2 * 0.9) + (0.1 * fast_pid_set);
        // Pset2 = (Pset2*0.7 )+ (0.3 * Pset); //Parker con lettura lenta

        pid_error = Pset2 - Pmeas;
        pid_integral += pid_error;
        if ((pid_integral * PID_I) > 4095)
            pid_integral = (4095 / PID_I);
        if ((pid_integral * PID_I) < -4095)
            pid_integral = -(4095 / PID_I);

        pid_out = PID_P * pid_error + PID_I * pid_integral + PID_D * (pid_error - pid_prec);

        //pid_outb = pid_outb * 0.8 + pid_out*0.2;
        pid_outb = pid_out;
        if (pid_outb < 0)
            pid_outb = 0;
        pid_outb = pid_outb + 500;
        if (pid_outb > 4090)
            pid_outb = 4090;

        pid_prec = pid_error;

        if (Pset == 0)
            ledcWrite(0, 0);
        else
            ledcWrite(0, pid_outb);
    }

    PIDMonitor = pid_outb;
}

void loop()
{
    static float pmem[6];
    static float VenturiFlux;
    static uint32_t last_loop_time;
    static uint8_t RestStateMachine = 0;
    static int serverhanging = 0; // Used to monitor how long the client is hanging
    static int serverhangingrestartdelay = 500; // delay after which we discard a hanging client
    static uint32_t sensor_read_last_time = 0;

    static float old_delta;
    static float pplow_old;

    core_config.target_pressure = core_config.BreathMode == M_BREATH_FORCED ? core_config.target_pressure_auto : core_config.target_pressure_assist;

    //Check connection with raspberry when RUN
    if (__CONSOLE_MODE == false) {
        if ((core_config.run) && (__WDENABLE == true)) {
            if (millis() > watchdog_time + 5000) {
                TriggerAlarm(ALARM_GUI_WDOG);
            }
        }
    }

    if (read_pressure_sensor(0) != 0) {
        TriggerAlarm(UNABLE_TO_READ_SENSOR_PRESSURE);
    }

    PressureControlLoop_PRESSIN();


    if (millis() > sensor_read_last_time + 20) {

        if (read_pressure_sensor(1) != 0) {
            TriggerAlarm(UNABLE_TO_READ_SENSOR_PRESSURE);
        }

        PressureControlLoop_PRESSIN_SLOW();
        pplow = 0.90 * pplow + pressure[1].last_pressure * 0.1;
        pres_peak = pplow > pres_peak ? pplow : pres_peak;
        pmem[5] = pmem[4];
        pmem[4] = pmem[3];
        pmem[3] = pmem[2];
        pmem[2] = pmem[1];
        pmem[1] = pmem[0];
        pmem[0] = pplow;
        delta = pplow - pplow_old;
        delta2 = (delta - old_delta) * 100;
        pplow_old = pmem[5];
        old_delta = delta;
        dgb_delta = delta2;

        float flux;
        //if (MeasureFlux(&flux) == false)
        //{
        //
        //}
        i2c_MuxSelect(IIC_MUX_FLOW1);
        if (use_Sensirion_Backup) {
            //Backup (OLD) sensore
            if (MeasureFlux(&flux) == false) {
                TriggerAlarm(UNABLE_TO_READ_SENSOR_FLUX);
            }
            temperature = 24;
        }
        else {
            //Normal Sensiro Sensor
            if (!MeasureFlux_SFM3019(&sfm3019_inhale, &flux, &temperature)) {
                TriggerAlarm(UNABLE_TO_READ_SENSOR_FLUX);
            }
        }

        gasflux[0].last_flux = flux; //gasflux[0].last_flux * 0.5 + (0.5*flux);

        //DBG_print(1,String(millis()) + "," + String(gasflux[0].last_flux) + "," + String(pressure[0].last_pressure)+ "," + String(valve1_status) + "," + String(valve2_status));
        //DBG_print(1, "PRESSURE:      " + String(pressure[0].last_pressure));
        sensor_read_last_time = millis();
        gasflux[0].read_millis = millis();

        //if (millis()-peaktime<1000)

        fluxpeak = fluxpeak > gasflux[0].last_flux ? fluxpeak : gasflux[0].last_flux;
        dgb_peaktime = fluxpeak;

        i2c_MuxSelect(pressure_sensor_i2c_mux[2]);
        if (read_pressure_sensor(2) != 0) {
            TriggerAlarm(UNABLE_TO_READ_SENSOR_VENTURI);
        }
        float dp, vf;
        dp = pressure[2].last_pressure;
        vf = 0.1513 * (dp * dp * dp) - 3.3424 * (dp * dp) + 41.657 * dp;
        VenturiFlux = 0.8 * VenturiFlux + 0.2 * vf;

        switch (tidal_volume_c.TidalStatus) {
        case 0:
            tidal_volume_c.TidalVolume = 0;
            tidal_volume_c.InspVolumeSensirion = 0;
            tidal_volume_c.InspVolumeVenturi = 0;
            tidal_volume_c.TidalCorrection = 1;
            tidal_volume_c.FLUX = gasflux[0].last_flux;
            break;
        case 1:
            tidal_volume_c.TidalVolume += gasflux[0].last_flux;
            tidal_volume_c.InspVolumeSensirion += gasflux[0].last_flux;
            tidal_volume_c.InspVolumeVenturi += vf;
            tidal_volume_c.FLUX = gasflux[0].last_flux;
            break;

        case 2:
            tidal_volume_c.ExpVolumeVenturi += vf;
            if (tidal_volume_c.TidalCorrection > 0) {
                tidal_volume_c.TidalVolume += (vf / tidal_volume_c.TidalCorrection) * tidal_volume_c.AutoZero;
                tidal_volume_c.FLUX = (VenturiFlux / tidal_volume_c.TidalCorrection);
            }
            break;
        }

        StatsUpdate();
        i2c_MuxSelect(IIC_MUX_P_FASTLOOP);
    }

    //DBG_print(1,String(gasflux[0].last_flux) + "," + String(pressure[0].last_pressure)+ "," + String(PIDMonitor/2) + "," + String(valve2_status)+"," +
    //String(dbg_state_machine*10)+"," + String(dgb_delta*100) + "," + String(dbg_trigger*50)+ "," + String(dgb_peaktime));

    if (__CONSOLE_MODE == true) {
        String ts = __ADDTimeStamp ? String(millis()) + "," : "";
        DBG_print(1, ts + String(gasflux[0].last_flux) + "," + String(pressure[0].last_pressure) + "," + String(pressure[1].last_pressure) + "," + String(PIDMonitor * 100 / 4096) + "," + String(PIDMonitor2) + "," + String(valve2_status) + "," + String(VenturiFlux) + "," + String(tidal_volume_c.FLUX) + "," + String(tidal_volume_c.TidalVolume * 0.02) + "," + String(dgb_delta));
    }
    //String(dgb_delta*100) + "," + String(dbg_trigger*50));

    if (Serial.available()) {
        // Read out string from the serial monitor
        String input = Serial.readStringUntil('\n');

        // Echo the user input
        // Serial.print("# ");
        //  Serial.println(input);

        // Parse the user input into the CLI
        cli.parse(input);
    }
}

int read_pressure_sensor(int idx)
{
    if (idx < N_PRESSURE_SENSORS) {

        if (SIMULATE_SENSORS == 1) {
        }
        else {
            uint8_t i2c_address = pressure_sensor_i2c_address[idx];
            int32_t raw_temp;
            int32_t raw_pressure;
            float T;
            float P;

            if (Convert_5525DSO((int)i2c_address, &raw_temp, &raw_pressure, true)) {
                CalibrateDate_5525DSO(PRES_SENS_CT[idx], raw_temp, raw_pressure, &T, &P);

                pressure[idx].last_pressure = P;
                pressure[idx].read_millis = millis();
            }
        }

        return 0;
    }
    else
        return -1;
}

int valve_contol(valves valve, int level)
{
    if (valve == VALVE_IN) {
        if (level == VALVE_CLOSE) {
            valve1_status = 0;
            //digitalWrite(VALVE_IN_PIN, LOW);
            Pset = 0;
            DBG_print(3, "VALVE IN CLOSED");
        }
        else if (level == VALVE_OPEN) {
            valve1_status = 100;
            //digitalWrite(VALVE_IN_PIN, HIGH);
            Pset = core_config.target_pressure;
            ;
            DBG_print(3, "VALVE IN OPENED");
        }
        else {
            //PWM MODE TO BE TESTED
        }

        //is there a way to have a feedback that the valve is working
        //insert here
        return 0;
    }
    else {
        if (valve == VALVE_OUT) {
            if (level == VALVE_CLOSE) {
                valve2_status = 0;
                digitalWrite(VALVE_OUT_PIN, HIGH);

                DBG_print(3, "VALVE OUT CLOSED");
            }
            else if (level == VALVE_OPEN) {
                valve2_status = 100;
                digitalWrite(VALVE_OUT_PIN, LOW);

                DBG_print(3, "VALVE OUT OPENED");
            }
            else {
                //PWM MODE TO BE TESTED
            }

            //is there a way to have a feedback that the valve is working
            //insert here
            return 0;
        }
        else {
            return -1;
        }
    }
}

void __service_i2c_detect()
{
    byte error, address;
    int nDevices;
    Serial.println("Scanning... I2C");
    nDevices = 0;
    for (address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();
        if (error == 0) {
            Serial.print("I2C device found at address 0x");
            if (address < 16) {
                Serial.print("0");
            }
            Serial.println(address, HEX);
            nDevices++;
        }
        else if (error == 4) {
            Serial.print("Unknow error at address 0x");
            if (address < 16) {
                Serial.print("0");
            }
            Serial.println(address, HEX);
        }
    }
    if (nDevices == 0) {
        Serial.println("No I2C devices found\n");
    }
    else {
        Serial.println("done\n");
    }
}

bool ReadCalibration_5525DSO(int address, t_5525DSO_calibration_table* ct)
{
    bool bres = true;
    int error;
    for (int i = 0; i < 6; i++) {
        Wire.beginTransmission(address);
        Wire.write(0xA0 + ((i + 1) << 1)); // MSB
        error = Wire.endTransmission();
        Wire.requestFrom(address, 2);
        byte MSB = Wire.read();
        byte LSB = Wire.read();
        error = Wire.endTransmission();
        ct->C[i] = (MSB << 8) + LSB;
    }

    return bres;
}

bool Reset_5525DSO(int address)
{
    int error;
    Wire.beginTransmission(address);
    Wire.write(0x1E); // MSB
    error = Wire.endTransmission();
    return true;
}

bool FirstConversion_5525DSO(int address)
{
    Wire.beginTransmission(address);
    Wire.write(0x58); // MSB
    Wire.endTransmission();
    return true;
}

bool Convert_5525DSO(int address, int32_t* temp, int32_t* pressure, bool read_temp)
{

    bool bres = true;
    int error;
    byte b1, b2, b3;

    Wire.beginTransmission(address);
    Wire.write(0x42); // MSB
    error = Wire.endTransmission();

    delay(2);

    Wire.beginTransmission(address);
    Wire.write(0x00); // MSB
    error = Wire.endTransmission();

    delay(1);

    error = Wire.requestFrom(address, 3, true);
    if (error < 3)
        return false;
    b1 = Wire.read();
    b2 = Wire.read();
    b3 = Wire.read();

    *pressure = (b1 << 16) + (b2 << 8) + b3;

    if (read_temp == true) {

        Wire.beginTransmission(address);
        Wire.write(0x52); // MSB
        error = Wire.endTransmission();

        delay(2);

        Wire.beginTransmission(address);
        Wire.write(0x00); // MSB
        error = Wire.endTransmission();

        delay(1);

        error = Wire.requestFrom(address, 3, true);
        if (error < 3)
            return false;
        b1 = Wire.read();
        b2 = Wire.read();
        b3 = Wire.read();
        *temp = (b1 << 16) + (b2 << 8) + b3;
    }

    return true;
}

void CalibrateDate_5525DSO(t_5525DSO_calibration_table CT, int32_t raw_temp, int32_t raw_pressure, float* T, float* P)
{
    int32_t Q1 = CT.Q[0];
    int32_t Q2 = CT.Q[1];
    int32_t Q3 = CT.Q[2];
    int32_t Q4 = CT.Q[3];
    int32_t Q5 = CT.Q[4];
    int32_t Q6 = CT.Q[5];

    int32_t C1 = CT.C[0];
    int32_t C2 = CT.C[1];
    int32_t C3 = CT.C[2];
    int32_t C4 = CT.C[3];
    int32_t C5 = CT.C[4];
    int32_t C6 = CT.C[5];

    int32_t dT;
    int64_t OFF;
    int64_t SENS;

    int32_t Temp;
    int64_t Pres;
    //Serial.println(raw_temp);
    //Serial.println(raw_pressure);
    dT = raw_temp - (C5 * pow(2, Q5));
    Temp = 2000 + ((dT * C6) / pow(2, Q6));
    OFF = (C2 * pow(2, Q2)) + ((C4 * dT) / (pow(2, Q4)));
    SENS = (C1 * pow(2, Q1)) + ((C3 * dT) / (pow(2, Q3)));
    Pres = (((raw_pressure * SENS) / (pow(2, 21))) - OFF) / (pow(2, 15));

    *T = ((float)Temp) / 100.0;
    *P = ((float)Pres) / 10000.0 * 68.9476;
    *P = *P - CT.ZERO;
}

void MeasureFluxInit()
{
    Wire.begin();
    Wire.beginTransmission(byte(flow_sensor_i2c_address[0])); // transmit to device with I2C mI2cAddress
    Wire.beginTransmission(byte(flow_sensor_i2c_address[0])); // transmit to device with I2C mI2cAddress
    Wire.write(byte(0x10)); //
    Wire.write(byte(0x00)); //
    Wire.endTransmission();
}

uint8_t crc8(const uint8_t data, uint8_t crc)
{
    crc ^= data;
    for (uint8_t i = 8; i; --i) {
        crc = (crc & 0x80)
            ? (crc << 1) ^ 0x31
            : (crc << 1);
    }
    return crc;
}

float MeasureFluxRaw()
{
    Wire.requestFrom(flow_sensor_i2c_address[0], 3); // read 3 bytes from device with address 0x40
    uint16_t a = Wire.read(); // first received byte stored here. The variable "uint16_t" can hold 2 bytes, this will be relevant later
    uint8_t b = Wire.read(); // second received byte stored here
    uint8_t crc = Wire.read(); // crc value stored here
    uint8_t mycrc = 0xFF; // initialize crc variable

    a = (a << 8) | b; // combine the two received bytes to a 16bit integer value
    // a >>= 2; // remove the two least significant bits
    //float Flow = (float)a;
    int Flow = a;
    return Flow;
}

bool MeasureFlux(float* Flow)
{
    unsigned int result = MeasureFluxRaw();
    int offset = 32000; // Offset for the sensor
    float scale = 140.0; // Scale factor for Air and N2 is 140.0, O2 is 142.8
    *Flow = ((float)result - offset) / scale;
    return true;
}

bool InitFlowMeter_SFM3019(SfmConfig* sfm3019)
{
    const char* driver_version = sfm_common_get_driver_version();
    if (driver_version) {
        DBG_print(1, "SFM driver version " + String(driver_version));
    }
    else {
        DBG_print(1, "SFM fatal: Getting driver version failed");
        return false;
    }

    /* Initialize I2C bus */
    sensirion_i2c_init();

    /* Reset all I2C devices */
    int16_t error = sensirion_i2c_general_call_reset();
    if (error) {
        DBG_print(1, "General call reset failed");
        return false;
    }
    /* Wait for the SFM3019 to initialize */
    sensirion_sleep_usec(SFM3019_SOFT_RESET_TIME_US);

    uint32_t timeout = millis();
    while (sfm3019_probe()) {
        DBG_print(1, "SFM sensor probing failed");
        if (millis() - timeout > 1000)
            return false;
        delay(10);
    }

    uint32_t product_number = 0;
    uint8_t serial_number[8] = {};
    error = sfm_common_read_product_identifier(SFM3019_I2C_ADDRESS,
        &product_number, &serial_number);
    if (error) {
        DBG_print(1, "Failed to read product identifier");
        return false;
    }
    else {
        DBG_print(1, "product: " + String(product_number));
        for (size_t i = 0; i < 8; ++i) {
            DBG_print(1, String(serial_number[i]));
        }
    }

    *sfm3019 = sfm3019_create();

    error = sfm_common_start_continuous_measurement(
        sfm3019, SFM3019_CMD_START_CONTINUOUS_MEASUREMENT_AIR);
    if (error) {
        Serial.println("Failed to start measurement");
    }

    /* Wait for the first measurement to be available. Wait for
     * SFM3019_MEASUREMENT_WARM_UP_TIME_US instead for more reliable results */
    delay(100);
    return true;
}

bool MeasureFlux_SFM3019(SfmConfig* sfm3019, float* Flow, float* T)
{
    int16_t flow_raw;
    int16_t temperature_raw;
    uint16_t status;
    int16_t error;
    error = sfm_common_read_measurement_raw(sfm3019, &flow_raw,
        &temperature_raw, &status);
    if (error) {
        //Serial.println("Error while reading measurement");
        return false;
    }
    else {
        float flow;
        float temperature;
        error = sfm_common_convert_flow_float(sfm3019, flow_raw, &flow);
        if (error) {
            //Serial.println("Error while converting flow");
            return false;
        }
        *T = sfm_common_convert_temperature_float(temperature_raw);
        *Flow = flow;
        //Serial.println("Flow: " + String(flow)  + " flow_raw: " + String(flow_raw) + " T: " +String(temperature) + " Traw: " +String(temperature_raw) + " status: " + String(status));
    }
}

void i2c_MuxSelect(uint8_t i)
{
    if (i > 7)
        return;

    Wire.beginTransmission(TCAADDR);
    Wire.write(1 << i);
    Wire.endTransmission();
}

void TidalReset()
{
    tidal_volume_c.TidalStatus = 0;
    tidal_volume_c.TidalVolume = 0;
    tidal_volume_c.InspVolumeSensirion = 0;
    tidal_volume_c.InspVolumeVenturi = 0;
    tidal_volume_c.ExpVolumeVenturi = 0;
    tidal_volume_c.TidalCorrection = 1;
    tidal_volume_c.TidalStatus = 0;
}
void TidalInhale()
{
    tidal_volume_c.TidalStatus = 1;
}

void TidalExhale()
{
    if (tidal_volume_c.InspVolumeSensirion > 0)
        tidal_volume_c.TidalCorrection = tidal_volume_c.InspVolumeVenturi / tidal_volume_c.InspVolumeSensirion;
    else
        tidal_volume_c.TidalCorrection = 1;

    tidal_volume_c.TidalStatus = 2;
}

void ResetStats()
{
    __stat_param.start_time = millis();
    __stat_param.overshoot = -1000;
    __stat_param.final_error = -1000;
    __stat_param.t10a = -1000;
    __stat_param.t50a = -1000;
    __stat_param.t90a = -1000;
    __stat_param.t10b = -1000;
    __stat_param.t50b = -1000;
    __stat_param.t90b = -1000;
    __stat_param.overshoot_length = 0;
    __stat_param.phase = 0;
    __stat_param.time_to_peak = 0;

    __stat_param.flux_t10a = -1000;
    __stat_param.flux_t90a = -1000;
    __stat_param.flux_t10b = -1000;
    __stat_param.flux_t90b = -1000;
}

void ResetStatsBegin()
{
    __stat_param.overshoot_avg = 0;
    __stat_param.overshoot_length_avg = 0;
    __stat_param.final_error_avg = 0;
    __stat_param.t1050_avg = 0;
    __stat_param.t1090_avg = 0;
    __stat_param.tpeak_avg = 0;
    __stat_param.t9010_avg = 0;
    __stat_param.t9050_avg = 0;
    __stat_param.peep_avg = 0;
    __stat_param.t10_avg = 0;
    __stat_param.time_to_peak_avg = 0;
    __stat_param.peep_avg = 0;
    __stat_param.flux_peak_avg = 0;
    __stat_param.flux_t1090_avg = 0;
    __stat_param.flux_t9010_avg = 0;
    __stat_param.flux_peak = 0;
    __stat_param.mean_cnt = 0;
}

void StatPhaseExpire()
{
    __stat_param.phase = 1;
}

void StatEndCycle()
{
    __stat_param.phase = 2;
    if (__stat_param.t10a > 0)
        __stat_param.t10_avg += __stat_param.t10a;
    if ((__stat_param.t50a - __stat_param.t10a) > 0)
        __stat_param.t1050_avg += (__stat_param.t50a - __stat_param.t10a);
    if ((__stat_param.t90a - __stat_param.t10a) > 0)
        __stat_param.t1090_avg += (__stat_param.t90a - __stat_param.t10a);
    if ((__stat_param.t10b - __stat_param.t90b) > 0)
        __stat_param.t9010_avg += (__stat_param.t10b - __stat_param.t90b);
    if ((__stat_param.t50b - __stat_param.t90b) > 0)
        __stat_param.t1090_avg += (__stat_param.t50b - __stat_param.t90b);

    __stat_param.tpeak_avg += currentP_Peak;
    __stat_param.peep_avg += last_peep;
    __stat_param.overshoot_length_avg += __stat_param.overshoot_length;
    __stat_param.final_error_avg += __stat_param.final_error;
    __stat_param.time_to_peak_avg += __stat_param.time_to_peak;
    __stat_param.overshoot_avg += __stat_param.overshoot - core_config.target_pressure;

    __stat_param.flux_peak_avg += currentVM;
    if ((__stat_param.flux_t90a - __stat_param.flux_t10a) > 0)
        __stat_param.flux_t1090_avg += (__stat_param.flux_t90a - __stat_param.flux_t10a);
    if ((__stat_param.flux_t10b - __stat_param.flux_t90b) > 0)
        __stat_param.flux_t9010_avg += (__stat_param.flux_t10b - __stat_param.flux_t90b);

    __stat_param.flux_peak = currentVM;

    __stat_param.mean_cnt++;
}

void StatsUpdate()
{
    if (__stat_param.phase == 0) {
        if ((pressure[1].last_pressure >= 0.1 * core_config.target_pressure) && (__stat_param.t10a == -1000))
            __stat_param.t10a = millis() - __stat_param.start_time;

        if ((pressure[1].last_pressure >= 0.5 * core_config.target_pressure) && (__stat_param.t50a == -1000))
            __stat_param.t50a = millis() - __stat_param.start_time;

        if ((pressure[1].last_pressure >= 0.9 * core_config.target_pressure) && (__stat_param.t90a == -1000))
            __stat_param.t90a = millis() - __stat_param.start_time;

        if (__stat_param.overshoot < pressure[1].last_pressure) {
            __stat_param.overshoot = pressure[1].last_pressure;
            __stat_param.time_to_peak = millis() - __stat_param.start_time;
        }

        if (pressure[1].last_pressure >= (core_config.target_pressure + 1)) {
            __stat_param.overshoot_length += 20;
        }
        __stat_param.final_error = pressure[1].last_pressure - core_config.target_pressure;

        if ((gasflux[0].last_flux >= 0.1 * __stat_param.flux_peak) && (__stat_param.flux_t10a == -1000))
            __stat_param.flux_t10a = millis() - __stat_param.start_time;

        if ((gasflux[0].last_flux >= 0.9 * __stat_param.flux_peak) && (__stat_param.flux_t90a == -1000))
            __stat_param.flux_t90a = millis() - __stat_param.start_time;
    }

    if (__stat_param.phase == 1) {
        if ((pressure[1].last_pressure - last_peep <= 0.1 * core_config.target_pressure) && (__stat_param.t10b == -1000))
            __stat_param.t10b = millis() - __stat_param.start_time;

        if ((pressure[1].last_pressure - last_peep <= 0.5 * core_config.target_pressure) && (__stat_param.t50b == -1000))
            __stat_param.t50b = millis() - __stat_param.start_time;

        if ((pressure[1].last_pressure - last_peep <= 0.9 * core_config.target_pressure) && (__stat_param.t90b == -1000))
            __stat_param.t90b = millis() - __stat_param.start_time;

        if ((gasflux[0].last_flux <= 0.1 * __stat_param.flux_peak) && (__stat_param.flux_t10b == -1000))
            __stat_param.flux_t10b = millis() - __stat_param.start_time;

        if ((gasflux[0].last_flux <= 0.9 * __stat_param.flux_peak) && (__stat_param.flux_t90b == -1000))
            __stat_param.flux_t90b = millis() - __stat_param.start_time;
    }
}
