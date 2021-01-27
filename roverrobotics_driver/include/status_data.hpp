#pragma once
struct statusData {
    unsigned long long int time;
    int motor1_id;
    int motor2_id;
    int motor3_id;
    int motor4_id;
    float motor1_rpm;
    float motor2_rpm;
    float motor3_rpm;
    float motor4_rpm;
    float motor1_current;
    float motor2_current;
    float motor3_current;
    float motor4_current;
    float motor1_temp;
    float motor2_temp;
    float motor3_temp;
    float motor4_temp;
    float battery_voltage;
    float power;
    float charge_status;
    float mos1_temp;
    float mos2_temp;
    float mos3_temp;
    float mos4_temp;
};