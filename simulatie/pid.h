


#pragma once
#include "imgui.h"
#include <vector>

// ════════════════════════════════════════════
//  Zwaartepunt model
// ════════════════════════════════════════════
struct CoMModel
{
    float mass_frame = 0.0f;
    float h_frame = 0.9f;

    float mass_rider = 15.0f;
    float h_rider = 0.9f;

    float EffectiveL() const
    {
        return h_rider;
    }

    float TotalMass() const
    {
        return mass_rider + mass_frame;
    }
};


// ════════════════════════════════════════════
//  PID Controller (FIXED SIGNATURE)
// ════════════════════════════════════════════
struct PIDController
{
    float kp = 80.0f;
    float ki = 0.5f;
    float kd = 8.0f;

    float integral = 0.0f;
    float prev_error = 0.0f;

    float p_term = 0.0f;
    float i_term = 0.0f;
    float d_term = 0.0f;

    void Reset();

    // ❗ FIX: extra parameter theta_dot_deg
    float Update(float error_deg, float theta_dot_deg, float dt);
};


// ════════════════════════════════════════════
//  Pendulum (FIXED SIGNATURE)
// ════════════════════════════════════════════
struct Pendulum
{
    CoMModel com;

    float theta = 0.0f;
    float theta_dot = 0.0f;

    float com_x_offset = 0.0f;

    float wind_force = 0.0f;
    float wind_timer = 0.0f;

    bool  fallen = false;

    float max_force = 15.0f;
    float deadzone_deg = 0.5f;

    float motor1_raw = 0.0f;
    float motor2_raw = 0.0f;

    void Reset(float init_angle_deg);

    // ❗ FIX: max_force toegevoegd
    void Step(float pid_output,
        float arm_height,
        float motor_base,
        float motor_trim,
        float wind_max,
        float max_force,
        float dt);

    ImVec2 TipPosition(ImVec2 pivot, float scale) const;
    ImVec2 CoMPosition(ImVec2 pivot, float scale) const;
};


// ════════════════════════════════════════════
//  Scrolling buffer
// ════════════════════════════════════════════
struct ScrollingBuffer
{
    int max_size;
    int offset;
    std::vector<float> data;

    explicit ScrollingBuffer(int max = 3000);

    void AddPoint(float val);
    void Clear();
};


// ════════════════════════════════════════════
namespace MijnApp
{
    void RenderUI();
}


