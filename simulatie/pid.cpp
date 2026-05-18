#include "pid.h"
#include "imgui.h"


#include <cmath>
#include <cstdlib>
#include <algorithm>
#include <string>

// ════════════════════════════════════════════
//  Globale constanten
// ════════════════════════════════════════════
static constexpr float PI = 3.14159265f;
static constexpr float G = 9.81f;
static constexpr float DT = 0.005f;
static constexpr float MAX_ANGLE = PI / 2.0f;
static constexpr int   STEPS_PER_FRAME = 4;

static constexpr float BIKE_HEIGHT = 1.7f;
static constexpr float STANG_LENGTH = 0.10f;

static constexpr float DSHOT_MIN = 0.0f;
static constexpr float DSHOT_MAX_VAL = 2047.0f;

// ════════════════════════════════════════════
//  Hulpfuncties
// ════════════════════════════════════════════
static float RandF(float lo, float hi)
{
    return lo + (hi - lo) * ((float)rand() / (float)RAND_MAX);
}

static float Clamp(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}


// ════════════════════════════════════════════
void PIDController::Reset()
{
    integral = 0.0f;
    prev_error = 0.0f;
    p_term = 0.0f;
    i_term = 0.0f;
    d_term = 0.0f;
}

float PIDController::Update(float error_deg, float theta_dot_deg, float dt)
{
    // P-term
    p_term = kp * error_deg;

    // I-term met anti-windup
    integral += error_deg * dt;
    float integral_limit = (ki > 0.0001f) ? (500.0f / ki) : 1e6f;
    integral = Clamp(integral, -integral_limit, integral_limit);
    i_term = ki * integral;

    // D-term op hoeksnelheid (geen derivative kick bij setpoint-wijziging)
    d_term = kd * theta_dot_deg;

    float out = p_term + i_term + d_term;
    return Clamp(out, -1000.0f, 1000.0f);
}


// ════════════════════════════════════════════
//  Pendulum — implementatie
// ════════════════════════════════════════════
void Pendulum::Reset(float init_angle_deg)
{
    theta = init_angle_deg * PI / 180.0f;
    theta_dot = 0.0f;
    wind_force = 0.0f;
    wind_timer = 0.0f;
    fallen = false;
    motor1_raw = 0.0f;
    motor2_raw = 0.0f;
}

void Pendulum::Step(float pid_output, float arm_height,
    float motor_base, float motor_trim,
    float wind_max, float max_force, float dt)
{
    if (fallen) return;

    // ── Wind ─────────────────────────────────────────────────────────────
    wind_timer -= dt;
    if (wind_timer <= 0.0f)
    {
        wind_force = RandF(-wind_max, wind_max);
        wind_timer = RandF(0.5f, 2.0f);
    }

    // ── Netto kracht uit PID output ───────────────────────────────────────

    float F_net = (pid_output / 1000.0f) * max_force;

    // ── Visuele DSHOT waarden (puur voor display) ─────────────────────────

    if (pid_output >= 0.0f)
    {
        // Motor 2 dominant (fiets valt naar rechts)
        float m2_frac = Clamp(pid_output / 1000.0f, 0.0f, 1.0f);
        motor1_raw = motor_base;
        motor2_raw = motor_base + m2_frac * (DSHOT_MAX_VAL - motor_base) - motor_trim;
    }
    else
    {
        // Motor 1 dominant (fiets valt naar links)
        float m1_frac = Clamp(-pid_output / 1000.0f, 0.0f, 1.0f);
        motor1_raw = motor_base + m1_frac * (DSHOT_MAX_VAL - motor_base) + motor_trim;
        motor2_raw = motor_base;
    }
    motor1_raw = Clamp(motor1_raw, motor_base, DSHOT_MAX_VAL);
    motor2_raw = Clamp(motor2_raw, motor_base, DSHOT_MAX_VAL);

    // ── Massa en traagheidsmoment ─────────────────────────────────────────
    float m = com.TotalMass();
    float Ly = com.EffectiveL();
    float Lx = com_x_offset;
    float I = m * (Lx * Lx + Ly * Ly);

    // ── Koppels ───────────────────────────────────────────────────────────
    float tau_gravity = m * G * (Lx * cosf(theta) + Ly * sinf(theta));
    float tau_wind = wind_force * Ly;
    float tau_motor = -F_net * arm_height;

    // ── Euler-integratie ──────────────────────────────────────────────────
    float theta_ddot = (tau_gravity + tau_wind + tau_motor) / I;
    theta_dot += theta_ddot * dt;
    theta_dot *= 0.999f;
    theta += theta_dot * dt;

    if (fabsf(theta) > MAX_ANGLE)
        fallen = true;
}

ImVec2 Pendulum::TipPosition(ImVec2 pivot, float scale) const
{
    return ImVec2(
        pivot.x + sinf(theta) * BIKE_HEIGHT * scale,
        pivot.y - cosf(theta) * BIKE_HEIGHT * scale
    );
}

ImVec2 Pendulum::CoMPosition(ImVec2 pivot, float scale) const
{
    float Ly = com.EffectiveL();
    float Lx = com_x_offset;
    float x_world = Lx * cosf(theta) + Ly * sinf(theta);
    float y_world = -Lx * sinf(theta) + Ly * cosf(theta);
    return ImVec2(
        pivot.x + x_world * scale,
        pivot.y - y_world * scale
    );
}


// ════════════════════════════════════════════
//  ScrollingBuffer — implementatie
// ════════════════════════════════════════════
ScrollingBuffer::ScrollingBuffer(int max)
    : max_size(max), offset(0)
{
    data.reserve(max);
}

void ScrollingBuffer::AddPoint(float val)
{
    if ((int)data.size() < max_size)
        data.push_back(val);
    else
    {
        data[offset] = val;
        offset = (offset + 1) % max_size;
    }
}

void ScrollingBuffer::Clear()
{
    data.clear();
    offset = 0;
}


// ════════════════════════════════════════════
//  Simulatiestaat
// ════════════════════════════════════════════
namespace
{
    Pendulum      g_pendulum;
    PIDController g_pid;

    bool  g_running = false;
    float g_sim_time = 0.0f;
    float g_wind_max = 1.0f;
    float g_init_angle = 5.0f;
    float g_target_angle = 0.0f;

    float g_motor_base = 100.0f;
    float g_motor_trim = 0.0f;
    float g_mount_height = 0.94f;
    float g_max_force = 80.0f;

    float g_last_pid_output = 0.0f;

    ScrollingBuffer g_log_t{ 3000 };
    ScrollingBuffer g_log_theta{ 3000 };
    ScrollingBuffer g_log_motor1{ 3000 };
    ScrollingBuffer g_log_motor2{ 3000 };
    ScrollingBuffer g_log_pterm{ 3000 };
    ScrollingBuffer g_log_iterm{ 3000 };
    ScrollingBuffer g_log_dterm{ 3000 };
    ScrollingBuffer g_log_wind{ 3000 };

    bool g_show_theta = true;
    bool g_show_motor1 = true;
    bool g_show_motor2 = true;
    bool g_show_pterm = false;
    bool g_show_iterm = false;
    bool g_show_dterm = false;

    void SimReset()
    {
        g_pendulum.Reset(g_init_angle);
        g_pid.Reset();
        g_sim_time = 0.0f;
        g_last_pid_output = 0.0f;
        g_running = false;
        g_log_t.Clear();
        g_log_theta.Clear();
        g_log_motor1.Clear();
        g_log_motor2.Clear();
        g_log_pterm.Clear();
        g_log_iterm.Clear();
        g_log_dterm.Clear();
        g_log_wind.Clear();
    }

    void SimStep()
    {
        if (g_pendulum.fallen) { g_running = false; return; }

        float theta_deg = g_pendulum.theta * 180.0f / PI;
        float theta_dot_deg = g_pendulum.theta_dot * 180.0f / PI;
        float error_deg = theta_deg - g_target_angle;

        float output = g_pid.Update(error_deg, theta_dot_deg, DT);
        g_last_pid_output = output;

        float arm_h = g_mount_height * BIKE_HEIGHT;

        g_pendulum.Step(output, arm_h,
            g_motor_base, g_motor_trim,
            g_wind_max, g_max_force, DT);
        g_sim_time += DT;

        g_log_t.AddPoint(g_sim_time);
        g_log_theta.AddPoint(g_pendulum.theta * 180.0f / PI);
        g_log_motor1.AddPoint(g_pendulum.motor1_raw);
        g_log_motor2.AddPoint(g_pendulum.motor2_raw);
        g_log_pterm.AddPoint(g_pid.p_term);
        g_log_iterm.AddPoint(g_pid.i_term);
        g_log_dterm.AddPoint(g_pid.d_term);
        g_log_wind.AddPoint(g_pendulum.wind_force);
    }

    // ════════════════════════════════════════════
    //  Dynamische y-as range berekening
    // ════════════════════════════════════════════
    static void ComputeDynamicRange(const ScrollingBuffer& buf,
        float min_half_range,
        float padding_factor,
        float& out_min, float& out_max)
    {
        float peak = min_half_range;
        for (float v : buf.data)
        {
            float a = fabsf(v);
            if (a > peak) peak = a;
        }
        float half = peak * padding_factor;
        if (half < min_half_range) half = min_half_range;
        out_min = -half;
        out_max = half;
    }

    // ════════════════════════════════════════════
    //  Tekenroutine — fiets canvas
    // ════════════════════════════════════════════
    void DrawPendulum(ImDrawList* dl, ImVec2 pivot, float scale)
    {
        auto& p = g_pendulum;
        const float L_tip = BIKE_HEIGHT;

        ImVec2 tip = p.TipPosition(pivot, scale);
        ImVec2 com = p.CoMPosition(pivot, scale);

        // ── Grondlijn ─────────────────────────────────────────────────────
        dl->AddLine(
            ImVec2(pivot.x - 140, pivot.y),
            ImVec2(pivot.x + 140, pivot.y),
            IM_COL32(70, 70, 70, 255), 1.5f
        );

        // ── Referentielijn verticaal ───────────────────────────────────────
        dl->AddLine(
            pivot,
            ImVec2(pivot.x, pivot.y - L_tip * scale * 1.1f),
            IM_COL32(40, 40, 60, 200), 1.0f
        );

        // ── Fiets pool ────────────────────────────────────────────────────
        dl->AddLine(pivot, tip, IM_COL32(210, 210, 210, 255), 5.0f);

        // ── Zwaartepunt ───────────────────────────────────────────────────
        dl->AddCircleFilled(com, 10.0f, IM_COL32(255, 200, 0, 220));
        dl->AddCircle(com, 10.0f, IM_COL32(255, 230, 100, 180), 0, 2.0f);
        dl->AddText(ImVec2(com.x + 13, com.y - 7), IM_COL32(255, 200, 0, 200), "CoM");

        // ── Top van fiets ─────────────────────────────────────────────────
        dl->AddCircleFilled(tip, 10.0f, IM_COL32(220, 60, 60, 255));
        dl->AddCircle(tip, 10.0f, IM_COL32(255, 120, 120, 160), 0, 1.5f);

        // ── Pivot ─────────────────────────────────────────────────────────
        dl->AddCircleFilled(pivot, 6.0f, IM_COL32(255, 255, 255, 220));

        // ── Drone stang ───────────────────────────────────────────────────
        float stang_half_px = (STANG_LENGTH * 0.5f) * scale;

        ImVec2 stang_mid(
            pivot.x + sinf(p.theta) * L_tip * scale * g_mount_height,
            pivot.y - cosf(p.theta) * L_tip * scale * g_mount_height
        );
        float perp_x = cosf(p.theta);
        float perp_y = sinf(p.theta);

        ImVec2 lp(stang_mid.x - perp_x * stang_half_px, stang_mid.y - perp_y * stang_half_px);
        ImVec2 rp(stang_mid.x + perp_x * stang_half_px, stang_mid.y + perp_y * stang_half_px);
        dl->AddLine(lp, rp, IM_COL32(150, 150, 150, 200), 2.5f);

        // ── Stuwkracht pijlen ─────────────────────────────────────────────
        // Motor 1 (links, lp): stuwkracht naar links (-perp richting)
        // Motor 2 (rechts, rp): stuwkracht naar rechts (+perp richting)
        auto DrawArrow = [&](ImVec2 origin, float dshot_val, float motor_base,
            ImU32 col_arrow, float dir_sign)
        {
            float frac = Clamp((dshot_val - motor_base) / (DSHOT_MAX_VAL - motor_base + 1.0f), 0.0f, 1.0f);
            if (frac < 0.01f) return;
            float arrow_len = frac * 70.0f;

            ImVec2 arrow_tip(
                origin.x + perp_x * arrow_len * dir_sign,
                origin.y + perp_y * arrow_len * dir_sign
            );
            dl->AddLine(origin, arrow_tip, col_arrow, 2.5f);

            float bx = (origin.x - arrow_tip.x);
            float by = (origin.y - arrow_tip.y);
            float len = sqrtf(bx * bx + by * by);
            if (len < 0.01f) return;
            bx /= len; by /= len;
            float sx = -by, sy = bx;
            float head = 10.0f;
            dl->AddLine(arrow_tip,
                ImVec2(arrow_tip.x + bx * head + sx * head * 0.5f,
                    arrow_tip.y + by * head + sy * head * 0.5f), col_arrow, 2.5f);
            dl->AddLine(arrow_tip,
                ImVec2(arrow_tip.x + bx * head - sx * head * 0.5f,
                    arrow_tip.y + by * head - sy * head * 0.5f), col_arrow, 2.5f);

            char nbuf[24];
            snprintf(nbuf, sizeof(nbuf), "%.0f", dshot_val);
            dl->AddText(ImVec2(arrow_tip.x + 4, arrow_tip.y - 7), col_arrow, nbuf);
        };

        // Motor 1 links, stuwkracht naar links (-perp)
        DrawArrow(lp, p.motor1_raw, g_motor_base, IM_COL32(55, 138, 221, 230), -1.0f);
        // Motor 2 rechts, stuwkracht naar rechts (+perp)
        DrawArrow(rp, p.motor2_raw, g_motor_base, IM_COL32(226, 75, 74, 230), +1.0f);

        // ── Motor kleuren ─────────────────────────────────────────────────
        bool m1_on = p.motor1_raw > g_motor_base + 10.0f;
        bool m2_on = p.motor2_raw > g_motor_base + 10.0f;
        ImU32 col_l = m1_on ? IM_COL32(55, 138, 221, 255) : IM_COL32(70, 70, 70, 180);
        ImU32 col_r = m2_on ? IM_COL32(226, 75, 74, 255) : IM_COL32(70, 70, 70, 180);
        dl->AddCircleFilled(lp, 10.0f, col_l);
        dl->AddCircleFilled(rp, 10.0f, col_r);
        dl->AddText(ImVec2(lp.x - 3, lp.y + 13), IM_COL32(200, 200, 200, 200), "M1");
        dl->AddText(ImVec2(rp.x - 4, rp.y + 13), IM_COL32(200, 200, 200, 200), "M2");

        // Stuwkrachteffect actieve motor (glow ringen)
        if (m1_on)
        {
            float intensity = Clamp((p.motor1_raw - g_motor_base) / (DSHOT_MAX_VAL - g_motor_base + 1.0f), 0.0f, 1.0f);
            for (int i = 1; i <= 3; ++i)
            {
                ImVec2 c(lp.x - perp_x * i * 13.0f * intensity,
                    lp.y - perp_y * i * 13.0f * intensity);
                dl->AddCircle(c, i * 5.5f * intensity,
                    IM_COL32(55, 138, 221, (int)(70 / i)), 0, 1.5f);
            }
        }
        if (m2_on)
        {
            float intensity = Clamp((p.motor2_raw - g_motor_base) / (DSHOT_MAX_VAL - g_motor_base + 1.0f), 0.0f, 1.0f);
            for (int i = 1; i <= 3; ++i)
            {
                ImVec2 c(rp.x + perp_x * i * 13.0f * intensity,
                    rp.y + perp_y * i * 13.0f * intensity);
                dl->AddCircle(c, i * 5.5f * intensity,
                    IM_COL32(226, 75, 74, (int)(70 / i)), 0, 1.5f);
            }
        }

        // ── Teksten ───────────────────────────────────────────────────────
        char buf[96];
        float deg = p.theta * 180.0f / PI;
        snprintf(buf, sizeof(buf), "theta = %.1f deg  (target %.1f)", deg, g_target_angle);
        dl->AddText(
            ImVec2(pivot.x - 120, pivot.y - L_tip * scale - 28),
            IM_COL32(240, 240, 240, 200), buf
        );

        snprintf(buf, sizeof(buf), "CoM = (%.2f, %.2f) m  |  massa = %.0f kg",
            p.com_x_offset, p.com.EffectiveL(), p.com.TotalMass());
        dl->AddText(
            ImVec2(pivot.x - 120, pivot.y - L_tip * scale - 14),
            IM_COL32(255, 200, 0, 180), buf
        );

        snprintf(buf, sizeof(buf), "M1: %.0f  M2: %.0f  PID: %.1f  F: %.1fN",
            p.motor1_raw, p.motor2_raw, g_last_pid_output,
            (g_last_pid_output / 1000.0f) * g_max_force);
        dl->AddText(
            ImVec2(pivot.x - 120, pivot.y - L_tip * scale - 42),
            IM_COL32(150, 200, 255, 200), buf
        );

        if (p.fallen)
        {
            dl->AddText(
                ImVec2(pivot.x - 45, pivot.y - 40),
                IM_COL32(226, 75, 74, 255), "GEVALLEN!"
            );
        }

        // ── Wind pijlen ───────────────────────────────────────────────────
        if (fabsf(p.wind_force) > 0.05f)
        {
            float dir = p.wind_force > 0.0f ? 1.0f : -1.0f;
            float str = fabsf(p.wind_force) / (g_wind_max + 0.01f);
            ImU32 wc = IM_COL32(29, 158, 117, (int)(200 * str));
            float wx = pivot.x;
            float wy = pivot.y - p.com.EffectiveL() * scale * 0.85f;
            for (int row = -1; row <= 1; ++row)
            {
                float yo = wy + row * 14.0f;
                dl->AddLine(ImVec2(wx - dir * 30, yo), ImVec2(wx + dir * 30, yo), wc, 2.0f);
                dl->AddLine(ImVec2(wx + dir * 30, yo), ImVec2(wx + dir * 20, yo - 5), wc, 2.0f);
                dl->AddLine(ImVec2(wx + dir * 30, yo), ImVec2(wx + dir * 20, yo + 5), wc, 2.0f);
            }
        }
    }

    // ════════════════════════════════════════════
    //  Grafiek — fixed y-as
    // ════════════════════════════════════════════
    void DrawGraph(const char* title,
        ScrollingBuffer& ybuf,
        ImU32  line_color,
        float  fixed_y_min, float fixed_y_max,
        float  width, float height)
    {
        ImDrawList* dl = ImGui::GetWindowDrawList();
        ImVec2 pos = ImGui::GetCursorScreenPos();
        float  margin_l = 42.0f;

        ImVec2 graph_pos = ImVec2(pos.x + margin_l, pos.y);
        ImVec2 graph_size = ImVec2(width - margin_l, height);

        dl->AddRectFilled(graph_pos,
            ImVec2(graph_pos.x + graph_size.x, graph_pos.y + graph_size.y),
            IM_COL32(15, 15, 20, 255));
        dl->AddRect(graph_pos,
            ImVec2(graph_pos.x + graph_size.x, graph_pos.y + graph_size.y),
            IM_COL32(60, 60, 70, 200));

        float y_range = fixed_y_max - fixed_y_min;
        for (int t = 0; t <= 4; ++t)
        {
            float v = fixed_y_min + t * y_range / 4.0f;
            float py = graph_pos.y + graph_size.y * (1.0f - (v - fixed_y_min) / y_range);
            py = Clamp(py, graph_pos.y, graph_pos.y + graph_size.y);
            dl->AddLine(ImVec2(graph_pos.x, py),
                ImVec2(graph_pos.x + graph_size.x, py),
                IM_COL32(35, 35, 45, 200), 0.5f);
            char lbl[16]; snprintf(lbl, sizeof(lbl), "%.0f", v);
            dl->AddText(ImVec2(pos.x, py - 6), IM_COL32(160, 160, 170, 200), lbl);
        }

        if (fixed_y_min < 0.0f && fixed_y_max > 0.0f)
        {
            float py = graph_pos.y + graph_size.y * (1.0f - (0.0f - fixed_y_min) / y_range);
            dl->AddLine(ImVec2(graph_pos.x, py),
                ImVec2(graph_pos.x + graph_size.x, py),
                IM_COL32(80, 80, 90, 180), 1.0f);
        }

        if (g_log_t.data.size() >= 2 && ybuf.data.size() >= 2)
        {
            float t_max = g_sim_time;
            float t_min = std::max(0.0f, t_max - 10.0f);
            size_t n = std::min(g_log_t.data.size(), ybuf.data.size());

            for (size_t i = 1; i < n; ++i)
            {
                float t0 = g_log_t.data[i - 1];
                float t1 = g_log_t.data[i];
                if (t1 < t_min) continue;

                auto tx = [&](float t) {
                    return graph_pos.x + graph_size.x * (t - t_min) / (t_max - t_min + 0.001f);
                };
                auto ty = [&](float v) {
                    float frac = (v - fixed_y_min) / (y_range + 0.001f);
                    return graph_pos.y + graph_size.y * (1.0f - frac);
                };

                float x0 = Clamp(tx(t0), graph_pos.x, graph_pos.x + graph_size.x);
                float x1 = Clamp(tx(t1), graph_pos.x, graph_pos.x + graph_size.x);
                float y0 = Clamp(ty(ybuf.data[i - 1]), graph_pos.y, graph_pos.y + graph_size.y);
                float y1 = Clamp(ty(ybuf.data[i]), graph_pos.y, graph_pos.y + graph_size.y);

                dl->AddLine(ImVec2(x0, y0), ImVec2(x1, y1), line_color, 1.8f);
            }
        }

        dl->AddText(ImVec2(graph_pos.x + 4, graph_pos.y + 4),
            IM_COL32(200, 200, 210, 200), title);

        ImGui::Dummy(ImVec2(width, height + 4));
    }

} // einde anonieme namespace


// ════════════════════════════════════════════
//  MijnApp::RenderUI
// ════════════════════════════════════════════
namespace MijnApp
{

    void RenderUI()
    {
        if (g_running)
        {
            for (int i = 0; i < STEPS_PER_FRAME; ++i)
                SimStep();
        }

        ImGui::SetNextWindowPos(ImGui::GetMainViewport()->WorkPos);
        ImGui::SetNextWindowSize(ImGui::GetMainViewport()->WorkSize);
        ImGui::SetNextWindowBgAlpha(1.0f);

        ImGuiWindowFlags main_flags =
            ImGuiWindowFlags_NoTitleBar |
            ImGuiWindowFlags_NoCollapse |
            ImGuiWindowFlags_NoResize |
            ImGuiWindowFlags_NoMove |
            ImGuiWindowFlags_NoBringToFrontOnFocus |
            ImGuiWindowFlags_NoNavFocus |
            ImGuiWindowFlags_NoScrollbar |
            ImGuiWindowFlags_NoScrollWithMouse;

        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
        ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
        ImGui::Begin("##FietsSim", nullptr, main_flags);
        ImGui::PopStyleVar(2);

        float total_w = ImGui::GetContentRegionAvail().x;
        float total_h = ImGui::GetContentRegionAvail().y;
        float col_canvas = total_w * 0.33f;
        float col_graphs = total_w * 0.40f;
        float col_params = total_w * 0.27f;

        // ════════════════════════════════════════════
        //  KOLOM 1 — Canvas
        // ════════════════════════════════════════════
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(8, 8));
        ImGui::BeginChild("##Canvas", ImVec2(col_canvas, total_h), false,
            ImGuiWindowFlags_NoScrollbar);
        ImGui::PopStyleVar();
        {
            ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "FIETS SIMULATIE");
            ImGui::Separator();

            ImVec2 canvas_pos = ImGui::GetCursorScreenPos();
            ImVec2 canvas_avail = ImGui::GetContentRegionAvail();
            float  canvas_h = canvas_avail.y - 74.0f;
            if (canvas_h < 100) canvas_h = 100;

            ImDrawList* dl = ImGui::GetWindowDrawList();
            dl->AddRectFilled(canvas_pos,
                ImVec2(canvas_pos.x + col_canvas - 16, canvas_pos.y + canvas_h),
                IM_COL32(12, 12, 18, 255));

            ImVec2 pivot(canvas_pos.x + (col_canvas - 16) * 0.5f,
                canvas_pos.y + canvas_h * 0.78f);
            float  scale = canvas_h * 0.46f;

            DrawPendulum(dl, pivot, scale);
            ImGui::InvisibleButton("##canvas_area", ImVec2(col_canvas - 16, canvas_h));

            // ── Motor krachtbalken ────────────────────────────────────────
            // Beide namen netjes boven hun eigen balk, dan balken naast elkaar
            float bar_w = (col_canvas - 32.0f) * 0.46f;
            float dshot_range = DSHOT_MAX_VAL - g_motor_base + 1.0f;
            float m1_frac = Clamp((g_pendulum.motor1_raw - g_motor_base) / dshot_range, 0.0f, 1.0f);
            float m2_frac = Clamp((g_pendulum.motor2_raw - g_motor_base) / dshot_range, 0.0f, 1.0f);

            float col1_x = 8.0f;
            float col2_x = col_canvas * 0.50f;

            // Namen
            ImGui::SetCursorPosX(col1_x);
            ImGui::TextColored(ImVec4(0.55f, 0.78f, 1.0f, 1.0f), "Motor 1 (links, \xe2\x86\x90)");
            ImGui::SameLine(col2_x);
            ImGui::TextColored(ImVec4(1.0f, 0.45f, 0.45f, 1.0f), "Motor 2 (rechts, \xe2\x86\x92)");

            // Balken
            ImGui::SetCursorPosX(col1_x);
            ImGui::PushStyleColor(ImGuiCol_PlotHistogram, ImVec4(0.21f, 0.54f, 0.87f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4(0.10f, 0.10f, 0.10f, 1.0f));
            ImGui::ProgressBar(m1_frac, ImVec2(bar_w, 14), "");
            ImGui::PopStyleColor(2);

            ImGui::SameLine(col2_x);
            ImGui::PushStyleColor(ImGuiCol_PlotHistogram, ImVec4(0.89f, 0.29f, 0.29f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4(0.10f, 0.10f, 0.10f, 1.0f));
            ImGui::ProgressBar(m2_frac, ImVec2(bar_w, 14), "");
            ImGui::PopStyleColor(2);

            // DSHOT waarden
            char dshotbuf[72];
            snprintf(dshotbuf, sizeof(dshotbuf), "M1: %.0f  M2: %.0f  (idle = %.0f)",
                g_pendulum.motor1_raw, g_pendulum.motor2_raw, g_motor_base);
            ImGui::SetCursorPosX(col1_x);
            ImGui::TextColored(ImVec4(0.6f, 0.6f, 0.6f, 1.0f), "%s", dshotbuf);

            // Netto kracht
            float F_display = (g_last_pid_output / 1000.0f) * g_max_force;
            ImGui::SetCursorPosX(col1_x);
            ImGui::TextColored(ImVec4(0.8f, 0.9f, 0.6f, 1.0f),
                "Netto kracht: %.1f N  (%s)",
                fabsf(F_display),
                F_display > 0.1f ? "M2 actief, top \xe2\x86\x90" :
                F_display < -0.1f ? "M1 actief, top \xe2\x86\x92" : "neutraal");
        }
        ImGui::EndChild();

        ImGui::SameLine(0, 0);

        // ════════════════════════════════════════════
        //  KOLOM 2 — Grafieken + Live waarden
        // ════════════════════════════════════════════
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(8, 8));
        ImGui::BeginChild("##Graphs", ImVec2(col_graphs, total_h), false,
            ImGuiWindowFlags_NoScrollbar);
        ImGui::PopStyleVar();
        {
            ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "GRAFIEKEN  (laatste 10s)");
            ImGui::Separator();
            ImGui::Spacing();

            float gw = col_graphs - 20.0f;
            float gh_main = 100.0f;
            float gh_small = 72.0f;

            // ── Dynamische y-as voor theta ────────────────────────────────
            float theta_ymin, theta_ymax;
            ComputeDynamicRange(g_log_theta, 3.0f, 1.5f, theta_ymin, theta_ymax);

            if (g_show_theta)
                DrawGraph("Hoek theta (deg)", g_log_theta,
                    IM_COL32(220, 80, 80, 255),
                    theta_ymin, theta_ymax, gw, gh_main);

            // Motor 1 & 2: VASTE y-as [0, 2047] voor onderlinge vergelijking
            if (g_show_motor1)
                DrawGraph("Motor 1 DSHOT (links, \xe2\x86\x90)", g_log_motor1,
                    IM_COL32(55, 138, 221, 255),
                    0.0f, DSHOT_MAX_VAL, gw, gh_small);

            if (g_show_motor2)
                DrawGraph("Motor 2 DSHOT (rechts, \xe2\x86\x92)", g_log_motor2,
                    IM_COL32(226, 75, 74, 255),
                    0.0f, DSHOT_MAX_VAL, gw, gh_small);

            // ── Dynamische y-as voor PID termen ──────────────────────────
            float p_ymin, p_ymax, i_ymin, i_ymax, d_ymin, d_ymax;
            ComputeDynamicRange(g_log_pterm, 50.0f, 1.5f, p_ymin, p_ymax);
            ComputeDynamicRange(g_log_iterm, 3.0f, 1.5f, i_ymin, i_ymax);
            ComputeDynamicRange(g_log_dterm, 50.0f, 1.5f, d_ymin, d_ymax);

            if (g_show_pterm)
                DrawGraph("P-term (kp * theta_err_deg)", g_log_pterm,
                    IM_COL32(80, 200, 140, 255),
                    p_ymin, p_ymax, gw, gh_small);

            if (g_show_iterm)
                DrawGraph("I-term", g_log_iterm,
                    IM_COL32(230, 180, 50, 255),
                    i_ymin, i_ymax, gw, gh_small);

            if (g_show_dterm)
                DrawGraph("D-term (kd * theta_dot_deg)", g_log_dterm,
                    IM_COL32(140, 100, 240, 255),
                    d_ymin, d_ymax, gw, gh_small);

            // ── Toggle knoppen ────────────────────────────────────────────
            ImGui::Separator();
            ImGui::Spacing();
            ImGui::Text("Toon:");
            ImGui::SameLine();

            auto ToggleBtn = [](const char* lbl, bool& flag, ImVec4 col) {
                bool off = !flag;
                if (off) ImGui::PushStyleVar(ImGuiStyleVar_Alpha, 0.35f);
                ImGui::PushStyleColor(ImGuiCol_Button,
                    ImVec4(col.x * 0.3f, col.y * 0.3f, col.z * 0.3f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered,
                    ImVec4(col.x * 0.5f, col.y * 0.5f, col.z * 0.5f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_Text, col);
                if (ImGui::SmallButton(lbl)) flag = !flag;
                ImGui::PopStyleColor(3);
                if (off) ImGui::PopStyleVar();
            };

            ToggleBtn("Hoek", g_show_theta, ImVec4(0.86f, 0.31f, 0.31f, 1.0f)); ImGui::SameLine();
            ToggleBtn("M1", g_show_motor1, ImVec4(0.21f, 0.54f, 0.87f, 1.0f)); ImGui::SameLine();
            ToggleBtn("M2", g_show_motor2, ImVec4(0.89f, 0.29f, 0.29f, 1.0f)); ImGui::SameLine();
            ToggleBtn("P-term", g_show_pterm, ImVec4(0.31f, 0.78f, 0.55f, 1.0f)); ImGui::SameLine();
            ToggleBtn("I-term", g_show_iterm, ImVec4(0.90f, 0.71f, 0.20f, 1.0f)); ImGui::SameLine();
            ToggleBtn("D-term", g_show_dterm, ImVec4(0.55f, 0.39f, 0.94f, 1.0f));

            // ── Live waarden onder grafieken ──────────────────────────────
            ImGui::Separator();
            ImGui::Spacing();
            ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "LIVE WAARDEN");
            ImGui::Separator();
            ImGui::Spacing();

            float deg = g_pendulum.theta * 180.0f / PI;
            float dot_deg = g_pendulum.theta_dot * 180.0f / PI;
            float err_display = deg - g_target_angle;

            ImVec4 angle_col = fabsf(deg) < 3.0f ? ImVec4(0.3f, 1.0f, 0.3f, 1.0f)
                : fabsf(deg) < 15.0f ? ImVec4(1.0f, 0.7f, 0.0f, 1.0f)
                : ImVec4(1.0f, 0.2f, 0.2f, 1.0f);

            // Twee kolommen voor compactheid
            float lv_col2 = gw * 0.50f;

            ImGui::TextColored(angle_col,
                "theta    = %+.2f deg", deg);
            ImGui::SameLine(lv_col2);
            ImGui::Text("theta'   = %+.2f deg/s", dot_deg);

            ImGui::Text("error    = %+.2f deg", err_display);
            ImGui::SameLine(lv_col2);
            ImGui::Text("PID out  = %+.2f", g_last_pid_output);

            float F_net_lv = (g_last_pid_output / 1000.0f) * g_max_force;
            ImGui::Text("F netto  = %+.1f N", F_net_lv);
            ImGui::SameLine(lv_col2);
            ImGui::Text("t        = %.2f s", g_sim_time);

            ImGui::Spacing();
            ImGui::TextColored(ImVec4(0.31f, 0.78f, 0.55f, 1.0f),
                "P = %+.2f", g_pid.p_term);
            ImGui::SameLine(lv_col2);
            ImGui::TextColored(ImVec4(0.90f, 0.71f, 0.20f, 1.0f),
                "I = %+.2f", g_pid.i_term);
            ImGui::TextColored(ImVec4(0.55f, 0.39f, 0.94f, 1.0f),
                "D = %+.2f  (kd*%.1f deg/s)", g_pid.d_term, dot_deg);

            ImGui::Spacing();
            ImGui::Text("M1: %.0f DSHOT   M2: %.0f DSHOT",
                g_pendulum.motor1_raw, g_pendulum.motor2_raw);
            ImGui::Text("Wind: %.2f N", g_pendulum.wind_force);

            // Fysische kengetallen
            {
                float m = g_pendulum.com.TotalMass();
                float L = g_pendulum.com.EffectiveL();
                float omega_n = sqrtf(G / L);
                float H = g_mount_height * BIKE_HEIGHT;
                float tau_ratio = (g_max_force * H) / (m * G * L);
                ImGui::Spacing();
                ImGui::TextDisabled("omega_n = %.2f rad/s (tau=%.2fs)  |  F_max/F_grav = %.1fx",
                    omega_n, 1.0f / omega_n, tau_ratio);
            }

            if (g_pendulum.fallen)
            {
                ImGui::Spacing();
                ImGui::TextColored(ImVec4(0.89f, 0.29f, 0.29f, 1.0f), "FIETS GEVALLEN!");
            }
        }
        ImGui::EndChild();

        ImGui::SameLine(0, 0);

        // ════════════════════════════════════════════
        //  KOLOM 3 — Parameters & Bediening
        // ════════════════════════════════════════════
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(10, 8));
        ImGui::BeginChild("##Params", ImVec2(col_params, total_h), false,
            ImGuiWindowFlags_NoScrollbar);
        ImGui::PopStyleVar();
        {
            ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "PARAMETERS");
            ImGui::Separator();

            // ── PID ───────────────────────────────────────────────────────
            ImGui::Spacing();
            ImGui::TextColored(ImVec4(0.6f, 0.8f, 1.0f, 1.0f), "PID Regelaar");
            ImGui::TextDisabled("error = theta_deg - target  (graden)");
            ImGui::TextDisabled("D op hoeksnelheid (geen derivative kick)");

            ImGui::SliderFloat("Kp##pid", &g_pid.kp, 0.0f, 500.0f, "%.1f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(
                    "Proportioneel: directe reactie op hoekfout in GRADEN.\n"
                    "Bij theta=5deg: P-term = kp * 5\n"
                    "Kp=100 -> P-term=500 -> 50%% van max motorvermogen.\n"
                    "Aanbevolen: 100-300.");

            ImGui::SliderFloat("Ki##pid", &g_pid.ki, 0.0f, 20.0f, "%.3f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(
                    "Integraal: corrigeert blijvende drift (bijv. CoM offset).\n"
                    "Anti-windup actief: integral gecapped op 500/Ki.\n"
                    "Aanbevolen: 1-5.");

            ImGui::SliderFloat("Kd##pid", &g_pid.kd, 0.0f, 100.0f, "%.1f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(
                    "Differentiaal: dempend op hoeksnelheid (deg/s).\n"
                    "D-term = kd * theta_dot_deg\n"
                    "Verhindert overswing. Aanbevolen: 20-50.");

            ImGui::SliderFloat("Target (deg)", &g_target_angle, -10.0f, 10.0f, "%.1f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(
                    "Doelhoek. Als CoM offset X != 0 heeft de fiets\n"
                    "een natuurlijk evenwicht bij een kleine hoek.\n"
                    "Zet target op die hoek voor stabiel gedrag.");

            ImGui::Spacing();
            ImGui::Text("Presets:");
            ImGui::SameLine();
            if (ImGui::SmallButton("Zacht"))
            {
                g_pid.kp = 80.0f;  g_pid.ki = 1.0f;  g_pid.kd = 20.0f;
            }
            ImGui::SameLine();
            if (ImGui::SmallButton("Normaal"))
            {
                g_pid.kp = 150.0f; g_pid.ki = 3.0f;  g_pid.kd = 30.0f;
            }
            ImGui::SameLine();
            if (ImGui::SmallButton("Agressief"))
            {
                g_pid.kp = 300.0f; g_pid.ki = 8.0f;  g_pid.kd = 60.0f;
            }

            ImGui::Separator();

            // ── Zwaartepunt ───────────────────────────────────────────────
            ImGui::Spacing();
            ImGui::TextColored(ImVec4(1.0f, 0.85f, 0.2f, 1.0f), "Zwaartepunt (CoM)");

            ImGui::SliderFloat("CoM hoogte Y (m)", &g_pendulum.com.h_rider,
                0.3f, BIKE_HEIGHT, "%.2f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(
                    "Verticale hoogte CoM boven grond.\n"
                    "Hogere CoM = onstabielere slinger.\n"
                    "tau_grav = m*g*L*sin(theta)");

            ImGui::SliderFloat("CoM offset X (m)", &g_pendulum.com_x_offset,
                -0.3f, 0.3f, "%.3f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(
                    "Horizontale offset van CoM t.o.v. spil.\n"
                    "Geeft een statische evenwichtshoek:\n"
                    "   theta_eq = -atan(Lx/Ly)\n"
                    "Zet Target op deze waarde voor geen I-windup.");

            g_pendulum.com.mass_frame = 0.0f;
            g_pendulum.com.h_frame = g_pendulum.com.h_rider;

            ImGui::SliderFloat("Massa (kg)", &g_pendulum.com.mass_rider, 2.0f, 80.0f, "%.0f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(
                    "Totale massa fiets + lading.\n"
                    "Hogere massa -> groter traagheidsmoment I = m*L2\n"
                    "-> motor heeft MINDER autoriteit per kg\n"
                    "-> zwaardere fiets is MOEILIJKER te balanceren.\n"
                    "Vrije valsnelheid is massa-onafhankelijk (Galileo).");

            {
                float Lx = g_pendulum.com_x_offset;
                float Ly = g_pendulum.com.EffectiveL();
                float theta_eq = -atanf(Lx / (Ly + 0.001f)) * 180.0f / PI;
                ImGui::TextColored(ImVec4(1.0f, 0.85f, 0.2f, 1.0f),
                    "Y=%.2fm  X=%.3fm  M=%.0fkg  eq=%.2fdeg",
                    Ly, Lx, g_pendulum.com.TotalMass(), theta_eq);
                ImGui::TextDisabled("  Zet Target op %.2f voor drift-vrij", theta_eq);
            }

            ImGui::Separator();

            // ── Motor instellingen ────────────────────────────────────────
            ImGui::Spacing();
            ImGui::TextColored(ImVec4(0.7f, 1.0f, 0.7f, 1.0f), "Drone Motor");
            ImGui::TextDisabled("Stang: %.0f cm | M1 links | M2 rechts",
                STANG_LENGTH * 100.0f);

            ImGui::SliderFloat("Max kracht (N)", &g_max_force, 1.0f, 200.0f, "%.1f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(
                    "Maximale stuwkracht bij PID output = 1000.\n"
                    "F_net = (pid_output / 1000) * max_kracht\n"
                    "Benodigde kracht bij 10 deg:\n"
                    "  2kg: ~3N  |  20kg: ~34N  |  80kg: ~136N");

            ImGui::SliderFloat("Montagehoogte", &g_mount_height, 0.10f, 0.99f, "%.2f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(
                    "Fractie van BIKE_HEIGHT (%.1fm) voor de stang.\n"
                    "tau_motor = F_net * (frac * %.1fm)\n"
                    "Hoger = meer hefboom = betere controle.\n"
                    "0.94 = ~1.6m (vlak onder top).",
                    BIKE_HEIGHT, BIKE_HEIGHT);
            ImGui::TextDisabled("  => %.2f m boven grond", g_mount_height * BIKE_HEIGHT);

            ImGui::SliderFloat("Motor idle", &g_motor_base, 0.0f, 300.0f, "%.0f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("DSHOT idle waarde (visueel). Geen effect op kracht.");

            ImGui::SliderFloat("Motor trim", &g_motor_trim, -200.0f, 200.0f, "%.0f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Trim offset op DSHOT waarden (visueel).");

            ImGui::Separator();

            // ── Verstoringen ──────────────────────────────────────────────
            ImGui::Spacing();
            ImGui::TextColored(ImVec4(0.6f, 0.9f, 0.7f, 1.0f), "Verstoringen");

            ImGui::SliderFloat("Wind max (N)", &g_wind_max, 0.0f, 20.0f, "%.1f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Willekeurige windkracht. tau_wind = F_wind * CoM_hoogte.");

            ImGui::SliderFloat("Starthoek (deg)", &g_init_angle, -20.0f, 20.0f, "%.1f");

            ImGui::Separator();

            // ── Bediening ─────────────────────────────────────────────────
            ImGui::Spacing();
            ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "Bediening");
            ImGui::Spacing();

            const char* start_lbl = g_running ? "Pauze"
                : (g_pendulum.fallen ? "Start (reset)" : "Start");
            if (ImGui::Button(start_lbl, ImVec2(-1, 32)))
            {
                if (g_pendulum.fallen) SimReset();
                g_running = !g_running;
            }

            ImGui::Spacing();
            if (ImGui::Button("Reset", ImVec2(-1, 28))) SimReset();

            ImGui::Spacing();
            if (ImGui::Button("<- Duw links", ImVec2(-1, 28))) g_pendulum.theta_dot -= 1.0f;
            if (ImGui::Button("Duw rechts ->", ImVec2(-1, 28))) g_pendulum.theta_dot += 1.0f;

            ImGui::Spacing();
            ImGui::TextDisabled("Duw = plotse stoot van 1 rad/s.");
            ImGui::TextDisabled("Stang: %.0fcm | Fiets: %.1fm",
                STANG_LENGTH * 100.0f, BIKE_HEIGHT);

            if (g_pendulum.fallen)
            {
                ImGui::Spacing();
                ImGui::TextColored(ImVec4(0.89f, 0.29f, 0.29f, 1.0f),
                    "Fiets gevallen!\nDruk Start om te resetten.");
            }
        }
        ImGui::EndChild();

        ImGui::End();
    }

} // namespace MijnApp


