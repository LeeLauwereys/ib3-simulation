
//TODO:
//berekenen hoeveel het zwaartepunt buiten het evenwichtlijn verschuift tijdens het verdraaien van het (stuur)

//thrust berekenen berekenen:
// T = Ct * rho * n² * D^4
// T = thrust (newton)
// Ct = thrustcoeff (~0,1 voor hobby drones)
// rho = luchtdichtheid (1,225 kg/m³)
// n = aantal toerental in rev/s
// D = rotordiameter



#include "pid.h"
#include "imgui.h"
#include "implot.h"

#include <cmath>
#include <cstdlib>
#include <algorithm>
#include <string>

// ════════════════════════════════════════════
//  Globale constanten
// ════════════════════════════════════════════
static constexpr float PI = 3.14159265f;
static constexpr float G = 9.81f;    // m/s² — valversnelling
static constexpr float DT = 0.005f;   // s    — simulatiestap (kleiner = nauwkeuriger)
static constexpr float MAX_ANGLE = PI / 2.0f;// rad  — 90° => fiets gevallen
static constexpr int   STEPS_PER_FRAME = 4;        // fysica-stappen per render-frame


// ════════════════════════════════════════════
//  Hulpfuncties
// ════════════════════════════════════════════

// Willekeurig getal tussen lo en hi
static float RandF(float lo, float hi)
{
    return lo + (hi - lo) * ((float)rand() / (float)RAND_MAX);
}

// Clip een waarde tussen min en max
static float Clamp(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}


// ════════════════════════════════════════════
//  PIDController — implementatie
// ════════════════════════════════════════════

void PIDController::Reset()
{
    integral = 0.0f;
    prev_error = 0.0f;
}

float PIDController::Update(float error, float dt)
{
    // Integraalterm: accumuleer de fout over tijd
    integral += error * dt;

    // Differentiaalterm: snelheid van foutverandering
    float derivative = (error - prev_error) / dt;
    prev_error = error;

    // PID formule: u = Kp*e + Ki*∫e + Kd*de/dt
    return kp * error + ki * integral + kd * derivative;
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
}

void Pendulum::Step(float motor_output, float arm_height, float wind_max, float dt)
{
    if (fallen) return;

    // ── Wind: update elke 0.5–2 seconden naar een nieuwe random waarde ──
    wind_timer -= dt;
    if (wind_timer <= 0.0f)
    {
        wind_force = RandF(-wind_max, wind_max);
        wind_timer = RandF(0.5f, 2.0f);
    }

    // Clip motor output op maximale drone kracht
    float force = Clamp(motor_output, -max_force, max_force);

    // ── Traagheidsmoment van een puntstmassa op afstand L ──
    // I = m * L²   (waarbij L = effectieve hoogte zwaartepunt)
    float L = com.EffectiveL();
    float m = com.TotalMass();
    float I = m * L * L;

    // ── Koppels (torques) op de fiets ──
    // Zwaartekracht wil de fiets verder laten vallen: τ = m*g*L*sin(θ)
    float tau_gravity = m * G * L * sinf(theta);

    // Wind duwt de fiets zijwaarts: τ = F_wind * L
    float tau_wind = wind_force * L;

    // Motor corrigeert: negatief teken = tegengesteld aan helling
    //float tau_motor = -force * L;
    // In Pendulum::Step() — vervang tau_motor:
    //float mount_h = g_motor.mount_height * p.com.h_rider * 1.15f; // hoogte stang in meters
    float tau_motor = -force * arm_height;   // was: -force * L  (CoM hoogte)

    // ── Bewegingsvergelijking: I * θ'' = Σ(torques) ──
    float theta_ddot = (tau_gravity + tau_wind + tau_motor) / I;

    // ── Euler-integratie ──
    theta_dot += theta_ddot * dt;
    theta_dot *= 0.999f;          // kleine luchtweerstandsdemping
    theta += theta_dot * dt;

    // Check of de fiets gevallen is (> 90°)
    if (fabsf(theta) > MAX_ANGLE)
        fallen = true;
}

ImVec2 Pendulum::TipPosition(ImVec2 pivot, float scale) const
{
    // Bovenste eindpunt van de fiets (totale hoogte = CoM.h_rider * 1.15 benadering)
    float L_tip = com.h_rider * 1.15f;  // iets boven de berijder = top fiets
    return ImVec2(
        pivot.x + sinf(theta) * L_tip * scale,
        pivot.y - cosf(theta) * L_tip * scale
    );
}

ImVec2 Pendulum::CoMPosition(ImVec2 pivot, float scale) const
{
    // Zwaartepunt op de effectieve hoogte L
    float L = com.EffectiveL();
    return ImVec2(
        pivot.x + sinf(theta) * L * scale,
        pivot.y - cosf(theta) * L * scale
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
        // Overschrijf oudste waarde (circulaire buffer)
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
//  Module-level simulatiestaat
//  (anonieme namespace = niet zichtbaar buiten dit .cpp bestand)
// ════════════════════════════════════════════
namespace
{
    Pendulum      g_pendulum;           // de fiets + CoM
    PIDController g_pid;               // de regelaar
    DroneMotor    g_motor;             // drone motor specificaties

    bool  g_running = false;     // loopt de simulatie?
    float g_sim_time = 0.0f;      // verstreken simulatietijd (s)
    float g_wind_max = 1.0f;      // maximale windkracht (N)
    float g_init_angle = 5.0f;      // starthoek bij reset (graden)

    // Data logs voor grafieken (ringbuffers)
    ScrollingBuffer g_log_t{ 3000 };  // tijd
    ScrollingBuffer g_log_theta{ 3000 };  // hoek in graden
    ScrollingBuffer g_log_motor{ 3000 };  // PID output (N)
    ScrollingBuffer g_log_motorL{ 3000 };  // motor links (positief N)
    ScrollingBuffer g_log_motorR{ 3000 };  // motor rechts (negatief N)
    ScrollingBuffer g_log_wind{ 3000 };  // windkracht

    float g_last_motor_output = 0.0f;    // laatste PID output, voor visualisatie

    // ── Reset de volledige simulatie ─────────────────────────────────────
    void SimReset()
    {
        g_pendulum.Reset(g_init_angle);
        g_pid.Reset();
        g_sim_time = 0.0f;
        g_last_motor_output = 0.0f;
        g_running = false;
        g_log_t.Clear();
        g_log_theta.Clear();
        g_log_motor.Clear();
        g_log_motorL.Clear();
        g_log_motorR.Clear();
        g_log_wind.Clear();
    }

    // ── Één simulatiestap ─────────────────────────────────────────────────
    void SimStep()
    {
        if (g_pendulum.fallen) { g_running = false; return; }

        // Bereken PID output (fout = huidige hoek, doel = 0° = rechtop)
        float error = g_pendulum.theta;
        float output = g_pid.Update(error, DT);
        output = Clamp(output, -g_pendulum.max_force, g_pendulum.max_force);
        g_last_motor_output = output;

        // Pas fysica toe
        float arm_h = g_motor.mount_height * g_pendulum.com.h_rider * 1.15f;
        g_pendulum.Step(output, arm_h, g_wind_max, DT);
        g_sim_time += DT;

        // Splits motor output in linker/rechter activatie (0–1)
        float left_norm = (output > 0.0f) ? (output / g_pendulum.max_force) : 0.0f;
        float right_norm = (output < 0.0f) ? (-output / g_pendulum.max_force) : 0.0f;

        // Log alles voor de grafieken
        g_log_t.AddPoint(g_sim_time);
        g_log_theta.AddPoint(g_pendulum.theta * 180.0f / PI);
        g_log_motor.AddPoint(output);
        g_log_motorL.AddPoint(left_norm * g_pendulum.max_force);
        g_log_motorR.AddPoint(-right_norm * g_pendulum.max_force);
        g_log_wind.AddPoint(g_pendulum.wind_force);
    }


    // ════════════════════════════════════════════
    //  Tekenroutine: fiets canvas
    // ════════════════════════════════════════════
    void DrawPendulum(ImDrawList* dl, ImVec2 pivot, float scale)
    {
        auto& p = g_pendulum;
        ImVec2 tip = p.TipPosition(pivot, scale);
        ImVec2 com = p.CoMPosition(pivot, scale);

        // ── Grondlijn ────────────────────────────────────────────────────
        dl->AddLine(
            ImVec2(pivot.x - 140, pivot.y),
            ImVec2(pivot.x + 140, pivot.y),
            IM_COL32(70, 70, 70, 255), 1.5f
        );

        // ── Stippellijn verticaal (referentie) ───────────────────────────
        dl->AddLine(
            pivot,
            ImVec2(pivot.x, pivot.y - p.com.h_rider * 1.2f * scale),
            IM_COL32(50, 50, 50, 200), 1.0f
        );

        // ── Fiets pool (wit) ──────────────────────────────────────────────
        dl->AddLine(pivot, tip, IM_COL32(210, 210, 210, 255), 5.0f);

        // ── Zwaartepunt (gele ster / bol) ────────────────────────────────
        // Het CoM is het meest kritische punt: hoe hoger, hoe sneller de val
        dl->AddCircleFilled(com, 10.0f, IM_COL32(255, 200, 0, 220));   // geel = CoM
        dl->AddCircle(com, 10.0f, IM_COL32(255, 230, 100, 180), 0, 2.0f);
        // Label
        dl->AddText(
            ImVec2(com.x + 13, com.y - 7),
            IM_COL32(255, 200, 0, 200), "CoM"
        );

        // ── Top van de fiets (rood) ───────────────────────────────────────
        dl->AddCircleFilled(tip, 10.0f, IM_COL32(220, 60, 60, 255));
        dl->AddCircle(tip, 10.0f, IM_COL32(255, 120, 120, 160), 0, 1.5f);

        // ── Pivot (bodempunt / wielcontact) ──────────────────────────────
        dl->AddCircleFilled(pivot, 6.0f, IM_COL32(255, 255, 255, 220));

        // ── Drone stang (haaks op de pool, variabele hoogte + lengte) ────────
        float mid_frac = g_motor.mount_height;          //hoogte instelbaar
        float L_tip = p.com.h_rider * 1.15f;            //halve meter lengte
        float stang_len = g_motor.arm_length * scale;
        ImVec2 stang_mid(
            pivot.x + sinf(p.theta) * L_tip * scale * mid_frac,
            pivot.y - cosf(p.theta) * L_tip * scale * mid_frac
        );
        float perp_x = cosf(p.theta);
        float perp_y = sinf(p.theta);

        ImVec2 lp(stang_mid.x - perp_x * stang_len, stang_mid.y - perp_y * stang_len);
        ImVec2 rp(stang_mid.x + perp_x * stang_len, stang_mid.y + perp_y * stang_len);
        dl->AddLine(lp, rp, IM_COL32(150, 150, 150, 200), 2.5f);


        // ── Newton-pijlen (thrust visualisatie) ──────────────────────────────
        // Richting: loodrecht op de stang = langs de pool omhoog
        float thrust_dir_x = perp_x;   // langs de stang = horizontaal
        float thrust_dir_y = perp_y;

        auto DrawArrow = [&](ImVec2 origin, float force_n, float max_force, ImU32 col_arrow)
        {
            if (fabsf(force_n) < 0.1f) return;
            float frac = fabsf(force_n) / (max_force + 0.01f);
            float arrow_len = frac * 80.0f;

            float dir = (force_n >= 0.0f) ? 1.0f : -1.0f;

            ImVec2 arrow_tip(
                origin.x + thrust_dir_x * arrow_len * dir,
                origin.y + thrust_dir_y * arrow_len * dir
            );

            dl->AddLine(origin, arrow_tip, col_arrow, 2.5f);

            float head = 10.0f;
            float back_x = origin.x - arrow_tip.x;
            float back_y = origin.y - arrow_tip.y;
            float len = sqrtf(back_x * back_x + back_y * back_y);
            if (len < 0.01f) return;
            back_x /= len; back_y /= len;

            float side_x = -back_y, side_y = back_x;
            dl->AddLine(arrow_tip,
                ImVec2(arrow_tip.x + back_x * head + side_x * head * 0.5f,
                    arrow_tip.y + back_y * head + side_y * head * 0.5f),
                col_arrow, 2.5f);
            dl->AddLine(arrow_tip,
                ImVec2(arrow_tip.x + back_x * head - side_x * head * 0.5f,
                    arrow_tip.y + back_y * head - side_y * head * 0.5f),
                col_arrow, 2.5f);

            char nbuf[24];
            snprintf(nbuf, sizeof(nbuf), "%.1fN", fabsf(force_n));
            dl->AddText(ImVec2(arrow_tip.x + 4, arrow_tip.y - 7), col_arrow, nbuf);
        };

        //float force_L = (g_last_motor_output > 0.0f) ? g_last_motor_output : 0.0f;
        //float force_R = (g_last_motor_output < 0.0f) ? -g_last_motor_output : 0.0f;

        float force_L = (g_last_motor_output < 0.0f) ? -g_last_motor_output : 0.0f;
        float force_R = (g_last_motor_output > 0.0f) ? g_last_motor_output : 0.0f;


        DrawArrow(lp, force_L, g_pendulum.max_force, IM_COL32(55, 138, 221, 230));
        DrawArrow(rp, -force_R, g_pendulum.max_force, IM_COL32(226, 75, 74, 230));

        // ── Motor kleuren: blauw = links actief, rood = rechts actief ────────

        // ── Motor kleuren: blauw = links actief, rood = rechts actief ────
        bool left_on = g_last_motor_output < -0.5f;
        bool right_on = g_last_motor_output > 0.5f;
        float intensity = fabsf(g_last_motor_output) / (g_pendulum.max_force + 0.01f);

        ImU32 col_l = left_on ? IM_COL32(55, 138, 221, 255) : IM_COL32(70, 70, 70, 180);
        ImU32 col_r = right_on ? IM_COL32(226, 75, 74, 255) : IM_COL32(70, 70, 70, 180);

        dl->AddCircleFilled(lp, 10.0f, col_l);
        dl->AddCircleFilled(rp, 10.0f, col_r);
        dl->AddText(ImVec2(lp.x - 3, lp.y + 13), IM_COL32(200, 200, 200, 200), "L");
        dl->AddText(ImVec2(rp.x - 4, rp.y + 13), IM_COL32(200, 200, 200, 200), "R");

        // ── Blaas-effect actieve motor ────────────────────────────────────
        if (left_on)
        {
            for (int i = 1; i <= 3; ++i)
            {
                ImVec2 c(lp.x - perp_x * i * 13.0f * intensity,
                    lp.y - perp_y * i * 13.0f * intensity);
                dl->AddCircle(c, i * 5.5f * intensity,
                    IM_COL32(55, 138, 221, (int)(70 / i)), 0, 1.5f);
            }
        }
        if (right_on)
        {
            for (int i = 1; i <= 3; ++i)
            {
                ImVec2 c(rp.x + perp_x * i * 13.0f * intensity,
                    rp.y + perp_y * i * 13.0f * intensity);
                dl->AddCircle(c, i * 5.5f * intensity,
                    IM_COL32(226, 75, 74, (int)(70 / i)), 0, 1.5f);
            }
        }

        // ── Hoek tekst ────────────────────────────────────────────────────
        char buf[48];
        snprintf(buf, sizeof(buf), "theta = %.1f deg", p.theta * 180.0f / PI);
        dl->AddText(
            ImVec2(pivot.x - 110, pivot.y - L_tip * scale - 28),
            IM_COL32(240, 240, 240, 200), buf
        );

        // CoM hoogte als extra info
        snprintf(buf, sizeof(buf), "CoM  = %.2f m", p.com.EffectiveL());
        dl->AddText(
            ImVec2(pivot.x - 110, pivot.y - L_tip * scale - 14),
            IM_COL32(255, 200, 0, 180), buf
        );

        // ── Gevallen-tekst ────────────────────────────────────────────────
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
                dl->AddLine(ImVec2(wx - dir * 30, yo),
                    ImVec2(wx + dir * 30, yo), wc, 2.0f);
                dl->AddLine(ImVec2(wx + dir * 30, yo),
                    ImVec2(wx + dir * 20, yo - 5), wc, 2.0f);
                dl->AddLine(ImVec2(wx + dir * 30, yo),
                    ImVec2(wx + dir * 20, yo + 5), wc, 2.0f);
            }
        }
    }

} // einde anonieme namespace


// ════════════════════════════════════════════
//  Hulp: teken een scrollende grafiek via ImPlot
//  (werkt met de nieuwe ImPlot API die ImPlotSpec gebruikt)
// ════════════════════════════════════════════

/*
static void PlotScrollLine(const char* label,
    ScrollingBuffer& xbuf,
    ScrollingBuffer& ybuf,
    ImVec4 color,
    float weight = 1.5f)
{
    if (xbuf.data.empty()) return;

    // Nieuwe ImPlot API: stijl via ImPlotSpec struct
    ImPlotSpec spec;
    //spec.Color = color;
    //spec.Weight = weight;

    ImPlot::PlotLine(label,
        xbuf.data.data(),
        ybuf.data.data(),
        (int)xbuf.data.size(),
        spec);
}

*/


// ════════════════════════════════════════════
//  MijnApp::RenderUI   — wordt elke frame aangeroepen
// ════════════════════════════════════════════
namespace MijnApp
{

    void RenderUI()
    {
        // ── Simulatie updaten ─────────────────────────────────────────────────
        if (g_running)
        {
            for (int i = 0; i < STEPS_PER_FRAME; ++i)
                SimStep();
        }

        ImGuiIO& io = ImGui::GetIO();

        // ════════════════════════════════════════════
        //  VASTE LAYOUT — geen docking, geen titelbalk
        //  Zelfde aanpak als je Matrix-app:
        //  één groot venster dat het volledige scherm vult,
        //  met vaste child-windows erin.
        // ════════════════════════════════════════════
        ImGui::SetNextWindowPos(ImGui::GetMainViewport()->WorkPos);
        ImGui::SetNextWindowSize(ImGui::GetMainViewport()->WorkSize);
        ImGui::SetNextWindowBgAlpha(1.0f);

        ImGuiWindowFlags main_flags =
            ImGuiWindowFlags_NoTitleBar |  // geen titelbalk
            ImGuiWindowFlags_NoCollapse |  // niet inklapbaar
            ImGuiWindowFlags_NoResize |  // niet resizable
            ImGuiWindowFlags_NoMove |  // niet versleepbaar
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

        // ── Kolom-breedtes ───────────────────────────────────────────────────
        float col_canvas = total_w * 0.38f;  // canvas: simulatie-animatie
        float col_graphs = total_w * 0.37f;  // grafieken
        float col_params = total_w * 0.25f;  // parameters & bediening

        // ════════════════════════════════════════════
        //  KOLOM 1 — Simulatie canvas
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
            float canvas_h = canvas_avail.y - 55.0f;  // ruimte voor motor bars onderaan
            if (canvas_h < 100) canvas_h = 100;

            // Achtergrond van het canvas
            ImDrawList* dl = ImGui::GetWindowDrawList();
            dl->AddRectFilled(canvas_pos,
                ImVec2(canvas_pos.x + col_canvas - 16, canvas_pos.y + canvas_h),
                IM_COL32(15, 15, 15, 255));

            // Pivot-punt (wielen) en schaal berekenen
            ImVec2 pivot(canvas_pos.x + (col_canvas - 16) * 0.5f,
                canvas_pos.y + canvas_h * 0.78f);
            float  scale = canvas_h * 0.48f;

            DrawPendulum(dl, pivot, scale);

            // Onzichtbare knop zodat de cursor vooruit gaat
            ImGui::InvisibleButton("##canvas_area", ImVec2(col_canvas - 16, canvas_h));

            // ── Motor activatie bars ──────────────────────────────────────────
            float bar_w = (col_canvas - 40) * 0.42f;
            float lx = 8.0f, rx = col_canvas * 0.52f;

            float left_frac = (g_last_motor_output < -0.01f)
                ? Clamp(-g_last_motor_output / g_pendulum.max_force, 0, 1)
                : 0.0f;

            float right_frac = (g_last_motor_output > 0.01f)
                ? Clamp(g_last_motor_output / g_pendulum.max_force, 0, 1)
                : 0.0f;

            ImGui::SetCursorPosX(lx);
            ImGui::TextColored(ImVec4(0.55f, 0.78f, 1.0f, 1.0f), "Motor L");
            ImGui::SetCursorPosX(lx);
            ImGui::PushStyleColor(ImGuiCol_PlotHistogram, ImVec4(0.21f, 0.54f, 0.87f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4(0.10f, 0.10f, 0.10f, 1.0f));
            ImGui::ProgressBar(left_frac, ImVec2(bar_w, 14), "");
            ImGui::PopStyleColor(2);

            ImGui::SameLine(rx);
            ImGui::TextColored(ImVec4(1.0f, 0.45f, 0.45f, 1.0f), "Motor R");
            ImGui::SameLine(rx);
            ImGui::PushStyleColor(ImGuiCol_PlotHistogram, ImVec4(0.89f, 0.29f, 0.29f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4(0.10f, 0.10f, 0.10f, 1.0f));
            ImGui::ProgressBar(right_frac, ImVec2(bar_w, 14), "");
            ImGui::PopStyleColor(2);
        }
        ImGui::EndChild();

        ImGui::SameLine(0, 0);

        // ════════════════════════════════════════════
        //  KOLOM 2 — Grafieken
        // ════════════════════════════════════════════

        ImGui::Begin("Hoek in tijd");

        // Canvas
        ImDrawList* dl = ImGui::GetWindowDrawList();
        ImVec2 pos = ImGui::GetCursorScreenPos();
        ImVec2 size = ImVec2(500, 200);

        // Achtergrond
        dl->AddRectFilled(pos, ImVec2(pos.x + size.x, pos.y + size.y), IM_COL32(30, 30, 30, 255));
        dl->AddRect(pos, ImVec2(pos.x + size.x, pos.y + size.y), IM_COL32(255, 255, 255, 255));

        // Tijd window (laatste 10 sec)
        float t_min = std::max(0.0f, g_sim_time - 10.0f);
        float t_max = g_sim_time;

        // Y-schaal: zoom in op ±15°
        float y_min = -15.0f;
        float y_max = 15.0f;

        // Plot hoeklijn
        if (!g_log_theta.data.empty())
        {
            for (size_t i = 1; i < g_log_theta.data.size(); ++i)
            {
                float t0 = g_log_t.data[i - 1];
                float t1 = g_log_t.data[i];
                if (t1 < t_min) continue;

                float x0 = pos.x + size.x * (t0 - t_min) / (t_max - t_min);
                float x1 = pos.x + size.x * (t1 - t_min) / (t_max - t_min);

                float y0 = pos.y + size.y * (y_max - g_log_theta.data[i - 1]) / (y_max - y_min);
                float y1 = pos.y + size.y * (y_max - g_log_theta.data[i]) / (y_max - y_min);

                dl->AddLine(ImVec2(x0, y0), ImVec2(x1, y1), IM_COL32(220, 50, 50, 255), 2.0f);
            }
        }

        // X-as en Y-as lijnen
        dl->AddLine(ImVec2(pos.x, pos.y + size.y / 2), ImVec2(pos.x + size.x, pos.y + size.y / 2), IM_COL32(255, 255, 255, 100), 1.0f); // X-as (0°)
        dl->AddLine(ImVec2(pos.x, pos.y), ImVec2(pos.x, pos.y + size.y), IM_COL32(255, 255, 255, 100), 1.0f); // Y-as

        // Y-as ticks ±5°, ±10°, ±15°
        for (float angle = -15.0f; angle <= 15.0f; angle += 5.0f)
        {
            float y = pos.y + size.y * (y_max - angle) / (y_max - y_min);
            dl->AddLine(ImVec2(pos.x - 5, y), ImVec2(pos.x + 5, y), IM_COL32(255, 255, 255, 150), 1.0f);
            char buf[8]; sprintf(buf, "%.0f", angle);
            dl->AddText(ImVec2(pos.x - 30, y - 7), IM_COL32(255, 255, 255, 255), buf);
        }

        // X-as ticks (tijd)
        int n_ticks = 5;
        for (int i = 0; i <= n_ticks; ++i)
        {
            float t_tick = t_min + i * (t_max - t_min) / n_ticks;
            float x = pos.x + size.x * (t_tick - t_min) / (t_max - t_min);
            dl->AddLine(ImVec2(x, pos.y + size.y - 5), ImVec2(x, pos.y + size.y + 5), IM_COL32(255, 255, 255, 150), 1.0f);
            char buf[16]; sprintf(buf, "%.1f", t_tick);
            dl->AddText(ImVec2(x - 10, pos.y + size.y + 5), IM_COL32(255, 255, 255, 255), buf);
        }

        // Legenda
        dl->AddRectFilled(ImVec2(pos.x + 10, pos.y + 10), ImVec2(pos.x + 150, pos.y + 50), IM_COL32(20, 20, 20, 200));
        dl->AddRect(ImVec2(pos.x + 10, pos.y + 10), ImVec2(pos.x + 150, pos.y + 50), IM_COL32(255, 255, 255, 200));
        dl->AddLine(ImVec2(pos.x + 15, pos.y + 25), ImVec2(pos.x + 35, pos.y + 25), IM_COL32(220, 50, 50, 255), 2.0f);
        dl->AddText(ImVec2(pos.x + 40, pos.y + 18), IM_COL32(255, 255, 255, 255), "Hoek θ (deg)");

        ImGui::Dummy(ImVec2(size.x, size.y + 20)); // ruimte voor X-as labels
        ImGui::End();

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

            // ── PID ──────────────────────────────────────────────────────────
            ImGui::Spacing();
            ImGui::TextColored(ImVec4(0.6f, 0.8f, 1.0f, 1.0f), "PID Regelaar");

            // Kp tot 200 zodat je de motor agressief kunt aansturen
            ImGui::SliderFloat("Kp##pid", &g_pid.kp, 0.0f, 500.0f, "%.1f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Proportioneel: directe kracht op basis van hoek.\nHoger = agressievere reactie op kleine hoek.");

            ImGui::SliderFloat("Ki##pid", &g_pid.ki, 0.0f, 20.0f, "%.2f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Integraal: corrigeert langzame drift.\nTe hoog = opbouwende oscillaties.");

            ImGui::SliderFloat("Kd##pid", &g_pid.kd, 0.0f, 50.0f, "%.1f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Differentiaal: dempt snelle bewegingen.\nTe hoog = traag en overgedempte reactie.");

            // PID presets
            ImGui::Spacing();
            ImGui::Text("Presets:");
            ImGui::SameLine();
            if (ImGui::SmallButton("Zacht")) { g_pid.kp = 8;   g_pid.ki = 0.2f; g_pid.kd = 5; }
            ImGui::SameLine();
            if (ImGui::SmallButton("Normaal")) { g_pid.kp = 15;  g_pid.ki = 0.5f; g_pid.kd = 8; }
            ImGui::SameLine();
            if (ImGui::SmallButton("Agressief")) { g_pid.kp = 80; g_pid.ki = 1.0f; g_pid.kd = 20; }

            ImGui::Separator();

            // ── Zwaartepunt ───────────────────────────────────────────────────
            ImGui::Spacing();
            ImGui::TextColored(ImVec4(1.0f, 0.85f, 0.2f, 1.0f), "Zwaartepunt (CoM)");

            ImGui::SliderFloat("Frame massa (kg)", &g_pendulum.com.mass_frame, 1.0f, 30.0f, "%.1f");
            ImGui::SliderFloat("Frame hoogte (m)", &g_pendulum.com.h_frame, 0.1f, 0.8f, "%.2f");
            ImGui::SliderFloat("Rijder massa (kg)", &g_pendulum.com.mass_rider, 0.0f, 100.0f, "%.1f");
            ImGui::SliderFloat("Rijder hoogte (m)", &g_pendulum.com.h_rider, 0.5f, 1.8f, "%.2f");

            // Toon het berekende zwaartepunt live
            float com_h = g_pendulum.com.EffectiveL();
            float total_m = g_pendulum.com.TotalMass();
            ImGui::TextColored(ImVec4(1.0f, 0.85f, 0.2f, 1.0f),
                "=> CoM = %.2f m   Massa = %.1f kg", com_h, total_m);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Hoger CoM = fiets valt sneller.\nLager CoM = meer stabiliteit.");

            ImGui::Separator();

            // ── Drone motor thrust ────────────────────────────────────────────
            ImGui::Spacing();
            ImGui::TextColored(ImVec4(0.7f, 1.0f, 0.7f, 1.0f), "Drone Motor");

            ImGui::SliderFloat("Diameter (mm)", &g_motor.diameter_mm, 50.0f, 250.0f, "%.0f");
            ImGui::SliderFloat("KV-rating", &g_motor.kv_rating, 500.0f, 5000.0f, "%.0f");
            ImGui::SliderFloat("Spanning (V)", &g_motor.voltage, 7.4f, 25.2f, "%.1f");
            ImGui::SliderFloat("Ct coeff", &g_motor.Ct, 0.03f, 0.18f, "%.3f");
            ImGui::SliderFloat("Stanglengte (m)", &g_motor.arm_length, 0.10f, 1.0f, "%.2f");
            ImGui::SliderFloat("Montagehoogte", &g_motor.mount_height, 0.10f, 0.95f, "%.2f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Fractie van de poollengte:\n0.0 = grond, 1.0 = top fiets");

            // Bereken en toon de thrust live
            g_motor.throttle = 1.0f;  // max thrust voor weergave
            float thrust_max = g_motor.MaxThrust();

            // Stel max_force in op basis van de twee motoren samen
            // (factor 2 = twee motoren aan de stang)
            g_pendulum.max_force = thrust_max * 2.0f;

            ImGui::TextColored(ImVec4(0.7f, 1.0f, 0.7f, 1.0f),
                "Max thrust 1 motor: %.2f N", thrust_max);
            ImGui::TextColored(ImVec4(0.7f, 1.0f, 0.7f, 1.0f),
                "Max kracht (2x):    %.2f N", g_pendulum.max_force);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(
                    "T = Ct * rho * n^2 * D^4\n"
                    "rho = 1.225 kg/m3\n"
                    "n = KV * V / 60  (rev/s)\n"
                    "D = diameter in meter"
                );

            ImGui::Separator();

            // ── Verstoringen ──────────────────────────────────────────────────
            ImGui::Spacing();
            ImGui::TextColored(ImVec4(0.6f, 0.9f, 0.7f, 1.0f), "Verstoringen");

            ImGui::SliderFloat("Wind max (N)", &g_wind_max, 0.0f, 20.0f, "%.1f");
            ImGui::SliderFloat("Starthoek (deg)", &g_init_angle, -20.0f, 20.0f, "%.1f");

            ImGui::Separator();

            // ── Live metingen ─────────────────────────────────────────────────
            ImGui::Spacing();
            ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "Live waarden");

            float deg = g_pendulum.theta * 180.0f / PI;
            ImVec4 angle_col = fabsf(deg) < 15 ? ImVec4(0.3f, 1.0f, 0.3f, 1.0f)
                : fabsf(deg) < 45 ? ImVec4(1.0f, 0.7f, 0.0f, 1.0f)
                : ImVec4(1.0f, 0.2f, 0.2f, 1.0f);
            ImGui::TextColored(angle_col, "theta     = %.2f deg", deg);
            ImGui::Text("theta'    = %.2f deg/s",
                g_pendulum.theta_dot * 180.0f / PI);
            ImGui::Text("PID out   = %.2f N", g_last_motor_output);
            ImGui::Text("Wind      = %.2f N", g_pendulum.wind_force);
            ImGui::Text("Tijd      = %.2f s", g_sim_time);

            ImGui::Separator();

            // ── Bediening ─────────────────────────────────────────────────────
            ImGui::Spacing();
            ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "Bediening");
            ImGui::Spacing();

            // Start / Pauze knop
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
            // Duw-knoppen: simuleren een plotse stoot of windstoot
            if (ImGui::Button("<- Duw links", ImVec2(-1, 28)))
                g_pendulum.theta_dot -= 1.0f;
            if (ImGui::Button("Duw rechts ->", ImVec2(-1, 28)))
                g_pendulum.theta_dot += 1.0f;

            ImGui::Spacing();
            ImGui::TextDisabled("Duw = plotse stoot simuleren.");

            // Gevallen-melding
            if (g_pendulum.fallen)
            {
                ImGui::Spacing();
                ImGui::TextColored(ImVec4(0.89f, 0.29f, 0.29f, 1.0f),
                    "Fiets gevallen!\nDruk Start om te resetten.");
            }
        }
        ImGui::EndChild();

        ImGui::End();  // einde hoofdvenster
    }

} // namespace MijnApp
