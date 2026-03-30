#pragma once

#include "imgui.h"
#include <vector>
#include <string>

// ─────────────────────────────────────────────
//  PID Regelaar
// ─────────────────────────────────────────────
struct PIDController
{
    float kp = 15.0f;   // proportioneel  — hoe hard de motor reageert op de fouthoek
    float ki = 0.5f;    // integraal      — corrigeert langzame drift
    float kd = 8.0f;    // differentiaal  — dempt oscillaties

    float integral = 0.0f;
    float prev_error = 0.0f;

    void  Reset();
    float Update(float error, float dt);
};

// ─────────────────────────────────────────────
//  Zwaartepunt (Center of Mass)
//
//  De fiets bestaat uit twee massa's:
//    - frame:    massa_frame op hoogte h_frame
//    - berijder: massa_rider op hoogte h_rider
//
//  Het gecombineerde CoM bepaalt hoe snel de fiets valt.
//  Hoger CoM = instabielere fiets = snellere val.
// ─────────────────────────────────────────────
struct CenterOfMass
{
    float mass_frame = 10.0f;  // kg  — massa van frame + wielen
    float h_frame = 0.5f;   // m   — hoogte CoM van het frame (laag, zwaartepunt wielen/frame)
    float mass_rider = 5.0f;   // kg  — massa berijder (of extra lading)
    float h_rider = 1.2f;   // m   — hoogte CoM berijder (zadel + romp)

    // Berekent de gewogen gemiddelde hoogte van het totale zwaartepunt
    float EffectiveL() const
    {
        float total = mass_frame + mass_rider;
        if (total < 0.01f) return 0.8f;
        return (mass_frame * h_frame + mass_rider * h_rider) / total;
    }

    // Totale massa van het systeem
    float TotalMass() const { return mass_frame + mass_rider; }
};

// ─────────────────────────────────────────────
//  Drone motor thrust berekening
//
//  Formule: T = Ct * rho * n² * D⁴
//    Ct  = thrust coëfficiënt (~0.09 voor hobby propellers)
//    rho = luchtdichtheid (1.225 kg/m³ op zeeniveau)
//    n   = toerental in rev/s (= RPM / 60)
//    D   = rotordiameter in meter
//
//  De effectieve zijwaartse kracht op de fiets hangt ook af van
//  de hoek van de stang t.o.v. verticaal — hier benaderen we die als ≈ 1
//  (horizontale montage).
// ─────────────────────────────────────────────
struct DroneMotor
{
    float diameter_mm = 127.0f;   // mm  — rotordiameter (5 inch = 127 mm)
    float kv_rating = 2300.0f;  // KV  — motor KV-waarde (RPM per Volt)
    float voltage = 14.8f;    // V   — accuspanning (4S LiPo = 14.8V)
    float throttle = 0.5f;     // 0–1 — throttle percentage
    float Ct = 0.09f;    // thrust coëfficiënt (afh. van propeller pitch/vorm)
    float rho = 1.225f;   // kg/m³ luchtdichtheid
    float arm_length = 0.10f;   // m — halve stanglengte (horizontaal), blijft vast
    float mount_height = 0.45f;   // 0.0–1.0 — fractie van de poollengte waar de stang zit

    // Thrust in Newton bij het huidige throttle-niveau
    float ThrustNewton() const
    {
        float D = diameter_mm / 1000.0f;               // mm -> m
        float rpm = kv_rating * voltage * throttle;    // toerental bij dit throttle
        float n = rpm / 60.0f;                       // rev/s
        return Ct * rho * (n * n) * (D * D * D * D);  // T = Ct * rho * n² * D⁴
    }

    // Max thrust bij 100% throttle (voor normalisatie)
    float MaxThrust() const
    {
        float D = diameter_mm / 1000.0f;
        float rpm = kv_rating * voltage;               // throttle = 1.0
        float n = rpm / 60.0f;
        return Ct * rho * (n * n) * (D * D * D * D);
    }
};

// ─────────────────────────────────────────────
//  Fysica — omgekeerde slinger (fiets)
// ─────────────────────────────────────────────
struct Pendulum
{
    float max_force = 30.0f;   // N    — maximale kracht die de drone-motoren kunnen leveren

    float theta = 0.0f;    // rad  — kantelhoek (+ = valt rechts, - = valt links)
    float theta_dot = 0.0f;    // rad/s— hoeksnelheid
    float wind_force = 0.0f;    // N    — huidige random windkracht
    float wind_timer = 0.0f;    // s    — timer tot volgende windwissel
    bool  fallen = false;   // true als |theta| > 90° (fiets op de grond)

    // CoM-model: bepaalt massa en hoogte zwaartepunt
    CenterOfMass com;

    void   Reset(float init_angle_deg = 5.0f);
    void   Step(float motor_output,float arm_height, float wind_max, float dt);

    // Schermcoordinaat van het bovenste eindpunt (top fiets)
    ImVec2 TipPosition(ImVec2 pivot, float scale) const;

    // Schermcoordinaat van het zwaartepunt (tussenliggend op de pool)
    ImVec2 CoMPosition(ImVec2 pivot, float scale) const;
};

// ─────────────────────────────────────────────
//  Ringbuffer voor scrollende grafieken
//  Werkt als een circulaire buffer zodat oudere data automatisch
//  overschreven wordt — geheugengebruik blijft constant.
// ─────────────────────────────────────────────
struct ScrollingBuffer
{
    int                max_size;
    int                offset;   // schrijfpositie in de circulaire buffer
    std::vector<float> data;

    explicit ScrollingBuffer(int max = 2000);
    void AddPoint(float val);
    void Clear();
};

// ─────────────────────────────────────────────
//  Hoofd applicatie namespace
// ─────────────────────────────────────────────
namespace MijnApp
{
    void RenderUI();  // wordt elke frame aangeroepen vanuit de main loop
}
