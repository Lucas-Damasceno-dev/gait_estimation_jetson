#pragma once

#include <vector>
#include <cmath>
#include <numeric>
#include <algorithm>

struct GaitFeatureVector {
    // 16 canais normalizados de pressão (palmilha direita/esquerda)
    std::vector<float> pressure_normalized;
    
    // Métricas espaciais de pressão
    float heel_ratio;       // Razão de carga no Calcanhar (0.0 - 1.0)
    float midfoot_ratio;    // Razão de carga no Mediopé (0.0 - 1.0)
    float forefoot_ratio;   // Razão de carga no Antepé (0.0 - 1.0)
    float cop_y;            // Centro de Pressão longitudinal (Center of Pressure)
    float total_pressure;   // Força total estimada

    // Orientação / Atitude dos segmentos (em radianos)
    float thigh_pitch;
    float shank_pitch;
    float foot_pitch;

    // Velocidades angulares estimadas (derivadas numéricas)
    float thigh_pitch_vel;
    float shank_pitch_vel;
    float foot_pitch_vel;
};

inline GaitFeatureVector extract_gait_features(
    const std::vector<uint16_t>& raw_pressure,
    float thigh_pitch,
    float shank_pitch,
    float foot_pitch,
    float dt = 0.01f)
{
    static float prev_thigh_pitch = 0.0f;
    static float prev_shank_pitch = 0.0f;
    static float prev_foot_pitch = 0.0f;

    GaitFeatureVector feat;
    feat.pressure_normalized.resize(16, 0.0f);

    float sum_total = 0.0f;
    float sum_heel = 0.0f;
    float sum_midfoot = 0.0f;
    float sum_forefoot = 0.0f;
    float weighted_y = 0.0f;

    constexpr float V_MAX = 4095.0f; // Fundo de escala ADC 12 bits

    // Posições longitudinais reais normalizadas (0.0 = Calcanhar posterior, 1.0 = Ponta dos dedos)
    // Mapeamento anatômico Legact FS-INS-16Z (sensores aos pares na mesma linha transversal)
    static constexpr float SENSOR_Y_NORM[16] = {
        0.0000f, 0.0000f, 0.1471f, 0.1471f, // 0..3: Calcanhar
        0.3235f, 0.3235f, 0.5000f, 0.5000f, // 4..7: Mediopé
        0.6765f, 0.6765f, 0.6765f, 0.6765f, // 8..11: Metatarso / Antepé
        0.8529f, 0.8529f, 0.8529f, 1.0000f  // 12..15: Falanges / Halux
    };

    for (size_t i = 0; i < 16 && i < raw_pressure.size(); ++i) {
        float norm_val = std::min(std::max(static_cast<float>(raw_pressure[i]) / V_MAX, 0.0f), 1.0f);
        feat.pressure_normalized[i] = norm_val;
        sum_total += norm_val;

        // Distribuição anatômica Legact FS-INS-16Z:
        // 0..3: Calcanhar (Rearfoot)
        // 4..7: Mediopé (Midfoot)
        // 8..15: Antepé / Dedos (Forefoot / Toes)
        if (i < 4) sum_heel += norm_val;
        else if (i < 8) sum_midfoot += norm_val;
        else sum_forefoot += norm_val;

        // Ponderação do Centro de Pressão longitudinal baseado na anatomia real
        float y_pos = (i < 16) ? SENSOR_Y_NORM[i] : 0.5f;
        weighted_y += norm_val * y_pos;
    }

    feat.total_pressure = sum_total;
    if (sum_total > 1e-4f) {
        feat.heel_ratio = sum_heel / sum_total;
        feat.midfoot_ratio = sum_midfoot / sum_total;
        feat.forefoot_ratio = sum_forefoot / sum_total;
        feat.cop_y = weighted_y / sum_total;
    } else {
        feat.heel_ratio = 0.0f;
        feat.midfoot_ratio = 0.0f;
        feat.forefoot_ratio = 0.0f;
        feat.cop_y = 0.5f;
    }

    feat.thigh_pitch = thigh_pitch;
    feat.shank_pitch = shank_pitch;
    feat.foot_pitch = foot_pitch;

    // Velocidades angulares
    if (dt > 1e-5f) {
        feat.thigh_pitch_vel = (thigh_pitch - prev_thigh_pitch) / dt;
        feat.shank_pitch_vel = (shank_pitch - prev_shank_pitch) / dt;
        feat.foot_pitch_vel = (foot_pitch - prev_foot_pitch) / dt;
    } else {
        feat.thigh_pitch_vel = 0.0f;
        feat.shank_pitch_vel = 0.0f;
        feat.foot_pitch_vel = 0.0f;
    }

    prev_thigh_pitch = thigh_pitch;
    prev_shank_pitch = shank_pitch;
    prev_foot_pitch = foot_pitch;

    return feat;
}
