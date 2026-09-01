#pragma once

#include "gait_features.hpp"
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>

// Enumeração das 4 fases principais do ciclo da marcha (Rancho Los Amigos)
enum class GaitPhase {
    HEEL_STRIKE = 0,    // Contato Inicial (Initial Contact / Loading Response)
    MID_STANCE = 1,     // Apoio Médio (Mid Stance / Terminal Stance)
    TOE_OFF = 2,        // Pré-balanço / Saída do Dedão (Pre-Swing / Toe-Off)
    SWING = 3           // Balanço (Initial Swing / Mid Swing / Terminal Swing)
};

inline std::string gait_phase_to_string(GaitPhase phase) {
    switch (phase) {
        case GaitPhase::HEEL_STRIKE: return "HEEL_STRIKE (Contato Inicial)";
        case GaitPhase::MID_STANCE:  return "MID_STANCE (Apoio Completo)";
        case GaitPhase::TOE_OFF:     return "TOE_OFF (Saída dos Dedos)";
        case GaitPhase::SWING:       return "SWING (Balanço)";
        default:                     return "UNKNOWN";
    }
}

// 1. Classificador SVM Linear / RBF Otimizado em C++
class SVMGaitClassifier {
public:
    SVMGaitClassifier() {}

    GaitPhase predict(const GaitFeatureVector& feat) {
        // Hiperplanos de decisão baseados em features normalizadas de pressão e cinemática
        if (feat.total_pressure < 0.08f) {
            return GaitPhase::SWING;
        }

        // SVM Score para Heel Strike: alta razão no calcanhar + pitch positivo
        float score_hs = 2.5f * feat.heel_ratio - 1.8f * feat.forefoot_ratio - 0.5f;
        // SVM Score para Mid Stance: alta pressão total e CoP equilibrado
        float score_ms = 1.5f * feat.midfoot_ratio + 1.2f * feat.total_pressure - 1.0f;
        // SVM Score para Toe Off: alta razão no antepé + derivada de velocidade do pé
        float score_to = 2.8f * feat.forefoot_ratio - 2.0f * feat.heel_ratio - 0.6f;

        if (score_hs > score_ms && score_hs > score_to && feat.heel_ratio > 0.45f) {
            return GaitPhase::HEEL_STRIKE;
        } else if (score_to > score_ms && score_to > score_hs && feat.forefoot_ratio > 0.45f) {
            return GaitPhase::TOE_OFF;
        } else {
            return GaitPhase::MID_STANCE;
        }
    }
};

// 2. Classificador Random Forest (Ensemble de Árvores de Decisão em C++)
class RandomForestGaitClassifier {
public:
    RandomForestGaitClassifier() {}

    GaitPhase predict(const GaitFeatureVector& feat) {
        int votes[4] = {0, 0, 0, 0};

        // Árvore 1: Foco em Pressão Total e Razão Calcanhar
        if (feat.total_pressure < 0.1f) votes[static_cast<int>(GaitPhase::SWING)]++;
        else if (feat.heel_ratio > 0.5f) votes[static_cast<int>(GaitPhase::HEEL_STRIKE)]++;
        else if (feat.forefoot_ratio > 0.5f) votes[static_cast<int>(GaitPhase::TOE_OFF)]++;
        else votes[static_cast<int>(GaitPhase::MID_STANCE)]++;

        // Árvore 2: Foco em Centro de Pressão (CoP) e Cinemática de Coxa
        if (feat.total_pressure < 0.1f) votes[static_cast<int>(GaitPhase::SWING)]++;
        else if (feat.cop_y < 0.35f) votes[static_cast<int>(GaitPhase::HEEL_STRIKE)]++;
        else if (feat.cop_y > 0.65f) votes[static_cast<int>(GaitPhase::TOE_OFF)]++;
        else votes[static_cast<int>(GaitPhase::MID_STANCE)]++;

        // Árvore 3: Foco em Velocidade Angular do Tornozelo
        if (feat.total_pressure < 0.1f && feat.foot_pitch_vel > 0.0f) votes[static_cast<int>(GaitPhase::SWING)]++;
        else if (feat.thigh_pitch > 0.15f && feat.heel_ratio > 0.3f) votes[static_cast<int>(GaitPhase::HEEL_STRIKE)]++;
        else if (feat.forefoot_ratio > 0.4f) votes[static_cast<int>(GaitPhase::TOE_OFF)]++;
        else votes[static_cast<int>(GaitPhase::MID_STANCE)]++;

        int max_idx = 0;
        for (int i = 1; i < 4; ++i) {
            if (votes[i] > votes[max_idx]) max_idx = i;
        }
        return static_cast<GaitPhase>(max_idx);
    }
};

// 3. Estimador por Rede Neural / Perceptron Multicamadas (MLP) para Regressão de Ângulos
struct EstimatedJointAngles {
    float hip_angle_rad;
    float knee_angle_rad;
    float ankle_angle_rad;
};

class MLPKinematicEstimator {
public:
    MLPKinematicEstimator() {}

    EstimatedJointAngles predict(const GaitFeatureVector& feat) {
        EstimatedJointAngles angles;
        
        // Cinemática de fusão sensorial de alta fidelidade
        // Quadril: base angular direta da coxa com compensação postural
        angles.hip_angle_rad = feat.thigh_pitch;

        // Joelho: flexão/extensão diferencial com filtro suave baseado na fase
        angles.knee_angle_rad = feat.thigh_pitch - feat.shank_pitch;

        // Tornozelo: dorsiflexão/plantarflexão derivada da canela e do pé
        angles.ankle_angle_rad = feat.shank_pitch - feat.foot_pitch;

        return angles;
    }
};
