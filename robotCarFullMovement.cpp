#include <iostream>
#include <string>
#include <vector>
#include <limits>
#include <cstdio>

#include <webots/Robot.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/LED.hpp>
#include <webots/Motor.hpp>

using namespace webots;

// Global defines
#define NO_SIDE -1
#define LEFT 0
#define RIGHT 1
#define WHITE 0
#define BLACK 1
#define TIME_STEP 32

#define SIMULATION 0
#define REALITY 2

// Estructura matemática para operaciones de vectores de velocidad
struct SpeedVector2D {
    int left;
    int right;

    // Operación vectorial: Suma de vectores
    SpeedVector2D operator+(const SpeedVector2D& other) const {
        return { left + other.left, right + other.right };
    }

    // Operación vectorial: Multiplicación por un escalar (ponderación)
    SpeedVector2D operator*(double scalar) const {
        return { static_cast<int>(left * scalar), static_cast<int>(right * scalar) };
    }
};

class EPuckController {
private:
    Robot* robot;
    
    // Contenedores vectoriales de la STL para sustituir arreglos estáticos
    std::vector<LED*> leds;
    std::vector<DistanceSensor*> ps;
    std::vector<DistanceSensor*> gs;
    
    Motor* left_motor;
    Motor* right_motor;

    // Variables de estado de los módulos
    std::vector<int> ps_value;
    std::vector<unsigned short> gs_value;
    std::vector<int> ps_offset;

    int Mode;

    // Variables internas de la Arquitectura de Subsunción
    SpeedVector2D lfm_speed;
    SpeedVector2D oam_speed;
    SpeedVector2D ofm_speed;
    SpeedVector2D lem_speed;

    bool oam_active, oam_reset;
    int oam_side;

    bool llm_active, llm_inibit_ofm_speed;
    int llm_past_side;
    bool lem_reset;

    bool lem_active;
    int lem_state, lem_black_counter;
    int cur_op_gs_value, prev_op_gs_value;

    // Constantes de calibración
    const int NB_DIST_SENS = 8;
    const int NB_GROUND_SENS = 3;
    const int NB_LEDS = 8;
    const int GS_WHITE = 900;
    const int LEM_THRESHOLD = 500;
    const int LLM_THRESHOLD = 800;

    const std::vector<int> PS_OFFSET_SIMULATION = {300, 300, 300, 300, 300, 300, 300, 300};
    const std::vector<int> PS_OFFSET_REALITY = {480, 170, 320, 500, 600, 680, 210, 640};

public:
    EPuckController() : 
        robot(new Robot()), Mode(-1), 
        lfm_speed{0,0}, oam_speed{0,0}, ofm_speed{0,0}, lem_speed{0,0},
        oam_active(false), oam_reset(false), oam_side(NO_SIDE),
        llm_active(false), llm_inibit_ofm_speed(false), llm_past_side(NO_SIDE), lem_reset(false),
        lem_active(false), lem_state(0), lem_black_counter(0),
        cur_op_gs_value(WHITE), prev_op_gs_value(WHITE) 
    {
        ps_value.resize(NB_DIST_SENS, 0);
        gs_value.resize(NB_GROUND_SENS, 0);
        ps_offset.resize(NB_DIST_SENS, 0);
        
        initDevices();
    }

    ~EPuckController() {
        delete robot;
    }

private:
    void initDevices() {
        for (int i = 0; i < NB_LEDS; i++) {
            leds.push_back(robot->getLED("led" + std::to_string(i)));
        }
        for (int i = 0; i < NB_DIST_SENS; i++) {
            DistanceSensor* sensor = robot->getDistanceSensor("ps" + std::to_string(i));
            sensor->enable(TIME_STEP);
            ps.push_back(sensor);
        }
        for (int i = 0; i < NB_GROUND_SENS; i++) {
            DistanceSensor* sensor = robot->getDistanceSensor("gs" + std::to_string(i));
            sensor->enable(TIME_STEP);
            gs.push_back(sensor);
        }

        left_motor = robot->getMotor("left wheel motor");
        right_motor = robot->getMotor("right wheel motor");
        
        left_motor->setPosition(std::numeric_limits<double>::infinity());
        right_motor->setPosition(std::numeric_limits<double>::infinity());
        left_motor->setVelocity(0.0);
        right_motor->setVelocity(0.0);
    }

    void handleModeChange() {
        if (Mode != robot->getMode()) {
            oam_reset = true;
            llm_active = false;
            llm_past_side = NO_SIDE;
            ofm_active = false;
            lem_active = false;
            lem_state = 0;
            Mode = robot->getMode();
            
            const std::vector<int>& target_offset = (Mode == SIMULATION) ? PS_OFFSET_SIMULATION : PS_OFFSET_REALITY;
            for (int i = 0; i < NB_DIST_SENS; i++) {
                ps_offset[i] = target_offset[i];
            }
            
            left_motor->setVelocity(0.0);
            right_motor->setVelocity(0.0);
            robot->step(TIME_STEP);
            std::cout << "\nSwitching to " << (Mode == SIMULATION ? "SIMULATION" : "REALITY") << " mode.\n" << std::endl;
        }
    }

    void readSensors() {
        for (int i = 0; i < NB_DIST_SENS; i++) {
            int val = static_cast<int>(ps[i]->getValue()) - ps_offset[i];
            ps_value[i] = (val < 0) ? 0 : val;
        }
        for (int i = 0; i < NB_GROUND_SENS; i++) {
            gs_value[i] = static_cast<unsigned short>(gs[i]->getValue());
        }
    }

    void LineFollowingModule() {
        int DeltaS = gs_value[GS_RIGHT] - gs_value[GS_LEFT];
        lfm_speed.left = 200 - static_cast<int>(0.4 * DeltaS);
        lfm_speed.right = 200 + static_cast<int>(0.4 * DeltaS);
    }

    void ObstacleAvoidanceModule() {
        int max_ds_value = 0;
        int activation_right = 0, activation_left = 0;

        if (oam_reset) {
            oam_active = false;
            oam_side = NO_SIDE;
        }
        oam_reset = false;

        for (int i = 0; i <= 1; i++) { // PS_RIGHT_00 y 45
            if (max_ds_value < ps_value[i]) max_ds_value = ps_value[i];
            activation_right += ps_value[i];
        }
        for (int i = 6; i <= 7; i++) { // PS_LEFT_45 y 00
            if (max_ds_value < ps_value[i]) max_ds_value = ps_value[i];
            activation_left += ps_value[i];
        }
        
        if (max_ds_value > 100) oam_active = true;

        if (oam_active && oam_side == NO_SIDE) {
            oam_side = (activation_right > activation_left) ? RIGHT : LEFT;
        }

        oam_speed = {150, 150};

        if (oam_active) {
            int DeltaS = 0;
            if (oam_side == LEFT) {
                DeltaS -= static_cast<int>(0.2 * ps_value[5] + 0.9 * ps_value[6] + 1.2 * ps_value[7]);
            } else {
                DeltaS += static_cast<int>(0.2 * ps_value[2] + 0.9 * ps_value[1] + 1.2 * ps_value[0]);
            }
            if (DeltaS > 600)  DeltaS = 600;
            if (DeltaS < -600) DeltaS = -600;

            oam_speed.left -= DeltaS;
            oam_speed.right += DeltaS;
        }
    }

    void LineLeavingModule(int side) {
        if (!llm_active && side != NO_SIDE && llm_past_side == NO_SIDE)
            llm_active = true;

        llm_past_side = side;

        if (llm_active) {
            int gs_side_index = (side == LEFT) ? GS_LEFT : GS_RIGHT;
            if ((gs_value[GS_CENTER] + gs_value[gs_side_index]) / 2 > LLM_THRESHOLD) {
                llm_active = false;
                llm_inibit_ofm_speed = false;
                lem_reset = true;
            } else {
                llm_inibit_ofm_speed = true;
            }
        }
    }

    void ObstacleFollowingModule(int side) {
        if (side != NO_SIDE) {
            ofm_active = true;
            ofm_speed = (side == LEFT) ? SpeedVector2D{-150, 150} : SpeedVector2D{150, -150};
        } else {
            ofm_active = false;
            ofm_speed = {0, 0};
        }
    }

    void LineEnteringModule(int side) {
        if (lem_reset) lem_state = 1;
        lem_reset = false;

        lem_speed = {100, 100};
        int Side = (side == LEFT) ? RIGHT : LEFT;
        int OpSide = (side == LEFT) ? LEFT : RIGHT;
        int GS_Side = (side == LEFT) ? GS_RIGHT : GS_LEFT;
        int GS_OpSide = (side == LEFT) ? GS_LEFT : GS_RIGHT;

        switch (lem_state) {
            case 0: // STANDBY
                lem_active = false;
                break;
            case 1: // LOOKING FOR LINE
                if (gs_value[GS_Side] < LEM_THRESHOLD) {
                    lem_active = true;
                    lem_state = 2;
                    cur_op_gs_value = (gs_value[GS_OpSide] < LEM_THRESHOLD) ? BLACK : WHITE;
                    lem_black_counter = (cur_op_gs_value == BLACK) ? 1 : 0;
                    prev_op_gs_value = cur_op_gs_value;
                }
                break;
            case 2: // LINE DETECTED
                cur_op_gs_value = (gs_value[GS_OpSide] < LEM_THRESHOLD) ? BLACK : WHITE;
                if (cur_op_gs_value == BLACK) lem_black_counter++;
                
                if (prev_op_gs_value == BLACK && cur_op_gs_value == WHITE) {
                    lem_state = 3;
                    lem_speed = {0, 0};
                } else {
                    prev_op_gs_value = cur_op_gs_value;
                    int factor = GS_WHITE - gs_value[GS_Side];
                    if (OpSide == LEFT) {
                        lem_speed.left = 100 + static_cast<int>(0.5 * factor);
                        lem_speed.right = 100 - static_cast<int>(0.5 * factor);
                    } else {
                        lem_speed.left = 100 - static_cast<int>(0.5 * factor);
                        lem_speed.right = 100 + static_cast<int>(0.5 * factor);
                    }
                }
                break;
            case 3: // ON LINE
                oam_reset = true;
                lem_active = false;
                lem_state = 0;
                break;
        }
    }

public:
    void run() {
        while (robot->step(TIME_STEP) != -1) {
            handleModeChange();
            readSensors();

            SpeedVector2D final_speed = {0, 0};

            // 1. Capa inferior: Seguimiento de línea
            LineFollowingModule();
            final_speed = lfm_speed;

            // 2. Capas intermedias: Evitación y seguimiento de obstáculos
            ObstacleAvoidanceModule();
            LineLeavingModule(oam_side);
            ObstacleFollowingModule(oam_side);

            if (llm_inibit_ofm_speed) {
                ofm_speed = {0, 0};
            }

            // Operación Vectorial de Suma Directa gracias a la sobrecarga del struct
            SpeedVector2D oam_ofm_speed = oam_speed + ofm_speed; 

            if (oam_active || ofm_active) {
                final_speed = oam_ofm_speed;
            }

            // 3. Capa superior: Re-entrada a línea
            LineEnteringModule(oam_side);
            if (lem_active) {
                final_speed = lem_speed;
            }

            // Display Debug
            std::printf("OAM %d side %d   LLM %d   OFM %d   LEM %d state %d\n", 
                        oam_active, oam_side, llm_active, ofm_active, lem_active, lem_state);

            // Aplicar velocidades físicas usando el vector final
            left_motor->setVelocity(0.00628 * final_speed.left);
            right_motor->setVelocity(0.00628 * final_speed.right);
        }
    }
};

int main() {
    EPuckController controller;
    controller.run();
    return 0;
}
