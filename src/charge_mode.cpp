#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>
#include <semphr.h>
#include <u8g2.h>
#include <math.h>

#include "app.h"
#include "FloatRingBuffer.h"
#include "mini_12864_module.h"
#include "display.h"
#include "scale.h"
#include "motors.h"
#include "charge_mode.h"
#include "eeprom.h"
#include "neopixel_led.h"
#include "profile.h"
#include "common.h"
#include "servo_gate.h"


uint8_t charge_weight_digits[] = {0, 0, 0, 0, 0};

charge_mode_config_t charge_mode_config;

// Scale related
extern scale_config_t scale_config;
extern servo_gate_t servo_gate;


const eeprom_charge_mode_data_t default_charge_mode_data = {
    .charge_mode_data_rev = 0,

    .coarse_stop_threshold = 5,
    .fine_stop_threshold = 0.03,

    .set_point_sd_margin = 0.02,
    .set_point_mean_margin = 0.02,
    .coarse_stop_gate_ratio = 0,   // NEW
    .decimal_places = DP_2,

    // Precharges
    .precharge_enable = false,
    .precharge_time_ms = 1000,
    .precharge_speed_rps = 2,

    // LED related
    .neopixel_normal_charge_colour = RGB_COLOUR_GREEN,        // green
    .neopixel_under_charge_colour = RGB_COLOUR_YELLOW,        // yellow
    .neopixel_over_charge_colour = RGB_COLOUR_RED,            // red
    .neopixel_not_ready_colour = RGB_COLOUR_BLUE,             // blue
};

// Valeur de validation "legacy" volontairement improbable : le bloc est ensuite valide par CRC32
#define EEPROM_CHARGE_TUNING_DATA_REV   0x5A17

const eeprom_charge_tuning_data_t default_charge_tuning_data = {
    .charge_tuning_data_rev = 0,

    .tolerance_over = 0.05,
    .tolerance_under = 0.05,

    .topup_enable = true,
    .topup_max_attempts = 5,
    .topup_speed_rps = 0,           // 0 = vitesse min du trickler fin du profil
    .topup_min_pulse_ms = 150,
    .topup_max_pulse_ms = 1500,
    .settle_timeout_ms = 6000,

    .learning_enable = true,
    .aim_bias = 0.02,
    .learning_max_offset = 0.5,
};


// ---------------------------------------------------------------------------
// Etat d'execution (memoire vive uniquement)
// ---------------------------------------------------------------------------

typedef enum {
    CHARGE_RESULT_OK = 0,
    CHARGE_RESULT_UNDER = 1,
    CHARGE_RESULT_OVER = 2,
} charge_result_t;

// Apprentissage du point d'arret (poudre "en vol")
static float learned_inflight = 0.0f;           // Moyenne glissante (unite balance)
static uint32_t learned_samples = 0;
static float topup_rate_per_ms = 0.0f;          // Poudre ajoutee par ms d'impulsion (0 = inconnu)

// Total de poudre consommee (depuis le demarrage ou le dernier effacement)
static float total_powder_weight = 0.0f;
static uint32_t total_powder_charges = 0;

// Le resultat de la derniere charge est affiche a l'ecran jusqu'a la charge suivante
static bool has_last_result = false;

// Resultat de la derniere charge (utilise pour la LED)
static float last_final_weight = 0.0f;
static charge_result_t last_charge_result = CHARGE_RESULT_OK;
static uint8_t last_charge_topups = 0;

// Historique des charges
#define CHARGE_HISTORY_SIZE 100

typedef struct {
    uint32_t seq;
    float target;
    float final_weight;
    float elapsed_s;
    float stop_offset;
    uint8_t topups;
    uint8_t result;
} charge_history_entry_t;

static charge_history_entry_t charge_history[CHARGE_HISTORY_SIZE];
static uint32_t charge_history_count = 0;       // Nombre d'entrees valides (<= CHARGE_HISTORY_SIZE)
static uint32_t charge_history_head = 0;        // Prochaine position d'ecriture
static uint32_t charge_history_seq = 0;         // Numero de la derniere charge


static void charge_history_add(float target, float final_weight, float elapsed_s, float stop_offset, uint8_t topups, charge_result_t result) {
    charge_history_entry_t entry;
    entry.seq = charge_history_seq + 1;
    entry.target = target;
    entry.final_weight = final_weight;
    entry.elapsed_s = elapsed_s;
    entry.stop_offset = stop_offset;
    entry.topups = topups;
    entry.result = (uint8_t) result;

    charge_history[charge_history_head] = entry;
    charge_history_head = (charge_history_head + 1) % CHARGE_HISTORY_SIZE;
    if (charge_history_count < CHARGE_HISTORY_SIZE) {
        charge_history_count += 1;
    }
    charge_history_seq += 1;
}

static void charge_history_clear(void) {
    total_powder_weight = 0.0f;
    total_powder_charges = 0;
    charge_history_count = 0;
    charge_history_head = 0;
    charge_history_seq = 0;
}


static charge_result_t classify_charge(float final_weight) {
    const float eps = 0.0005f;  // Tolere les erreurs d'arrondi des flottants
    float deviation = final_weight - charge_mode_config.target_charge_weight;

    if (deviation > charge_mode_config.eeprom_charge_tuning_data.tolerance_over - eps) {
        return CHARGE_RESULT_OVER;
    }
    if (deviation < -(charge_mode_config.eeprom_charge_tuning_data.tolerance_under - eps)) {
        return CHARGE_RESULT_UNDER;
    }
    return CHARGE_RESULT_OK;
}


// Variation minimale (en unite de la balance) pour corriger le resultat apres coup
#define CUP_WATCH_MIN_CHANGE 0.015f

static void charge_mode_apply_result_led(void);

// Corrige le poids final de la derniere charge (poudre tombee apres la mesure)
static void charge_mode_update_final_weight(float new_weight) {
    float delta = new_weight - last_final_weight;
    charge_result_t old_result = last_charge_result;

    last_final_weight = new_weight;
    last_charge_result = classify_charge(new_weight);

    // La poudre arrivee en retard fait partie de la poudre "en vol" : on corrige l'apprentissage
    // (seulement si la charge n'a pas eu de complement, sinon la poudre vient des impulsions).
    eeprom_charge_tuning_data_t * tuning = &charge_mode_config.eeprom_charge_tuning_data;
    if (tuning->learning_enable && last_charge_topups == 0 && learned_samples > 0 && fabsf(delta) < 0.2f) {
        learned_inflight = fminf(fmaxf(0.0f, learned_inflight + 0.3f * delta), tuning->learning_max_offset);
        profile_extra_t * extra = profile_get_selected_extra();
        extra->learned_inflight = learned_inflight;
        profile_extra_data_save();
    }

    // Met a jour la derniere entree de l'historique et le total de poudre
    if (charge_history_count > 0) {
        uint32_t last_idx = (charge_history_head + CHARGE_HISTORY_SIZE - 1) % CHARGE_HISTORY_SIZE;
        if (charge_history[last_idx].seq == charge_history_seq) {
            charge_history[last_idx].final_weight = new_weight;
            charge_history[last_idx].result = (uint8_t) last_charge_result;
        }
    }
    if (total_powder_charges > 0) {
        total_powder_weight += delta;
    }

    if (last_charge_result != old_result) {
        charge_mode_apply_result_led();
    }
}


// Configures
TaskHandle_t scale_measurement_render_task_handler = NULL;
static char title_string[30];

static TickType_t charge_start_tick = 0;
static float last_charge_elapsed_seconds = 0.0f;

// Menu system
extern AppState_t exit_state;
extern QueueHandle_t encoder_event_queue;
extern neopixel_led_config_t neopixel_led_config;


// Definitions
typedef enum {
    CHARGE_MODE_EVENT_NO_EVENT = (1 << 0),
    CHARGE_MODE_EVENT_UNDER_CHARGE = (1 << 1),
    CHARGE_MODE_EVENT_OVER_CHARGE = (1 << 2),
} ChargeModeEventBit_t;


static void format_elapsed_time(char *buffer, size_t len, TickType_t start_tick) {
    TickType_t now = xTaskGetTickCount();
    uint32_t elapsed_ticks = now - start_tick;

    // Tick to milliseconds
    float elapsed_seconds = (float)(elapsed_ticks * portTICK_PERIOD_MS) / 1000.0f;

    snprintf(buffer, len, "%.2f s", elapsed_seconds);
}


void scale_measurement_render_task(void *p) {
    char current_weight_string[WEIGHT_STRING_LEN];
    char time_buffer[16];

    u8g2_t *display_handler = get_display_handler();

    while (true) {
        TickType_t last_render_tick = xTaskGetTickCount();

        u8g2_ClearBuffer(display_handler);

        // Set font for title and timer
        u8g2_SetFont(display_handler, u8g2_font_helvB08_tr);

        // Format the timer string based on current state
        if (charge_mode_config.charge_mode_state == CHARGE_MODE_WAIT_FOR_COMPLETE) {
            format_elapsed_time(time_buffer, sizeof(time_buffer), charge_start_tick);
        } else if (charge_mode_config.charge_mode_state == CHARGE_MODE_WAIT_FOR_CUP_REMOVAL ||
                   charge_mode_config.charge_mode_state == CHARGE_MODE_WAIT_FOR_CUP_RETURN ||
                   charge_mode_config.charge_mode_state == CHARGE_MODE_WAIT_FOR_ZERO) {
            snprintf(time_buffer, sizeof(time_buffer), "%.2f s", last_charge_elapsed_seconds);
        } else {
            snprintf(time_buffer, sizeof(time_buffer), "--.- s");
        }

        // Calculate x positions
        uint8_t screen_width = u8g2_GetDisplayWidth(display_handler);
        uint8_t time_width = u8g2_GetStrWidth(display_handler, time_buffer);

        // Draw title on left
        u8g2_DrawStr(display_handler, 5, 10, title_string);

        // Draw timer on right edge
        u8g2_DrawStr(display_handler, screen_width - time_width - 5, 10, time_buffer);  // 5 px padding from edge

        // Draw line under title
        u8g2_DrawHLine(display_handler, 0, 13, screen_width);

        // Current weight (only show values > -1.0)
        memset(current_weight_string, 0x0, sizeof(current_weight_string));
        float scale_measurement = scale_get_current_measurement();
        if (scale_measurement > -1.0) {
            float_to_string(current_weight_string, scale_measurement, charge_mode_config.eeprom_charge_mode_data.decimal_places);
        } else {
            strcpy(current_weight_string, "---");
        }

        // Draw current weight value
        u8g2_SetFont(display_handler, u8g2_font_profont22_tf);
        u8g2_DrawStr(display_handler, 26, 35, current_weight_string);

        // Draw profile name
        profile_t *current_profile = profile_get_selected();
        u8g2_SetFont(display_handler, u8g2_font_helvR08_tr);
        u8g2_DrawStr(display_handler, 5, 61, current_profile->name);

        // Resultat de la derniere charge (petite ligne sous le poids, le poids reste en gros)
        if (has_last_result && charge_mode_config.charge_mode_state != CHARGE_MODE_WAIT_FOR_COMPLETE) {
            char result_string[24];
            float deviation = last_final_weight - charge_mode_config.target_charge_weight;
            if (charge_mode_config.eeprom_charge_mode_data.decimal_places == DP_3) {
                snprintf(result_string, sizeof(result_string), "Ecart %+.3f", deviation);
            }
            else {
                snprintf(result_string, sizeof(result_string), "Ecart %+.2f", deviation);
            }
            u8g2_DrawStr(display_handler, 5, 49, result_string);

            const char * verdict = "OK";
            if (last_charge_result == CHARGE_RESULT_OVER) {
                verdict = "TROP HAUT";
            }
            else if (last_charge_result == CHARGE_RESULT_UNDER) {
                verdict = "TROP BAS";
            }
            uint8_t verdict_width = u8g2_GetStrWidth(display_handler, verdict);
            u8g2_DrawStr(display_handler, screen_width - verdict_width - 5, 49, verdict);
        }

        u8g2_SendBuffer(display_handler);

        vTaskDelayUntil(&last_render_tick, pdMS_TO_TICKS(20));
    }
}


void charge_mode_wait_for_zero() {
    // Set colour to not ready
    neopixel_led_set_colour(
        neopixel_led_config.eeprom_neopixel_led_metadata.default_led_colours.mini12864_backlight_colour,
        charge_mode_config.eeprom_charge_mode_data.neopixel_not_ready_colour, 
        charge_mode_config.eeprom_charge_mode_data.neopixel_not_ready_colour, 
        true
    );
    
    // Wait for 5 measurements and wait for stable
    FloatRingBuffer data_buffer(10);

    // Update current status
    snprintf(title_string, sizeof(title_string), "Waiting for Zero");

    // Stop condition: 10 stable measurements in 200ms apart (2 seconds minimum)
    while (true) {
        TickType_t last_measurement_tick = xTaskGetTickCount();

        // Non block waiting for the input
        ButtonEncoderEvent_t button_encoder_event = button_wait_for_input(false);
        if (button_encoder_event == BUTTON_RST_PRESSED) {
            charge_mode_config.charge_mode_state = CHARGE_MODE_EXIT;
            return;
        }
        else if (button_encoder_event == BUTTON_ENCODER_PRESSED) {
            scale_config.scale_handle->force_zero();
        }

        // Perform measurement (max delay 300 seconds   )
        float current_measurement;
        if (scale_block_wait_for_next_measurement(300, &current_measurement)){
            data_buffer.enqueue(current_measurement);
        }

        // Generate stop condition
        if (data_buffer.getCounter() >= 10){
            if (data_buffer.getSd() < charge_mode_config.eeprom_charge_mode_data.set_point_sd_margin && 
                abs(data_buffer.getMean()) < charge_mode_config.eeprom_charge_mode_data.set_point_mean_margin) {
                break;
            }
        }

        // Wait for minimum 300 ms (but can skip if previously wait already)
        vTaskDelayUntil(&last_measurement_tick, pdMS_TO_TICKS(300));
    }

    charge_mode_config.charge_mode_state = CHARGE_MODE_WAIT_FOR_COMPLETE;
}

typedef enum {
    SETTLE_STABLE = 0,      // Balance stable
    SETTLE_TIMEOUT = 1,     // Pas stable dans le temps imparti (on utilise la moyenne)
    SETTLE_ABORTED = 2,     // L'utilisateur a demande l'arret
} settle_result_t;

/*
 * Attend que la balance soit stable : 5 mesures consecutives avec un ecart-type
 * inferieur a "Set Point Accuracy (Standard Deviation)".
 * Retourne la moyenne de ces mesures dans *stable_weight.
 */
// Le poids est considere stable seulement s'il n'a pas bouge pendant STABLE_WINDOW_MS :
// une poudre qui tombe lentement (quelques grains par seconde) n'est plus prise pour un poids stable.
#define STABLE_WINDOW_MS        1500
#define STABLE_MAX_SAMPLES      64
#define STABLE_MIN_RANGE        0.016f   // ~1 graduation de FX-120i en grains (scintillement toléré)
#define STABLE_MAX_DRIFT        0.012f   // derive max entre debut et fin de fenetre (< 1 graduation)

static settle_result_t wait_for_stable_weight(float * stable_weight, uint32_t timeout_ms) {
    static float weights[STABLE_MAX_SAMPLES];
    static TickType_t ticks[STABLE_MAX_SAMPLES];
    size_t count = 0;

    TickType_t start_tick = xTaskGetTickCount();
    float last_weight = scale_get_current_measurement();
    float sd_margin = charge_mode_config.eeprom_charge_mode_data.set_point_sd_margin;
    float max_range = fmaxf(2.0f * sd_margin, STABLE_MIN_RANGE);

    while (true) {
        ButtonEncoderEvent_t button_encoder_event = button_wait_for_input(false);
        if (button_encoder_event == BUTTON_RST_PRESSED) {
            return SETTLE_ABORTED;
        }

        float current_weight;
        if (scale_block_wait_for_next_measurement(200, &current_weight)) {
            TickType_t now = xTaskGetTickCount();
            last_weight = current_weight;

            // Ajoute la mesure (on retire la plus ancienne si le tableau est plein)
            if (count == STABLE_MAX_SAMPLES) {
                memmove(&weights[0], &weights[1], (STABLE_MAX_SAMPLES - 1) * sizeof(float));
                memmove(&ticks[0], &ticks[1], (STABLE_MAX_SAMPLES - 1) * sizeof(TickType_t));
                count -= 1;
            }
            weights[count] = current_weight;
            ticks[count] = now;
            count += 1;

            // On ne garde que les mesures de la fenetre (en gardant celle qui couvre son debut)
            size_t first = 0;
            while (first + 1 < count &&
                   (now - ticks[first + 1]) * portTICK_PERIOD_MS >= STABLE_WINDOW_MS) {
                first += 1;
            }
            if (first > 0) {
                memmove(&weights[0], &weights[first], (count - first) * sizeof(float));
                memmove(&ticks[0], &ticks[first], (count - first) * sizeof(TickType_t));
                count -= first;
            }

            uint32_t covered_ms = (now - ticks[0]) * portTICK_PERIOD_MS;
            if (count >= 5 && covered_ms >= STABLE_WINDOW_MS) {
                float w_min = weights[0], w_max = weights[0];
                for (size_t i = 1; i < count; i++) {
                    w_min = fminf(w_min, weights[i]);
                    w_max = fmaxf(w_max, weights[i]);
                }

                // Moyenne du premier et du dernier tiers pour detecter une lente montee
                size_t third = count / 3;
                if (third < 1) {
                    third = 1;
                }
                float head = 0.0f, tail = 0.0f;
                for (size_t i = 0; i < third; i++) {
                    head += weights[i];
                    tail += weights[count - 1 - i];
                }
                head /= third;
                tail /= third;

                if ((w_max - w_min) <= max_range && fabsf(tail - head) <= STABLE_MAX_DRIFT) {
                    *stable_weight = tail;
                    return SETTLE_STABLE;
                }
            }
        }

        uint32_t elapsed_ms = (xTaskGetTickCount() - start_tick) * portTICK_PERIOD_MS;
        if (elapsed_ms >= timeout_ms) {
            *stable_weight = last_weight;
            return SETTLE_TIMEOUT;
        }
    }
}


/*
 * Apres l'arret des moteurs : attend la stabilisation, apprend la poudre "en vol",
 * puis complete par petites impulsions du trickler fin jusqu'a la zone verte.
 * Retourne false si l'utilisateur a demande l'arret.
 */
static bool charge_mode_settle_and_topup(float weight_at_stop, float fine_min_speed, profile_extra_t * extra, uint8_t * topups_done) {
    eeprom_charge_tuning_data_t * tuning = &charge_mode_config.eeprom_charge_tuning_data;
    float target = charge_mode_config.target_charge_weight;
    *topups_done = 0;

    // Laisser retomber la poudre encore en vol
    snprintf(title_string, sizeof(title_string), "Settling...");
    vTaskDelay(pdMS_TO_TICKS(300));

    float stable_weight;
    if (wait_for_stable_weight(&stable_weight, tuning->settle_timeout_ms) == SETTLE_ABORTED) {
        return false;
    }

    // Apprentissage : quantite arrivee apres l'arret des moteurs
    if (tuning->learning_enable) {
        float inflight_sample = stable_weight - weight_at_stop;

        // On ignore les valeurs aberrantes (lecture parasite, coupelle touchee...)
        if (inflight_sample > -0.02f && inflight_sample < tuning->learning_max_offset + 0.1f) {
            inflight_sample = fmaxf(0.0f, inflight_sample);
            if (learned_samples == 0) {
                learned_inflight = inflight_sample;
            }
            else {
                learned_inflight = 0.7f * learned_inflight + 0.3f * inflight_sample;
            }
            learned_samples += 1;
        }
    }

    // Complement automatique
    if (tuning->topup_enable) {
        float topup_speed = tuning->topup_speed_rps > 0 ? tuning->topup_speed_rps : fine_min_speed;
        uint32_t fallback_pulse_ms = tuning->topup_min_pulse_ms;
        // Le tube fin a recule a l'arret : il revient une seule fois avant la premiere impulsion,
        // ne recule pas entre les impulsions, et recule une seule fois a la fin.
        bool fine_retracted = extra->fine_retract_rev > 0;
        bool fine_moved_forward = false;

        while (*topups_done < tuning->topup_max_attempts &&
               classify_charge(stable_weight) == CHARGE_RESULT_UNDER) {

            float missing = target - stable_weight;

            // On vise le milieu de la marge basse pour ne pas depasser
            float wanted = fmaxf(missing - 0.5f * tuning->tolerance_under, 0.25f * missing);

            uint32_t pulse_ms;
            if (topup_rate_per_ms > 0) {
                pulse_ms = (uint32_t) (wanted / topup_rate_per_ms);
            }
            else {
                pulse_ms = fallback_pulse_ms;
            }
            if (pulse_ms < tuning->topup_min_pulse_ms) {
                pulse_ms = tuning->topup_min_pulse_ms;
            }
            if (pulse_ms > tuning->topup_max_pulse_ms) {
                pulse_ms = tuning->topup_max_pulse_ms;
            }

            *topups_done += 1;
            snprintf(title_string, sizeof(title_string), "Top-up %u/%lu",
                     (unsigned) *topups_done, (unsigned long) tuning->topup_max_attempts);

            // Avant la premiere impulsion seulement : on ramene le tube fin en position
            if (fine_retracted) {
                motor_rotate_blocking(SELECT_FINE_TRICKLER_MOTOR, extra->fine_retract_rev, extra->retract_speed_rps);
                fine_retracted = false;
                fine_moved_forward = true;
            }

            // Impulsion du trickler fin
            motor_set_speed(SELECT_FINE_TRICKLER_MOTOR, topup_speed);
            vTaskDelay(pdMS_TO_TICKS(pulse_ms));
            motor_set_speed(SELECT_FINE_TRICKLER_MOTOR, 0);
            vTaskDelay(pdMS_TO_TICKS(300));

            float previous_weight = stable_weight;
            if (wait_for_stable_weight(&stable_weight, tuning->settle_timeout_ms) == SETTLE_ABORTED) {
                return false;
            }

            // Apprentissage du debit des impulsions
            float gained = stable_weight - previous_weight;
            if (gained > 0.005f) {
                float rate_sample = gained / (float) pulse_ms;
                if (topup_rate_per_ms > 0) {
                    topup_rate_per_ms = 0.6f * topup_rate_per_ms + 0.4f * rate_sample;
                }
                else {
                    topup_rate_per_ms = rate_sample;
                }
            }
            else {
                // Rien n'est tombe : impulsion suivante plus longue
                if (topup_rate_per_ms > 0) {
                    topup_rate_per_ms *= 0.5f;
                }
                fallback_pulse_ms = pulse_ms * 2;
            }
        }

        // Fin des impulsions : un seul recul du tube fin
        if (fine_moved_forward) {
            motor_rotate_blocking(SELECT_FINE_TRICKLER_MOTOR, -extra->fine_retract_rev, extra->retract_speed_rps);
        }
    }

    last_final_weight = stable_weight;
    last_charge_result = classify_charge(stable_weight);
    return true;
}


void charge_mode_wait_for_complete() {

    charge_start_tick = xTaskGetTickCount();
    has_last_result = false;

    // Set colour to under charge
    neopixel_led_set_colour(
        neopixel_led_config.eeprom_neopixel_led_metadata.default_led_colours.mini12864_backlight_colour,
        charge_mode_config.eeprom_charge_mode_data.neopixel_under_charge_colour, 
        charge_mode_config.eeprom_charge_mode_data.neopixel_under_charge_colour, 
        true
    );

    // If the servo gate is used then it has to be opened
    if (servo_gate.eeprom_servo_gate_config.servo_gate_enable) {
        servo_gate_set_ratio(SERVO_GATE_RATIO_OPEN, false);
    }
    // Update current status
    char target_weight_string[WEIGHT_STRING_LEN];
    float_to_string(target_weight_string, charge_mode_config.target_charge_weight, charge_mode_config.eeprom_charge_mode_data.decimal_places);

    snprintf(title_string, sizeof(title_string), 
             "Target: %s", 
             target_weight_string);

    // Read trickling parameter from the current profile
    profile_t * current_profile = profile_get_selected();

    // Find the minimum of max speed from the motor and the profile
    float coarse_trickler_max_speed = fmin(get_motor_max_speed(SELECT_COARSE_TRICKLER_MOTOR),
                                           current_profile->coarse_max_flow_speed_rps);
    float coarse_trickler_min_speed = fmax(get_motor_min_speed(SELECT_COARSE_TRICKLER_MOTOR),
                                           current_profile->coarse_min_flow_speed_rps);
    float fine_trickler_max_speed = fmin(get_motor_max_speed(SELECT_FINE_TRICKLER_MOTOR),
                                         current_profile->fine_max_flow_speed_rps);
    float fine_trickler_min_speed = fmax(get_motor_min_speed(SELECT_FINE_TRICKLER_MOTOR),
                                         current_profile->fine_min_flow_speed_rps);

    // Define PID terms
    float coarse_trickler_integral = 0.0f;
    float fine_trickler_integral = 0.0f;
    float coarse_trickler_last_error = 0.0f;
    float fine_trickler_last_error = 0.0f;

    // Calculate target weight for coarse trickler
    // The coarse trickler is suppose to stop ahead of the target weight by an offset
    float coarse_trickler_target_charge_weight = fmaxf(0.0f, charge_mode_config.target_charge_weight - charge_mode_config.eeprom_charge_mode_data.coarse_stop_threshold);

    // Point d'arret du trickler fin : appris automatiquement, sinon "Fine Trickler Stop Threshold"
    eeprom_charge_tuning_data_t * tuning = &charge_mode_config.eeprom_charge_tuning_data;
    // Apprentissage memorise pour ce profil (conserve apres extinction)
    profile_extra_t * extra = profile_get_selected_extra();
    learned_inflight = extra->learned_inflight;
    learned_samples = extra->learned_samples;
    topup_rate_per_ms = extra->topup_rate_per_ms;
    if (!isfinite(learned_inflight) || !isfinite(topup_rate_per_ms)) {
        learned_inflight = 0.0f;
        learned_samples = 0;
        topup_rate_per_ms = 0.0f;
    }

    // Memorise la cible pour ce profil (proposee a la prochaine utilisation)
    bool extra_changed = false;
    if (fabsf(extra->last_target_weight - charge_mode_config.target_charge_weight) > 0.0005f) {
        extra->last_target_weight = charge_mode_config.target_charge_weight;
        extra_changed = true;
    }

    float fine_stop_offset = charge_mode_config.eeprom_charge_mode_data.fine_stop_threshold;
    if (tuning->learning_enable && learned_samples > 0) {
        fine_stop_offset = fminf(fmaxf(0.0f, learned_inflight + tuning->aim_bias), tuning->learning_max_offset);
    }
    float weight_at_stop = 0.0f;
    uint8_t stop_confirm_count = 0;     // Filtre anti-lecture parasite

    TickType_t last_sample_tick = xTaskGetTickCount();
    TickType_t current_sample_tick = last_sample_tick;
    bool should_coarse_trickler_move = true;

    while (true) {
        // Non block waiting for the input
        ButtonEncoderEvent_t button_encoder_event = button_wait_for_input(false);
        if (button_encoder_event == BUTTON_RST_PRESSED) {
            charge_mode_config.charge_mode_state = CHARGE_MODE_EXIT;
            return;
        }

        // Run the PID controlled loop to start charging
        // Perform the measurement
        float current_weight;
        if (!scale_block_wait_for_next_measurement(200, &current_weight)) {
            // If no measurement within 200ms then poll the button and retry
            continue;
        }
        current_sample_tick = xTaskGetTickCount();

        float coarse_trickler_error = coarse_trickler_target_charge_weight - current_weight;
        float fine_trickler_error = charge_mode_config.target_charge_weight - current_weight;

        // Fine & Coarse trickler stop condition
        // Il faut 2 lectures de suite au-dessus du point d'arret : une lecture parasite isolee
        // (vibration, grain qui frappe le plateau) ne doit pas arreter la charge.
        if (fine_trickler_error < fine_stop_offset) {
            stop_confirm_count += 1;
            if (stop_confirm_count < 2) {
                continue;  // On garde la vitesse actuelle et on attend la lecture suivante
            }

            // Stop all motors
            motor_set_speed(SELECT_FINE_TRICKLER_MOTOR, 0);
            motor_set_speed(SELECT_COARSE_TRICKLER_MOTOR, 0);

            weight_at_stop = current_weight;

            // Recul des tubes pour retenir la poudre au bord
            if (should_coarse_trickler_move && extra->coarse_retract_rev > 0) {
                motor_rotate_blocking(SELECT_COARSE_TRICKLER_MOTOR, -extra->coarse_retract_rev, extra->retract_speed_rps);
            }
            if (extra->fine_retract_rev > 0) {
                motor_rotate_blocking(SELECT_FINE_TRICKLER_MOTOR, -extra->fine_retract_rev, extra->retract_speed_rps);
            }
            break;
        }
        stop_confirm_count = 0;

        // Coarse trickler stop condition
        if (coarse_trickler_error <= 0 &&
                 should_coarse_trickler_move) {

            should_coarse_trickler_move = false;
            motor_set_speed(SELECT_COARSE_TRICKLER_MOTOR, 0);

            // NEW: When the coarse trickler stops, move the servo gate to a configured ratio
            // Ratio convention: 0.0 = open, 1.0 = close
            if (servo_gate.eeprom_servo_gate_config.servo_gate_enable) {
                float r = charge_mode_config.eeprom_charge_mode_data.coarse_stop_gate_ratio;

                servo_gate_set_ratio(r, false); // don't block the charge loop
            }

            // Recul du tube principal pour retenir la poudre au bord
            if (extra->coarse_retract_rev > 0) {
                motor_rotate_blocking(SELECT_COARSE_TRICKLER_MOTOR, -extra->coarse_retract_rev, extra->retract_speed_rps);
            }
        }
    

        // Update fine trickler speed
        float elapse_time_ms = (current_sample_tick - last_sample_tick) / portTICK_RATE_MS;
        fine_trickler_integral += fine_trickler_error;
        float fine_trickler_derivative = (fine_trickler_error - fine_trickler_last_error) / elapse_time_ms;

        // Update fine trickler speed
        float new_p = current_profile->fine_kp * fine_trickler_error;
        float new_i = current_profile->fine_ki * fine_trickler_integral;
        float new_d = current_profile->fine_kd * fine_trickler_derivative;
        float new_speed = fmaxf(fine_trickler_min_speed, fminf(new_p + new_i + new_d, fine_trickler_max_speed));
        motor_set_speed(SELECT_FINE_TRICKLER_MOTOR, new_speed);

        // Update coarse trickler speed
        if (should_coarse_trickler_move) {
            coarse_trickler_integral += coarse_trickler_error;
            float coarse_trickler_derivative = (coarse_trickler_error - coarse_trickler_last_error) / elapse_time_ms;

            new_p = current_profile->coarse_kp * coarse_trickler_error;
            new_i = current_profile->coarse_ki * coarse_trickler_integral;
            new_d = current_profile->coarse_kd * coarse_trickler_derivative ;

            new_speed = fmaxf(coarse_trickler_min_speed, fminf(new_p + new_i + new_d, coarse_trickler_max_speed));

            motor_set_speed(SELECT_COARSE_TRICKLER_MOTOR, new_speed);
        }

        // Record state
        last_sample_tick = current_sample_tick;
        fine_trickler_last_error = fine_trickler_error;
        coarse_trickler_last_error = coarse_trickler_error;
    }

    // Stabilisation, apprentissage et complement automatique (avant de fermer la gate)
    uint8_t topups_done = 0;
    if (!charge_mode_settle_and_topup(weight_at_stop, fine_trickler_min_speed, extra, &topups_done)) {
        motor_set_speed(SELECT_FINE_TRICKLER_MOTOR, 0);
        charge_mode_config.charge_mode_state = CHARGE_MODE_EXIT;
        return;
    }

    last_charge_topups = topups_done;

    // Stop the timer
    TickType_t now = xTaskGetTickCount();
    TickType_t elapsed_ticks = now - charge_start_tick;
    last_charge_elapsed_seconds = (float)(elapsed_ticks * portTICK_PERIOD_MS) / 1000.0f;

    // Historique et total de poudre
    charge_history_add(charge_mode_config.target_charge_weight, last_final_weight,
                       last_charge_elapsed_seconds, fine_stop_offset, topups_done, last_charge_result);
    if (last_final_weight > 0) {
        total_powder_weight += last_final_weight;
        total_powder_charges += 1;
    }
    has_last_result = true;

    // Sauvegarde de l'apprentissage pour ce profil
    if (extra->learned_inflight != learned_inflight ||
        extra->learned_samples != learned_samples ||
        extra->topup_rate_per_ms != topup_rate_per_ms) {
        extra->learned_inflight = learned_inflight;
        extra->learned_samples = learned_samples;
        extra->topup_rate_per_ms = topup_rate_per_ms;
        extra_changed = true;
    }
    if (extra_changed) {
        profile_extra_data_save();
    }

    // Close the gate if the servo gate is present
    if (servo_gate.eeprom_servo_gate_config.servo_gate_enable) {
    servo_gate_set_ratio(SERVO_GATE_RATIO_CLOSED, true);
    }

    // Precharge
    if (charge_mode_config.eeprom_charge_mode_data.precharge_enable &&
    servo_gate.eeprom_servo_gate_config.servo_gate_enable) {
        // Set a fixed delay between closing the gate and precharge to allow the gate to fully close
        vTaskDelay(pdMS_TO_TICKS(500));

        // Start the pre-charge
        motor_set_speed(SELECT_COARSE_TRICKLER_MOTOR, charge_mode_config.eeprom_charge_mode_data.precharge_speed_rps);
        vTaskDelay(pdMS_TO_TICKS(charge_mode_config.eeprom_charge_mode_data.precharge_time_ms));

        motor_set_speed(SELECT_COARSE_TRICKLER_MOTOR, 0);
    }
    else {
        vTaskDelay(pdMS_TO_TICKS(20));  // Wait for other tasks to complete  
    }

    charge_mode_config.charge_mode_state = CHARGE_MODE_WAIT_FOR_CUP_REMOVAL;
}

// Allume la LED et met a jour les drapeaux selon le resultat de la derniere charge
static void charge_mode_apply_result_led(void) {
    // Update LED colour before moving to the next stage
    // Over charged
    if (last_charge_result == CHARGE_RESULT_OVER) {
        neopixel_led_set_colour(
            neopixel_led_config.eeprom_neopixel_led_metadata.default_led_colours.mini12864_backlight_colour,
            charge_mode_config.eeprom_charge_mode_data.neopixel_over_charge_colour,
            charge_mode_config.eeprom_charge_mode_data.neopixel_over_charge_colour,
            true
        );

        // Set over charge
        charge_mode_config.charge_mode_event &= ~CHARGE_MODE_EVENT_UNDER_CHARGE;
        charge_mode_config.charge_mode_event |= CHARGE_MODE_EVENT_OVER_CHARGE;
    }
    // Under charged : hors tolerance aussi -> meme couleur que la surcharge (rouge)
    // La couleur "Under Charge" reste utilisee pendant la charge en cours.
    else if (last_charge_result == CHARGE_RESULT_UNDER) {
        neopixel_led_set_colour(
            neopixel_led_config.eeprom_neopixel_led_metadata.default_led_colours.mini12864_backlight_colour,
            charge_mode_config.eeprom_charge_mode_data.neopixel_over_charge_colour,
            charge_mode_config.eeprom_charge_mode_data.neopixel_over_charge_colour,
            true
        );

        // Set under charge flag
        charge_mode_config.charge_mode_event &= ~CHARGE_MODE_EVENT_OVER_CHARGE;
        charge_mode_config.charge_mode_event |= CHARGE_MODE_EVENT_UNDER_CHARGE;

    }
    // Normal
    else {
        neopixel_led_set_colour(
            neopixel_led_config.eeprom_neopixel_led_metadata.default_led_colours.mini12864_backlight_colour, 
            charge_mode_config.eeprom_charge_mode_data.neopixel_normal_charge_colour, 
            charge_mode_config.eeprom_charge_mode_data.neopixel_normal_charge_colour, 
            true
        );

        // Clear over and under charge bit
        charge_mode_config.charge_mode_event &= ~(CHARGE_MODE_EVENT_UNDER_CHARGE | CHARGE_MODE_EVENT_OVER_CHARGE);
    }
}


void charge_mode_wait_for_cup_removal() {
    // Update current status
    snprintf(title_string, sizeof(title_string), "Remove Cup");

    FloatRingBuffer data_buffer(5);

    // Post charge analysis: le poids final stable a deja ete mesure (apres complement eventuel)
    // et compare aux tolerances "Precision de charge".

    charge_mode_apply_result_led();

    // Stop condition: 5 stable measurements in 300ms apart (1.5 seconds minimum)
    while (true) {
        TickType_t last_sample_tick = xTaskGetTickCount();

        // Non block waiting for the input
        ButtonEncoderEvent_t button_encoder_event = button_wait_for_input(false);
        if (button_encoder_event == BUTTON_RST_PRESSED) {
            charge_mode_config.charge_mode_state = CHARGE_MODE_EXIT;
            return;
        }

        // Perform measurement
        float current_weight;
        if (!scale_block_wait_for_next_measurement(200, &current_weight)) {
            // If no measurement within 200ms then poll the button and retry
            continue;
        }
        data_buffer.enqueue(current_weight);

        // Generate stop condition
        if (data_buffer.getCounter() >= 5) {
            if (data_buffer.getSd() < charge_mode_config.eeprom_charge_mode_data.set_point_sd_margin && 
                data_buffer.getMean() + 10 < charge_mode_config.eeprom_charge_mode_data.set_point_mean_margin){
                break;
            }

            // Suivi du poids tant que la coupelle est sur le plateau : si de la poudre tombe en retard,
            // le resultat (ecran, LED, historique) est corrige avec le vrai poids final.
            float stable_mean = data_buffer.getMean();
            if (has_last_result &&
                data_buffer.getSd() < charge_mode_config.eeprom_charge_mode_data.set_point_sd_margin &&
                stable_mean > charge_mode_config.target_charge_weight * 0.5f &&
                fabsf(stable_mean - last_final_weight) > CUP_WATCH_MIN_CHANGE) {
                charge_mode_update_final_weight(stable_mean);
            }
        }

        // Wait for next measurement
        vTaskDelayUntil(&last_sample_tick, pdMS_TO_TICKS(300));
    }

    // Reset LED to default colour
    neopixel_led_set_colour(neopixel_led_config.eeprom_neopixel_led_metadata.default_led_colours.mini12864_backlight_colour,
                            neopixel_led_config.eeprom_neopixel_led_metadata.default_led_colours.led1_colour,
                            neopixel_led_config.eeprom_neopixel_led_metadata.default_led_colours.led2_colour,
                            true);

    charge_mode_config.charge_mode_state = CHARGE_MODE_WAIT_FOR_CUP_RETURN;
}

void charge_mode_wait_for_cup_return() { 
    // Set colour to not ready
    neopixel_led_set_colour(
        neopixel_led_config.eeprom_neopixel_led_metadata.default_led_colours.mini12864_backlight_colour, 
        charge_mode_config.eeprom_charge_mode_data.neopixel_not_ready_colour, 
        charge_mode_config.eeprom_charge_mode_data.neopixel_not_ready_colour, 
        true
    );

    snprintf(title_string, sizeof(title_string), "Return Cup");


    FloatRingBuffer data_buffer(5);

    while (true) {
        TickType_t last_sample_tick = xTaskGetTickCount();

        // Non block waiting for the input
        ButtonEncoderEvent_t button_encoder_event = button_wait_for_input(false);
        if (button_encoder_event == BUTTON_RST_PRESSED) {
            charge_mode_config.charge_mode_state = CHARGE_MODE_EXIT;
            return;
        }
        else if (button_encoder_event == BUTTON_ENCODER_PRESSED) {
            scale_config.scale_handle->force_zero();
        }

        // Perform measurement
        float current_weight;
        if (!scale_block_wait_for_next_measurement(200, &current_weight)) {
            // If no measurement within 200ms then poll the button and retry
            continue;
        }

        if (current_weight >= 0) {
            break;
        }

        // Wait for next measurement
        vTaskDelayUntil(&last_sample_tick, pdMS_TO_TICKS(20));
    }

    charge_mode_config.charge_mode_state = CHARGE_MODE_WAIT_FOR_ZERO;
}


// Pre-remplit les chiffres du menu "Select Charge Weight" avec la derniere cible du profil
void charge_mode_load_last_target_digits(void) {
    float target = profile_get_selected_extra()->last_target_weight;
    if (!(target > 0.0f) || !isfinite(target)) {
        return;
    }

    // Conversion en entier selon le nombre de decimales (2 dp : centiemes, 3 dp : milliemes)
    uint32_t value;
    if (charge_mode_config.eeprom_charge_mode_data.decimal_places == DP_3) {
        value = (uint32_t) lroundf(target * 1000.0f);
    }
    else {
        value = (uint32_t) lroundf(target * 100.0f);
    }
    if (value > 99999) {
        return;  // Ne tient pas sur 5 chiffres
    }
    for (int i = 0; i < 5; i++) {
        charge_weight_digits[i] = value % 10;
        value /= 10;
    }
}


uint8_t charge_mode_menu(bool charge_mode_skip_user_input) {
    // Create target weight, if the charge mode weight is built by charge_weight_digits
    if (!charge_mode_skip_user_input) {
        switch (charge_mode_config.eeprom_charge_mode_data.decimal_places) {
            case DP_2:
                charge_mode_config.target_charge_weight = charge_weight_digits[4] * 100 + \
                                                charge_weight_digits[3] * 10 + \
                                                charge_weight_digits[2] * 1 + \
                                                charge_weight_digits[1] * 0.1 + \
                                                charge_weight_digits[0] * 0.01;
                break;
            case DP_3:
                charge_mode_config.target_charge_weight = charge_weight_digits[4] * 10 + \
                                                charge_weight_digits[3] * 1 + \
                                                charge_weight_digits[2] * 0.1 + \
                                                charge_weight_digits[1] * 0.01 + \
                                                charge_weight_digits[0] * 0.001;
                break;
            default:
                charge_mode_config.target_charge_weight = 0;
                break;
        }
    }

    // If the display task is never created then we shall create one, otherwise we shall resume the task
    if (scale_measurement_render_task_handler == NULL) {
        // The render task shall have lower priority than the current one
        UBaseType_t current_task_priority = uxTaskPriorityGet(xTaskGetCurrentTaskHandle());
        xTaskCreate(scale_measurement_render_task, "Scale Measurement Render Task", configMINIMAL_STACK_SIZE, NULL, current_task_priority - 1, &scale_measurement_render_task_handler);
    }
    else {
        vTaskResume(scale_measurement_render_task_handler);
    }

    // Enable motor on entering the charge mode
    motor_enable(SELECT_COARSE_TRICKLER_MOTOR, true);
    motor_enable(SELECT_FINE_TRICKLER_MOTOR, true);
    
    charge_mode_config.charge_mode_state = CHARGE_MODE_WAIT_FOR_ZERO;

    bool quit = false;
    while (quit == false) {
        switch (charge_mode_config.charge_mode_state) {
            case CHARGE_MODE_WAIT_FOR_ZERO:
                charge_mode_wait_for_zero();
                break;
            case CHARGE_MODE_WAIT_FOR_COMPLETE:
                charge_mode_wait_for_complete();
                break;
            case CHARGE_MODE_WAIT_FOR_CUP_REMOVAL:
                charge_mode_wait_for_cup_removal();
                break;
            case CHARGE_MODE_WAIT_FOR_CUP_RETURN:
                charge_mode_wait_for_cup_return();
                break;
            case CHARGE_MODE_EXIT:
            default:
                quit = true;
                break;
        }
    }

    // Reset LED to default colour
    neopixel_led_set_colour(neopixel_led_config.eeprom_neopixel_led_metadata.default_led_colours.mini12864_backlight_colour,
                            neopixel_led_config.eeprom_neopixel_led_metadata.default_led_colours.led1_colour,
                            neopixel_led_config.eeprom_neopixel_led_metadata.default_led_colours.led2_colour,
                            true);

    // vTaskDelete(scale_measurement_render_handler);
    vTaskSuspend(scale_measurement_render_task_handler);

    // Diable motors on exiting the mode
    motor_enable(SELECT_COARSE_TRICKLER_MOTOR, false);
    motor_enable(SELECT_FINE_TRICKLER_MOTOR, false);

    return 1;  // return back to main menu
}


bool charge_mode_config_init(void) {
    bool is_ok = false;

    // Read charge mode config from EEPROM
    is_ok = load_config(EEPROM_CHARGE_MODE_BASE_ADDR, &charge_mode_config.eeprom_charge_mode_data, &default_charge_mode_data, sizeof(charge_mode_config.eeprom_charge_mode_data), EEPROM_CHARGE_MODE_DATA_REV);
    if (!is_ok) {
        printf("Unable to read charge mode configuration\n");
        return is_ok;
    }

    // Register to eeprom save all
    eeprom_register_handler(charge_mode_config_save);

    // Reglages "Precision de charge" (bloc EEPROM separe)
    is_ok = load_config(EEPROM_CHARGE_TUNING_BASE_ADDR, &charge_mode_config.eeprom_charge_tuning_data, &default_charge_tuning_data, sizeof(charge_mode_config.eeprom_charge_tuning_data), EEPROM_CHARGE_TUNING_DATA_REV);
    if (!is_ok) {
        printf("Unable to read charge tuning configuration\n");
        memcpy(&charge_mode_config.eeprom_charge_tuning_data, &default_charge_tuning_data, sizeof(default_charge_tuning_data));
    }
    eeprom_register_handler(charge_tuning_config_save);

    return true;
}


bool charge_tuning_config_save(void) {
    return save_config(EEPROM_CHARGE_TUNING_BASE_ADDR, &charge_mode_config.eeprom_charge_tuning_data, sizeof(eeprom_charge_tuning_data_t));
}


bool http_rest_charge_tuning_config(struct fs_file *file, int num_params, char *params[], char *values[]) {
    // Mappings
    // t0 (float): tolerance_over
    // t1 (float): tolerance_under
    // t2 (bool): topup_enable
    // t3 (int): topup_max_attempts
    // t4 (float): topup_speed_rps (0 = vitesse min du profil)
    // t5 (int): topup_min_pulse_ms
    // t6 (int): topup_max_pulse_ms
    // t7 (int): settle_timeout_ms
    // t8 (bool): learning_enable
    // t9 (float): aim_bias
    // t10 (float): learning_max_offset
    // ee (bool): save to eeprom

    static char json_buffer[384];
    bool save_to_eeprom = false;
    eeprom_charge_tuning_data_t * t = &charge_mode_config.eeprom_charge_tuning_data;

    for (int idx = 0; idx < num_params; idx += 1) {
        if (strcmp(params[idx], "t0") == 0) {
            t->tolerance_over = fabsf(strtof(values[idx], NULL));
        }
        else if (strcmp(params[idx], "t1") == 0) {
            t->tolerance_under = fabsf(strtof(values[idx], NULL));
        }
        else if (strcmp(params[idx], "t2") == 0) {
            t->topup_enable = string_to_boolean(values[idx]);
        }
        else if (strcmp(params[idx], "t3") == 0) {
            long v = strtol(values[idx], NULL, 10);
            t->topup_max_attempts = (uint32_t) (v < 0 ? 0 : (v > 50 ? 50 : v));
        }
        else if (strcmp(params[idx], "t4") == 0) {
            t->topup_speed_rps = fmaxf(0.0f, strtof(values[idx], NULL));
        }
        else if (strcmp(params[idx], "t5") == 0) {
            long v = strtol(values[idx], NULL, 10);
            t->topup_min_pulse_ms = (uint32_t) (v < 10 ? 10 : v);
        }
        else if (strcmp(params[idx], "t6") == 0) {
            long v = strtol(values[idx], NULL, 10);
            t->topup_max_pulse_ms = (uint32_t) (v < 10 ? 10 : v);
        }
        else if (strcmp(params[idx], "t7") == 0) {
            long v = strtol(values[idx], NULL, 10);
            t->settle_timeout_ms = (uint32_t) (v < 1000 ? 1000 : v);
        }
        else if (strcmp(params[idx], "t8") == 0) {
            bool new_learning_enable = string_to_boolean(values[idx]);
            // Desactiver/reactiver l'apprentissage repart de zero (pour le profil en cours)
            if (new_learning_enable != t->learning_enable) {
                profile_extra_t * extra = profile_get_selected_extra();
                extra->learned_inflight = 0.0f;
                extra->learned_samples = 0;
                extra->topup_rate_per_ms = 0.0f;
                learned_samples = 0;
                learned_inflight = 0.0f;
                topup_rate_per_ms = 0.0f;
                profile_extra_data_save();
            }
            t->learning_enable = new_learning_enable;
        }
        else if (strcmp(params[idx], "t9") == 0) {
            t->aim_bias = strtof(values[idx], NULL);
        }
        else if (strcmp(params[idx], "t10") == 0) {
            t->learning_max_offset = fmaxf(0.0f, strtof(values[idx], NULL));
        }
        else if (strcmp(params[idx], "ee") == 0) {
            save_to_eeprom = string_to_boolean(values[idx]);
        }
    }

    // Coherence : impulsion min <= impulsion max
    if (t->topup_max_pulse_ms < t->topup_min_pulse_ms) {
        t->topup_max_pulse_ms = t->topup_min_pulse_ms;
    }

    if (save_to_eeprom) {
        charge_tuning_config_save();
    }

    snprintf(json_buffer, sizeof(json_buffer),
             "%s"
             "{\"t0\":%0.3f,\"t1\":%0.3f,\"t2\":%s,\"t3\":%lu,\"t4\":%0.3f,\"t5\":%lu,\"t6\":%lu,"
             "\"t7\":%lu,\"t8\":%s,\"t9\":%0.3f,\"t10\":%0.3f}",
             http_json_header,
             t->tolerance_over,
             t->tolerance_under,
             boolean_to_string(t->topup_enable),
             (unsigned long) t->topup_max_attempts,
             t->topup_speed_rps,
             (unsigned long) t->topup_min_pulse_ms,
             (unsigned long) t->topup_max_pulse_ms,
             (unsigned long) t->settle_timeout_ms,
             boolean_to_string(t->learning_enable),
             t->aim_bias,
             t->learning_max_offset);

    size_t data_length = strlen(json_buffer);
    file->data = json_buffer;
    file->len = data_length;
    file->index = data_length;
    file->flags = FS_FILE_FLAGS_HEADER_INCLUDED;

    return true;
}


bool http_rest_charge_history(struct fs_file *file, int num_params, char *params[], char *values[]) {
    // Parametres
    // clr (bool): efface l'historique
    //
    // Reponse
    // n: nombre d'entrees, seq: numero de la derniere charge
    // inflight: poudre "en vol" apprise, samples: nb d'echantillons, rate: debit des complements (par seconde)
    // t0/t1: tolerances + / -
    // h: [[seq, cible, poids final, complements, duree (s), resultat (0=ok,1=bas,2=haut), point d'arret], ...]
    //    de la plus recente a la plus ancienne

    static char json_buffer[64 + 300 + CHARGE_HISTORY_SIZE * 72];

    for (int idx = 0; idx < num_params; idx += 1) {
        if (strcmp(params[idx], "clr") == 0 && string_to_boolean(values[idx])) {
            charge_history_clear();
        }
    }

    profile_extra_t * history_extra = profile_get_selected_extra();
    size_t len = 0;
    int written = snprintf(json_buffer, sizeof(json_buffer),
                           "%s"
                           "{\"n\":%lu,\"seq\":%lu,\"inflight\":%0.3f,\"samples\":%lu,\"rate\":%0.4f,"
                           "\"total\":%0.3f,\"total_n\":%lu,"
                           "\"t0\":%0.3f,\"t1\":%0.3f,\"h\":[",
                           http_json_header,
                           (unsigned long) charge_history_count,
                           (unsigned long) charge_history_seq,
                           history_extra->learned_inflight,
                           (unsigned long) history_extra->learned_samples,
                           history_extra->topup_rate_per_ms * 1000.0f,
                           total_powder_weight,
                           (unsigned long) total_powder_charges,
                           charge_mode_config.eeprom_charge_tuning_data.tolerance_over,
                           charge_mode_config.eeprom_charge_tuning_data.tolerance_under);
    if (written > 0) {
        len = (size_t) written;
    }

    uint32_t count = charge_history_count;
    for (uint32_t i = 0; i < count && len < sizeof(json_buffer); i++) {
        uint32_t pos = (charge_history_head + CHARGE_HISTORY_SIZE - 1 - i) % CHARGE_HISTORY_SIZE;
        charge_history_entry_t * e = &charge_history[pos];
        written = snprintf(json_buffer + len, sizeof(json_buffer) - len,
                           "%s[%lu,%0.3f,%0.3f,%u,%0.1f,%u,%0.3f]",
                           i == 0 ? "" : ",",
                           (unsigned long) e->seq,
                           e->target,
                           e->final_weight,
                           (unsigned) e->topups,
                           e->elapsed_s,
                           (unsigned) e->result,
                           e->stop_offset);
        if (written <= 0 || (size_t) written >= sizeof(json_buffer) - len - 3) {
            json_buffer[len] = '\0';  // Pas assez de place : on s'arrete proprement
            break;
        }
        len += (size_t) written;
    }

    if (len < sizeof(json_buffer) - 3) {
        json_buffer[len++] = ']';
        json_buffer[len++] = '}';
        json_buffer[len] = '\0';
    }

    size_t data_length = strlen(json_buffer);
    file->data = json_buffer;
    file->len = data_length;
    file->index = data_length;
    file->flags = FS_FILE_FLAGS_HEADER_INCLUDED;

    return true;
}


bool charge_mode_config_save(void) {
    bool is_ok = save_config(EEPROM_CHARGE_MODE_BASE_ADDR, &charge_mode_config.eeprom_charge_mode_data, sizeof(eeprom_charge_mode_data_t));
    return is_ok;
}



bool http_rest_charge_mode_config(struct fs_file *file, int num_params, char *params[], char *values[]) {
    // Mappings
    // c1 (str): neopixel_normal_charge_colour
    // c2 (str): neopixel_under_charge_colour
    // c3 (str): neopixel_over_charge_colour
    // c4 (str): neopixel_not_ready_colour

    // c5 (float): coarse_stop_threshold
    // c6 (float): fine_stop_threshold
    // c7 (float): set_point_sd_margin
    // c8 (float): set_point_mean_margin
    // c9 (int): decimal point enum
    // c10 (bool): precharge_enable
    // c11 (int): precharge_time_ms
    // c12 (float): precharge_speed_rps
    // c13 (float): coarse_stop_gate_ratio
    // ee (bool): save to eeprom

    static char charge_mode_json_buffer[256];
    bool save_to_eeprom = false;

    // Control
    for (int idx = 0; idx < num_params; idx += 1) {
        if (strcmp(params[idx], "c5") == 0) {
            charge_mode_config.eeprom_charge_mode_data.coarse_stop_threshold = strtof(values[idx], NULL);
        }
        else if (strcmp(params[idx], "c6") == 0) {
            charge_mode_config.eeprom_charge_mode_data.fine_stop_threshold = strtof(values[idx], NULL);
        }
        else if (strcmp(params[idx], "c7") == 0) {
            charge_mode_config.eeprom_charge_mode_data.set_point_sd_margin = strtof(values[idx], NULL);
        }
        else if (strcmp(params[idx], "c8") == 0) {
            charge_mode_config.eeprom_charge_mode_data.set_point_mean_margin = strtof(values[idx], NULL);
        }
        else if (strcmp(params[idx], "c9") == 0) {
            charge_mode_config.eeprom_charge_mode_data.decimal_places = (decimal_places_t) atoi(values[idx]);
        }
        
        // Pre charge related settings
        else if (strcmp(params[idx], "c10") == 0) {
            charge_mode_config.eeprom_charge_mode_data.precharge_enable = string_to_boolean(values[idx]);
        }
        else if (strcmp(params[idx], "c11") == 0) {
            charge_mode_config.eeprom_charge_mode_data.precharge_time_ms = strtol(values[idx], NULL, 10);
        }
        else if (strcmp(params[idx], "c12") == 0) {
            charge_mode_config.eeprom_charge_mode_data.precharge_speed_rps = strtof(values[idx], NULL);
        }
        else if (strcmp(params[idx], "c13") == 0) {
            charge_mode_config.eeprom_charge_mode_data.coarse_stop_gate_ratio = strtof(values[idx], NULL);
        }


        // LED related settings
        else if (strcmp(params[idx], "c1") == 0) {
            charge_mode_config.eeprom_charge_mode_data.neopixel_normal_charge_colour._raw_colour = hex_string_to_decimal(values[idx]);
        }
        else if (strcmp(params[idx], "c2") == 0) {
            charge_mode_config.eeprom_charge_mode_data.neopixel_under_charge_colour._raw_colour = hex_string_to_decimal(values[idx]);
        }
        else if (strcmp(params[idx], "c3") == 0) {
            charge_mode_config.eeprom_charge_mode_data.neopixel_over_charge_colour._raw_colour = hex_string_to_decimal(values[idx]);
        }
        else if (strcmp(params[idx], "c4") == 0) {
            charge_mode_config.eeprom_charge_mode_data.neopixel_not_ready_colour._raw_colour = hex_string_to_decimal(values[idx]);
        }
        else if (strcmp(params[idx], "ee") == 0) {
            save_to_eeprom = string_to_boolean(values[idx]);
        }
    }
    
    // Perform action
    if (save_to_eeprom) {
        charge_mode_config_save();
    }

    // Response
    snprintf(charge_mode_json_buffer, 
             sizeof(charge_mode_json_buffer),
             "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\n\r\n"
             "{\"c1\":\"#%06lx\",\"c2\":\"#%06lx\",\"c3\":\"#%06lx\",\"c4\":\"#%06lx\","
             "\"c5\":%.3f,\"c6\":%.3f,\"c7\":%.3f,\"c8\":%.3f,\"c9\":%d,\"c10\":%s,\"c11\":%ld,\"c12\":%0.3f,\"c13\":%0.3f}",
             charge_mode_config.eeprom_charge_mode_data.neopixel_normal_charge_colour._raw_colour,
             charge_mode_config.eeprom_charge_mode_data.neopixel_under_charge_colour._raw_colour,
             charge_mode_config.eeprom_charge_mode_data.neopixel_over_charge_colour._raw_colour,
             charge_mode_config.eeprom_charge_mode_data.neopixel_not_ready_colour._raw_colour,
             charge_mode_config.eeprom_charge_mode_data.coarse_stop_threshold,
             charge_mode_config.eeprom_charge_mode_data.fine_stop_threshold,
             charge_mode_config.eeprom_charge_mode_data.set_point_sd_margin,
             charge_mode_config.eeprom_charge_mode_data.set_point_mean_margin,
             charge_mode_config.eeprom_charge_mode_data.decimal_places,
             boolean_to_string(charge_mode_config.eeprom_charge_mode_data.precharge_enable),
             charge_mode_config.eeprom_charge_mode_data.precharge_time_ms,
             charge_mode_config.eeprom_charge_mode_data.precharge_speed_rps,
             charge_mode_config.eeprom_charge_mode_data.coarse_stop_gate_ratio);

    size_t data_length = strlen(charge_mode_json_buffer);
    file->data = charge_mode_json_buffer;
    file->len = data_length;
    file->index = data_length;
    file->flags = FS_FILE_FLAGS_HEADER_INCLUDED;

    return true;
}


bool http_rest_charge_mode_state(struct fs_file *file, int num_params, char *params[], char *values[]) {
    // Mappings
    // s0 (float): Charge weight set point (unitless)
    // s1 (float): Current weight (unitless)
    // s2 (charge_mode_state_t | int): Charge mode state
    // s3 (uint32_t): Charge mode event
    // s4 (string): Profile Name
    // s5 (string): Elapsed time in seconds, live during charging
    // s6 (float): Derniere cible utilisee avec le profil selectionne

    static char charge_mode_json_buffer[200];  // Increased to fit s5, s6
    char elapsed_time_buffer[16] = {0};

    // Control
    for (int idx = 0; idx < num_params; idx += 1) {
        if (strcmp(params[idx], "s0") == 0) {
            charge_mode_config.target_charge_weight = strtof(values[idx], NULL);
        }
        else if (strcmp(params[idx], "s2") == 0) {
            charge_mode_state_t new_state = (charge_mode_state_t) atoi(values[idx]);

            // Exit
            if (new_state == CHARGE_MODE_EXIT && charge_mode_config.charge_mode_state != CHARGE_MODE_EXIT) {
                ButtonEncoderEvent_t button_event = BUTTON_RST_PRESSED;
                xQueueSend(encoder_event_queue, &button_event, portMAX_DELAY);
            }
            // Enter
            else if (new_state == CHARGE_MODE_WAIT_FOR_ZERO && charge_mode_config.charge_mode_state == CHARGE_MODE_EXIT) {
                // Set exit_status for the menu
                exit_state = APP_STATE_ENTER_CHARGE_MODE_FROM_REST;

                // Then signal the menu to stop
                ButtonEncoderEvent_t button_event = OVERRIDE_FROM_REST;
                xQueueSend(encoder_event_queue, &button_event, portMAX_DELAY);
            }

            charge_mode_config.charge_mode_state = new_state;
        }
    }

    // Handle the special case
    float current_measurement = scale_get_current_measurement();
    char weight_string[16];
    if (isnanf(current_measurement)) {
        sprintf(weight_string, "\"nan\"");
    }
    else if (isinff(current_measurement)) {
        sprintf(weight_string, "\"inf\"");
    }
    else {
        sprintf(weight_string, "%0.3f", current_measurement);
    }

    // Format elapsed time
    if (charge_mode_config.charge_mode_state == CHARGE_MODE_WAIT_FOR_COMPLETE) {
        TickType_t now = xTaskGetTickCount();
        float elapsed_seconds = (float)((now - charge_start_tick) * portTICK_PERIOD_MS) / 1000.0f;
        snprintf(elapsed_time_buffer, sizeof(elapsed_time_buffer), "%.2f", elapsed_seconds);
    } else {
        snprintf(elapsed_time_buffer, sizeof(elapsed_time_buffer), "%.2f", last_charge_elapsed_seconds);
    }

    // Response
    snprintf(charge_mode_json_buffer, 
             sizeof(charge_mode_json_buffer),
             "%s"
             "{\"s0\":%0.3f,\"s1\":%s,\"s2\":%d,\"s3\":%lu,\"s4\":\"%s\",\"s5\":\"%s\",\"s6\":%0.3f}",
             http_json_header,
             charge_mode_config.target_charge_weight,
             weight_string,
             (int) charge_mode_config.charge_mode_state,
             charge_mode_config.charge_mode_event,
             profile_get_selected()->name,
             elapsed_time_buffer,
             profile_get_selected_extra()->last_target_weight);

    // Clear events
    charge_mode_config.charge_mode_event = 0;

    size_t data_length = strlen(charge_mode_json_buffer);
    file->data = charge_mode_json_buffer;
    file->len = data_length;
    file->index = data_length;
    file->flags = FS_FILE_FLAGS_HEADER_INCLUDED;

    return true;
}
