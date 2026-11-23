#ifndef PROFILE_H_
#define PROFILE_H_

#include <stdint.h>
#include <stdbool.h>
#include "http_rest.h"


#define PROFILE_NAME_MAX_LEN    16
#define MAX_PROFILE_CNT         8

#define EEPROM_PROFILE_DATA_REV             1           // 16 bit

typedef struct
{  
    uint32_t rev;
    uint32_t compatibility;
    
    char name[PROFILE_NAME_MAX_LEN];

    float coarse_kp;
    float coarse_ki;
    float coarse_kd;

    float coarse_min_flow_speed_rps;
    float coarse_max_flow_speed_rps;

    float fine_kp;
    float fine_ki;
    float fine_kd;

    float fine_min_flow_speed_rps;
    float fine_max_flow_speed_rps;
} profile_t;


typedef struct {
    uint16_t profile_data_rev;
    uint16_t current_profile_idx;

    profile_t profiles[MAX_PROFILE_CNT];
} eeprom_profile_data_t;


// Donnees complementaires par profil, stockees dans un bloc EEPROM separe
// (pour ne pas modifier la structure des profils existants et les conserver)
typedef struct {
    // Recul des tubes a l'arret (en tours, 0 = desactive)
    float coarse_retract_rev;
    float fine_retract_rev;
    float retract_speed_rps;

    // Derniere charge utilisee avec ce profil
    float last_target_weight;

    // Apprentissage memorise
    float learned_inflight;
    uint32_t learned_samples;
    float topup_rate_per_ms;
} profile_extra_t;

typedef struct {
    uint16_t profile_extra_data_rev;   // Doit rester a 0 (validation CRC32)
    uint16_t reserved;
    profile_extra_t extras[MAX_PROFILE_CNT];
} eeprom_profile_extra_data_t;


#ifdef __cplusplus
extern "C" {
#endif



// Interface
bool profile_data_init(void);
bool profile_data_save();

profile_t * profile_select(uint8_t idx);
profile_t * profile_get_selected();
uint16_t profile_get_selected_idx();
profile_extra_t * profile_get_selected_extra();
bool profile_extra_data_save();

// REST interface
bool http_rest_profile_config(struct fs_file *file, int num_params, char *params[], char *values[]);
bool http_rest_profile_summary(struct fs_file *file, int num_params, char *params[], char *values[]);


#ifdef __cplusplus
}
#endif

#endif  // PROFILE_H_