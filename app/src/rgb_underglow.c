/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/settings/settings.h>

#include <math.h>
#include <stdlib.h>

#include <zephyr/logging/log.h>

#include <zephyr/drivers/led_strip.h>
#include <drivers/ext_power.h>

#include <zmk/rgb_underglow.h>

#include <zmk/activity.h>
#include <zmk/usb.h>
#include <zmk/event_manager.h>
#include <zmk/events/activity_state_changed.h>
#include <zmk/events/usb_conn_state_changed.h>
#include <zmk/workqueue.h>

#include <zmk/events/position_state_changed.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#if !DT_HAS_CHOSEN(zmk_underglow)

#error "A zmk,underglow chosen node must be declared"

#endif

#define STRIP_CHOSEN DT_CHOSEN(zmk_underglow)
#define STRIP_NUM_PIXELS DT_PROP(STRIP_CHOSEN, chain_length)

#define HUE_MAX 360
#define SAT_MAX 100
#define BRT_MAX 100

#define NUM_KEYS 42
#define KEYS_PER_HALF 21
#define MAX_RIPPLE_TREES 15
#define MAX_RIPPLE_FRAMES 10
#define MAX_RIPPLE_FRAME_DURATION 1
#define MAX_ADJACENCIES 5

#if !IS_ENABLED(CONFIG_ZMK_SPLIT) || IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
    static const int pos_to_led_map[NUM_KEYS] = {24, 23, 22, 21, 20, 19, -1, -1, -1, -1, -1, -1, 25, 18, 17, 16, 15, 14, -1, -1, -1, -1, -1, -1, 26, 13, 12, 11, 10, 9, -1, -1, -1, -1, -1, -1, 8, 7, 6, -1, -1, -1};
    static const int led_to_pos_map[STRIP_NUM_PIXELS] = {-1, -1, -1, -1, -1, -1, 38, 37, 36, 29, 28, 27, 26, 25, 17, 16, 15, 14, 13, 5, 4, 3, 2, 1, 0, 12, 24};
#else
    static const int pos_to_led_map[NUM_KEYS] = {-1, -1, -1, -1, -1, -1, 19, 20, 21, 22, 23, 24, -1, -1, -1, -1, -1, -1, 14, 15, 16, 17, 18, 25, -1, -1, -1, -1, -1, -1, 9, 10, 11, 12, 13, 26, -1, -1, -1, 6, 7, 8};
    static const int led_to_pos_map[STRIP_NUM_PIXELS] = {-1, -1, -1, -1, -1, -1, 39, 40, 41, 30, 31, 32, 33, 34, 18, 19, 20, 21, 22, 6, 7, 8, 9, 10, 11, 23, 35};
#endif
    static const int adjacencies[NUM_KEYS][MAX_ADJACENCIES] = {
        {1, 12, -1, -1, -1 },
        {0, 2, 13, -1, -1},
        {1, 3, 14, -1, -1},
        {2, 4, 15, -1, -1},
        {3, 5, 16, -1, -1},
        {4, 17, -1, -1, -1},
        {7, 18, -1, -1, -1},
        {6,8,19, -1, -1},
        {7,9,20, -1, -1},
        {8,10,21, -1, -1},
        {9,11,22, -1, -1},
        {10,23,  -1, -1, -1},
        {0,13,24,  -1, -1}, // inizio seconda riga
        {1,12,14,25,  -1},
        {2,13,15,26,  -1},
        {3,14,16,27,  -1},
        {4,15,17,28,  -1},
        {5,16,29,  -1, -1},
        {6,19,30, -1, -1},
        {7,18,20,31, -1},
        {8,19,21,32, -1},
        {9,20,22,33, -1},
        {10,21,23,34, -1},
        {11,22,35,  -1, -1},
        {12,25, -1, -1, -1}, // inizio terza riga
        {13,24,26, -1, -1},
        {14,25,27,  -1, -1},
        {15,26,28,36,  -1},
        {16,27,29,36,37},
        {17,28, 37,  -1, -1},
        {18,31,40,  -1, -1},
        {19,30,32, 40, 41},
        {20,31,33, 41,  -1},
        {21,32,34,  -1, -1},
        {22,33,35, -1, -1},
        {23,34, -1, -1, -1},
        {27,28,37, -1, -1}, // thumbs
        {28,29,36,38, -1},
        {37, -1, -1, -1, -1},
        {40,  -1, -1, -1, -1},
        {30,31,39,41,  -1},
        {31,32,40 -1, -1}
    };
static uint32_t heatmap_values[NUM_KEYS] = {0};
static uint32_t heatmap_value_sum = 1;
static short speed_mult = 1;
static short ripple_trees[MAX_RIPPLE_TREES][MAX_RIPPLE_FRAMES][NUM_KEYS];
static short occupied_trees[MAX_RIPPLE_TREES]; // frame counter for each tree
static short queued_trees[MAX_RIPPLE_TREES]; 
static short ripple_frame_duration[MAX_RIPPLE_TREES];



BUILD_ASSERT(CONFIG_ZMK_RGB_UNDERGLOW_BRT_MIN <= CONFIG_ZMK_RGB_UNDERGLOW_BRT_MAX,
             "ERROR: RGB underglow maximum brightness is less than minimum brightness");

enum rgb_underglow_effect {
    UNDERGLOW_EFFECT_SOLID,
    UNDERGLOW_EFFECT_BREATHE,
    UNDERGLOW_EFFECT_SPECTRUM,
    UNDERGLOW_EFFECT_SWIRL,
    UNDERGLOW_EFFECT_SWIRL_BI,
    // UNDERGLOW_EFFECT_WAVE,
    // UNDERGLOW_EFFECT_RESPONSIVE,
    UNDERGLOW_EFFECT_HEATMAP,
    UNDERGLOW_EFFECT_RIPPLE,
    UNDERGLOW_EFFECT_NUMBER // Used to track number of underglow effects
};

struct rgb_underglow_state {
    struct zmk_led_hsb color;
    uint8_t animation_speed;
    uint8_t current_effect;
    uint16_t animation_step;
    bool on;
};

static const struct device *led_strip;

static struct led_rgb pixels[STRIP_NUM_PIXELS];

static struct rgb_underglow_state state;

#if IS_ENABLED(CONFIG_ZMK_RGB_UNDERGLOW_EXT_POWER)
static const struct device *ext_power;
#endif

static struct zmk_led_hsb hsb_scale_min_max(struct zmk_led_hsb hsb) {
    hsb.b = CONFIG_ZMK_RGB_UNDERGLOW_BRT_MIN +
            (CONFIG_ZMK_RGB_UNDERGLOW_BRT_MAX - CONFIG_ZMK_RGB_UNDERGLOW_BRT_MIN) * hsb.b / BRT_MAX;
    return hsb;
}

static struct zmk_led_hsb hsb_scale_zero_max(struct zmk_led_hsb hsb) {
    hsb.b = hsb.b * CONFIG_ZMK_RGB_UNDERGLOW_BRT_MAX / BRT_MAX;
    return hsb;
}

static struct led_rgb hsb_to_rgb(struct zmk_led_hsb hsb) {
    float r, g, b;

    uint8_t i = hsb.h / 60;
    float v = hsb.b / ((float)BRT_MAX);
    float s = hsb.s / ((float)SAT_MAX);
    float f = hsb.h / ((float)HUE_MAX) * 6 - i;
    float p = v * (1 - s);
    float q = v * (1 - f * s);
    float t = v * (1 - (1 - f) * s);

    switch (i % 6) {
    case 0:
        r = v;
        g = t;
        b = p;
        break;
    case 1:
        r = q;
        g = v;
        b = p;
        break;
    case 2:
        r = p;
        g = v;
        b = t;
        break;
    case 3:
        r = p;
        g = q;
        b = v;
        break;
    case 4:
        r = t;
        g = p;
        b = v;
        break;
    case 5:
        r = v;
        g = p;
        b = q;
        break;
    }

    struct led_rgb rgb = {r : r * 255, g : g * 255, b : b * 255};

    return rgb;
}

static bool find_in_list(int element, int* list, size_t arr_size){
    for (size_t i = 0; i < arr_size; i++){
        if(element == list[i]){
            return true;
        }
    }
    return false;
}

static void zmk_rgb_underglow_effect_solid() {
    for (int i = 0; i < STRIP_NUM_PIXELS; i++) {
        pixels[i] = hsb_to_rgb(hsb_scale_min_max(state.color));
    }
}

static void zmk_rgb_underglow_effect_breathe() {
    for (int i = 0; i < STRIP_NUM_PIXELS; i++) {
        struct zmk_led_hsb hsb = state.color;
        hsb.b = abs(state.animation_step - 1200) / 12;

        pixels[i] = hsb_to_rgb(hsb_scale_zero_max(hsb));
    }

    state.animation_step += state.animation_speed * 10;

    if (state.animation_step > 2400) {
        state.animation_step = 0;
    }
}

static void zmk_rgb_underglow_effect_spectrum() {
    for (int i = 0; i < STRIP_NUM_PIXELS; i++) {
        struct zmk_led_hsb hsb = state.color;
        hsb.h = state.animation_step;

        pixels[i] = hsb_to_rgb(hsb_scale_min_max(hsb));
    }

    state.animation_step += state.animation_speed;
    state.animation_step = state.animation_step % HUE_MAX;
}

static void zmk_rgb_underglow_effect_swirl() {
    for (int i = 0; i < STRIP_NUM_PIXELS; i++) {
        struct zmk_led_hsb hsb = state.color;
        hsb.h = (HUE_MAX / STRIP_NUM_PIXELS * i + state.animation_step) % HUE_MAX;

        pixels[i] = hsb_to_rgb(hsb_scale_min_max(hsb));
    }

    state.animation_step += state.animation_speed * 2;
    state.animation_step = state.animation_step % HUE_MAX;
}

static void zmk_rgb_underglow_effect_swirl_bi() {

    const uint32_t hue_high = 320;
    const uint32_t hue_low = 240;

    // Hue range: 240 - 300
    for (int i = 0; i < STRIP_NUM_PIXELS; i++) {
        struct zmk_led_hsb hsb = state.color;
        hsb.h = (((hue_high-hue_low) / STRIP_NUM_PIXELS) * i + state.animation_step);
        if (hsb.h > hue_high) {
            hsb.h = hue_high - (hsb.h - hue_high); // remove the overshoot
        }
        hsb.h = hsb.h % HUE_MAX; // should never do anything
        pixels[i] = hsb_to_rgb(hsb_scale_min_max(hsb));
    }

    // Triangle wave
    if (state.animation_step >= hue_high) {
        speed_mult *= -1;
    } else if (state.animation_step < hue_low) {
        speed_mult *= -1;
    }
    state.animation_step += state.animation_speed*speed_mult;
    // state.animation_step = state.animation_step % HUE_MAX;
}

static void zmk_rgb_underglow_effect_wave() {
    // hardcoded for splitkb aurora corne    
    int GROUP_0[6] = {3,4,5,6,7,8}; 
    int GROUP_1[6] = {9,10,11,12,13,26};
    int GROUP_2[7] = {14,15,16,17,18,25,0};
    int GROUP_3[8] = {19,20,21,22,23,24,1,2};

    for (int i = 0; i < STRIP_NUM_PIXELS; i++) {
        struct zmk_led_hsb hsb = state.color;
        int group = -1;
        
        if (find_in_list(i, GROUP_0, 6)) group = 0;
        if (find_in_list(i, GROUP_1, 6)) group = 1;
        if (find_in_list(i, GROUP_2, 7)) group = 2;
        if (find_in_list(i, GROUP_3, 8)) group = 3;
        // 2400 steps

        int shift = 100; // default

        switch (group) {
        case 0:
            shift = 120;
            break;
        case 1:
            shift = 70;
            break;
        case 2:
            shift = 45;
            break;
        case 3:
            shift = 20;
            break;
        default:
            shift = 100;
            break;
        };

        hsb.b = CLAMP((-abs(state.animation_step - 1200) / 12) + shift, 0, BRT_MAX);
        pixels[i] = hsb_to_rgb(hsb_scale_zero_max(hsb));
    }

    state.animation_step += state.animation_speed * 10;

    if (state.animation_step > 2400) {
        state.animation_step = 0;
    }
}


// dynamic effect code
static short prepare_ripple(int position);
static void get_adjacent(int key, short* adjacencies_found);

static int rgb_underglow_position_state_changed_listener(const zmk_event_t *eh);

ZMK_LISTENER(rgb_underglow_dynamic, rgb_underglow_position_state_changed_listener);
ZMK_SUBSCRIPTION(rgb_underglow_dynamic, zmk_position_state_changed);

static int rgb_underglow_position_state_changed_listener(const zmk_event_t *eh) {

    short returned_tree = -1;
    if(
        // state.current_effect != UNDERGLOW_EFFECT_RESPONSIVE 
        // && 
        state.current_effect != UNDERGLOW_EFFECT_HEATMAP
        &&
        state.current_effect != UNDERGLOW_EFFECT_RIPPLE
        ){
        return ZMK_EV_EVENT_BUBBLE;
    }
    struct zmk_position_state_changed *ev = as_zmk_position_state_changed(eh);
    if(ev == NULL){ 
        return ZMK_EV_EVENT_BUBBLE;
    }

    // UNDERGLOW_EFFECT_HEATMAP
    if (state.current_effect == UNDERGLOW_EFFECT_HEATMAP) {

        if (ev->position < 0 || ev->position >= NUM_KEYS){ // my total number of keys 
            // don't increment anything
        } else if (ev->state) {
            heatmap_values[ev->position] += 1;
            heatmap_value_sum += 1;
        }
        return ZMK_EV_EVENT_BUBBLE;
    }

    // UNDERGLOW_EFFECT_RIPPLE
    // Create a data structure that's a list of frames originating from the pressed key
    else if (state.current_effect == UNDERGLOW_EFFECT_RIPPLE) {
        if (ev->position < 0 || ev->position >= NUM_KEYS){ // my total number of keys 
            // nothing
        }
        else if (ev->state) // key down
        {
            // prepare ripple
            returned_tree = prepare_ripple(ev->position); // race condition here but who cares
            if (returned_tree != -1) queued_trees[returned_tree] = ev->position; // if free slot ok, else return doing nothing
        }
        else if (!ev->state) // key up
        {
            for(int i = 0; i < MAX_RIPPLE_TREES; i++){
                if(queued_trees[i] == ev->position){
                    occupied_trees[i] = 0;
                    queued_trees[i] = -1;
                }
            }
            // activate queued ripples
        }
        return ZMK_EV_EVENT_BUBBLE;
    }

    // UNDERGLOW_EFFECT_RESPONSIVE
    // if (state.current_effect == UNDERGLOW_EFFECT_RESPONSIVE) {

    //     struct zmk_led_hsb other_color = state.color; 
    //     other_color.h = (other_color.h + HUE_MAX/2) % HUE_MAX; // shifted by half the hue space
    //     // other_color.b = CONFIG_ZMK_RGB_UNDERGLOW_BRT_MIN;
    //     int pixel_to_light = 0;

    //     if (ev->position < 0 || ev->position >= NUM_KEYS){ // my total number of keys 
    //         pixel_to_light = 0; // DEBUG in case it fails
    //     } else {
    //         pixel_to_light = pos_to_led_map[ev->position];
    //     }

    //     if(pixel_to_light == -1){ // not on the half that should light up
    //         return ZMK_EV_EVENT_BUBBLE;
    //     }

    //     if(ev->state){ //on 
    //         pixels[pixel_to_light] = hsb_to_rgb(hsb_scale_min_max(other_color));
    //     } else { // off
    //         pixels[pixel_to_light] = hsb_to_rgb(hsb_scale_min_max(state.color));
    //     }
    //     return ZMK_EV_EVENT_BUBBLE;
    // }
    return ZMK_EV_EVENT_BUBBLE;
}

static short prepare_ripple(int position){
    /*
    * Starting from position, creates a list of frames of adjacent keys
    * Returns the index of the used tree slot
    */
    // find a free tree
    int free_tree = -1;
    for (int i = 0; i < MAX_RIPPLE_TREES; i++){
        if(occupied_trees[i] == -1){
            free_tree = i;
            break;
        }
    }
    if(free_tree == -1){
        return -1; // no free tree
    }

    // reset the tree
    for(int i = 0; i < MAX_RIPPLE_FRAMES; i++){
        for(int j = 0; j < NUM_KEYS; j++){
            ripple_trees[free_tree][i][j] = -1;
        }
    }

    ripple_trees[free_tree][0][0] = position; // first frame
    bool used_keys[NUM_KEYS];
    short adjacencies_found[NUM_KEYS];
    short frame = 1;
    short cur_key = -1;
    short cur_free_pos = -1;
    bool done = false;
    for(int i = 0; i < NUM_KEYS; i++){
        used_keys[i] = false;
    }
    used_keys[position] = true;
    
    while(!done){
        // begin preparing
        cur_free_pos = 0;
        done = true;

        // begin frame
        for(int j = 0; j < NUM_KEYS; j++){
            // for each key in the previous frame, get its neighbours
            cur_key = ripple_trees[free_tree][frame-1][j];

            if (cur_key == -1){
                frame += 1;
                break; // done with the frame
            } 
            done = false;
            get_adjacent(cur_key, adjacencies_found);

            for(int k = 0; k < NUM_KEYS; k++){
                if (adjacencies_found[k] == -1) {
                    break;
                }
                if (!used_keys[adjacencies_found[k]]) {
                    ripple_trees[free_tree][frame][cur_free_pos] = adjacencies_found[k];
                    cur_free_pos += 1;
                    used_keys[adjacencies_found[k]] = true;
                }
            }
        }
    }
    return free_tree;
}

static void get_adjacent(int key, short* adjacencies_found){
    for (int i = 0; i < NUM_KEYS; i++){
        adjacencies_found[i] = -1;
    }
    for (int i = 0; i < MAX_ADJACENCIES; i++){
        adjacencies_found[i] = adjacencies[key][i];
    }
}

static void zmk_rgb_underglow_effect_heatmap() {
    // refresehes the static pixels with calculated values
    for (int i = 0; i < 6; i++) { // backlight is refreshed always
        pixels[i] = hsb_to_rgb(hsb_scale_min_max(state.color));
    }
    float percentage = 0;
    int hue_range = HUE_MAX/2;

    for (int i = 6; i < STRIP_NUM_PIXELS; i++){
        struct zmk_led_hsb hsb = state.color;
        // multiply by 8 to give some importance to all values as they will be spread out over many keys
        // instead of a linear thing it could be like a softmax of some sort
        percentage = 8.0 * heatmap_values[led_to_pos_map[i]] / (float)heatmap_value_sum; // from 0 to 8
        // clamp
        percentage = CLAMP(percentage, 0.0, 1.0);

        hsb.h = (hsb.h + (int)(percentage * hue_range)) % HUE_MAX; 
        pixels[i] = hsb_to_rgb(hsb_scale_min_max(hsb));
    }
}

static void zmk_rgb_underglow_effect_ripple() {
    // reset lit pixels to other colour
    // struct zmk_led_hsb hsb = state.color;
    // hsb.h = (hsb.h + HUE_MAX/2) % HUE_MAX;
    // for (int i = 0; i < STRIP_NUM_PIXELS; i++) {
    //     pixels[i] = hsb_to_rgb(hsb_scale_min_max(hsb));
    // }
    for (int i = 0; i < STRIP_NUM_PIXELS; i++) {
        pixels[i] = (struct led_rgb){r : 0, g : 0, b : 0};
    }

    struct zmk_led_hsb hsb = state.color;
    int led_to_light;

    // light held keys
    for (int i=0; i < MAX_RIPPLE_TREES; i++){ // for each slot
        if (queued_trees[i] == -1) continue;
        led_to_light = pos_to_led_map[queued_trees[i]];
        if (led_to_light == -1) continue;
        pixels[led_to_light] = hsb_to_rgb(hsb_scale_min_max(hsb));
    }
    // light correct pixels
    for (int i=0; i < MAX_RIPPLE_TREES; i++){ // for each slot
        if (occupied_trees[i] == -1) continue;
        short frame = occupied_trees[i];
        // what colour to use
        hsb = state.color;
        hsb.b = ((float)(100 - (frame * 20)))/100*hsb.b; // 10% less bright than current_brightness each frame

        for (int idx=0; idx < NUM_KEYS; idx++){  // this is a list of keys that should light up
            if (ripple_trees[i][frame][idx] == -1) break; // frame is done
            led_to_light = pos_to_led_map[ripple_trees[i][frame][idx]];
            if (led_to_light == -1) continue; // safety

            pixels[led_to_light] = hsb_to_rgb(hsb_scale_min_max(hsb));
        }
        // might be too fast to see, add this secondary buffer
        ripple_frame_duration[i] += 1;
        if (ripple_frame_duration[i] >= MAX_RIPPLE_FRAME_DURATION){
            ripple_frame_duration[i] = 0;
            occupied_trees[i] += 1; // go to next frame            
        }
        if (occupied_trees[i] >= MAX_RIPPLE_FRAMES){
            occupied_trees[i] = -1; // free the slot
        }
    }
}

// static void zmk_rgb_underglow_effect_responsive() {
//     // refresehes the static pixels
//     for (int i = 0; i < 6; i++) { // only the backlight
//         pixels[i] = hsb_to_rgb(hsb_scale_min_max(state.color));
//     }
// }

static void zmk_rgb_underglow_tick(struct k_work *work) {
    switch (state.current_effect) {
    case UNDERGLOW_EFFECT_SOLID:
        zmk_rgb_underglow_effect_solid();
        break;
    case UNDERGLOW_EFFECT_BREATHE:
        zmk_rgb_underglow_effect_breathe();
        break;
    case UNDERGLOW_EFFECT_SPECTRUM:
        zmk_rgb_underglow_effect_spectrum();
        break;
    case UNDERGLOW_EFFECT_SWIRL:
        zmk_rgb_underglow_effect_swirl();
        break;
    case UNDERGLOW_EFFECT_SWIRL_BI:
        zmk_rgb_underglow_effect_swirl_bi();
        break;
    // case UNDERGLOW_EFFECT_WAVE:
    //     zmk_rgb_underglow_effect_wave();
    //     break;
    // case UNDERGLOW_EFFECT_RESPONSIVE:
    //     zmk_rgb_underglow_effect_responsive();
    //     break;
    case UNDERGLOW_EFFECT_HEATMAP:
        zmk_rgb_underglow_effect_heatmap();
        break;
    case UNDERGLOW_EFFECT_RIPPLE:
        zmk_rgb_underglow_effect_ripple();
        break;
    }

    int err = led_strip_update_rgb(led_strip, pixels, STRIP_NUM_PIXELS);
    if (err < 0) {
        LOG_ERR("Failed to update the RGB strip (%d)", err);
    }
}

K_WORK_DEFINE(underglow_tick_work, zmk_rgb_underglow_tick);

static void zmk_rgb_underglow_tick_handler(struct k_timer *timer) {
    if (!state.on) {
        return;
    }

    k_work_submit_to_queue(zmk_workqueue_lowprio_work_q(), &underglow_tick_work);
}

K_TIMER_DEFINE(underglow_tick, zmk_rgb_underglow_tick_handler, NULL);

#if IS_ENABLED(CONFIG_SETTINGS)
static int rgb_settings_set(const char *name, size_t len, settings_read_cb read_cb, void *cb_arg) {
    const char *next;
    int rc;

    if (settings_name_steq(name, "state", &next) && !next) {
        if (len != sizeof(state)) {
            return -EINVAL;
        }

        rc = read_cb(cb_arg, &state, sizeof(state));
        if (rc >= 0) {
            return 0;
        }

        return rc;
    }

    return -ENOENT;
}

struct settings_handler rgb_conf = {.name = "rgb/underglow", .h_set = rgb_settings_set};

static void zmk_rgb_underglow_save_state_work() {
    settings_save_one("rgb/underglow/state", &state, sizeof(state));
}

static struct k_work_delayable underglow_save_work;
#endif

static int zmk_rgb_underglow_init(const struct device *_arg) {
    led_strip = DEVICE_DT_GET(STRIP_CHOSEN);

#if IS_ENABLED(CONFIG_ZMK_RGB_UNDERGLOW_EXT_POWER)
    ext_power = device_get_binding("EXT_POWER");
    if (ext_power == NULL) {
        LOG_ERR("Unable to retrieve ext_power device: EXT_POWER");
    }
#endif

    state = (struct rgb_underglow_state){
        color : {
            h : CONFIG_ZMK_RGB_UNDERGLOW_HUE_START,
            s : CONFIG_ZMK_RGB_UNDERGLOW_SAT_START,
            b : CONFIG_ZMK_RGB_UNDERGLOW_BRT_START,
        },
        animation_speed : CONFIG_ZMK_RGB_UNDERGLOW_SPD_START,
        current_effect : CONFIG_ZMK_RGB_UNDERGLOW_EFF_START,
        animation_step : 0,
        on : IS_ENABLED(CONFIG_ZMK_RGB_UNDERGLOW_ON_START)
    };

#if IS_ENABLED(CONFIG_SETTINGS)
    settings_subsys_init();

    int err = settings_register(&rgb_conf);
    if (err) {
        LOG_ERR("Failed to register the ext_power settings handler (err %d)", err);
        return err;
    }

    k_work_init_delayable(&underglow_save_work, zmk_rgb_underglow_save_state_work);

    settings_load_subtree("rgb/underglow");
#endif

#if IS_ENABLED(CONFIG_ZMK_RGB_UNDERGLOW_AUTO_OFF_USB)
    state.on = zmk_usb_is_powered();
#endif

    if (state.on) {
        k_timer_start(&underglow_tick, K_NO_WAIT, K_MSEC(50));
    }

    return 0;
}

int zmk_rgb_underglow_save_state() {
#if IS_ENABLED(CONFIG_SETTINGS)
    int ret = k_work_reschedule(&underglow_save_work, K_MSEC(CONFIG_ZMK_SETTINGS_SAVE_DEBOUNCE));
    return MIN(ret, 0);
#else
    return 0;
#endif
}

int zmk_rgb_underglow_get_state(bool *on_off) {
    if (!led_strip)
        return -ENODEV;

    *on_off = state.on;
    return 0;
}

int zmk_rgb_underglow_on() {
    if (!led_strip)
        return -ENODEV;

#if IS_ENABLED(CONFIG_ZMK_RGB_UNDERGLOW_EXT_POWER)
    if (ext_power != NULL) {
        int rc = ext_power_enable(ext_power);
        if (rc != 0) {
            LOG_ERR("Unable to enable EXT_POWER: %d", rc);
        }
    }
#endif

    state.on = true;
    state.animation_step = 0;
    k_timer_start(&underglow_tick, K_NO_WAIT, K_MSEC(50));

    return zmk_rgb_underglow_save_state();
}

static void zmk_rgb_underglow_off_handler(struct k_work *work) {
    for (int i = 0; i < STRIP_NUM_PIXELS; i++) {
        pixels[i] = (struct led_rgb){r : 0, g : 0, b : 0};
    }

    led_strip_update_rgb(led_strip, pixels, STRIP_NUM_PIXELS);
}

K_WORK_DEFINE(underglow_off_work, zmk_rgb_underglow_off_handler);

int zmk_rgb_underglow_off() {
    if (!led_strip)
        return -ENODEV;

#if IS_ENABLED(CONFIG_ZMK_RGB_UNDERGLOW_EXT_POWER)
    if (ext_power != NULL) {
        int rc = ext_power_disable(ext_power);
        if (rc != 0) {
            LOG_ERR("Unable to disable EXT_POWER: %d", rc);
        }
    }
#endif

    k_work_submit_to_queue(zmk_workqueue_lowprio_work_q(), &underglow_off_work);

    k_timer_stop(&underglow_tick);
    state.on = false;

    return zmk_rgb_underglow_save_state();
}

int zmk_rgb_underglow_calc_effect(int direction) {
    return (state.current_effect + UNDERGLOW_EFFECT_NUMBER + direction) % UNDERGLOW_EFFECT_NUMBER;
}

int zmk_rgb_underglow_initialize_effect() {
    switch (state.current_effect) {
        // case UNDERGLOW_EFFECT_RESPONSIVE:
        //     for (int i = 0; i < STRIP_NUM_PIXELS; i++) {
        //         pixels[i] = hsb_to_rgb(hsb_scale_min_max(state.color));
        //     }
        //     break;
        case UNDERGLOW_EFFECT_HEATMAP:
            heatmap_value_sum = 1; // to avoid zero division
            for (int i = 0; i < NUM_KEYS; i++) {
                heatmap_values[i] = 0;
            }
            break;
        case UNDERGLOW_EFFECT_SWIRL_BI:
            state.animation_step = 240; // start at 240
            speed_mult = 1;
            break;
        case UNDERGLOW_EFFECT_RIPPLE:
            for (int i = 0; i < MAX_RIPPLE_TREES; i++){
                occupied_trees[i] = -1;
                queued_trees[i] = -1;
                ripple_frame_duration[i] = 0;
            }
            break;
        default:
            break;
    }

    return 0;
}

int zmk_rgb_underglow_select_effect(int effect) {
    if (!led_strip)
        return -ENODEV;

    if (effect < 0 || effect >= UNDERGLOW_EFFECT_NUMBER) {
        return -EINVAL;
    }

    state.current_effect = effect;
    state.animation_step = 0;
    zmk_rgb_underglow_initialize_effect();

    return zmk_rgb_underglow_save_state();
}

int zmk_rgb_underglow_cycle_effect(int direction) {
    return zmk_rgb_underglow_select_effect(zmk_rgb_underglow_calc_effect(direction));
}

int zmk_rgb_underglow_toggle() {
    return state.on ? zmk_rgb_underglow_off() : zmk_rgb_underglow_on();
}

int zmk_rgb_underglow_set_hsb(struct zmk_led_hsb color) {
    if (color.h > HUE_MAX || color.s > SAT_MAX || color.b > BRT_MAX) {
        return -ENOTSUP;
    }

    state.color = color;

    return 0;
}

struct zmk_led_hsb zmk_rgb_underglow_calc_hue(int direction) {
    struct zmk_led_hsb color = state.color;

    color.h += HUE_MAX + (direction * CONFIG_ZMK_RGB_UNDERGLOW_HUE_STEP);
    color.h %= HUE_MAX;

    return color;
}

struct zmk_led_hsb zmk_rgb_underglow_calc_sat(int direction) {
    struct zmk_led_hsb color = state.color;

    int s = color.s + (direction * CONFIG_ZMK_RGB_UNDERGLOW_SAT_STEP);
    if (s < 0) {
        s = 0;
    } else if (s > SAT_MAX) {
        s = SAT_MAX;
    }
    color.s = s;

    return color;
}

struct zmk_led_hsb zmk_rgb_underglow_calc_brt(int direction) {
    struct zmk_led_hsb color = state.color;

    int b = color.b + (direction * CONFIG_ZMK_RGB_UNDERGLOW_BRT_STEP);
    color.b = CLAMP(b, 0, BRT_MAX);

    return color;
}

int zmk_rgb_underglow_change_hue(int direction) {
    if (!led_strip)
        return -ENODEV;

    state.color = zmk_rgb_underglow_calc_hue(direction);

    return zmk_rgb_underglow_save_state();
}

int zmk_rgb_underglow_change_sat(int direction) {
    if (!led_strip)
        return -ENODEV;

    state.color = zmk_rgb_underglow_calc_sat(direction);

    return zmk_rgb_underglow_save_state();
}

int zmk_rgb_underglow_change_brt(int direction) {
    if (!led_strip)
        return -ENODEV;

    state.color = zmk_rgb_underglow_calc_brt(direction);

    return zmk_rgb_underglow_save_state();
}

int zmk_rgb_underglow_change_spd(int direction) {
    if (!led_strip)
        return -ENODEV;

    if (state.animation_speed == 1 && direction < 0) {
        return 0;
    }

    state.animation_speed += direction;

    if (state.animation_speed > 5) {
        state.animation_speed = 5;
    }

    return zmk_rgb_underglow_save_state();
}

#if IS_ENABLED(CONFIG_ZMK_RGB_UNDERGLOW_AUTO_OFF_IDLE) ||                                          \
    IS_ENABLED(CONFIG_ZMK_RGB_UNDERGLOW_AUTO_OFF_USB)
static int rgb_underglow_auto_state(bool *prev_state, bool new_state) {
    if (state.on == new_state) {
        return 0;
    }
    if (new_state) {
        state.on = *prev_state;
        *prev_state = false;
        return zmk_rgb_underglow_on();
    } else {
        state.on = false;
        *prev_state = true;
        return zmk_rgb_underglow_off();
    }
}

static int rgb_underglow_event_listener(const zmk_event_t *eh) {

#if IS_ENABLED(CONFIG_ZMK_RGB_UNDERGLOW_AUTO_OFF_IDLE)
    if (as_zmk_activity_state_changed(eh)) {
        static bool prev_state = false;
        return rgb_underglow_auto_state(&prev_state,
                                        zmk_activity_get_state() == ZMK_ACTIVITY_ACTIVE);
    }
#endif

#if IS_ENABLED(CONFIG_ZMK_RGB_UNDERGLOW_AUTO_OFF_USB)
    if (as_zmk_usb_conn_state_changed(eh)) {
        static bool prev_state = false;
        return rgb_underglow_auto_state(&prev_state, zmk_usb_is_powered());
    }
#endif

    return -ENOTSUP;
}

ZMK_LISTENER(rgb_underglow, rgb_underglow_event_listener);
#endif // IS_ENABLED(CONFIG_ZMK_RGB_UNDERGLOW_AUTO_OFF_IDLE) ||
       // IS_ENABLED(CONFIG_ZMK_RGB_UNDERGLOW_AUTO_OFF_USB)

#if IS_ENABLED(CONFIG_ZMK_RGB_UNDERGLOW_AUTO_OFF_IDLE)
ZMK_SUBSCRIPTION(rgb_underglow, zmk_activity_state_changed);
#endif

#if IS_ENABLED(CONFIG_ZMK_RGB_UNDERGLOW_AUTO_OFF_USB)
ZMK_SUBSCRIPTION(rgb_underglow, zmk_usb_conn_state_changed);
#endif

SYS_INIT(zmk_rgb_underglow_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
