#include "music.h"

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "config.h"

// Time, Note, Duration, Instrument (onlinesequencer.net schematic format)
// 0 D4 8 0;0 D5 8 0;0 G4 8 0;8 C5 2 0;10 B4 2 0;12 G4 2 0;14 F4 1 0;15 G4 17 0;16 D4 8 0;24 C4 8 0

struct MusicPlayer {
    int* tones;
    int* durations;
    size_t len;

    int note_index;
    int cur_note_length;
};

void init_music_player(struct MusicPlayer* music, int* tones, int* durations, size_t len)
{
    assert(len >= 1);
    music->tones = tones;
    music->durations = durations;
    music->len = len;

    music->note_index = 0;
    music->cur_note_length = durations[0];
}

void stop_music() {
    
}

void reset_music_player(struct MusicPlayer* music)
{
    music->note_index = 0;
    music->cur_note_length = music->durations[1];
}

void tick_music_player(struct MusicPlayer* music)
{
    int freq = music->tones[music->note_index];
    float clkdiv = (125.0e6 / 65536) / freq;

    if(clkdiv < 1.0)
        clkdiv = 1.0;
    if(freq != TONE_PAUSE) {
        pwm_set_clkdiv(pwm_gpio_to_slice_num(BUZZER_PIN), clkdiv);
        pwm_set_gpio_level(BUZZER_PIN, DUTY);
    } else {
        pwm_set_gpio_level(BUZZER_PIN, 0);
    }

    if(music->cur_note_length == 0) {
        music->note_index = (music->note_index + 1) % music->len;
    }
}