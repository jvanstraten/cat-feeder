#pragma once

#include <Arduino.h>
#include <ArduinoHA.h>
#include <Adafruit_GC9A01A.h>

#include "fsm.h"
#include "pins.h"

class UserInterface {
private:
    /**
     * Low-level display driver.
     */
    Adafruit_GC9A01A tft;

    /**
     * Reference to the state machine that we're representing.
     */
    StateMachine &fsm;

    /**
     * Reference to the MQTT connection.
     */
    HAMqtt &mqtt;

    /**
     * To reduce the duration of individual update calls, the display is
     * updated piecewise. This tracks which piece we're updating next.
     */
    uint8_t display_update_state = 0;

    /**
     * Error report that we're displaying this update cycle.
     */
    StateMachine::ErrorReport error_report = {};

    /**
     * State report that we're displaying this update cycle.
     */
    StateMachine::StateReport state_report = {};

    /**
     * Feed report that we're displaying this update cycle.
     */
    StateMachine::FeedReport feed_report = {};

    /**
     * String representation of the feed report.
     */
    char feed_report_string[21] = {};

    /**
     * String representation of system status: error messages from state
     * machine, or otherwise the WiFi state.
     */
    char status_string[21] = {};

    /**
     * Whether the status string above is "idle enough" to be grayed out.
     */
    bool status_grayed = false;

    /**
     * Screen foreground color for this update cycle.
     */
    uint16_t color_fg = 0;

    /**
     * Screen grayed-out foreground color for this update cycle.
     */
    uint16_t color_gr = 0;

    /**
     * Screen background color for this update cycle.
     */
    uint16_t color_bg = 0;

    /**
     * Backlight brightness.
     */
    uint8_t brightness = 0;

    /**
     * Renders a single line of text.
     */
    void render_line(size_t y, const char *buffer, size_t scale, bool grayed=false);

    /**
     * Preprocess what should be on the screen.
     */
    void display_preprocess();

    /**
     * Updates the display.
     */
    void display_update();

    /**
     * Trivially debounced button class.
     */
    class Button {
    private:
        const uint8_t pin;
        uint8_t state = 0;
    public:
        explicit Button(int pin);
        void begin();
        bool update();
    };

    /**
     * Debounce logic for set button, used to reset deficit.
     */
    Button key_set {PIN_KEY_SET};

    /**
     * Debounce logic for feed button, used to feed manually.
     */
    Button key_feed {PIN_KEY_FEED};

    /**
     * Debounce logic for up button, used to tare reservoir.
     */
    Button key_up {PIN_KEY_UP};

    /**
     * Debounce logic for down button, used to tare bowl.
     */
    Button key_down {PIN_KEY_DOWN};

    /**
     * Debounce logic for (un)lock button, used to exit maintenance or reset.
     */
    Button key_lock {PIN_KEY_LOCK};

    /**
     * Debounce logic for microphone button, used to enter maintenance mode.
     */
    Button key_mic {PIN_KEY_MIC};

public:
    /**
     * Constructor.
     */
    UserInterface(StateMachine &fsm, HAMqtt &mqtt);

    /**
     * Initializes things.
     */
    void begin();

    /**
     * Updates the user interface.
     */
    void update();
};
