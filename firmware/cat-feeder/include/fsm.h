#pragma once

#include <Arduino.h>
#include <ArduinoHA.h>
#include "loadcell.h"

//#define DEBUG_FSM

/**
 * Main feeding logic.
 */
class StateMachine {
public:
    /**
     * Result of previous feed.
     */
    enum struct FeedResult {
        /**
         * No feed has been performed yet.
         */
        NONE,

        /**
         * Successful feed; arg is amount of milligrams.
         */
        SUCCESS,

        /**
         * Failed to initiate feed due to noise on sensors; arg is number of
         * consecutively failed attempts.
         */
        SENSOR_RETRY,
    };

    /**
     * Report for result of previous feed.
     */
    struct FeedReport {
        FeedResult result;
        int arg;
        unsigned long millis;
    };

    /**
     * Feeding status report. If large is set, only up to 10 characters of
     * detail1 will be used and detail2 is unused, allowing this text to be
     * printed at 2x scale where detail1+detail2 would normally be.
     */
    struct StateReport {
        char header[21];
        char detail1[21];
        char detail2[21];
        bool large;
    };

    /**
     * Severity level for an error message.
     */
    enum struct ErrorSeverity {
        OKAY,
        WARNING,
        ERROR
    };

    /**
     * Status report.
     */
    struct ErrorReport {
        const char *message;
        ErrorSeverity severity;
    };

private:

    /**
     * Loadcell driver.
     */
    Loadcell loadcell;

    /**
     * State machine state.
     */
    enum class State {
        /**
         * Waiting for command or next feed.
         */
        IDLE,

        /**
         * Tare the reservoir after sensor stabilizes from pressing the button.
         */
        IDLE_TARE_RESERVOIR_WAIT,

        /**
         * Perform empty reservoir load cell measurement.
         */
        IDLE_TARE_RESERVOIR,

        /**
         * Perform empty bowl load cell measurement.
         */
        IDLE_TARE_BOWL,

        /**
         * Perform idle reservoir load cell measurement to update MQTT data.
         */
        IDLE_MEASURE_RESERVOIR,

        /**
         * Perform idle bowl load cell measurement to update MQTT data.
         */
        IDLE_MEASURE_BOWL,

        /**
         * Wait time for reservoir stabilization after button press.
         */
        FEED_PRE_MEASURE_WAIT,

        /**
         * Take pre-feed reservoir load cell sample.
         */
        FEED_PRE_MEASURE_RESERVOIR,

        /**
         * Take pre-feed bowl load cell sample.
         */
        FEED_PRE_MEASURE_BOWL,

        /**
         * Feeding motor running, waiting for limit switch release, if it
         * wasn't already released initially.
         */
        FEED_RUN_SYNC,

        /**
         * Feeding motor running, waiting for limit switch assert.
         */
        FEED_RUN_A,

        /**
         * Feeding motor running, waiting for limit switch release.
         */
        FEED_RUN_B,

        /**
         * Feeding motor running, additional time after limit release.
         */
        FEED_RUN_C,

        /**
         * Wait for things to settle before taking load cell measurements.
         */
        FEED_POST_WAIT,

        /**
         * Take post-feed bowl load cell sample.
         */
        FEED_POST_MEASURE_BOWL,

        /**
         * Take post-feed reservoir load cell sample.
         */
        FEED_POST_MEASURE_RESERVOIR,
    };

    /**
     * Current main state.
     */
    State state = State::IDLE;

    /**
     * Maintenance mode or error state.
     */
    enum class MaintenanceMode {
        /**
         * Not in maintenance mode, feeding normally.
         */
        OPERATIONAL,

        /**
         * Manually put in maintenance mode. No automatic feeding, manual feeds
         * do not update deficit, and sensors are updated more frequently while
         * idle.
         */
        MAINTENANCE,

        /**
         * Hopper seems to be jammed. Feeding is stopped until maintenance is
         * performed.
         */
        JAMMED,

    };

    /**
     * Current maintenance state.
     */
    MaintenanceMode maintenance_mode = MaintenanceMode::OPERATIONAL;

    /**
     * Excessive standard deviation in reservoir weight readout.
     */
    bool error_reservoir_stddev = false;

    /**
     * Excessive standard deviation in bowl weight readout.
     */
    bool error_bowl_stddev = false;

    /**
     * Loadcell readout timed out.
     */
    bool error_loadcell_timeout = false;

    /**
     * Loadcell readout timed out.
     */
    bool error_loadcell_disagree = false;

    /**
     * Loadcell readout timed out.
     */
    bool error_loadcell_unreasonable = false;

    /**
     * Motor limit switch readout.
     */
    bool error_limit_switch = false;

    /**
     * Whether there has been a power loss since the last reset.
     */
#ifdef DEBUG_FSM
    bool error_power_loss = false;
#else
    bool error_power_loss = true;
#endif

    /**
     * Clears the above error flags.
     */
    void error_reset();

    /**
     * Amount of grams to feed per day.
     */
    int32_t grams_per_day = 60;

    /**
     * Number of milliseconds remaining before deficit is incremented.
     */
    int32_t deficit_ms_remain = 0;

    /**
     * Feeding deficit in milligrams.
     */
    int32_t deficit_mg = 0;

    /**
     * Deficit threshold for auto-feeding.
     */
    int32_t deficit_threshold_mg = 0;

    /**
     * Previous value of millis().
     */
    unsigned long update_prev_millis = 0;

    /**
     * Amount of time passed since the reservoir sensor was read.
     */
    unsigned long millis_since_reservoir_read = std::numeric_limits<unsigned long>::max() / 2;

    /**
     * Amount of time passed since the bowl sensor was read.
     */
    unsigned long millis_since_bowl_read = std::numeric_limits<unsigned long>::max() / 2;

    /**
     * Amount of time passed since the last time we attempted to feed.
     */
    unsigned long millis_since_feed_attempt = 0;

    /**
     * Amount of time passed since last state transition.
     */
    unsigned long millis_since_transition = 0;

    /**
     * Milliseconds since we've last forced an MQTT string update.
     */
    unsigned long millis_since_mqtt_string_update = 0;

    /**
     * Number of times a state has been retried.
     */
    uint16_t state_retries = 0;

    /**
     * Number of times in a row that we've failed to feed due to excessive
     * sensor stddev, presumed to be due to cat.
     */
    uint16_t feed_sensor_retries = 0;

    /**
     * Number of times in a row that we've failed to dispense a reasonable
     * amount of kibble according to sensors.
     */
    uint16_t feed_jammed_retries = 0;

    /**
     * Maximum number of retries for feeding.
     */
    static constexpr uint16_t FEED_MAX_RETRIES = 3;

    /**
     * Weight in grams of reservoir before feeding cycle.
     */
    float feed_reservoir_pre = 0.0f;

    /**
     * Weight in grams of reservoir after feeding cycle.
     */
    float feed_reservoir_post = 0.0f;

    /**
     * Weight in grams of bowl before feeding cycle.
     */
    float feed_bowl_pre = 0.0f;

    /**
     * Weight in grams of bowl after feeding cycle. Only used if
     * feed_bowl_post_valid is set, because of high cat interference potential.
     */
    float feed_bowl_post = 0.0f;

    /**
     * Whether the post-measurement for the bowl is valid.
     */
    bool feed_bowl_post_valid = false;

    /**
     * Information about the most recent feed attempt.
     */
    FeedReport feed_report = {FeedResult::NONE, 0, 0};

    /**
     * Motor takes about 2 seconds to do one cycle. If state transitions take
     * much longer than that we time out.
     */
    static constexpr unsigned long FEED_RUN_TIMEOUT_MILLIS = 3000;

    /**
     * If we're not getting feedback from the limit switch, run the motor for
     * the expected amount of time.
     */
    static constexpr unsigned long FEED_RUN_LIMP_MILLIS = 2000;

    /**
     * After limit switch release, run this amount of extra time.
     */
    static constexpr unsigned long FEED_RUN_POST_MILLIS = 10;

    /**
     * Wait this amount of time after the motor run before doing the
     * post-feed load cell measurements.
     */
    static constexpr unsigned long FEED_TO_MEASURE_MILLIS = 800;

    /**
     * Minimum time between feed attempts.
     */
#ifdef DEBUG_FSM
    static constexpr unsigned long FEED_COOLDOWN_MILLIS = 3000;
#else
    static constexpr unsigned long FEED_COOLDOWN_MILLIS = 5 * 60 * 1000;
#endif
    /**
     * Assumed weight in grams for a feeding cycle if the sensor values aren't
     * reasonable.
     */
    static constexpr float FEED_ASSUMED_WEIGHT_GRAMS = 9.0f;

    /**
     * Maximum disagreement between sensors.
     */
    static constexpr float FEED_MAX_DISAGREE_GRAMS = 5.0f;

    /**
     * Returns whether we're operating in loadcell limp mode. That is, load
     * cells aren't used; portion size is assumed. If we are in a limp mode,
     * return value is a string representing why.
     */
    [[nodiscard]] const char *loadcell_limp_mode() const;

    /**
     * Reasons why feeding might be blocked from the idle state.
     */
    enum struct FeedBlockReason {
        NOT_BLOCKED,
        MAINTENANCE,
        JAMMED,
        COOLDOWN,
        DEFICIT
    };

    /**
     * Returns whether we need to auto-feed.
     */
    [[nodiscard]] FeedBlockReason need_to_feed() const;

    /**
     * Transitions to the given state.
     */
    void transition(State new_state);

    /**
     * Handles loadcell result. Returns whether the loadcell readout is complete.
     */
    bool handle_loadcell_readout();

    /**
     * Returns the estimated dispensed weight after a feeding cycle.
     */
    float estimate_dispensed_weight_grams();

    /**
     * Completes a feeding cycle.
     */
    void complete_feed();

public:

    /**
     * Published floating-point value.
     */
    class PublishedFloatSensor {
    private:
        friend class StateMachine;

        /**
         * Most recent value.
         */
        float value = 0.0f;

        /**
         * MQTT manager.
         */
        HASensorNumber mqtt;

        /**
         * Sets the value, optionally forcing MQTT update.
         */
        void set(float new_value, bool force = false);

    public:
        PublishedFloatSensor(const char *unique_id, const char *name, const char *unit, const char *icon, int16_t expiry, HABaseDeviceType::NumberPrecision precision = HABaseDeviceType::PrecisionP1);

        [[nodiscard]] float get() const;
    };

    /**
     * Last value of reservoir weight.
     */
    PublishedFloatSensor reservoir_mean{"reservoir_weight", "Reservoir weight", "g", "mdi:scale", 0, HABaseDeviceType::PrecisionP1};

    /**
     * Last value of reservoir weight standard deviation.
     */
    PublishedFloatSensor reservoir_stddev{"reservoir_weight_stddev", "Reservoir weight stddev", "g", "mdi:sigma-lower", 0, HABaseDeviceType::PrecisionP1};

    /**
     * Last value of bowl weight.
     */
    PublishedFloatSensor bowl_mean{"bowl_weight", "Bowl weight", "g", "mdi:scale", 0, HABaseDeviceType::PrecisionP1};

    /**
     * Last value of bowl weight standard deviation.
     */
    PublishedFloatSensor bowl_stddev{"bowl_weight_stddev", "Bowl weight stddev", "g", "mdi:sigma-lower", 0, HABaseDeviceType::PrecisionP1};

    /**
     * Current deficit.
     */
    PublishedFloatSensor mqtt_deficit{"deficit", "Deficit", "g", "mdi:sigma", 0, HABaseDeviceType::PrecisionP1};

    /**
     * Last feed amount.
     */
    PublishedFloatSensor mqtt_last_feed{"last_feed", "Last amount fed", "g", "mdi:scale", 0, HABaseDeviceType::PrecisionP1};

    /**
     * Grams per day feedback.
     */
    PublishedFloatSensor mqtt_grams_per_day{"grams_per_day_fb", "Actual grams per day", "g", "mdi:food-drumstick", 0, HABaseDeviceType::PrecisionP0};

    /**
     * Published binary value.
     */
    class PublishedBinarySensor {
    private:
        friend class StateMachine;

        /**
         * Most recent value.
         */
        bool value = false;

        /**
         * MQTT manager.
         */
        HABinarySensor mqtt;

        /**
         * Sets the value, optionally forcing MQTT update.
         */
        void set(bool new_value, bool force = false);

    public:
        PublishedBinarySensor(const char *unique_id, const char *name, const char *icon, int16_t expiry);

        [[nodiscard]] bool get() const;
    };

    /**
     * Whether feeding is in progress.
     */
    PublishedBinarySensor mqtt_feeding{"feeding", "Currently feeding", "mdi:food-drumstick", 0};

    /**
     * Whether maintenance is in progress.
     */
    PublishedBinarySensor mqtt_maintenance{"maintenance", "Maintenance mode", "mdi:cog", 0};

    /**
     * Whether we're jammed.
     */
    PublishedBinarySensor mqtt_jammed{"jammed", "Jammed", "mdi:alert", 0};

    /**
     * Published string value.
     */
    template <size_t BUF_SIZE>
    class PublishedStringSensor {
    private:
        friend class StateMachine;

        /**
         * Most recent value.
         */
        char value[BUF_SIZE] = {0};

        /**
         * MQTT manager.
         */
        HASensor mqtt;

        /**
         * Sets the value, optionally forcing MQTT update.
         */
        void set(const char *new_value, bool force = false) {
            if (force || strcmp(value, new_value)) {
                snprintf(value, sizeof(value), "%s", new_value);
                mqtt.setValue(new_value);
            }
        }

    public:
        PublishedStringSensor(const char *unique_id, const char *name, const char *icon, const int16_t expiry) : mqtt(unique_id) {
            mqtt.setName(name);
            mqtt.setIcon(icon);
            mqtt.setExpireAfter(expiry);
        }

        [[nodiscard]] const char *get() const {
            return value;
        }
    };

    /**
     * Error message.
     */
    PublishedStringSensor<21> mqtt_error{"error", "Error message", "mdi:alert", 0};

    /**
     * Initializes the driver.
     */
    void begin();

    /**
     * Updates the state machine.
     */
    void update();

    /**
     * Resets to maintenance mode.
     */
    void enter_maintenance();

    /**
     * Whether we're currently in maintenance mode.
     */
    [[nodiscard]] bool maintenance() const;

    /**
     * Resets to maintenance mode and tares empty feeding reservoir.
     */
    void tare_reservoir();

    /**
     * Resets to maintenance mode and tares empty feeding bowl.
     */
    void tare_bowl();

    /**
     * Reset maintenance state, error state, etc.
     */
    void reset();

    /**
     * Starts a feeding cycle.
     */
    void feed();

    /**
     * Returns current deficit in milligrams.
     */
    [[nodiscard]] int32_t get_deficit() const;

    /**
     * Adjusts the feeding deficit by the given amount of milligrams. Positive
     * numbers result in (earlier) automatic feeding, negative numbers can be
     * used when fed manually.
     */
    void adjust_deficit(int32_t milligrams);

    /**
     * Returns current grams per day setting.
     */
    [[nodiscard]] int32_t get_grams_per_day() const;

    /**
     * Adjusts target grams per day.
     */
    void set_grams_per_day(int32_t new_grams_per_day);

    /**
     * Returns string representations of the current high-level state.
     */
    void get_state_report(StateReport &report) const;

    /**
     * Returns information about the previous feed.
     */
    [[nodiscard]] const FeedReport &get_feed_report() const;

    /**
     * Returns a string representation of the most severe error message along
     * with a severity level. Max 20 characters. Returns a null string along
     * with OKAY if there is no error to report.
     */
    [[nodiscard]] ErrorReport get_error_report() const;

};
