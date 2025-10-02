#pragma once

#include <Arduino.h>
#include <HX711.h>

/**
 * Loadcell/HX711 management class.
 */
class Loadcell {
public:
    enum class Sensor {
        RESERVOIR,
        BOWL,
    };

private:
    /**
     * Underlying driver.
     */
    HX711 hx711;

    /**
     * Number of samples to average.
     */
    static constexpr size_t NUM_SAMPLES = 32;

    /**
     * Gain to go from raw HX711 reservoir value to grams.
     */
    static constexpr float GAIN_RESERVOIR = -0.0020530327830519573f;

    /**
     * Gain to go from raw HX711 bowl value to grams.
     */
    static constexpr float GAIN_BOWL = 0.003227106961589246f;

    /**
     * Number of averaging samples remaining.
     */
    int samples_remaining = 0;

    /**
     * The sensor we're measuring.
     */
    Sensor sensor = Sensor::RESERVOIR;

    /**
     * Whether this measurement is a taring operation.
     */
    bool apply_tare = false;

    /**
     * List of samples.
     */
    int32_t samples[NUM_SAMPLES] = {};

    /**
     * Raw tare value for reservoir sensor.
     */
    int32_t tare_reservoir = std::numeric_limits<int32_t>::min();

    /**
     * Raw tare value for bowl sensor.
     */
    int32_t tare_bowl = std::numeric_limits<int32_t>::min();

    /**
     * Most recently measured mean.
     */
    float mean = 0.0f;

    /**
     * Most recently measured stddev.
     */
    float stddev = 0.0f;

    /**
     * Most recently measured mean in raw measurement unit.
     */
    int32_t mean_raw = 0;

public:
    /**
     * Initialize the driver.
     */
    void begin();

    /**
     * Start averaging loadcell data for the given loadcell. Any
     * previously-started measurement is stopped.
     */
    void start(Sensor sensor, bool tare = false);

    /**
     * Updates state machine from main loop.
     */
    void update();

    /**
     * Returns whether the loadcell readout logic is currently busy.
     */
    [[nodiscard]] bool is_busy() const;

    /**
     * Returns mean in grams.
     */
    [[nodiscard]] float get_mean() const;

    /**
     * Returns standard deviation in grams.
     */
    [[nodiscard]] float get_stddev() const;

    /**
     * Returns which sensor was most recently read.
     */
    [[nodiscard]] Sensor get_sensor() const;

    /**
     * Returns mean in raw measurement units.
     */
    [[nodiscard]] int32_t get_mean_raw() const;

    /**
     * Sets the tare value for the given sensor.
     */
    void set_tare_raw(Sensor sensor, int32_t raw);

};
