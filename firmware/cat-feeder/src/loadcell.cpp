#include "loadcell.h"
#include "pins.h"

void Loadcell::begin() {
    hx711.begin(PIN_LC_DATA, PIN_LC_CLK);
    hx711.read();
}

void Loadcell::start(Sensor target_sensor, bool tare) {
    sensor = target_sensor;
    if (sensor == Sensor::RESERVOIR) {
        hx711.set_gain(128);
    } else {
        hx711.set_gain(32);
    }
    hx711.read();
    samples_remaining = NUM_SAMPLES;
    apply_tare = tare;
}

void Loadcell::update() {
    if (!samples_remaining) return;
    if (!hx711.is_ready()) return;
    samples_remaining--;
    samples[samples_remaining] = hx711.read();
    if (samples_remaining) return;

    // Compute mean in raw measurement format.
    int64_t accum = NUM_SAMPLES / 2;
    for (auto sample : samples) {
        accum += sample;
    }
    mean_raw = static_cast<int32_t>(accum / static_cast<int64_t>(NUM_SAMPLES));

    // Compute variance in measurement format.
    accum = NUM_SAMPLES / 2;
    for (const auto sample : samples) {
        const int64_t diff = sample - mean_raw;
        accum += diff * diff;
    }
    const auto var = static_cast<float>(accum) / static_cast<float>(NUM_SAMPLES);

    // Figure out tare value and gain for selected sensor.
    int32_t tare = 0;
    float gain = 0.0f;
    switch (sensor) {
        case Sensor::RESERVOIR:
            if (apply_tare || tare_reservoir == std::numeric_limits<int32_t>::min()) tare_reservoir = mean_raw;
            tare = tare_reservoir;
            gain = GAIN_RESERVOIR;
            break;
        case Sensor::BOWL:
            if (apply_tare || tare_bowl == std::numeric_limits<int32_t>::min()) tare_bowl = mean_raw;
            tare = tare_bowl;
            gain = GAIN_BOWL;
            break;
    }

    // Compute mean and stddev in grams.
    mean = static_cast<float>(mean_raw - tare) * gain;
    stddev = sqrt(var) * abs(gain);
    Serial.printf("Measured sensor %d, raw %d, %.2g +/- %.2g\n", sensor, mean_raw, mean, stddev);
}

[[nodiscard]] bool Loadcell::is_busy() const {
    return samples_remaining > 0;
}

[[nodiscard]] float Loadcell::get_mean() const {
    return mean;
}

[[nodiscard]] float Loadcell::get_stddev() const {
    return stddev;
}

[[nodiscard]] Loadcell::Sensor Loadcell::get_sensor() const {
    return sensor;
}

[[nodiscard]] int32_t Loadcell::get_mean_raw() const {
    return mean_raw;
}

void Loadcell::set_tare_raw(const Sensor sensor, const int32_t raw) {
    switch (sensor) {
        case Sensor::RESERVOIR:
            tare_reservoir = raw;
            break;
        case Sensor::BOWL:
            tare_bowl = raw;
            break;
    }
}
