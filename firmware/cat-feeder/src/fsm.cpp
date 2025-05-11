#include "fsm.h"
#include "pins.h"

void StateMachine::error_reset() {
    error_reservoir_stddev = false;
    error_bowl_stddev = false;
    error_loadcell_timeout = false;
    error_loadcell_disagree = false;
    error_loadcell_unreasonable = false;
    error_limit_switch = false;
    error_power_loss = false;
}

[[nodiscard]] const char *StateMachine::loadcell_limp_mode() const {
    if (error_loadcell_timeout) return "Sensor timeout";
    if (error_reservoir_stddev) return "Reservoir noisy";
    if (error_bowl_stddev) return "Bowl noisy";
    if (error_loadcell_disagree) return "Sensor disagree";
    if (error_loadcell_unreasonable) return "Sensor sanity";
    return nullptr;
}

[[nodiscard]] StateMachine::FeedBlockReason StateMachine::need_to_feed() const {
    // Do not auto-feed during maintenance or fatal errors.
    switch (maintenance_mode) {
        case MaintenanceMode::OPERATIONAL: break;
        case MaintenanceMode::MAINTENANCE: return FeedBlockReason::MAINTENANCE;
        case MaintenanceMode::JAMMED: return FeedBlockReason::JAMMED;
    }

    // Auto-feed only if the deficit is more than the set maximum.
    if (deficit_mg < deficit_threshold_mg) return FeedBlockReason::DEFICIT;

    // Auto-feed cooldown.
    if (millis_since_feed_attempt < FEED_COOLDOWN_MILLIS) return FeedBlockReason::COOLDOWN;

    return FeedBlockReason::NOT_BLOCKED;
}

void StateMachine::transition(State new_state) {
    if (new_state == state) {
        state_retries++;
    } else {
        state_retries = 0;
    }
    Serial.printf("Transition to %d after %d, retry %d, maint %d\n", static_cast<int>(new_state), static_cast<int>(millis_since_transition), static_cast<int>(state_retries), static_cast<int>(maintenance_mode));
    switch (new_state) {
        case State::IDLE_TARE_RESERVOIR:
            loadcell.start(Loadcell::Sensor::RESERVOIR, true);
            break;
        case State::IDLE_TARE_BOWL:
            loadcell.start(Loadcell::Sensor::BOWL, true);
            break;
        case State::IDLE_MEASURE_RESERVOIR:
        case State::FEED_PRE_MEASURE_RESERVOIR:
        case State::FEED_POST_MEASURE_RESERVOIR:
            loadcell.start(Loadcell::Sensor::RESERVOIR);
            break;
        case State::IDLE_MEASURE_BOWL:
        case State::FEED_PRE_MEASURE_BOWL:
        case State::FEED_POST_MEASURE_BOWL:
            loadcell.start(Loadcell::Sensor::BOWL);
            break;
        default:
            break;
    }
    millis_since_transition = 0;
    state = new_state;
}

bool StateMachine::handle_loadcell_readout() {
    if (error_loadcell_timeout || millis_since_transition > 10000) {
        error_loadcell_timeout = true;
        return true;
    }
    if (loadcell.is_busy()) return false;
    switch (loadcell.get_sensor()) {
        case Loadcell::Sensor::RESERVOIR:
            reservoir_mean.set(loadcell.get_mean(), true);
            reservoir_stddev.set(loadcell.get_stddev(), true);
            millis_since_reservoir_read = 0;
            break;
        case Loadcell::Sensor::BOWL:
            bowl_mean.set(loadcell.get_mean(), true);
            bowl_stddev.set(loadcell.get_stddev(), true);
            millis_since_bowl_read = 0;
            break;
    }
    return true;
}

float StateMachine::estimate_dispensed_weight_grams() {

    // If the loadcell didn't work right before, use fallback value.
    if (loadcell_limp_mode()) {
        return FEED_ASSUMED_WEIGHT_GRAMS;
    }

    // Determine weight deltas.
    float dispensed_reservoir = feed_reservoir_pre - feed_reservoir_post;
    float dispensed_bowl = feed_bowl_post - feed_bowl_pre;

    // If the sensors disagree by too much, fail.
    if (abs(dispensed_reservoir - dispensed_bowl) > FEED_MAX_DISAGREE_GRAMS) {
        error_loadcell_disagree = true;
        return FEED_ASSUMED_WEIGHT_GRAMS;
    }
    float dispensed = (dispensed_reservoir + dispensed_bowl) / 2.0f;

    // Check reasonableness.
    if (dispensed < -2.0f || dispensed > FEED_ASSUMED_WEIGHT_GRAMS * 3.0f) {
        error_loadcell_unreasonable = true;
        return FEED_ASSUMED_WEIGHT_GRAMS;
    }

    return dispensed;
}

void StateMachine::complete_feed() {

    // Try to figure out how much kibble was dispensed.
    float dispensed_weight_grams = estimate_dispensed_weight_grams();

    // If the amount is too little, flag that the reservoir is probably
    // empty or something is jammed.
    if (dispensed_weight_grams > FEED_ASSUMED_WEIGHT_GRAMS * 0.3f) {
        feed_jammed_retries = 0;
        if (maintenance_mode == MaintenanceMode::JAMMED) {
            maintenance_mode = MaintenanceMode::OPERATIONAL;
        }
    } else {
        feed_jammed_retries++;
        if (feed_jammed_retries >= FEED_MAX_RETRIES) {
            maintenance_mode = MaintenanceMode::JAMMED;
        }
    }

    // Update deficit.
    int dispensed_weight_mg = static_cast<int>(dispensed_weight_grams * 1000.0f);
    deficit_mg -= dispensed_weight_mg;

    // State management.
    millis_since_feed_attempt = 0;
    feed_sensor_retries = 0;
    feed_report.result = FeedResult::SUCCESS;
    feed_report.arg = dispensed_weight_mg;
    feed_report.millis = millis();
    mqtt_last_feed.set(dispensed_weight_grams, true);
    transition(State::IDLE);
}

void StateMachine::PublishedFloatSensor::set(const float new_value, const bool force) {
    value = new_value;
    mqtt.setValue(value, force);
}

StateMachine::PublishedFloatSensor::PublishedFloatSensor(const char *unique_id, const char *name, const char *unit, const char *icon, const int16_t expiry, const HABaseDeviceType::NumberPrecision precision) : mqtt(unique_id, precision) {
    mqtt.setName(name);
    mqtt.setIcon(icon);
    mqtt.setUnitOfMeasurement(unit);
    mqtt.setExpireAfter(expiry);
}

[[nodiscard]] float StateMachine::PublishedFloatSensor::get() const {
    return value;
}

void StateMachine::PublishedBinarySensor::set(const bool new_value, const bool force) {
    value = new_value;
    mqtt.setState(value, force);
}

StateMachine::PublishedBinarySensor::PublishedBinarySensor(const char *unique_id, const char *name, const char *icon, const int16_t expiry) : mqtt(unique_id) {
    mqtt.setName(name);
    mqtt.setIcon(icon);
    mqtt.setExpireAfter(expiry);
}

[[nodiscard]] bool StateMachine::PublishedBinarySensor::get() const {
    return value;
}

void StateMachine::begin() {
    // Initialize motor control pins.
    pinMode(PIN_LIMIT, INPUT_PULLUP);
    pinMode(PIN_MOTOR, OUTPUT);
    digitalWrite(PIN_MOTOR, LOW);

    // Initialize loadcell driver.
    loadcell.begin();
    loadcell.set_tare_raw(Loadcell::Sensor::RESERVOIR, -754589);
    loadcell.set_tare_raw(Loadcell::Sensor::BOWL, 31485);

    // Initialize time delta logic.
    update_prev_millis = millis();
}

void StateMachine::update() {
    // Update owned lower-level drivers.
    loadcell.update();

    // Figure out time delta.
    const unsigned long current_millis = millis();
    const auto delta_millis = static_cast<int32_t>(current_millis - update_prev_millis);
    update_prev_millis = current_millis;

    // Update deficit.
    deficit_ms_remain -= delta_millis;
    while (deficit_ms_remain < 0) {
        deficit_ms_remain += 86400 / grams_per_day;
        deficit_mg += 1;
    }
    mqtt_deficit.set(static_cast<float>(deficit_mg) / 1000.0f);

    // Update MQTT status.
    bool force_update = false;
    if (millis_since_mqtt_string_update > 5000) {
        force_update = true;
        millis_since_mqtt_string_update = 0;
    } else {
        millis_since_mqtt_string_update += delta_millis;
    }
    switch (state) {
        case State::IDLE:
        case State::IDLE_TARE_RESERVOIR_WAIT:
        case State::IDLE_TARE_RESERVOIR:
        case State::IDLE_TARE_BOWL:
        case State::IDLE_MEASURE_RESERVOIR:
        case State::IDLE_MEASURE_BOWL:
            mqtt_feeding.set(false);
            break;
        case State::FEED_PRE_MEASURE_WAIT:
        case State::FEED_PRE_MEASURE_RESERVOIR:
        case State::FEED_PRE_MEASURE_BOWL:
        case State::FEED_RUN_SYNC:
        case State::FEED_RUN_A:
        case State::FEED_RUN_B:
        case State::FEED_RUN_C:
        case State::FEED_POST_WAIT:
        case State::FEED_POST_MEASURE_BOWL:
        case State::FEED_POST_MEASURE_RESERVOIR:
            mqtt_feeding.set(true);
            // Avoid long stuff while doing motor stuff :/
            force_update = false;
            break;
    }
    mqtt_maintenance.set(maintenance_mode == MaintenanceMode::MAINTENANCE, force_update);
    mqtt_jammed.set(maintenance_mode == MaintenanceMode::JAMMED, force_update);
    auto er = get_error_report();
    mqtt_error.set(er.message ? er.message : "No error", force_update);
    mqtt_grams_per_day.set(grams_per_day, force_update);

    // Update regular timers.
    millis_since_reservoir_read += delta_millis;
    millis_since_bowl_read += delta_millis;
    millis_since_feed_attempt += delta_millis;
    millis_since_transition += delta_millis;

    // Read limit switch.
    bool limit = digitalRead(PIN_LIMIT) == HIGH;

    // Handle state machine.
    bool motor = false;
    switch (state) {
        case State::IDLE: {
            // Check if we need to do an automatic feed.
            if (need_to_feed() == FeedBlockReason::NOT_BLOCKED) {
                feed();
                break;
            }

            // Check if we need to sample one of our sensors. Read sensors
            // continuously while in maintenance mode, otherwise read once
            // every five minutes.
            unsigned long sensor_read_cooldown = 5 * 60 * 1000;
            if (maintenance_mode == MaintenanceMode::MAINTENANCE) {
                sensor_read_cooldown = 0;
            }
            if (millis_since_reservoir_read > millis_since_bowl_read) {
                if (millis_since_reservoir_read > sensor_read_cooldown) {
                    transition(State::IDLE_MEASURE_RESERVOIR);
                }
            } else {
                if (millis_since_bowl_read > sensor_read_cooldown) {
                    transition(State::IDLE_MEASURE_BOWL);
                }
            }
            break;
        }

        case State::IDLE_TARE_RESERVOIR_WAIT:
            if (millis_since_transition > 2000) {
                transition(State::IDLE_TARE_RESERVOIR);
            }
            break;

        case State::IDLE_TARE_RESERVOIR:
            if (handle_loadcell_readout()) {
                transition(State::IDLE);
            }
            break;

        case State::IDLE_MEASURE_RESERVOIR:
        case State::IDLE_TARE_BOWL:
        case State::IDLE_MEASURE_BOWL:
            if (handle_loadcell_readout()) {
                transition(State::IDLE);
            }
            break;

        case State::FEED_PRE_MEASURE_WAIT:
            if (millis_since_transition > 2000) {
                transition(State::FEED_PRE_MEASURE_RESERVOIR);
            }
            break;

        case State::FEED_PRE_MEASURE_RESERVOIR:
            if (!loadcell_limp_mode()) {
                if (!handle_loadcell_readout()) {
                    break;
                }
                if (loadcell.get_stddev() < 1.0) {
                    feed_reservoir_pre = loadcell.get_mean();
                    transition(State::FEED_PRE_MEASURE_BOWL);
                    break;
                }
                if (state_retries < 5) {
                    transition(state);
                    break;
                }
                if (feed_sensor_retries <= FEED_MAX_RETRIES) {
                    millis_since_feed_attempt = 0;
                    feed_sensor_retries++;
                    feed_report.result = FeedResult::SENSOR_RETRY;
                    feed_report.arg = feed_sensor_retries;
                    feed_report.millis = millis();
                    transition(State::IDLE);
                    break;
                }
            }

            // Limp mode.
            error_reservoir_stddev = true;
            transition(State::FEED_PRE_MEASURE_BOWL);
            break;

        case State::FEED_PRE_MEASURE_BOWL:
            if (!loadcell_limp_mode()) {
                if (!handle_loadcell_readout()) {
                    break;
                }
                if (loadcell.get_stddev() < 1.0) {
                    feed_bowl_pre = loadcell.get_mean();
                    transition(State::FEED_RUN_SYNC);
                    break;
                }
                if (state_retries < 5) {
                    transition(state);
                    break;
                }
                if (feed_sensor_retries <= FEED_MAX_RETRIES) {
                    millis_since_feed_attempt = 0;
                    feed_sensor_retries++;
                    feed_report.result = FeedResult::SENSOR_RETRY;
                    feed_report.arg = feed_sensor_retries;
                    feed_report.millis = millis();
                    transition(State::IDLE);
                    break;
                }
            }

            // Limp mode.
            error_bowl_stddev = true;
            transition(State::FEED_RUN_SYNC);
            break;

        case State::FEED_RUN_SYNC:
            motor = true;
            if (!error_limit_switch) {
                if (!limit) {
                    transition(State::FEED_RUN_A);
                    break;
                }
                if (millis_since_transition < FEED_RUN_TIMEOUT_MILLIS) {
                    break;
                }
            } else {
                if (millis_since_transition > FEED_RUN_LIMP_MILLIS) {
                    transition(State::FEED_POST_WAIT);
                }
                break;
            }

            // Error. Assume motor already moved a bunch and continue.
            error_limit_switch = true;
            transition(State::FEED_POST_WAIT);
            break;

        case State::FEED_RUN_A:
            motor = true;
            if (limit && millis_since_transition > 50) {
                transition(State::FEED_RUN_B);
                break;
            }
            if (millis_since_transition < FEED_RUN_TIMEOUT_MILLIS) {
                break;
            }

            // Error. Assume motor already moved a bunch and continue.
            error_limit_switch = true;
            transition(State::FEED_POST_WAIT);
            break;

        case State::FEED_RUN_B:
            motor = true;
            if (!limit && millis_since_transition > 50) {
                transition(State::FEED_RUN_C);
                break;
            }
            if (millis_since_transition < FEED_RUN_TIMEOUT_MILLIS) {
                break;
            }

            // Error. Assume motor already moved a bunch and continue.
            error_limit_switch = true;
            transition(State::FEED_POST_WAIT);
            break;

        case State::FEED_RUN_C:
            motor = true;
            if (millis_since_transition > FEED_RUN_POST_MILLIS) {
                transition(State::FEED_POST_WAIT);
            }
            break;

        case State::FEED_POST_WAIT:
            if (millis_since_transition > FEED_TO_MEASURE_MILLIS) {
                transition(State::FEED_POST_MEASURE_BOWL);
            }
            break;

        case State::FEED_POST_MEASURE_BOWL:
            if (!loadcell_limp_mode()) {
                if (!handle_loadcell_readout()) {
                    break;
                }
                if (loadcell.get_stddev() < 1.0) {
                    feed_bowl_post = loadcell.get_mean();
                    transition(State::FEED_POST_MEASURE_RESERVOIR);
                    break;
                }
                if (state_retries < 5) {
                    transition(state);
                    break;
                }
            }

            // Limp mode.
            error_bowl_stddev = true;
            transition(State::FEED_POST_MEASURE_RESERVOIR);
            break;

        case State::FEED_POST_MEASURE_RESERVOIR:
            if (!loadcell_limp_mode()) {
                if (!handle_loadcell_readout()) {
                    break;
                }
                if (loadcell.get_stddev() < 1.0) {
                    feed_reservoir_post = loadcell.get_mean();
                    complete_feed();
                    break;
                }
                if (state_retries < 5) {
                    transition(state);
                    break;
                }
            }

            // Limp mode.
            error_reservoir_stddev = true;
            complete_feed();
            break;
    }

    // Update motor state.
    digitalWrite(PIN_MOTOR, motor);
}

void StateMachine::enter_maintenance() {
    error_reset();
    maintenance_mode = MaintenanceMode::MAINTENANCE;
    transition(State::IDLE);
}

[[nodiscard]] bool StateMachine::maintenance() const {
    return maintenance_mode == MaintenanceMode::MAINTENANCE;
}

void StateMachine::tare_reservoir() {
    maintenance_mode = MaintenanceMode::MAINTENANCE;
    transition(State::IDLE_TARE_RESERVOIR_WAIT);
}

void StateMachine::tare_bowl() {
    maintenance_mode = MaintenanceMode::MAINTENANCE;
    transition(State::IDLE_TARE_BOWL);
}

void StateMachine::reset() {
    error_reset();
    maintenance_mode = MaintenanceMode::OPERATIONAL;
    transition(State::IDLE);
    state_retries = 0;
    feed_jammed_retries = 0;
    feed_sensor_retries = 0;
}

void StateMachine::feed() {
    transition(State::FEED_PRE_MEASURE_WAIT);
}

[[nodiscard]] int32_t StateMachine::get_deficit() const {
    return deficit_mg;
}

void StateMachine::adjust_deficit(int32_t milligrams) {
    deficit_mg += milligrams;
}

[[nodiscard]] int32_t StateMachine::get_grams_per_day() const {
    return grams_per_day;
}

void StateMachine::set_grams_per_day(int32_t new_grams_per_day) {
    grams_per_day = new_grams_per_day;
}

/**
 * Returns a string representation of the current high-level state.
 */
void StateMachine::get_state_report(StateReport &report) const {
    report.header[0] = 0;
    report.detail1[0] = 0;
    report.detail2[0] = 0;
    report.large = false;
    int progress = 0;
    switch (state) {
        case State::IDLE:
        case State::IDLE_MEASURE_RESERVOIR:
        case State::IDLE_MEASURE_BOWL:
            if (feed_report.result == FeedResult::SUCCESS && (millis() - feed_report.millis) < 10000) {
                strcpy(report.header, "Feed result");
                snprintf(report.detail1, sizeof(report.detail1), "R %+7.1fg %+7.1fg", feed_reservoir_pre, feed_reservoir_post - feed_reservoir_pre);
                snprintf(report.detail2, sizeof(report.detail2), "B %+7.1fg %+7.1fg", feed_bowl_pre, feed_bowl_post - feed_bowl_pre);
                return;
            }
            switch (need_to_feed()) {
                case FeedBlockReason::MAINTENANCE:
                    strcpy(report.header, "Maintenance");
                    goto details_maintenance;
                case FeedBlockReason::JAMMED:
                    strcpy(report.detail1, "JAMMED");
                    report.large = true;
                    return;
                case FeedBlockReason::COOLDOWN: {
                    strcpy(report.header, "Cooldown");
                    unsigned long remain = FEED_COOLDOWN_MILLIS - millis_since_feed_attempt;
                    if (remain < FEED_COOLDOWN_MILLIS) {
                        unsigned long seconds = remain / 1000;
                        unsigned long minutes = seconds / 60;
                        seconds -= minutes * 60;
                        snprintf(report.detail1, sizeof(report.detail1), "%d:%02d", static_cast<int>(minutes), static_cast<int>(seconds));
                    }
                    report.large = true;
                    return;
                }
                case FeedBlockReason::DEFICIT: {
                    strcpy(report.header, "Deficit");
                    int deficit = deficit_mg - deficit_threshold_mg;
                    snprintf(report.detail1, sizeof(report.detail1), "%dmg", deficit);
                    report.large = true;
                    return;
                }
                default:
                    progress = 0;
                    goto details_feeding;
            }
        case State::IDLE_TARE_RESERVOIR_WAIT:
        case State::IDLE_TARE_RESERVOIR:
            strcpy(report.header, "Tare reservoir");
            goto details_maintenance;
        case State::IDLE_TARE_BOWL:
            strcpy(report.header, "Tare bowl");
            goto details_maintenance;
        details_maintenance:
            snprintf(report.detail1, sizeof(report.detail1), "%+7.1fg +/-%6.1fg", reservoir_mean.get(), reservoir_stddev.get());
            snprintf(report.detail2, sizeof(report.detail2), "%+7.1fg +/-%6.1fg", bowl_mean.get(), bowl_stddev.get());
            return;
        case State::FEED_PRE_MEASURE_WAIT:
            progress = 0;
            goto details_feeding;
        case State::FEED_PRE_MEASURE_RESERVOIR:
            progress = 1;
            goto details_feeding;
        case State::FEED_PRE_MEASURE_BOWL:
            progress = 2;
            goto details_feeding;
        case State::FEED_RUN_SYNC:
            progress = 3;
            goto details_feeding;
        case State::FEED_RUN_A:
            progress = 4;
            goto details_feeding;
        case State::FEED_RUN_B:
            progress = 5;
            goto details_feeding;
        case State::FEED_RUN_C:
            progress = 6;
            goto details_feeding;
        case State::FEED_POST_WAIT:
            progress = 7;
            goto details_feeding;
        case State::FEED_POST_MEASURE_BOWL:
            progress = 8;
            goto details_feeding;
        case State::FEED_POST_MEASURE_RESERVOIR:
            progress = 9;
            goto details_feeding;
        details_feeding:
            strcpy(report.header, "Feeding");
            for (int i = 0; i < 10; i++) {
                report.detail1[i] = i < progress ? '#' : '-';
            }
            report.detail1[10] = 0;
            report.large = true;
            return;
    }
}

[[nodiscard]] const StateMachine::FeedReport &StateMachine::get_feed_report() const {
    return feed_report;
}

[[nodiscard]] StateMachine::ErrorReport StateMachine::get_error_report() const {
    // Errors.
    if (error_limit_switch) return {"Motor timeout", ErrorSeverity::ERROR};
    if (const auto mode = loadcell_limp_mode()) return {mode, ErrorSeverity::ERROR};
    if (error_power_loss) return {"Power loss", ErrorSeverity::ERROR};
    if (maintenance_mode == MaintenanceMode::JAMMED) return { "Jammed/empty", ErrorSeverity::ERROR };

    // Warnings.
    if (feed_jammed_retries) return { "Jammed/empty?", ErrorSeverity::WARNING };
    if (reservoir_mean.get() < 250.0f) return { "Reservoir low", ErrorSeverity::WARNING };

    // Operational.
    return { nullptr, ErrorSeverity::OKAY };
}
