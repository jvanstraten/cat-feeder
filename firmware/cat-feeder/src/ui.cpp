#include "ui.h"

#include <WiFi.h>

void UserInterface::render_line(const size_t y, const char *buffer, const size_t scale, const bool grayed) {
    size_t w = strlen(buffer) * 6u * scale;
    const size_t h = 8u * scale;
    if (w > 240) w = 240;
    const size_t x = (240 - w) / 2;
    const size_t r = 240 - x - w;
    if (x) tft.fillRect(0, y, x, h, color_bg);
    if (r) tft.fillRect(x + w, y, r, h, color_bg);
    if (!w) return;
    tft.setCursor(x, y);
    tft.setTextColor(grayed ? color_gr : color_fg, color_bg);
    tft.setTextSize(scale);
    tft.setTextWrap(false);
    tft.print(buffer);
}

void UserInterface::display_preprocess() {
    error_report = fsm.get_error_report();
    state_report = {};
    fsm.get_state_report(state_report);
    feed_report = fsm.get_feed_report();
    feed_report_string[0] = 0;
    switch (feed_report.result) {
        case StateMachine::FeedResult::NONE:
            strcpy(feed_report_string, "None");
            break;
        case StateMachine::FeedResult::SUCCESS: {
            unsigned long ms = millis() - feed_report.millis;
            int s = static_cast<int>(ms / 1000);
            int m = s / 60;
            s -= m * 60;
            int h = m / 60;
            m -= h * 60;
            snprintf(feed_report_string, sizeof(feed_report_string), "%d:%02d:%02d   %7.1fg", h, m, s, static_cast<float>(feed_report.arg) / 1000.0f);
            break;
        }
        case StateMachine::FeedResult::SENSOR_RETRY:
            snprintf(feed_report_string, sizeof(feed_report_string), "Noise on sensor (x%d)", feed_report.arg);
            break;
    }

    // Pick colors based on severity.
    switch (error_report.severity) {
        case StateMachine::ErrorSeverity::OKAY:
            if (fsm.maintenance()) {
                color_fg = 0b0000011111111000;
                color_gr = 0b0000010000001100;
                color_bg = 0;
                brightness = 255;
            } else {
                color_fg = 0b1100011111100000;
                color_gr = 0b0110010000000000;
                color_bg = 0;
                brightness = 32;
            }
            break;
        case StateMachine::ErrorSeverity::WARNING:
            color_fg = 0;
            color_gr = 0b1000001000000000;
            color_bg = 0b1111110000000000;
            brightness = 255;
            break;
        case StateMachine::ErrorSeverity::ERROR:
            color_fg = 0;
            color_gr = 0b1000000000000000;
            color_bg = 0b1111100000000000;
            brightness = ((millis() >> 9) & 1) ? 255 : 32;
            break;
    }

    // Pick status message to print.
    status_grayed = false;
    if (error_report.message) {
        snprintf(status_string, sizeof(status_string), "%s", error_report.message);
    } else {
        switch (WiFi.status()) {
            case WL_IDLE_STATUS:
                strcpy(status_string, "WiFi: idle");
                break;
            case WL_NO_SSID_AVAIL:
                strcpy(status_string, "WiFi: no SSID");
                break;
            case WL_SCAN_COMPLETED:
                strcpy(status_string, "WiFi: scan complete");
                break;
            case WL_CONNECTED:
                switch (mqtt.getState()) {
                case HAMqtt::StateConnecting:
                    strcpy(status_string, "MQTT: connecting");
                    break;
                case HAMqtt::StateConnectionTimeout:
                    strcpy(status_string, "MQTT: conn. timeout");
                    break;
                case HAMqtt::StateConnectionLost:
                    strcpy(status_string, "MQTT: conn. lost");
                    break;
                case HAMqtt::StateConnectionFailed:
                    strcpy(status_string, "MQTT: connect failed");
                    break;
                case HAMqtt::StateDisconnected:
                    strcpy(status_string, "MQTT: disconnected");
                    break;
                case HAMqtt::StateConnected:
                    status_grayed = true;
                    snprintf(status_string, sizeof(status_string), "%s", WiFi.localIP().toString().c_str());
                    break;
                case HAMqtt::StateBadProtocol:
                    strcpy(status_string, "MQTT: bad protocol");
                    break;
                case HAMqtt::StateBadClientId:
                    strcpy(status_string, "MQTT: bad client ID");
                    break;
                case HAMqtt::StateUnavailable:
                    strcpy(status_string, "MQTT: unavailable");
                    break;
                case HAMqtt::StateBadCredentials:
                    strcpy(status_string, "MQTT: bad login");
                    break;
                case HAMqtt::StateUnauthorized:
                    strcpy(status_string, "MQTT: unauthorized");
                    break;
                }
                break;
            case WL_CONNECT_FAILED:
                strcpy(status_string, "WiFi: connect failed");
                break;
            case WL_CONNECTION_LOST:
                strcpy(status_string, "WiFi: conn. lost");
                break;
            case WL_DISCONNECTED:
                strcpy(status_string, "WiFi: disconnected");
                break;
            default:
                snprintf(status_string, sizeof(status_string), "WiFi: status %d", WiFi.status());
                break;
        }
    }
}

void UserInterface::display_update() {
    switch (display_update_state) {
        case 0:
            display_preprocess();
            break;

        case 1:
            render_line(68, "Last feed", 2, true);
            render_line(84, feed_report_string, 2);
            break;

        case 2:
            tft.fillRect(0, 100, 240, 8, color_bg);
            render_line(108, state_report.header, 2, true);
            break;

        case 3:
            if (state_report.large) {
                render_line(124, state_report.detail1, 4);
            } else {
                render_line(124, state_report.detail1, 2);
                render_line(140, state_report.detail2, 2);
            }
            break;

        case 4:
            tft.fillRect(0, 156, 240, 8, color_bg);
            render_line(164, status_string, 2, status_grayed);
            analogWrite(PIN_TFT_BL, brightness);
            // fallthrough

        default:
            display_update_state = 0;
            return;
    }
    display_update_state++;
}

UserInterface::Button::Button(const int pin) : pin(pin) {}

void UserInterface::Button::begin() {
    pinMode(pin, INPUT_PULLUP);
    state = 0;
}

bool UserInterface::Button::update() {
    if (digitalRead(pin) == LOW) {
        state = 3;
    } else if (state) {
        state--;
        if (!state) return true;
    }
    return false;
}

UserInterface::UserInterface(StateMachine &fsm, HAMqtt &mqtt) : tft(&SPI, PIN_TFT_DC, PIN_TFT_CS, PIN_TFT_RST), fsm(fsm), mqtt(mqtt) {
}

void UserInterface::begin() {
    display_update_state = 0;

    // Initialize pins.
    pinMode(PIN_FP_LED, OUTPUT);
    digitalWrite(PIN_FP_LED, LOW);
    pinMode(PIN_TFT_BL, OUTPUT);
    digitalWrite(PIN_TFT_BL, LOW);

    // Initialize key FSMs.
    key_set.begin();
    key_feed.begin();
    key_up.begin();
    key_down.begin();
    key_lock.begin();
    key_mic.begin();

    // Initialize display.
    SPI.setTX(PIN_TFT_SDA);
    SPI.setSCK(PIN_TFT_SCL);
    tft.begin();
    tft.setRotation(3);
    tft.fillRect(0, 60, 240, 120, 0);

}

void UserInterface::update() {

    // Update the display.
    display_update();

    // Update the keys.
    key_set.update(); // TODO
    if (key_feed.update()) fsm.feed();
    if (key_up.update()) fsm.tare_reservoir();
    if (key_down.update()) fsm.tare_bowl();
    if (key_lock.update()) fsm.reset();
    if (key_mic.update()) fsm.enter_maintenance();

}
