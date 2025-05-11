#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoHA.h>

#include "fsm.h"
#include "ui.h"

WiFiClient client;
HADevice device("catfeeder");
HAMqtt mqtt(client, device);
StateMachine fsm;
UserInterface ui(fsm, mqtt);

HAButton mqtt_feed {"feed"};
volatile bool mqtt_feed_flag = false;
void on_mqtt_feed(HAButton *sender) {
    (void)sender;
    mqtt_feed_flag = true;
}

HAButton mqtt_reset {"reset"};
volatile bool mqtt_reset_flag = false;
void on_mqtt_reset(HAButton *sender) {
    (void)sender;
    mqtt_reset_flag = true;
}

HAButton mqtt_maintenance {"enter_maintenance"};
volatile bool mqtt_maintenance_flag = false;
void on_mqtt_maintenance(HAButton *sender) {
    (void)sender;
    mqtt_maintenance_flag = true;
}

HANumber mqtt_grams_per_day {"grams_per_day", HABaseDeviceType::PrecisionP0};
volatile bool mqtt_grams_per_day_flag = false;
volatile int mqtt_grams_per_day_value = 0;
void on_mqtt_grams_per_day(const HANumeric number, HANumber *sender) {
    (void)sender;
    mqtt_grams_per_day_flag = true;
    mqtt_grams_per_day_value = number.toInt32();
}

HANumber mqtt_adjust_deficit_number {"adjust_deficit_amount", HABaseDeviceType::PrecisionP1};
volatile int32_t mqtt_adjust_deficit_amount = 0;
void on_mqtt_adjust_deficit_number(const HANumeric number, HANumber *sender) {
    (void)sender;
    mqtt_adjust_deficit_amount = static_cast<int32_t>(number.toFloat() * 1000.0f);
}

HAButton mqtt_adjust_deficit_button {"adjust_deficit_button"};
volatile bool mqtt_adjust_deficit_flag = false;
void on_mqtt_adjust_deficit_button(HAButton *sender) {
    (void)sender;
    mqtt_adjust_deficit_flag = true;
}

unsigned long last_wifi_reconnect = 0;

void wifi_connect() {
    WiFi.begin("TPL@PB40", "1Tilia5Nefit!");
}

void setup() {
    Serial.begin();
    fsm.begin();
    ui.begin();

    WiFi.mode(WIFI_STA);
    wifi_connect();

    device.setName("Cat feeder");
    device.enableSharedAvailability();
    device.enableLastWill();

    mqtt_feed.setName("Feed now");
    mqtt_feed.setIcon("mdi:food-drumstick");
    mqtt_feed.onCommand(on_mqtt_feed);

    mqtt_reset.setName("Reset state machine");
    mqtt_reset.setIcon("mdi:cog");
    mqtt_reset.onCommand(on_mqtt_reset);

    mqtt_maintenance.setName("Enter maintenance mode");
    mqtt_maintenance.setIcon("mdi:cog");
    mqtt_maintenance.onCommand(on_mqtt_maintenance);

    mqtt_grams_per_day.setName("Grams per day");
    mqtt_grams_per_day.setIcon("mdi:food-drumstick");
    mqtt_grams_per_day.setUnitOfMeasurement("g");
    mqtt_grams_per_day.setRetain(true);
    mqtt_grams_per_day.onCommand(on_mqtt_grams_per_day);
    mqtt_grams_per_day.setMin(0);
    mqtt_grams_per_day.setMax(150);
    mqtt_grams_per_day.setMode(HANumber::ModeBox);

    mqtt_adjust_deficit_number.setName("Adjust deficit by");
    mqtt_adjust_deficit_number.setIcon("mdi:delta");
    mqtt_adjust_deficit_number.setUnitOfMeasurement("g");
    mqtt_adjust_deficit_number.onCommand(on_mqtt_adjust_deficit_number);
    mqtt_adjust_deficit_number.setMin(-1000);
    mqtt_adjust_deficit_number.setMax(1000);
    mqtt_adjust_deficit_number.setMode(HANumber::ModeBox);

    mqtt_adjust_deficit_button.setName("Adjust deficit");
    mqtt_adjust_deficit_button.setIcon("mdi:delta");
    mqtt_adjust_deficit_button.onCommand(on_mqtt_adjust_deficit_button);

    mqtt.begin(IPAddress(192, 168, 1, 7), 1883, "jeroen", "Y0vzmMi90Q5egGzQFbfg");
}

void loop() {
    ui.update();
    fsm.update();
    mqtt.loop();

    if (mqtt_feed_flag) {
        mqtt_feed_flag = false;
        fsm.feed();
    }
    if (mqtt_reset_flag) {
        mqtt_reset_flag = false;
        fsm.reset();
    }
    if (mqtt_maintenance_flag) {
        mqtt_maintenance_flag = false;
        fsm.enter_maintenance();
    }
    if (mqtt_grams_per_day_flag) {
        mqtt_grams_per_day_flag = false;
        fsm.set_grams_per_day(mqtt_grams_per_day_value);
    }
    if (mqtt_adjust_deficit_flag) {
        mqtt_adjust_deficit_flag = false;
        fsm.adjust_deficit(mqtt_adjust_deficit_amount);
        mqtt_adjust_deficit_number.setState(static_cast<int32_t>(0), true);
    }

    if (WiFi.status() != WL_CONNECTED) {
        if ((millis() - last_wifi_reconnect) > 10000) {
            WiFi.begin("TPL@PB40", "1Tilia5Nefit!");
            last_wifi_reconnect = millis();
        }
    }

}
