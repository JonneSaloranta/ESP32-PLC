#include <WiFi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <FastLED_NeoPixel.h>

EventGroupHandle_t wifiEventGroup;
const int WIFI_CONNECTED_BIT = BIT0;

WiFiClient client;

const char *ssid = "ArduinoLabra";
const char *password = "12345678";

const char *server_ip = "192.168.1.101";
const int server_port = 12345;

const int emergencyPin = 18;

const int buzzer = 4;
const int beep_delay = 500;

const int limit1 = 22;
const int limit2 = 23;

const int m1dir = 27;
const int m1step = 26;
const int m2dir = 25;
const int m2step = 33;

const int ledPin = 32;
const int number_of_leds = 7;
const int brightness = 50;

unsigned long lastMillis = 0;

int max_brightness = 32;

CRGB leds[number_of_leds];
FastLED_NeoPixel_Variant strip(leds, number_of_leds);

uint32_t black = strip.Color(0, 0, 0);
uint32_t red = strip.Color(255, 0, 0);
uint32_t green = strip.Color(0, 255, 0);
uint32_t blue = strip.Color(0, 0, 255);
uint32_t yellow = strip.Color(255, 255, 0);
uint32_t purple = strip.Color(255, 0, 255);
uint32_t aqua = strip.Color(0, 255, 255);
uint32_t white = strip.Color(255, 255, 255);

enum State
{
    STOPPED,
    RUNNING,
    EMERGENCY,
    ERROR,
    CONNECTING_TO_WIFI,
    CONNECTED_TO_WIFI,
    CONNECTING_TO_SERVER,
    CONNECTED_TO_SERVER
};

enum State process_state;

String command = "";

// Function declarations
void maintainWiFi(void *parameter);
void handleClient(void *parameter);

void clear_command()
{
    command = "";
}

void number_of_flashes(uint32_t color, int ledPin, int flashes, int speed, bool used_in_core)
{
    strip.fill(color, 0, number_of_leds);
    for (int i = 0; i < flashes; i++)
    {
        strip.show();
        if (used_in_core)
        {
            vTaskDelay(speed / portTICK_PERIOD_MS);
        }
        else
        {
            delay(speed);
        }
        strip.fill(black, 0, number_of_leds);
        strip.show();
        if (used_in_core)
        {
            vTaskDelay(speed / portTICK_PERIOD_MS);
        }
        else
        {
            delay(speed);
        }
    }
}

void fade_flash(uint32_t color, int flashes, int speed, bool used_in_core)
{

    strip.fill(color, 0, number_of_leds);
    for (int i = 0; i < flashes; i++)
    {
        for (int j = 0; j < 255; j++)
        {
            strip.setBrightness(j);
            strip.show();
            if (used_in_core)
            {
                vTaskDelay(speed / portTICK_PERIOD_MS);
            }
            else
            {
                delay(speed);
            }
        }
        for (int j = 255; j > 0; j--)
        {
            strip.setBrightness(j);
            strip.show();
            if (used_in_core)
            {
                vTaskDelay(speed / portTICK_PERIOD_MS);
            }
            else
            {
                delay(speed);
            }
        }
    }
}

void set_led_color(uint32_t color)
{
    strip.fill(color, 0, number_of_leds);
    strip.show();
}

void set_led_color(uint32_t color, int led)
{
    strip.setPixelColor(led, color);
    strip.show();
}

void set_led_brightness(int brightness)
{
    strip.setBrightness(brightness);
    strip.show();
}

void setup_ready()
{
    tone(buzzer, 2500, 50);
    delay(200);
    noTone(buzzer);
    tone(buzzer, 500, 25);
    delay(200);
    noTone(buzzer);
    process_state = CONNECTING_TO_WIFI;
}

void ready_sound()
{
    tone(buzzer, 2500, 50);
    delay(200);
    noTone(buzzer);
    tone(buzzer, 500, 25);
    delay(200);
    noTone(buzzer);
}

void emergency_sound()
{
}

void emergency_state()
{
    Serial.println("Emergency state");
    number_of_flashes(red, ledPin, 1, 500, true);
    clear_command();
}

void set_direction(int motor, int direction)
{
    digitalWrite(motor, direction);
}

void move_motor(int motor, int steps, int delay_time)
{
    for (int i = 0; i < steps; i++)
    {
        digitalWrite(motor, HIGH);
        delayMicroseconds(delay_time);
        digitalWrite(motor, LOW);
        delayMicroseconds(delay_time);
    }
}

void setup()
{
    Serial.begin(115200);

    strip.begin(FastLED.addLeds<WS2812B, ledPin, GRB>(leds, number_of_leds));
    strip.setBrightness(max_brightness);

    // fade_flash(red, 1, 2, false);

    pinMode(m1step, OUTPUT);
    pinMode(m1dir, OUTPUT);
    pinMode(m2step, OUTPUT);
    pinMode(m2dir, OUTPUT);

    pinMode(limit1, INPUT_PULLUP);
    pinMode(limit2, INPUT_PULLUP);

    pinMode(emergencyPin, INPUT_PULLUP);

    pinMode(ledPin, OUTPUT);

    set_direction(m1dir, HIGH);
    move_motor(m1step, 100, 2000);

    // Create an event group
    wifiEventGroup = xEventGroupCreate();

    // Create tasks
    xTaskCreatePinnedToCore(
        maintainWiFi,
        "MaintainWiFi",
        10000,
        NULL,
        1,
        NULL,
        0);

    xTaskCreatePinnedToCore(
        handleClient,
        "HandleClient",
        10000,
        NULL,
        1,
        NULL,
        1);

    set_led_color(black);
    set_led_brightness(max_brightness);

    setup_ready();
}

void loop()
{
    // Empty loop, as tasks are handled in FreeRTOS tasks
}

void maintainWiFi(void *parameter)
{
    // Connect to WiFi
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.println("Connecting to WiFi..");
        number_of_flashes(blue, ledPin, 1, 1000, true);
    }
    process_state = CONNECTED_TO_WIFI;
    Serial.println("Connected to the WiFi");
    number_of_flashes(blue, ledPin, 2, 500, true);

    // Signal that WiFi is connected
    xEventGroupSetBits(wifiEventGroup, WIFI_CONNECTED_BIT);

    for (;;)
    {

        if (command.startsWith("reset"))
        {
            process_state = STOPPED;
        }

        if (process_state == EMERGENCY)
        {
            emergency_state();
            continue;
        }

        if (process_state == ERROR)
        {
            number_of_flashes(red, ledPin, 1, 500, true);
            continue;
        }

        if (process_state == STOPPED)
        {
            number_of_flashes(yellow, ledPin, 1, 1000, true);
            continue;
        }

        if (process_state == RUNNING)
        {
            continue;
        }

        if (digitalRead(emergencyPin) == HIGH && process_state != EMERGENCY)
        {
            process_state = EMERGENCY;
            continue;
        }

        set_led_color(black);

        if (millis() - lastMillis > 1000)
        {
            digitalWrite(ledPin, LOW);
        }

        if (command.startsWith("person:"))
        {
            digitalWrite(ledPin, HIGH);
            lastMillis = millis();
        }

        if (command.startsWith("motor:"))
        {
            int steps = command.substring(6).toInt();
            move_motor(m1step, steps, 2000);
            clear_command();
        }

        if (command.startsWith("dir:"))
        {
            int dir = command.substring(4).toInt();
            set_direction(m1dir, dir);
            clear_command();
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void handleClient(void *parameter)
{
    // Wait for the WiFi connection
    xEventGroupWaitBits(wifiEventGroup, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);

    // Now attempt to connect to the TCP server
    while (!client.connect(server_ip, server_port))
    {
        Serial.println("Connection to server failed, retrying...");
        fade_flash(red, 1, 500, true);
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
    process_state = CONNECTED_TO_SERVER;
    Serial.println("Connected to server");
    fade_flash(green, 1, 500, true);

    // Handle client communication
    while (true)
    {
        if (client.connected())
        {
            if (client.available())
            {

                command = client.readStringUntil('\n');
                Serial.println(command);
            }
        }
        else
        {
            // Attempt to reconnect if the connection is lost
            while (!client.connect(server_ip, server_port))
            {
                Serial.println("Reconnecting to server...");
                process_state = CONNECTING_TO_SERVER;
                fade_flash(blue, 1, 500, true);
                vTaskDelay(500 / portTICK_PERIOD_MS);
            }
        }
        vTaskDelay(1 / portTICK_PERIOD_MS); // Small delay to prevent task from hogging CPU
    }
}
