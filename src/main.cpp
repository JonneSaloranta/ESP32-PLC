#include <WiFi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <FastLED_NeoPixel.h>
#include "BasicStepperDriver.h"

EventGroupHandle_t wifiEventGroup;
const int WIFI_CONNECTED_BIT = BIT0;

WiFiClient client;

// const char *ssid = "ArduinoLabra";
// const char *password = "12345678";

// const char *server_ip = "192.168.1.101";
const char *server_ip = "192.168.1.100";
const int server_port = 12345;

const int emergencyPin = 18;

const int buzzer = 4;
const int beep_delay = 500;

const int limit1 = 22;
const int limit2 = 23;

#define MOTOR_STEPS 200
#define M1RPM 30
#define M2RPM 30
#define MICROSTEPS 1

const int M1DIR = 27;
const int M1STEP = 26;
BasicStepperDriver stepper1(MOTOR_STEPS, M1DIR, M1STEP);

const int M2DIR = 25;
const int M2STEP = 33;
BasicStepperDriver stepper2(MOTOR_STEPS, M2DIR, M2STEP);

#define BUFFER_SIZE 1024
char buffer[BUFFER_SIZE];
int bufferIndex = 0;

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

bool busy = false;

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

void emergency_state()
{
    Serial.println("Emergency state");
    number_of_flashes(red, ledPin, 1, 500, true);
    clear_command();
}

bool check_limit(int pin)
{
    return digitalRead(pin) != false;
}

void tip_holder()
{
    stepper2.rotate(-45);
    delay(500);
    stepper2.rotate(45);
    delay(500);
}

void left_90()
{
    stepper1.rotate(-90);
}

void right_90()
{
    stepper1.rotate(90);
}

void up_45()
{
    stepper2.rotate(45);
}

void down_45()
{
    stepper2.rotate(-45);
}

void calibrate()
{
    bool limit1Reached = false;
    bool limit2Reached = false;

    // Initialize stepper for calibration
    stepper2.rotate(180);
    delay(500);
    stepper2.rotate(-90);
    delay(500);

    // Continuously move stepper1 towards limit1
    while (!limit1Reached)
    {
        if (check_limit(limit1))
        {
            stepper1.move(1); // Move a single step towards limit1
            delay(10);        // Adjust delay for desired speed
        }
        else
        {
            stepper1.stop(); // Stop if limit switch is hit
            limit1Reached = true;
        }
    }

    delay(250);

    // Continuously move stepper1 towards limit2
    while (!limit2Reached)
    {
        if (check_limit(limit2))
        {
            stepper1.move(-1); // Move a single step towards limit2
            delay(10);         // Adjust delay for desired speed
        }
        else
        {
            stepper1.stop(); // Stop if limit switch is hit
            limit2Reached = true;
        }
    }

    delay(500);

    stepper1.rotate(95);
}

void setup()
{
    Serial.begin(115200);

    strip.begin(FastLED.addLeds<WS2812B, ledPin, GRB>(leds, number_of_leds));
    strip.setBrightness(max_brightness);

    // fade_flash(red, 1, 2, false);

    // pinMode(M1STEP, OUTPUT);
    // pinMode(M1DIR, OUTPUT);
    // pinMode(M2STEP, OUTPUT);
    // pinMode(M2DIR, OUTPUT);

    pinMode(limit1, INPUT_PULLUP);
    pinMode(limit2, INPUT_PULLUP);

    pinMode(emergencyPin, INPUT_PULLUP);

    pinMode(ledPin, OUTPUT);

    // set_direction(M1DIR, HIGH);
    // move_motor(M1STEP, 100, 2000);

    stepper1.begin(M1RPM, MICROSTEPS);
    stepper2.begin(M2RPM, MICROSTEPS);

    calibrate();

    tip_holder();

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

        if (command.startsWith("Reset"))
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

        // if (command.startsWith("motor:"))
        // {
        //     int steps = command.substring(6).toInt();
        //     move_motor(M1STEP, steps, 2000);
        //     clear_command();
        // }

        // if (command.startsWith("dir:"))
        // {
        //     int dir = command.substring(4).toInt();
        //     set_direction(M1DIR, dir);
        //     clear_command();
        // }

        if (command.startsWith("adapter:"))
        {
            if (!busy)
            {
                busy = true;
                left_90();
                vTaskDelay(100 / portTICK_PERIOD_MS);
                down_45();
                vTaskDelay(100 / portTICK_PERIOD_MS);
                up_45();
                vTaskDelay(100 / portTICK_PERIOD_MS);
                right_90();
                vTaskDelay(100 / portTICK_PERIOD_MS);
                clear_command();
                busy = false;
            }

            clear_command();
        }

        if (command.startsWith("battery:"))
        {
            if (!busy)
            {
                busy = true;
                down_45();
                vTaskDelay(100 / portTICK_PERIOD_MS);
                up_45();
                vTaskDelay(100 / portTICK_PERIOD_MS);
                clear_command();
                busy = false;
            }
        }

        if (command.startsWith("potentiometer:"))
        {
            if (!busy)
            {
                busy = true;
                right_90();
                vTaskDelay(100 / portTICK_PERIOD_MS);
                down_45();
                vTaskDelay(100 / portTICK_PERIOD_MS);
                up_45();
                vTaskDelay(100 / portTICK_PERIOD_MS);
                left_90();
                vTaskDelay(100 / portTICK_PERIOD_MS);
                clear_command();
                busy = false;
            }
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
        fade_flash(red, 1, 1, true);
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
    process_state = CONNECTED_TO_SERVER;
    Serial.println("Connected to a server");
    fade_flash(green, 1, 1, true);

    // Handle client communication
    while (true)
    {
        if (client.connected())
        {
            if (client.available())
            {
                char c = client.read(); // Read a character
                // Serial.println(c);
                if (c == '\n' || c == '\r' || bufferIndex == BUFFER_SIZE - 1)
                {
                    buffer[bufferIndex] = '\0'; // Null-terminate the string
                    if (bufferIndex > 0)        // Check if there's something in the buffer
                    {
                        command = String(buffer); // Assign to command
                        Serial.println(command);
                        // Reset buffer for the next command
                        bufferIndex = 0;
                        memset(buffer, 0, BUFFER_SIZE);
                    }
                }
                else
                {
                    buffer[bufferIndex++] = c; // Store character in buffer
                }
            }
        }
        else
        {
            // Attempt to reconnect if the connection is lost
            while (!client.connect(server_ip, server_port))
            {
                Serial.println("Reconnecting to server...");
                process_state = CONNECTING_TO_SERVER;
                fade_flash(blue, 1, 1, true);
                vTaskDelay(500 / portTICK_PERIOD_MS);
            }
        }
        vTaskDelay(1 / portTICK_PERIOD_MS); // Small delay to prevent task from hogging CPU
    }
}
