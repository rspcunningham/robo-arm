#include <Arduino.h>
#include "esp_camera.h"
#include "camera_pins.h"

// --- Configuration ---
#define SERIAL_BAUD    921600
#define FRAME_WIDTH    FRAMESIZE_QVGA  // 320x240 — good balance for UART
#define JPEG_QUALITY   12              // 10-63, lower = better quality, bigger file

// --- Framing protocol ---
// Each frame sent as:
//   [MAGIC 4B] [LENGTH 4B little-endian] [JPEG data] [CRC16 2B]
//
// MAGIC: 0xAA 0x55 0x01 0x00
// LENGTH: byte count of JPEG data only
// CRC16: CCITT over the JPEG data
static const uint8_t FRAME_MAGIC[] = {0xAA, 0x55, 0x01, 0x00};

// Simple CRC-16/CCITT
static uint16_t crc16(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
    }
    return crc;
}

static bool streaming = false;

static bool init_camera() {
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer   = LEDC_TIMER_0;
    config.pin_d0       = Y2_GPIO_NUM;
    config.pin_d1       = Y3_GPIO_NUM;
    config.pin_d2       = Y4_GPIO_NUM;
    config.pin_d3       = Y5_GPIO_NUM;
    config.pin_d4       = Y6_GPIO_NUM;
    config.pin_d5       = Y7_GPIO_NUM;
    config.pin_d6       = Y8_GPIO_NUM;
    config.pin_d7       = Y9_GPIO_NUM;
    config.pin_xclk     = XCLK_GPIO_NUM;
    config.pin_pclk     = PCLK_GPIO_NUM;
    config.pin_vsync    = VSYNC_GPIO_NUM;
    config.pin_href     = HREF_GPIO_NUM;
    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn     = PWDN_GPIO_NUM;
    config.pin_reset    = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;
    config.grab_mode    = CAMERA_GRAB_LATEST;

    // Use PSRAM for frame buffers (WROVER has 4MB PSRAM)
    if (psramFound()) {
        config.frame_size   = FRAME_WIDTH;
        config.jpeg_quality = JPEG_QUALITY;
        config.fb_count     = 2;  // double-buffer for smooth capture
        config.fb_location  = CAMERA_FB_IN_PSRAM;
    } else {
        // Fallback without PSRAM — smaller frames
        config.frame_size   = FRAMESIZE_QVGA;
        config.jpeg_quality = 16;
        config.fb_count     = 1;
        config.fb_location  = CAMERA_FB_IN_DRAM;
    }

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("Camera init failed: 0x%x\n", err);
        return false;
    }
    return true;
}

static void send_frame() {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        Serial.println("ERR:capture_failed");
        return;
    }

    uint32_t len = fb->len;
    uint16_t crc = crc16(fb->buf, len);

    // Write header
    Serial.write(FRAME_MAGIC, sizeof(FRAME_MAGIC));
    Serial.write((uint8_t *)&len, 4);

    // Write JPEG payload
    Serial.write(fb->buf, len);

    // Write CRC
    Serial.write((uint8_t *)&crc, 2);

    esp_camera_fb_return(fb);
}

static void handle_command(const String &cmd) {
    if (cmd == "START") {
        streaming = true;
        Serial.println("OK:streaming");
    } else if (cmd == "STOP") {
        streaming = false;
        Serial.println("OK:stopped");
    } else if (cmd == "SNAP") {
        // Single frame capture regardless of streaming state
        send_frame();
    } else if (cmd == "STATUS") {
        Serial.printf("OK:streaming=%d,psram=%d,psram_free=%u\n",
                       streaming, psramFound(),
                       psramFound() ? ESP.getFreePsram() : 0);
    } else if (cmd.startsWith("QUALITY ")) {
        int q = cmd.substring(8).toInt();
        if (q >= 10 && q <= 63) {
            sensor_t *s = esp_camera_sensor_get();
            s->set_quality(s, q);
            Serial.printf("OK:quality=%d\n", q);
        } else {
            Serial.println("ERR:quality_range_10-63");
        }
    } else if (cmd.startsWith("RESOLUTION ")) {
        String res = cmd.substring(11);
        framesize_t fs;
        if      (res == "QQVGA")  fs = FRAMESIZE_QQVGA;  // 160x120
        else if (res == "QVGA")   fs = FRAMESIZE_QVGA;   // 320x240
        else if (res == "CIF")    fs = FRAMESIZE_CIF;     // 400x296
        else if (res == "VGA")    fs = FRAMESIZE_VGA;     // 640x480
        else if (res == "SVGA")   fs = FRAMESIZE_SVGA;    // 800x600
        else {
            Serial.println("ERR:unknown_resolution");
            return;
        }
        sensor_t *s = esp_camera_sensor_get();
        s->set_framesize(s, fs);
        Serial.printf("OK:resolution=%s\n", res.c_str());
    } else {
        Serial.println("ERR:unknown_command");
    }
}

void setup() {
    Serial.begin(SERIAL_BAUD);

    // Disable flash LED
    pinMode(FLASH_GPIO_NUM, OUTPUT);
    digitalWrite(FLASH_GPIO_NUM, LOW);

    Serial.println("ESP32-CAM booting...");

    if (!init_camera()) {
        Serial.println("ERR:camera_init_failed");
        while (true) delay(1000);
    }

    Serial.println("OK:ready");
}

void loop() {
    // Check for serial commands
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        if (cmd.length() > 0) {
            handle_command(cmd);
        }
    }

    // Stream frames if active
    if (streaming) {
        send_frame();
    }
}
