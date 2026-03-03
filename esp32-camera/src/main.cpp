#include <Arduino.h>
#include <cstring>

#include "esp_camera.h"
#include "camera_pins.h"
#include "driver/spi_slave.h"
#include "esp_heap_caps.h"
#include "esp_rom_crc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

// Continuous capture + pipelined SPI transport.
// Each SPI transaction is fixed-size full duplex:
//   - Pi sends the next 8-byte request in MOSI bytes 0..7
//   - Pi receives the previous response in MISO bytes 0..N
// This removes the "request now, response later" turnaround race.

#define SERIAL_BAUD      115200
#define FRAME_WIDTH      FRAMESIZE_QVGA
#define JPEG_QUALITY     12

#define CMD_NOP              0x00
#define CMD_GET_HEADER       0x01
#define CMD_GET_FRAME_CHUNK  0x02
#define CMD_GET_STATUS       0x03
#define CMD_ACK_FRAME        0x04

#define PROTOCOL_MAGIC  0x314D4143u  // "CAM1"
#define STATUS_MAGIC    0x54415453u  // "STAT"
#define RESPONSE_MAGIC  0xA55AA55Au
#define PROTOCOL_VER    1u

#define REQUEST_LEN       8u
#define RESPONSE_HDR_LEN  8u
#define MAX_CHUNK_LEN     1024u
#define TRANSFER_LEN      (RESPONSE_HDR_LEN + MAX_CHUNK_LEN)
#define FRAME_BUF_CAP     (192u * 1024u)

struct RequestPacket {
    uint8_t cmd;
    uint8_t reserved;
    uint16_t length;
    uint32_t offset;
};

struct ResponseHeader {
    uint32_t magic;
    uint16_t length;
    uint8_t type;
    uint8_t flags;
};

struct FrameHeader {
    uint32_t magic;
    uint16_t version;
    uint16_t flags;
    uint32_t seq;
    uint32_t length;
    uint32_t crc32;
};

struct StatusPacket {
    uint32_t magic;
    uint32_t latest_seq;
    uint32_t frames_captured;
    uint32_t frames_served;
    uint32_t frames_dropped;
};

static uint8_t *live_buffers[2] = {nullptr, nullptr};
static uint8_t *stream_buffer = nullptr;
static uint8_t *spi_tx_buf = nullptr;
static uint8_t *spi_rx_buf = nullptr;

static SemaphoreHandle_t frame_mutex = nullptr;

static int latest_slot = 0;
static bool have_frame = false;
static size_t latest_len = 0;
static uint32_t latest_crc32 = 0;
static uint32_t latest_seq = 0;
static uint32_t acked_seq = 0;

static bool stream_valid = false;
static size_t stream_len = 0;
static uint32_t stream_crc32 = 0;
static uint32_t stream_seq = 0;

static uint32_t frames_captured = 0;
static uint32_t frames_served = 0;
static uint32_t frames_dropped = 0;

static void set_resp_ready(bool ready) {
    digitalWrite(SPI_READY_GPIO_NUM, ready ? HIGH : LOW);
}

static uint8_t *alloc_buffer(size_t size, bool dma) {
    int caps = MALLOC_CAP_8BIT;
    if (psramFound()) {
        caps |= MALLOC_CAP_SPIRAM;
    }
    if (dma) {
        caps = MALLOC_CAP_DMA;
    }

    uint8_t *buf = static_cast<uint8_t *>(heap_caps_malloc(size, caps));
    if (!buf && !dma) {
        buf = static_cast<uint8_t *>(heap_caps_malloc(size, MALLOC_CAP_8BIT));
    }
    return buf;
}

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

    if (psramFound()) {
        config.frame_size   = FRAME_WIDTH;
        config.jpeg_quality = JPEG_QUALITY;
        config.fb_count     = 2;
        config.fb_location  = CAMERA_FB_IN_PSRAM;
    } else {
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

static bool init_buffers() {
    live_buffers[0] = alloc_buffer(FRAME_BUF_CAP, false);
    live_buffers[1] = alloc_buffer(FRAME_BUF_CAP, false);
    stream_buffer = alloc_buffer(FRAME_BUF_CAP, false);
    spi_tx_buf = alloc_buffer(TRANSFER_LEN, true);
    spi_rx_buf = alloc_buffer(TRANSFER_LEN, true);
    frame_mutex = xSemaphoreCreateMutex();

    if (
        !live_buffers[0] || !live_buffers[1] || !stream_buffer ||
        !spi_tx_buf || !spi_rx_buf || !frame_mutex
    ) {
        Serial.println("ERR:buffer_init_failed");
        return false;
    }

    memset(spi_tx_buf, 0, TRANSFER_LEN);
    memset(spi_rx_buf, 0, TRANSFER_LEN);
    return true;
}

static bool init_spi() {
    pinMode(SPI_READY_GPIO_NUM, OUTPUT);
    digitalWrite(SPI_READY_GPIO_NUM, LOW);

    spi_bus_config_t buscfg = {};
    buscfg.mosi_io_num = SPI_MOSI_GPIO_NUM;
    buscfg.miso_io_num = SPI_MISO_GPIO_NUM;
    buscfg.sclk_io_num = SPI_SCLK_GPIO_NUM;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = TRANSFER_LEN;

    spi_slave_interface_config_t slvcfg = {};
    slvcfg.mode = 0;
    slvcfg.spics_io_num = SPI_CS_GPIO_NUM;
    slvcfg.queue_size = 1;
    slvcfg.flags = 0;

    esp_err_t err = spi_slave_initialize(HSPI_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) {
        Serial.printf("SPI init failed: 0x%x\n", err);
        return false;
    }
    return true;
}

static void begin_response(uint8_t type, uint16_t length, uint8_t flags = 0) {
    memset(spi_tx_buf, 0, TRANSFER_LEN);

    ResponseHeader header = {};
    header.magic = RESPONSE_MAGIC;
    header.length = length;
    header.type = type;
    header.flags = flags;
    memcpy(spi_tx_buf, &header, sizeof(header));
}

static void prepare_nop_response() {
    begin_response(CMD_NOP, 0);
}

static void prepare_header_response() {
    FrameHeader header = {};

    if (xSemaphoreTake(frame_mutex, portMAX_DELAY) == pdTRUE) {
        if (have_frame) {
            memcpy(stream_buffer, live_buffers[latest_slot], latest_len);
            stream_len = latest_len;
            stream_crc32 = latest_crc32;
            stream_seq = latest_seq;
            stream_valid = true;
        } else {
            stream_valid = false;
            stream_len = 0;
            stream_crc32 = 0;
            stream_seq = 0;
        }
        xSemaphoreGive(frame_mutex);
    }

    header.magic = PROTOCOL_MAGIC;
    header.version = PROTOCOL_VER;
    header.flags = stream_valid ? 0x0001u : 0;
    header.seq = stream_seq;
    header.length = stream_len;
    header.crc32 = stream_crc32;

    begin_response(CMD_GET_HEADER, sizeof(header), stream_valid ? 1 : 0);
    memcpy(spi_tx_buf + RESPONSE_HDR_LEN, &header, sizeof(header));
}

static void prepare_chunk_response(uint32_t offset, uint16_t length) {
    begin_response(CMD_GET_FRAME_CHUNK, length, stream_valid ? 1 : 0);

    if (!stream_valid || length == 0 || offset >= stream_len) {
        return;
    }

    size_t available = stream_len - offset;
    size_t copy_len = length;
    if (copy_len > available) {
        copy_len = available;
    }
    memcpy(spi_tx_buf + RESPONSE_HDR_LEN, stream_buffer + offset, copy_len);
}

static void prepare_status_response() {
    StatusPacket status = {};

    if (xSemaphoreTake(frame_mutex, portMAX_DELAY) == pdTRUE) {
        status.magic = STATUS_MAGIC;
        status.latest_seq = latest_seq;
        status.frames_captured = frames_captured;
        status.frames_served = frames_served;
        status.frames_dropped = frames_dropped;
        xSemaphoreGive(frame_mutex);
    }

    begin_response(CMD_GET_STATUS, sizeof(status), have_frame ? 1 : 0);
    memcpy(spi_tx_buf + RESPONSE_HDR_LEN, &status, sizeof(status));
}

static void prepare_ack_response(uint32_t seq) {
    if (xSemaphoreTake(frame_mutex, portMAX_DELAY) == pdTRUE) {
        if (seq > acked_seq) {
            acked_seq = seq;
        }
        xSemaphoreGive(frame_mutex);
    }

    stream_valid = false;
    begin_response(CMD_ACK_FRAME, 0);
}

static void prepare_response_for_request(const RequestPacket &req) {
    switch (req.cmd) {
        case CMD_GET_HEADER:
            prepare_header_response();
            break;
        case CMD_GET_FRAME_CHUNK:
            prepare_chunk_response(req.offset, req.length);
            break;
        case CMD_GET_STATUS:
            prepare_status_response();
            break;
        case CMD_ACK_FRAME:
            prepare_ack_response(req.offset);
            break;
        case CMD_NOP:
        default:
            prepare_nop_response();
            break;
    }
}

static void capture_task(void *) {
    int write_slot = 1;

    while (true) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        if (fb->len > FRAME_BUF_CAP) {
            if (xSemaphoreTake(frame_mutex, portMAX_DELAY) == pdTRUE) {
                frames_dropped += 1;
                xSemaphoreGive(frame_mutex);
            }
            esp_camera_fb_return(fb);
            continue;
        }

        uint32_t crc = esp_rom_crc32_le(0, fb->buf, fb->len);

        if (xSemaphoreTake(frame_mutex, portMAX_DELAY) == pdTRUE) {
            memcpy(live_buffers[write_slot], fb->buf, fb->len);
            latest_slot = write_slot;
            latest_len = fb->len;
            latest_crc32 = crc;
            latest_seq += 1;
            frames_captured += 1;
            have_frame = true;
            write_slot = 1 - latest_slot;
            xSemaphoreGive(frame_mutex);
        }

        esp_camera_fb_return(fb);
        taskYIELD();
    }
}

static void spi_task(void *) {
    prepare_nop_response();
    set_resp_ready(true);

    while (true) {
        memset(spi_rx_buf, 0, TRANSFER_LEN);

        spi_slave_transaction_t trans = {};
        trans.length = TRANSFER_LEN * 8;
        trans.tx_buffer = spi_tx_buf;
        trans.rx_buffer = spi_rx_buf;

        esp_err_t err = spi_slave_transmit(HSPI_HOST, &trans, portMAX_DELAY);
        set_resp_ready(false);
        if (err != ESP_OK) {
            prepare_nop_response();
            set_resp_ready(true);
            continue;
        }

        RequestPacket req = {};
        memcpy(&req, spi_rx_buf, sizeof(req));
        prepare_response_for_request(req);
        set_resp_ready(true);

        if (req.cmd == CMD_GET_FRAME_CHUNK && stream_valid) {
            if (xSemaphoreTake(frame_mutex, portMAX_DELAY) == pdTRUE) {
                frames_served += 1;
                xSemaphoreGive(frame_mutex);
            }
        }
    }
}

void setup() {
    Serial.begin(SERIAL_BAUD);
    delay(200);
    Serial.println("ESP32-CAM booting...");

    if (!init_camera()) {
        Serial.println("ERR:camera_init_failed");
        while (true) {
            delay(1000);
        }
    }

    if (!init_buffers()) {
        while (true) {
            delay(1000);
        }
    }

    if (!init_spi()) {
        Serial.println("ERR:spi_init_failed");
        while (true) {
            delay(1000);
        }
    }

    xTaskCreatePinnedToCore(capture_task, "capture_task", 8192, nullptr, 1, nullptr, 1);
    xTaskCreatePinnedToCore(spi_task, "spi_task", 6144, nullptr, 2, nullptr, 0);

    Serial.println("OK:ready");
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}
