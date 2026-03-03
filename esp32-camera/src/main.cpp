#include <Arduino.h>
#include <cstring>

#include "esp_camera.h"
#include "camera_pins.h"
#include "driver/gpio.h"
#include "driver/spi_slave.h"
#include "esp_heap_caps.h"
#include "esp_rom_crc.h"

// Camera defaults tuned for stable transfer over the new SPI transport.
#define SERIAL_BAUD      115200
#define FRAME_WIDTH      FRAMESIZE_QVGA
#define JPEG_QUALITY     12

#define CMD_GET_HEADER        0x01
#define CMD_GET_FRAME_CHUNK   0x02
#define CMD_GET_STATUS        0x03

#define PROTOCOL_MAGIC  0x314D4143u  // "CAM1"
#define STATUS_MAGIC    0x54415453u  // "STAT"
#define PROTOCOL_VER    1u
#define REQUEST_LEN     8u
#define HEADER_LEN      20u
#define STATUS_LEN      16u
#define MAX_CHUNK_LEN   4096u
#define FRAME_FLAGS_READY 0x0001u

struct RequestPacket {
    uint8_t cmd;
    uint8_t reserved;
    uint16_t length;
    uint32_t offset;
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
    uint32_t seq;
    uint32_t frames_captured;
    uint32_t frames_served;
};

static uint8_t *frame_buf = nullptr;
static size_t frame_buf_cap = 0;
static size_t latest_len = 0;
static uint32_t latest_crc32 = 0;
static uint32_t latest_seq = 0;
static bool frame_ready = false;

static uint32_t frames_captured = 0;
static uint32_t frames_served = 0;

static uint8_t *spi_tx_buf = nullptr;
static uint8_t *spi_rx_buf = nullptr;
static uint8_t *chunk_buf = nullptr;
static bool spi_ok = false;

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

static bool ensure_frame_capacity(size_t need) {
    if (need <= frame_buf_cap) {
        return true;
    }

    uint8_t *next = static_cast<uint8_t *>(
        heap_caps_realloc(frame_buf, need, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)
    );
    if (!next) {
        Serial.printf("ERR:frame_alloc=%u\n", static_cast<unsigned>(need));
        return false;
    }

    frame_buf = next;
    frame_buf_cap = need;
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
    buscfg.max_transfer_sz = MAX_CHUNK_LEN;

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

    spi_tx_buf = static_cast<uint8_t *>(heap_caps_malloc(MAX_CHUNK_LEN, MALLOC_CAP_DMA));
    spi_rx_buf = static_cast<uint8_t *>(heap_caps_malloc(MAX_CHUNK_LEN, MALLOC_CAP_DMA));
    chunk_buf = static_cast<uint8_t *>(heap_caps_malloc(MAX_CHUNK_LEN, MALLOC_CAP_DMA));
    if (!spi_tx_buf || !spi_rx_buf || !chunk_buf) {
        Serial.println("SPI DMA buffer alloc failed");
        return false;
    }

    spi_ok = true;
    return true;
}

static bool spi_exchange(size_t tx_len, size_t rx_len) {
    if (!spi_ok) {
        return false;
    }

    const size_t nbytes = tx_len > rx_len ? tx_len : rx_len;
    if (nbytes == 0 || nbytes > MAX_CHUNK_LEN) {
        return false;
    }

    memset(spi_rx_buf, 0, nbytes);

    spi_slave_transaction_t trans = {};
    trans.length = nbytes * 8;
    trans.tx_buffer = spi_tx_buf;
    trans.rx_buffer = spi_rx_buf;

    esp_err_t err = spi_slave_transmit(HSPI_HOST, &trans, pdMS_TO_TICKS(20));
    return err == ESP_OK;
}

static bool receive_request(RequestPacket *out) {
    memset(spi_tx_buf, 0, REQUEST_LEN);
    if (!spi_exchange(REQUEST_LEN, REQUEST_LEN)) {
        return false;
    }

    memcpy(out, spi_rx_buf, REQUEST_LEN);
    return true;
}

static bool send_bytes(const uint8_t *data, size_t len) {
    if (len > MAX_CHUNK_LEN) {
        return false;
    }

    memset(spi_tx_buf, 0, len);
    memcpy(spi_tx_buf, data, len);
    return spi_exchange(len, len);
}

static void publish_ready(bool ready) {
    frame_ready = ready;
    digitalWrite(SPI_READY_GPIO_NUM, ready ? HIGH : LOW);
}

static void capture_latest_frame() {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        return;
    }

    if (!ensure_frame_capacity(fb->len)) {
        esp_camera_fb_return(fb);
        return;
    }

    memcpy(frame_buf, fb->buf, fb->len);
    latest_len = fb->len;
    latest_crc32 = esp_rom_crc32_le(0, frame_buf, latest_len);
    latest_seq += 1;
    frames_captured += 1;
    publish_ready(true);

    esp_camera_fb_return(fb);
}

static void handle_get_header() {
    FrameHeader header = {};
    header.magic = PROTOCOL_MAGIC;
    header.version = PROTOCOL_VER;
    header.flags = frame_ready ? FRAME_FLAGS_READY : 0;
    header.seq = latest_seq;
    header.length = latest_len;
    header.crc32 = latest_crc32;
    send_bytes(reinterpret_cast<const uint8_t *>(&header), sizeof(header));
}

static void handle_get_frame_chunk(uint32_t offset, uint16_t length) {
    if (length > MAX_CHUNK_LEN) {
        length = MAX_CHUNK_LEN;
    }

    if (offset >= latest_len || length == 0) {
        memset(chunk_buf, 0, length);
        send_bytes(chunk_buf, length);
        return;
    }

    size_t available = latest_len - offset;
    size_t send_len = length;
    if (send_len > available) {
        send_len = available;
    }

    memcpy(chunk_buf, frame_buf + offset, send_len);
    if (send_len < length) {
        memset(chunk_buf + send_len, 0, length - send_len);
    }

    if (send_bytes(chunk_buf, length)) {
        frames_served += 1;
        if (offset + send_len >= latest_len) {
            publish_ready(false);
        }
    }
}

static void handle_get_status() {
    StatusPacket status = {};
    status.magic = STATUS_MAGIC;
    status.seq = latest_seq;
    status.frames_captured = frames_captured;
    status.frames_served = frames_served;
    send_bytes(reinterpret_cast<const uint8_t *>(&status), sizeof(status));
}

static void handle_request(const RequestPacket &req) {
    switch (req.cmd) {
        case CMD_GET_HEADER:
            handle_get_header();
            break;
        case CMD_GET_FRAME_CHUNK:
            handle_get_frame_chunk(req.offset, req.length);
            break;
        case CMD_GET_STATUS:
            handle_get_status();
            break;
        default:
            memset(chunk_buf, 0, 4);
            send_bytes(chunk_buf, 4);
            break;
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

    if (!init_spi()) {
        Serial.println("ERR:spi_init_failed");
        while (true) {
            delay(1000);
        }
    }

    Serial.println("OK:ready");
}

void loop() {
    capture_latest_frame();

    RequestPacket req = {};
    if (receive_request(&req)) {
        handle_request(req);
    }
}
