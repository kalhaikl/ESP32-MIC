/*
  ==========================================================================
  ESP32 WROOM-32U + INMP441 — TES MIC POLOS (tanpa WiFi/PTT/DSP)
  ==========================================================================

  Tujuan: memastikan wiring & mic-nya beres, TERPISAH dari kode PTT/UDP
  yang kompleks. Sketch ini cuma baca I2S dan cetak level suara mentah
  ke Serial Monitor terus-menerus, tanpa filter/gain/DSP apa pun.

  WIRING (sama seperti project utama):
    INMP441 VDD  -> 3V3
    INMP441 GND  -> GND
    INMP441 WS   -> GPIO25
    INMP441 SCK  -> GPIO26
    INMP441 SD   -> GPIO33
    INMP441 L/R  -> 3V3   (channel RIGHT)

  CARA PAKAI:
    1. Upload sketch ini, buka Serial Monitor (115200 baud).
    2. Diam sebentar, lihat baris RAW-MIN/RAW-MAX/RMS — itu noise floor.
    3. Lalu bicara / ketuk-ketuk dekat mic, lihat apakah angkanya naik
       jelas dan bar [#####.....] ikut memanjang.

  INTERPRETASI HASIL:
    - Kalau angka SELALU ~0 (RAW-MIN/MAX mentok di 0, RMS ~0.0) walau
      sudah diketuk/dibicarain keras dekat mic -> ini masalah HARDWARE/
      WIRING, bukan software. Coba urutan ini:
        a) Tukar CHANNEL_FORMAT di bawah: I2S_CHANNEL_FMT_ONLY_RIGHT
           <-> I2S_CHANNEL_FMT_ONLY_LEFT, upload ulang, tes lagi.
        b) Cek ulang solderan/jumper WS, SCK, SD - jangan sampai
           tertukar atau ada yang tidak kontak.
        c) Pastikan INMP441 benar dapat 3.3V (ukur pakai multimeter di
           pin VDD modul, bukan cuma di sumbernya).
        d) Coba pin I2S lain (kalau ada modul INMP441 cadangan, coba
           swap modulnya untuk pastikan bukan mic yang rusak).

    - Kalau ada angka bergerak (noise floor beberapa ratus/ribu) dan
      naik jelas saat bicara/ketuk -> wiring & mic AMAN, mic terdeteksi
      dengan baik. Tinggal balik ke sketch utama dan sesuaikan
      MIC_PRE_SHIFT / AUDIO_GAIN di sana berdasar rentang RAW yang
      terlihat di sini.
  ==========================================================================
*/

#include <Arduino.h>
#include "driver/i2s.h"
#include <math.h>

#define I2S_WS_PIN   25
#define I2S_SCK_PIN  26
#define I2S_SD_PIN   33

#define SAMPLE_RATE  8000
#define BUFFER_SIZE  256

#define I2S_PORT I2S_NUM_0

int32_t i2sRawBuf[BUFFER_SIZE];

void setup()
{
  Serial.begin(115200);
  delay(300);

  Serial.println();
  Serial.println("MIC TEST: INIT");

  i2s_config_t i2sConfig = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    // L/R -> 3V3 = channel RIGHT. Tukar ke ONLY_LEFT kalau hasilnya 0 terus.
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = BUFFER_SIZE,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };

  esp_err_t err = i2s_driver_install(I2S_PORT, &i2sConfig, 0, NULL);
  if (err != ESP_OK) {
    Serial.printf("MIC TEST: I2S DRIVER INSTALL ERROR 0x%04X\n", err);
    while (true) delay(1000);
  }

  i2s_pin_config_t pinConfig = {
    .bck_io_num = I2S_SCK_PIN,
    .ws_io_num = I2S_WS_PIN,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_SD_PIN
  };

  esp_err_t err2 = i2s_set_pin(I2S_PORT, &pinConfig);
  if (err2 != ESP_OK) {
    Serial.printf("MIC TEST: I2S SET PIN ERROR 0x%04X\n", err2);
    while (true) delay(1000);
  }

  i2s_zero_dma_buffer(I2S_PORT);

  Serial.println("MIC TEST: READY");
  Serial.println("Diamkan dulu beberapa detik utk lihat noise floor,");
  Serial.println("lalu bicara / ketuk-ketuk dekat mic.");
  Serial.println();
}

void loop()
{
  size_t bytesRead = 0;
  esp_err_t res = i2s_read(I2S_PORT, i2sRawBuf, sizeof(i2sRawBuf), &bytesRead, pdMS_TO_TICKS(100));

  if (res != ESP_OK) {
    Serial.println("MIC TEST: I2S READ ERROR");
    delay(200);
    return;
  }

  int samplesRead = bytesRead / sizeof(int32_t);
  if (samplesRead <= 0) return;

  int32_t rawMin = i2sRawBuf[0];
  int32_t rawMax = i2sRawBuf[0];
  double sumSq = 0.0;

  for (int i = 0; i < samplesRead; i++) {
    int32_t s = i2sRawBuf[i];
    if (s < rawMin) rawMin = s;
    if (s > rawMax) rawMax = s;
    sumSq += (double)s * (double)s;
  }

  int32_t peakToPeak = rawMax - rawMin;
  double rms = sqrt(sumSq / samplesRead);

  static uint32_t lastPrint = 0;
  if (millis() - lastPrint > 150) {
    lastPrint = millis();

    int barLen = 0;
    if (peakToPeak > 1) {
      barLen = (int)(log10((double)peakToPeak) * 8.0);
      if (barLen > 40) barLen = 40;
      if (barLen < 0) barLen = 0;
    }
    char bar[41];
    for (int i = 0; i < 40; i++) bar[i] = (i < barLen) ? '#' : '.';
    bar[40] = '\0';

    Serial.printf(
      "RAW-MIN=%9ld  RAW-MAX=%9ld  P2P=%9ld  RMS=%12.1f  [%s]\n",
      (long)rawMin, (long)rawMax, (long)peakToPeak, rms, bar
    );
  }
}
