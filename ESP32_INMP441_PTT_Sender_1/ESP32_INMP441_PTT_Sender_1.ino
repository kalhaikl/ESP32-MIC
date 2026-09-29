/*
  ==========================================================================
  ESP32 WROOM-32U + INMP441  —  PTT Mic Sender (Arduino IDE, bukan Tasmota)
  ==========================================================================

  Ini adalah port dari driver Tasmota ESP8266 (xdrv sisi mic) ke ESP32,
  dipakai berpasangan dengan firmware penerima ESP32 (Xdrv129, Tasmota)
  yang mendengarkan UDP di port 5000 dan memutar audio via I2S TX.

  WIRING (sesuai diagram):
    INMP441 VDD  -> 3V3
    INMP441 GND  -> GND
    INMP441 WS   -> GPIO25   (I2S WS / LRCLK)
    INMP441 SCK  -> GPIO26   (I2S SCK / BCLK)
    INMP441 SD   -> GPIO33   (I2S SD  / DIN ke ESP32)
    INMP441 L/R  -> 3V3      (pilih channel RIGHT)
    Tombol PTT   -> GPIO27   (ke GND, pakai internal pull-up)
    LED indikator-> GPIO32

  DSP CHAIN (dipertahankan sama persis dari versi ESP8266):
    - DC blocking filter
    - Biquad HPF (~180Hz) -> LPF (~3.6kHz) -> Peaking presence (+dB @2.5kHz)
    - Downward noise expander dengan noise floor adaptif
    - Howl/feedback guard (crest factor + zero-crossing-rate tonal detect)
    - Echo-chain guard (rentetan gate-open cepat -> attenuasi sementara)
    - AGC (target RMS tetap)
    - Soft clipper
    - Noise gate dengan hold time

  PERBEDAAN PENTING vs versi ESP8266 (supaya nyambung ke penerima ESP32
  yang sudah ada / Xdrv129):
    - Versi ESP8266 menambahkan header 2-byte sequence number di depan
      tiap paket audio. Firmware penerima ESP32 (Xdrv129) TIDAK melepas
      header itu — ia langsung menganggap seluruh payload sebagai sample
      PCM 16-bit mentah. Karena itu di sini header DIHAPUS, paket audio
      hanya berisi PCM 16-bit mono mentah, dan paket flush/stop tetap
      1 byte (sudah cocok dengan pengecekan "received <= 1" di penerima).
    - Karena ini sketch Arduino IDE biasa (bukan Tasmota), koneksi WiFi
      dikelola manual di sketch ini (isi SSID/PASSWORD di bawah).

  Catatan LED indikator:
    Sama seperti versi ESP8266, LED ini menyala saat unit menerima paket
    kecil ("ping") di UDP port 5001 dari speaker/penerima — dipakai
    sebagai indikator "lawan bicara sedang memutar audio" (mis. untuk
    half-duplex). Firmware penerima yang dilampirkan di percakapan ini
    belum mengirim ping semacam itu, jadi LED baru akan menyala kalau
    nanti ada perangkat yang mengirim paket kecil ke IP unit ini di port
    5001. Kalau maksudnya LED cukup menyala saat PTT ditekan (indikator
    "sedang transmit"), tinggal bilang, gampang diubah.

  CATATAN DRIVER I2S:
    Versi ini pakai driver I2S "legacy" (driver/i2s.h), BUKAN i2s_std.h.
    Arduino-ESP32 (framework Arduino, beda dengan ESP-IDF murni yang
    dipakai Tasmota) tidak mengekspos header driver/i2s_std.h ke sketch,
    walau versi core sudah 3.x sekalipun — header itu hanya tersedia kalau
    build langsung lewat ESP-IDF. Driver legacy ini didukung penuh di
    semua versi Arduino-ESP32 (2.x maupun 3.x), cuma akan muncul beberapa
    warning "deprecated" saat compile — itu normal, bukan error, aman
    diabaikan.

    Kalau nanti audio kebalik/hening padahal PEAK di Serial Monitor jalan,
    coba tukar I2S_CHANNEL_FMT_ONLY_RIGHT <-> ONLY_LEFT di MicI2sInit()
    (ini kuirk umum di banyak board INMP441, kadang polaritas L/R fisik
    tidak selalu sama dengan yang diharapkan driver).
  ==========================================================================
*/

#include <WiFi.h>
#include <WiFiUdp.h>
#include "driver/i2s.h"
#include <math.h>

// ---------------------------------------------------------------------
// KONFIGURASI WIFI  (WAJIB DIISI)
// ---------------------------------------------------------------------
#define WIFI_SSID      "bitio"
#define WIFI_PASSWORD  "Integralnaikkk"

// ---------------------------------------------------------------------
// KONFIGURASI TUJUAN AUDIO (harus sama dengan penerima / Xdrv129)
// ---------------------------------------------------------------------
#define AUDIO_IP    "192.168.24.101"   // IP ESP32 penerima
#define AUDIO_PORT  5000               // == AUDIO_UDP_PORT di penerima

#define SPEAKER_PING_LISTEN_PORT  5001 // port lokal utk terima "ping" dari speaker

// ---------------------------------------------------------------------
// PIN (sesuai wiring)
// ---------------------------------------------------------------------
#define I2S_WS_PIN   25
#define I2S_SCK_PIN  26
#define I2S_SD_PIN   33

#define BUTTON_PIN   27
#define LED_PIN      32

#define BUTTON_DEBOUNCE_MS 40

// ---------------------------------------------------------------------
// KONFIGURASI AUDIO / MIC
// ---------------------------------------------------------------------
#define SAMPLE_RATE   8000     // harus sama dgn AUDIO_SAMPLE_RATE penerima
#define BUFFER_SIZE   256

// INMP441 keluar 24-bit data left-justified dalam frame 32-bit.
// Shift ini mengubahnya jadi skala setara PCM 16-bit sebelum masuk DSP
// chain (yang konstanta-konstantanya dikalibrasi utk skala 16-bit).
// Diturunkan ke 8 (dari 16) karena versi sebelumnya kelewat agresif -
// level suara wajar bisa habis dibuang jadi 0 sebelum sempat difilter.
// Kalau PEAK masih 0 terus, lihat log "RAW MIN/MAX" untuk tahu apakah
// datanya memang 0 dari I2S (soal wiring/channel), atau cuma perlu
// MIC_PRE_SHIFT lebih rendah lagi (coba 4-10).
#define MIC_PRE_SHIFT  8

#define AUDIO_GAIN  4.0f

#define NOISE_GATE_PEAK  3000
#define GATE_HOLD_MS     200

static const float BQ_HPF_B0 =  0.90479f;
static const float BQ_HPF_B1 = -1.80958f;
static const float BQ_HPF_B2 =  0.90479f;
static const float BQ_HPF_A1 = -1.80066f;
static const float BQ_HPF_A2 =  0.81870f;

static const float BQ_LPF_B0 =  0.80063f;
static const float BQ_LPF_B1 =  1.60127f;
static const float BQ_LPF_B2 =  0.80063f;
static const float BQ_LPF_A1 =  1.56106f;
static const float BQ_LPF_A2 =  0.64137f;

static const float BQ_PEAK_B0 = 1.15701f;
static const float BQ_PEAK_B1 = 0.55990f;
static const float BQ_PEAK_B2 = 0.30612f;
static const float BQ_PEAK_A1 = 0.55990f;
static const float BQ_PEAK_A2 = 0.46315f;

#define HOWL_CREST_LOW      1.15f
#define HOWL_CREST_HIGH     1.65f
#define HOWL_PEAK_MIN       2700
#define HOWL_TRIGGER_COUNT  3
#define HOWL_ATTEN          0.07f
#define HOWL_HOLD_MS        600

#define ZCR_EMA_SMOOTH      0.3f
#define ZCR_DEV_THRESHOLD   0.12f
#define ZCR_WARMUP_COUNT    3

#define AGC_TARGET_RMS      5500.0f
#define AGC_MIN_MULT        0.7f
#define AGC_MAX_MULT        1.8f
#define AGC_SMOOTH          0.05f

#define ECHO_CHAIN_WINDOW_MS  1200
#define ECHO_CHAIN_TRIGGER    3
#define ECHO_CHAIN_HOLD_MS    900
#define ECHO_CHAIN_ATTEN      0.10f

#define NOISE_FLOOR_ADAPT     0.02f
#define NOISE_FLOOR_MIN       30.0f
#define EXPANDER_LOW_MULT     1.2f
#define EXPANDER_HIGH_MULT    3.0f
#define EXPANDER_FLOOR_ATTEN  0.12f
#define EXPANDER_SMOOTH       0.15f

#define SPEAKER_ACTIVE_LED_HOLD_MS  180

// ---------------------------------------------------------------------
// GLOBAL STATE
// ---------------------------------------------------------------------
WiFiUDP audioUdp;

#define I2S_PORT I2S_NUM_0

bool micStarted = false;

int32_t i2sRawBuf[BUFFER_SIZE];
int16_t audioBuffer[BUFFER_SIZE];

int32_t dc_filter = 0;

static float hpf_x1 = 0.0f, hpf_x2 = 0.0f, hpf_y1 = 0.0f, hpf_y2 = 0.0f;
static float lpf_x1 = 0.0f, lpf_x2 = 0.0f, lpf_y1 = 0.0f, lpf_y2 = 0.0f;
static float pk_x1 = 0.0f, pk_x2 = 0.0f, pk_y1 = 0.0f, pk_y2 = 0.0f;

static uint8_t  howlCount = 0;
static uint32_t howlHoldUntil = 0;
static bool     howlActiveLast = false;

static float   zcrEma = 40.0f;
static float   zcrDevEma = 999.0f;
static uint8_t zcrWarmup = 0;

static float agcMult = 1.0f;

static float noiseFloorRms = 200.0f;
static float expAttenSmoothed = 1.0f;

static uint32_t lastSpeakerActiveMs = 0;

static uint32_t burstTimestamps[4] = {0, 0, 0, 0};
static uint8_t  burstIdx = 0;
static uint32_t echoChainHoldUntil = 0;
static bool     prevGateOpen = false;
static bool     echoChainActiveLast = false;

static bool     debouncedPressed = false;
static bool     rawPressedLast = false;
static uint32_t lastRawChangeMs = 0;

// ---------------------------------------------------------------------
// SOFT CLIP
// ---------------------------------------------------------------------
static inline int16_t AudioSoftClip(float x)
{
  const float thr  = 25000.0f;
  const float maxv = 32700.0f;

  float ax = fabsf(x);
  float y;

  if (ax <= thr) {
    y = x;
  } else {
    float over   = ax - thr;
    float range  = maxv - thr;
    float shaped = thr + range * tanhf(over / range);
    y = (x < 0) ? -shaped : shaped;
  }

  if (y > 32767.0f)  y = 32767.0f;
  if (y < -32768.0f) y = -32768.0f;

  return (int16_t)y;
}

// ---------------------------------------------------------------------
// WIFI
// ---------------------------------------------------------------------
void WifiConnect()
{
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.printf("AUDIO: MENYAMBUNGKAN WIFI KE %s", WIFI_SSID);

  uint32_t startMs = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - startMs) < 20000) {
    delay(250);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.printf("AUDIO: WIFI OK, IP=%s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println();
    Serial.println("AUDIO: WIFI GAGAL (akan dicoba lagi di background)");
  }
}

void WifiMaintain()
{
  static uint32_t lastCheck = 0;
  if (millis() - lastCheck < 5000) return;
  lastCheck = millis();

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("AUDIO: WIFI TERPUTUS, MENCOBA RECONNECT");
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  }
}

// ---------------------------------------------------------------------
// I2S / MIC INIT
// ---------------------------------------------------------------------
bool MicI2sInit()
{
  i2s_config_t i2sConfig = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    // INMP441 L/R -> 3V3 artinya data dikirim di slot RIGHT.
    // Kalau ternyata hening/kebalik, tukar ke I2S_CHANNEL_FMT_ONLY_LEFT.
    .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,
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
    Serial.printf("AUDIO: I2S DRIVER INSTALL ERROR 0x%04X\n", err);
    return false;
  }

  i2s_pin_config_t pinConfig = {
    .bck_io_num = I2S_SCK_PIN,
    .ws_io_num = I2S_WS_PIN,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_SD_PIN
  };

  esp_err_t err2 = i2s_set_pin(I2S_PORT, &pinConfig);
  if (err2 != ESP_OK) {
    Serial.printf("AUDIO: I2S SET PIN ERROR 0x%04X\n", err2);
    i2s_driver_uninstall(I2S_PORT);
    return false;
  }

  i2s_zero_dma_buffer(I2S_PORT);

  return true;
}

void MicInit()
{
  Serial.println("AUDIO: MIC INIT");

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  if (MicI2sInit()) {

    audioUdp.begin(SPEAKER_PING_LISTEN_PORT);

    micStarted = true;

    Serial.println("AUDIO: MIC READY");
    Serial.printf(
      "AUDIO: RATE=%d BUFFER=%d GAIN=%.1f GATE=%d\n",
      SAMPLE_RATE, BUFFER_SIZE, AUDIO_GAIN, NOISE_GATE_PEAK
    );
    Serial.printf(
      "AUDIO: OPT BIQUAD-BPF PRESENCE NOISE-EXPANDER HOWL-GUARD ECHO-CHAIN-GUARD AGC-TARGET=%d LED-PIN=%d\n",
      (int)AGC_TARGET_RMS, LED_PIN
    );

  } else {
    Serial.println("AUDIO: I2S MIC FAILED");
  }
}

// ---------------------------------------------------------------------
// RESET SEMUA STATE FILTER/GUARD (dipanggil saat tombol PTT dilepas)
// ---------------------------------------------------------------------
void MicResetState()
{
  dc_filter = 0;

  hpf_x1 = 0.0f; hpf_x2 = 0.0f; hpf_y1 = 0.0f; hpf_y2 = 0.0f;
  lpf_x1 = 0.0f; lpf_x2 = 0.0f; lpf_y1 = 0.0f; lpf_y2 = 0.0f;
  pk_x1 = 0.0f; pk_x2 = 0.0f; pk_y1 = 0.0f; pk_y2 = 0.0f;

  howlCount = 0;
  howlHoldUntil = 0;
  agcMult = 1.0f;
  expAttenSmoothed = 1.0f;

  zcrEma = 40.0f;
  zcrDevEma = 999.0f;
  zcrWarmup = 0;

  burstTimestamps[0] = 0; burstTimestamps[1] = 0;
  burstTimestamps[2] = 0; burstTimestamps[3] = 0;
  burstIdx = 0;
  echoChainHoldUntil = 0;
  prevGateOpen = false;
}

// ---------------------------------------------------------------------
// LOOP UTAMA STREAMING
// ---------------------------------------------------------------------
void MicStream()
{
  if (!micStarted) return;

  // --- cek "ping" dari speaker (indikator LED) ---
  int pingSize = audioUdp.parsePacket();
  if (pingSize > 0) {
    uint8_t dummy[8];
    int n = audioUdp.read(dummy, sizeof(dummy));
    if (n > 0) {
      lastSpeakerActiveMs = millis();
    }
  }
  digitalWrite(
    LED_PIN,
    ((millis() - lastSpeakerActiveMs) < SPEAKER_ACTIVE_LED_HOLD_MS) ? HIGH : LOW
  );

  // --- baca & debounce tombol PTT ---
  static bool wasPressed = false;
  static uint32_t lastAboveGateMs = 0;

  bool rawPressed = (digitalRead(BUTTON_PIN) == LOW);
  if (rawPressed != rawPressedLast) {
    lastRawChangeMs = millis();
    rawPressedLast = rawPressed;
  }
  if ((millis() - lastRawChangeMs) > BUTTON_DEBOUNCE_MS) {
    debouncedPressed = rawPressed;
  }
  bool isPressed = debouncedPressed;

  if (!isPressed) {

    if (wasPressed) {
      // kirim marker flush (paket 1 byte, cocok dgn "received <= 1" di penerima)
      IPAddress destination;
      destination.fromString(AUDIO_IP);

      uint8_t flushMarker = 0xFF;
      for (uint8_t i = 0; i < 4; i++) {
        audioUdp.beginPacket(destination, AUDIO_PORT);
        audioUdp.write(&flushMarker, 1);
        audioUdp.endPacket();
      }

      lastAboveGateMs = 0;
      MicResetState();
    }

    wasPressed = false;
    return;
  }

  wasPressed = true;

  // --- baca satu blok sample dari mic (blocking, timeout 50ms) ---
  size_t bytesRead = 0;
  esp_err_t res = i2s_read(I2S_PORT, i2sRawBuf, sizeof(i2sRawBuf), &bytesRead, pdMS_TO_TICKS(50));
  if (res != ESP_OK) return;

  int samplesRead = bytesRead / sizeof(int32_t);
  if (samplesRead <= 0) return;

  int32_t bufferPeak = 0;

  double preSumSq = 0.0;
  int32_t prePeak = 0;

  float howlAtten     = (millis() < howlHoldUntil)      ? HOWL_ATTEN      : 1.0f;
  float expAtten       = expAttenSmoothed;
  float echoChainAtten = (millis() < echoChainHoldUntil) ? ECHO_CHAIN_ATTEN : 1.0f;

  int32_t zcrCount = 0;
  float prevFiltered = 0.0f;
  bool havePrevFiltered = false;

  int32_t rawMin = 0;
  int32_t rawMax = 0;

  for (int i = 0; i < samplesRead; i++) {

    int32_t rawSample = i2sRawBuf[i];
    if (i == 0) {
      rawMin = rawSample;
      rawMax = rawSample;
    } else {
      if (rawSample < rawMin) rawMin = rawSample;
      if (rawSample > rawMax) rawMax = rawSample;
    }

    int32_t sample = rawSample >> MIC_PRE_SHIFT;

    dc_filter += (sample - dc_filter) >> 7;
    sample -= dc_filter;

    float hpf_in = (float)sample;
    float hpf_out = BQ_HPF_B0 * hpf_in + BQ_HPF_B1 * hpf_x1 + BQ_HPF_B2 * hpf_x2
                  - BQ_HPF_A1 * hpf_y1 - BQ_HPF_A2 * hpf_y2;
    hpf_x2 = hpf_x1; hpf_x1 = hpf_in;
    hpf_y2 = hpf_y1; hpf_y1 = hpf_out;

    float lpf_in = hpf_out;
    float lpf_out = BQ_LPF_B0 * lpf_in + BQ_LPF_B1 * lpf_x1 + BQ_LPF_B2 * lpf_x2
                  - BQ_LPF_A1 * lpf_y1 - BQ_LPF_A2 * lpf_y2;
    lpf_x2 = lpf_x1; lpf_x1 = lpf_in;
    lpf_y2 = lpf_y1; lpf_y1 = lpf_out;

    float pk_in = lpf_out;
    float pk_out = BQ_PEAK_B0 * pk_in + BQ_PEAK_B1 * pk_x1 + BQ_PEAK_B2 * pk_x2
                 - BQ_PEAK_A1 * pk_y1 - BQ_PEAK_A2 * pk_y2;
    pk_x2 = pk_x1; pk_x1 = pk_in;
    pk_y2 = pk_y1; pk_y1 = pk_out;

    float filtered = pk_out;

    preSumSq += (double)filtered * (double)filtered;
    int32_t absFiltered = (int32_t)fabsf(filtered);
    if (absFiltered > prePeak) prePeak = absFiltered;

    if (havePrevFiltered) {
      if ((filtered >= 0.0f) != (prevFiltered >= 0.0f)) zcrCount++;
    }
    prevFiltered = filtered;
    havePrevFiltered = true;

    float amplified = filtered * AUDIO_GAIN * agcMult * howlAtten * expAtten * echoChainAtten;
    audioBuffer[i] = AudioSoftClip(amplified);

    int32_t a = abs((int32_t)audioBuffer[i]);
    if (a > bufferPeak) bufferPeak = a;
  }

  {
    float rms = sqrtf((float)(preSumSq / (double)samplesRead) + 1e-6f);
    float crest = (rms > 1.0f) ? ((float)prePeak / rms) : 0.0f;

    bool crestTonal = (prePeak >= HOWL_PEAK_MIN) &&
                       (crest >= HOWL_CREST_LOW) &&
                       (crest <= HOWL_CREST_HIGH);

    if (prePeak >= HOWL_PEAK_MIN) {
      float zcrDev = fabsf((float)zcrCount - zcrEma);
      zcrEma += ((float)zcrCount - zcrEma) * ZCR_EMA_SMOOTH;
      zcrDevEma += (zcrDev - zcrDevEma) * ZCR_EMA_SMOOTH;
      if (zcrWarmup < 250) zcrWarmup++;
    } else {
      zcrWarmup = 0;
      zcrDevEma = 999.0f;
    }
    float zcrRelDev = zcrDevEma / (zcrEma + 1.0f);
    bool zcrTonal = (prePeak >= HOWL_PEAK_MIN) &&
                     (zcrWarmup >= ZCR_WARMUP_COUNT) &&
                     (zcrEma > 5.0f) &&
                     (zcrRelDev < ZCR_DEV_THRESHOLD);

    bool tonal = crestTonal || zcrTonal;

    if (tonal) {
      if (howlCount < 250) howlCount++;
      if (howlCount >= HOWL_TRIGGER_COUNT) {
        howlHoldUntil = millis() + HOWL_HOLD_MS;
      }
    } else {
      howlCount = 0;
    }

    bool howlActiveNow = (millis() < howlHoldUntil);
    if (howlActiveNow && !howlActiveLast) {
      Serial.printf(
        "AUDIO: HOWL/FEEDBACK TERDETEKSI (crest=%d zcr=%d), GAIN DIREDAM SEMENTARA\n",
        crestTonal ? 1 : 0, zcrTonal ? 1 : 0
      );
    }
    howlActiveLast = howlActiveNow;

    if (rms > 50.0f) {
      float desiredMult = AGC_TARGET_RMS / rms;
      if (desiredMult < AGC_MIN_MULT) desiredMult = AGC_MIN_MULT;
      if (desiredMult > AGC_MAX_MULT) desiredMult = AGC_MAX_MULT;
      agcMult += (desiredMult - agcMult) * AGC_SMOOTH;
    }

    if (rms < noiseFloorRms * 1.5f) {
      noiseFloorRms += (rms - noiseFloorRms) * NOISE_FLOOR_ADAPT;
      if (noiseFloorRms < NOISE_FLOOR_MIN) noiseFloorRms = NOISE_FLOOR_MIN;
    }

    float lowBound  = noiseFloorRms * EXPANDER_LOW_MULT;
    float highBound = noiseFloorRms * EXPANDER_HIGH_MULT;
    float expTarget;

    if (rms <= lowBound) {
      expTarget = EXPANDER_FLOOR_ATTEN;
    } else if (rms >= highBound) {
      expTarget = 1.0f;
    } else {
      float t = (rms - lowBound) / (highBound - lowBound);
      expTarget = EXPANDER_FLOOR_ATTEN + t * (1.0f - EXPANDER_FLOOR_ATTEN);
    }
    expAttenSmoothed += (expTarget - expAttenSmoothed) * EXPANDER_SMOOTH;
  }

  static uint32_t lastGateLog = 0;
  if (millis() - lastGateLog > 1000) {
    Serial.printf(
      "AUDIO: MIC PEAK=%d GATE=%d AGC=%d NOISE-FLOOR=%d RAW-MIN=%d RAW-MAX=%d\n",
      (int)bufferPeak, NOISE_GATE_PEAK, (int)(agcMult * 100), (int)noiseFloorRms,
      (int)rawMin, (int)rawMax
    );
    lastGateLog = millis();
  }

  if (bufferPeak >= NOISE_GATE_PEAK) {
    lastAboveGateMs = millis();
  }

  bool gateOpen = (lastAboveGateMs != 0) &&
                  (millis() - lastAboveGateMs < GATE_HOLD_MS);

  if (gateOpen && !prevGateOpen) {
    uint32_t nowMs = millis();
    burstTimestamps[burstIdx] = nowMs;
    burstIdx = (burstIdx + 1) % 4;

    uint8_t recentCount = 0;
    for (uint8_t k = 0; k < 4; k++) {
      if (burstTimestamps[k] != 0 && (nowMs - burstTimestamps[k]) < ECHO_CHAIN_WINDOW_MS) {
        recentCount++;
      }
    }
    if (recentCount >= ECHO_CHAIN_TRIGGER) {
      echoChainHoldUntil = nowMs + ECHO_CHAIN_HOLD_MS;
    }
  }
  prevGateOpen = gateOpen;

  bool echoChainActiveNow = (millis() < echoChainHoldUntil);
  if (echoChainActiveNow && !echoChainActiveLast) {
    Serial.println("AUDIO: RENTETAN GEMA TERDETEKSI, MIC DIREDAM SEMENTARA");
  }
  echoChainActiveLast = echoChainActiveNow;

  if (!gateOpen) {
    return;
  }

  // --- kirim audio (PCM 16-bit mono mentah, tanpa header) ---
  IPAddress destination;
  destination.fromString(AUDIO_IP);

  audioUdp.beginPacket(destination, AUDIO_PORT);
  audioUdp.write((uint8_t *)audioBuffer, samplesRead * sizeof(int16_t));
  audioUdp.endPacket();
}

// ---------------------------------------------------------------------
// ARDUINO SETUP / LOOP
// ---------------------------------------------------------------------
void setup()
{
  Serial.begin(115200);
  delay(200);

  WifiConnect();
  MicInit();
}

void loop()
{
  WifiMaintain();
  MicStream();
}
