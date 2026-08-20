#include "wled.h"
#include "driver/i2s.h"

// ======================================================
// WS2811 I2S RX STREAMING TEST
//
// ESP32 Classic / Arduino-ESP32 2.x / IDF 4.4
//
// INPUT:
// GPIO25 = WS2811 DATA
//
// I2S generated clocks:
// GPIO16 = BCK   (leave unconnected)
// GPIO2  = WS    (leave unconnected)
//
// NO LED OUTPUT IN THIS TEST
// NO RMT RX
//
// Purpose:
// continuously sample WS2811 into DMA/RAM,
// removing the RMT frame-size limitation.
// ======================================================


// ======================================================
// PINS
// ======================================================

#define DATA_INPUT_GPIO 25

#define I2S_BCK_GPIO    16
#define I2S_WS_GPIO     2


// ======================================================
// I2S
// ======================================================

#define RX_I2S_PORT I2S_NUM_1


// ======================================================
// SAMPLING
// ======================================================
//
// Stereo × 16 bit:
//
// BCK = sample_rate × 2 × 16
//
// 312500 × 32 = 10,000,000 Hz
//
// Therefore:
//
// one sampled bit = 100 ns
// ======================================================

#define I2S_SAMPLE_RATE 312500

#define SAMPLE_CLOCK_HZ 10000000UL


// ======================================================
// TEST FRAME
// ======================================================
//
// Current short strip:
// 5 WS2811 IC
//
// 5 × 24 = 120 bits
// 5 × 3  = 15 data bytes
// ======================================================

#define TEST_ICS     5
#define FRAME_BYTES  (TEST_ICS * 3)
#define FRAME_BITS   (FRAME_BYTES * 8)


// ======================================================
// WS2811 TIMING
// ======================================================
//
// Previous real measurements:
//
// bit 0 HIGH:
// ~0.35 - 0.38 us
//
// bit 1 HIGH:
// ~1.25 us
//
// At 10 MHz:
//
// bit 0 HIGH ≈ 3-4 samples
// bit 1 HIGH ≈ 12-13 samples
//
// threshold = 8 samples
// ======================================================

#define ONE_HIGH_SAMPLES 8

// Reject impossible HIGH pulses
#define MIN_HIGH_SAMPLES 2
#define MAX_HIGH_SAMPLES 20


// ======================================================
// RESET / FRAME BOUNDARY
// ======================================================
//
// At 10 MHz:
//
// 50 us = 500 samples
//
// Increased from 300 to 500.
// ======================================================

#define RESET_LOW_SAMPLES 500


// ======================================================
// DMA READ BUFFER
// ======================================================

#define DMA_WORDS 512

static uint16_t dmaBuffer[DMA_WORDS];


// ======================================================
// FRAME BUFFER
// ======================================================

static uint8_t frameBuffer[FRAME_BYTES];


// ======================================================
// DECODER STATE
// ======================================================

static bool synced = false;

static bool currentLevel = false;

static uint32_t runLength = 0;

static uint32_t pendingHighLength = 0;

static bool havePendingHigh = false;

static uint16_t frameBitCount = 0;


// ======================================================
// STATISTICS
// ======================================================

static uint32_t framesGood = 0;
static uint32_t framesBad = 0;

static uint32_t invalidHigh = 0;

static uint32_t resetsSeen = 0;

static uint32_t dmaReads = 0;

static uint32_t lastDebugMs = 0;
static uint32_t lastFramePrintMs = 0;


// ======================================================
// RESET FRAME
// ======================================================

static void resetFrame()
{
  memset(
    frameBuffer,
    0,
    sizeof(frameBuffer)
  );

  frameBitCount = 0;
}


// ======================================================
// ADD ONE DECODED WS2811 BIT
// ======================================================

static void addBit(bool bit)
{
  if (!synced)
    return;


  if (frameBitCount >= FRAME_BITS)
  {
    return;
  }


  uint16_t byteIndex =
    frameBitCount >> 3;


  uint8_t bitPosition =
    7 - (frameBitCount & 7);


  if (bit)
  {
    frameBuffer[byteIndex] |=
      (1U << bitPosition);
  }


  frameBitCount++;
}


// ======================================================
// DECODE HIGH PULSE
// ======================================================

static void decodeHigh(
  uint32_t highSamples
)
{
  if (!synced)
    return;


  if (
    highSamples < MIN_HIGH_SAMPLES ||
    highSamples > MAX_HIGH_SAMPLES
  )
  {
    invalidHigh++;
    return;
  }


  bool bit =
    highSamples >= ONE_HIGH_SAMPLES;


  addBit(bit);
}


// ======================================================
// PRINT FRAME
// ======================================================

static void printFrame()
{
  uint32_t now = millis();


  if (
    now - lastFramePrintMs <
    1000
  )
  {
    return;
  }


  lastFramePrintMs = now;


  Serial.println();

  Serial.println(
    "========== I2S RX FRAME =========="
  );


  for (
    uint8_t i = 0;
    i < TEST_ICS;
    i++
  )
  {
    uint8_t p =
      i * 3;


    Serial.print("IC");
    Serial.print(i + 1);


    Serial.print(": G=");
    Serial.print(
      frameBuffer[p + 0]
    );


    Serial.print(" R=");
    Serial.print(
      frameBuffer[p + 1]
    );


    Serial.print(" B=");
    Serial.println(
      frameBuffer[p + 2]
    );
  }


  Serial.println(
    "=================================="
  );
}


// ======================================================
// FRAME BOUNDARY
// ======================================================

static void onResetGap()
{
  resetsSeen++;


  // ====================================================
  // IMPORTANT FIX
  //
  // If the last WS2811 HIGH pulse is still pending
  // when the long RESET LOW arrives, decode it first.
  //
  // This should prevent ending at 119/120 bits.
  // ====================================================

  if (havePendingHigh)
  {
    decodeHigh(
      pendingHighLength
    );

    havePendingHigh = false;

    pendingHighLength = 0;
  }


  // ====================================================
  // VALIDATE PREVIOUS FRAME
  // ====================================================

  if (synced)
  {
    if (
      frameBitCount == FRAME_BITS
    )
    {
      framesGood++;

      printFrame();
    }
    else if (
      frameBitCount != 0
    )
    {
      framesBad++;
    }
  }


  // ====================================================
  // START NEW FRAME
  // ====================================================

  resetFrame();

  synced = true;
}


// ======================================================
// PROCESS ONE DIGITAL SAMPLE
// ======================================================

static void processSample(
  bool level
)
{
  // First sample
  if (runLength == 0)
  {
    currentLevel = level;

    runLength = 1;

    return;
  }


  // Same level continues
  if (level == currentLevel)
  {
    runLength++;

    return;
  }


  // ====================================================
  // LEVEL CHANGED
  // ====================================================

  uint32_t completedRun =
    runLength;


  bool completedLevel =
    currentLevel;


  currentLevel = level;

  runLength = 1;


  // ====================================================
  // HIGH -> LOW
  //
  // Save HIGH length.
  // ====================================================

  if (completedLevel)
  {
    pendingHighLength =
      completedRun;

    havePendingHigh =
      true;

    return;
  }


  // ====================================================
  // LOW -> HIGH
  //
  // We now know:
  //
  // previous HIGH length
  // previous LOW length
  //
  // So one WS2811 bit can be decoded.
  // ====================================================

  uint32_t lowSamples =
    completedRun;


  // ----------------------------------------------------
  // Reset / frame boundary
  //
  // Handle RESET before normal bit progression.
  // ----------------------------------------------------

  if (
    lowSamples >= RESET_LOW_SAMPLES
  )
  {
    onResetGap();

    return;
  }


  // ----------------------------------------------------
  // Normal WS2811 bit
  // ----------------------------------------------------

  if (havePendingHigh)
  {
    decodeHigh(
      pendingHighLength
    );

    havePendingHigh =
      false;

    pendingHighLength =
      0;
  }
}


// ======================================================
// PROCESS ONE I2S WORD
// ======================================================
//
// Each 16-bit I2S word contains 16 consecutive
// serial samples from DATA_INPUT_GPIO.
// ======================================================

static void processWord(
  uint16_t word
)
{
  for (
    int8_t bit = 15;
    bit >= 0;
    bit--
  )
  {
    bool level =
      (word >> bit) & 0x01;


    processSample(level);
  }
}


// ======================================================
// SETUP I2S RX
// ======================================================

static bool setupI2S()
{
  i2s_config_t config = {};


  config.mode =
    (i2s_mode_t)(
      I2S_MODE_MASTER |
      I2S_MODE_RX
    );


  config.sample_rate =
    I2S_SAMPLE_RATE;


  config.bits_per_sample =
    I2S_BITS_PER_SAMPLE_16BIT;


  config.channel_format =
    I2S_CHANNEL_FMT_RIGHT_LEFT;


  config.communication_format =
    I2S_COMM_FORMAT_STAND_I2S;


  config.intr_alloc_flags =
    ESP_INTR_FLAG_LEVEL1;


  config.dma_buf_count =
    8;


  config.dma_buf_len =
    512;


  config.use_apll =
    false;


  config.tx_desc_auto_clear =
    false;


  config.fixed_mclk =
    0;


  esp_err_t err;


  err =
    i2s_driver_install(
      RX_I2S_PORT,
      &config,
      0,
      nullptr
    );


  if (err != ESP_OK)
  {
    Serial.print(
      "I2S DRIVER ERROR: "
    );

    Serial.println(err);

    return false;
  }


  // ====================================================
  // PIN ROUTING
  // ====================================================

  i2s_pin_config_t pins = {};


  pins.mck_io_num =
    I2S_PIN_NO_CHANGE;


  pins.bck_io_num =
    I2S_BCK_GPIO;


  pins.ws_io_num =
    I2S_WS_GPIO;


  pins.data_out_num =
    I2S_PIN_NO_CHANGE;


  pins.data_in_num =
    DATA_INPUT_GPIO;


  err =
    i2s_set_pin(
      RX_I2S_PORT,
      &pins
    );


  if (err != ESP_OK)
  {
    Serial.print(
      "I2S PIN ERROR: "
    );

    Serial.println(err);

    return false;
  }


  return true;
}


// ======================================================
// USERMOD
// ======================================================

class I2SStreamingRxUsermod :
  public Usermod
{

public:

  void setup() override
  {
    Serial.println();

    Serial.println(
      "================================"
    );

    Serial.println(
      "WS2811 I2S RX STREAMING TEST V2"
    );


    Serial.print(
      "DATA input: GPIO"
    );

    Serial.println(
      DATA_INPUT_GPIO
    );


    Serial.print(
      "I2S BCK: GPIO"
    );

    Serial.println(
      I2S_BCK_GPIO
    );


    Serial.print(
      "I2S WS: GPIO"
    );

    Serial.println(
      I2S_WS_GPIO
    );


    Serial.println(
      "BCK/WS: LEAVE UNCONNECTED"
    );


    Serial.println(
      "I2S port: 1"
    );


    Serial.println(
      "Sampling: ~10 MHz"
    );


    Serial.println(
      "RMT RX: NOT USED"
    );


    Serial.println(
      "LED output: DISABLED"
    );


    Serial.print(
      "Expected ICs: "
    );

    Serial.println(
      TEST_ICS
    );


    Serial.print(
      "Expected bits: "
    );

    Serial.println(
      FRAME_BITS
    );


    Serial.print(
      "Reset threshold: "
    );

    Serial.print(
      RESET_LOW_SAMPLES
    );

    Serial.println(
      " samples"
    );


    Serial.println(
      "Pending-last-bit fix: ENABLED"
    );


    Serial.println(
      "================================"
    );


    if (setupI2S())
    {
      Serial.println(
        "I2S RX READY"
      );
    }
    else
    {
      Serial.println(
        "I2S RX FAILED"
      );
    }


    Serial.println(
      "================================"
    );
  }


  void loop() override
  {
    size_t bytesRead = 0;


    esp_err_t err =
      i2s_read(
        RX_I2S_PORT,
        dmaBuffer,
        sizeof(dmaBuffer),
        &bytesRead,
        0
      );


    if (
      err == ESP_OK &&
      bytesRead > 0
    )
    {
      dmaReads++;


      size_t words =
        bytesRead /
        sizeof(uint16_t);


      for (
        size_t i = 0;
        i < words;
        i++
      )
      {
        processWord(
          dmaBuffer[i]
        );
      }
    }


    // ==================================================
    // STATUS
    // ==================================================

    uint32_t now =
      millis();


    if (
      now - lastDebugMs >=
      2000
    )
    {
      lastDebugMs = now;


      Serial.print(
        "Good: "
      );

      Serial.print(
        framesGood
      );


      Serial.print(
        "  Bad: "
      );

      Serial.print(
        framesBad
      );


      Serial.print(
        "  Reset: "
      );

      Serial.print(
        resetsSeen
      );


      Serial.print(
        "  InvalidHigh: "
      );

      Serial.print(
        invalidHigh
      );


      Serial.print(
        "  CurrentBits: "
      );

      Serial.print(
        frameBitCount
      );


      Serial.print(
        "/"
      );

      Serial.print(
        FRAME_BITS
      );


      Serial.print(
        "  DMAReads: "
      );

      Serial.println(
        dmaReads
      );
    }
  }


  uint16_t getId() override
  {
    return 0x5048;
  }
};


// ======================================================
// REGISTER
// ======================================================

static I2SStreamingRxUsermod
i2sStreamingRxUsermod;


REGISTER_USERMOD(
  i2sStreamingRxUsermod
);
