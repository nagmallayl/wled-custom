#include "wled.h"
#include "driver/i2s.h"

// ======================================================
// WS2811 I2S RX STREAMING TEST V3
//
// ESP32 Classic / GL-C-016WL-D
// Arduino-ESP32 2.x / ESP-IDF 4.4
//
// INPUT:
// GPIO25 = WS2811 DATA
//
// I2S clocks:
// GPIO16 = BCK   -> LEAVE UNCONNECTED
// GPIO2  = WS    -> LEAVE UNCONNECTED
//
// RX only - no LED output
// ======================================================


// ======================================================
// PINS
// ======================================================

#define DATA_INPUT_GPIO 25

#define I2S_BCK_GPIO 16
#define I2S_WS_GPIO  2


// ======================================================
// I2S
// ======================================================

#define RX_I2S_PORT I2S_NUM_1


// ======================================================
// SAMPLING
//
// 312500 × 32 = ~10 MHz BCK
// ~100 ns per serial sample
// ======================================================

#define I2S_SAMPLE_RATE 312500
#define SAMPLE_CLOCK_HZ 10000000UL


// ======================================================
// CURRENT TEST
//
// 5 WS2811 IC
// 5 × 24 = 120 bits
// ======================================================

#define TEST_ICS     5
#define FRAME_BYTES  (TEST_ICS * 3)
#define FRAME_BITS   (FRAME_BYTES * 8)


// ======================================================
// WS2811 DECODER
//
// Previous successful measurements:
//
// 0 HIGH ≈ 3-4 samples
// 1 HIGH ≈ 12-13 samples
//
// threshold = 8
// ======================================================

#define ONE_HIGH_SAMPLES 8

#define MIN_HIGH_SAMPLES 2
#define MAX_HIGH_SAMPLES 20


// ======================================================
// RESET
//
// 10 MHz:
// 500 samples ≈ 50 us
// ======================================================

#define RESET_LOW_SAMPLES 500


// ======================================================
// DMA
//
// V2:
// 8 × 512
//
// V3:
// 16 × 1024
// ======================================================

#define DMA_BUFFER_COUNT 16
#define DMA_BUFFER_LENGTH 1024

#define READ_WORDS 1024

static uint16_t dmaBuffer[READ_WORDS];


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

static uint32_t bad119 = 0;
static uint32_t badPartial = 0;
static uint32_t badOther = 0;

static uint32_t invalidHigh = 0;

static uint32_t resetsSeen = 0;

static uint32_t dmaReads = 0;

static uint32_t lastDebugMs = 0;
static uint32_t lastFramePrintMs = 0;


// Last completed bad frame
static uint16_t lastBadBits = 0;


// Min/max completed frame sizes
static uint16_t minBadBits = 0xFFFF;
static uint16_t maxBadBits = 0;


// ======================================================
// RESET FRAME BUFFER
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
// ADD ONE WS2811 BIT
// ======================================================

static void addBit(bool bit)
{
  if (!synced)
  {
    return;
  }


  // Ignore anything beyond expected frame size.
  if (frameBitCount >= FRAME_BITS)
  {
    return;
  }


  uint16_t byteIndex =
    frameBitCount >> 3;


  uint8_t bitPosition =
    7 - (frameBitCount & 0x07);


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
  {
    return;
  }


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
// PRINT GOOD FRAME
// ======================================================

static void printFrame()
{
  uint32_t now = millis();


  // Print max once per second.
  if (
    now - lastFramePrintMs < 1000
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
    uint8_t p = i * 3;


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
// CLASSIFY BAD COMPLETED FRAME
// ======================================================

static void registerBadFrame(
  uint16_t bits
)
{
  lastBadBits = bits;


  if (bits < minBadBits)
  {
    minBadBits = bits;
  }


  if (bits > maxBadBits)
  {
    maxBadBits = bits;
  }


  if (bits == FRAME_BITS - 1)
  {
    bad119++;
  }
  else if (
    bits > 0 &&
    bits < FRAME_BITS - 1
  )
  {
    badPartial++;
  }
  else
  {
    badOther++;
  }
}


// ======================================================
// RESET / FRAME BOUNDARY
// ======================================================

static void onResetGap()
{
  resetsSeen++;


  // ====================================================
  // LAST PENDING HIGH
  //
  // Decode the final HIGH pulse before evaluating
  // the completed frame.
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
  // EVALUATE THE FRAME THAT ACTUALLY ENDED
  // ====================================================

  if (synced)
  {
    uint16_t completedBits =
      frameBitCount;


    if (
      completedBits == FRAME_BITS
    )
    {
      framesGood++;

      printFrame();
    }
    else if (
      completedBits != 0
    )
    {
      registerBadFrame(
        completedBits
      );
    }
  }


  // ====================================================
  // PREPARE FOR NEXT FRAME
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


  // Same state continues
  if (level == currentLevel)
  {
    runLength++;

    return;
  }


  // ====================================================
  // EDGE DETECTED
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
  // Save HIGH duration.
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
  // ====================================================

  uint32_t lowSamples =
    completedRun;


  // ====================================================
  // RESET GAP
  //
  // If LOW is >= 50 us, this is the frame boundary.
  // ====================================================

  if (
    lowSamples >= RESET_LOW_SAMPLES
  )
  {
    onResetGap();

    return;
  }


  // ====================================================
  // NORMAL DATA BIT
  // ====================================================

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
// PROCESS I2S WORD
// ======================================================

static void processWord(
  uint16_t word
)
{
  // Keep same bit order as V2 because decoding
  // already proved correct.

  for (
    int8_t bit = 15;
    bit >= 0;
    bit--
  )
  {
    bool level =
      (word >> bit) & 0x01;


    processSample(
      level
    );
  }
}


// ======================================================
// SETUP I2S
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


  // ====================================================
  // V3 DMA SETTINGS
  // ====================================================

  config.dma_buf_count =
    DMA_BUFFER_COUNT;


  config.dma_buf_len =
    DMA_BUFFER_LENGTH;


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

    Serial.println(
      esp_err_to_name(err)
    );

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

    Serial.println(
      esp_err_to_name(err)
    );

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
      "WS2811 I2S RX STREAMING TEST V3"
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
      " samples (~50 us)"
    );


    Serial.print(
      "DMA buffers: "
    );

    Serial.print(
      DMA_BUFFER_COUNT
    );

    Serial.print(
      " x "
    );

    Serial.println(
      DMA_BUFFER_LENGTH
    );


    Serial.println(
      "Bad-frame diagnostics: ENABLED"
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
    // STATUS EVERY 2 SECONDS
    // ==================================================

    uint32_t now =
      millis();


    if (
      now - lastDebugMs >= 2000
    )
    {
      lastDebugMs = now;


      uint32_t totalBad =
        bad119 +
        badPartial +
        badOther;


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
        totalBad
      );


      Serial.print(
        "  Bad119: "
      );

      Serial.print(
        bad119
      );


      Serial.print(
        "  BadPartial: "
      );

      Serial.print(
        badPartial
      );


      Serial.print(
        "  BadOther: "
      );

      Serial.print(
        badOther
      );


      Serial.print(
        "  LastBadBits: "
      );

      Serial.print(
        lastBadBits
      );


      Serial.print(
        "  BadRange: "
      );


      if (minBadBits == 0xFFFF)
      {
        Serial.print(
          "none"
        );
      }
      else
      {
        Serial.print(
          minBadBits
        );

        Serial.print(
          "-"
        );

        Serial.print(
          maxBadBits
        );
      }


      Serial.print(
        "  InvalidHigh: "
      );

      Serial.print(
        invalidHigh
      );


      Serial.print(
        "  LiveBits: "
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
        "  Reset: "
      );

      Serial.print(
        resetsSeen
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
    return 0x5049;
  }
};


// ======================================================
// REGISTER USERMOD
// ======================================================

static I2SStreamingRxUsermod
i2sStreamingRxUsermod;


REGISTER_USERMOD(
  i2sStreamingRxUsermod
);
