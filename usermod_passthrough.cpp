#include "wled.h"
#include "driver/i2s.h"

// ======================================================
// WS2811 I2S RX STREAMING TEST V5
//
// ESP32 Classic / GL-C-016WL-D
//
// INPUT:
// GPIO25 = WS2811 DATA
//
// I2S:
// GPIO16 = BCK  -> LEAVE UNCONNECTED
// GPIO2  = WS   -> LEAVE UNCONNECTED
//
// RX ONLY
//
// Main change in V5:
// I2S RX + decoding runs in a dedicated FreeRTOS task.
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
// 312500 × 32 ≈ 10 MHz BCK
// ~100 ns per sampled serial bit
// ======================================================

#define I2S_SAMPLE_RATE 312500


// ======================================================
// TEST FRAME
//
// Current test:
// 5 WS2811 IC
// 15 bytes
// 120 bits
// ======================================================

#define TEST_ICS     5
#define FRAME_BYTES  (TEST_ICS * 3)
#define FRAME_BITS   (FRAME_BYTES * 8)


// ======================================================
// WS2811 TIMING
// ======================================================
//
// At ~10 MHz:
//
// 0 HIGH ≈ 3-4 samples
// 1 HIGH ≈ 12-13 samples
//
// Proven threshold:
// ======================================================

#define ONE_HIGH_SAMPLES 8

#define MIN_HIGH_SAMPLES 2
#define MAX_HIGH_SAMPLES 20


// ======================================================
// RESET GAP
//
// 1000 samples ≈ 100 us
// ======================================================

#define RESET_LOW_SAMPLES 1000


// ======================================================
// DMA
// ======================================================

#define DMA_BUFFER_COUNT 16
#define DMA_BUFFER_LENGTH 1024

// Local task read buffer
#define READ_WORDS 2048

static uint16_t dmaBuffer[READ_WORDS];


// ======================================================
// FRAME BUFFERS
// ======================================================

// Frame currently being decoded
static uint8_t workingFrame[FRAME_BYTES];

// Last known-good complete frame
static uint8_t lastGoodFrame[FRAME_BYTES];


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

static volatile uint32_t framesGood = 0;

static volatile uint32_t bad119 = 0;
static volatile uint32_t badPartial = 0;
static volatile uint32_t badOther = 0;

static volatile uint32_t invalidHigh = 0;

static volatile uint32_t resetsSeen = 0;

static volatile uint32_t dmaReads = 0;

static volatile uint32_t readErrors = 0;

static volatile uint16_t lastBadBits = 0;

static volatile uint16_t minBadBits = 0xFFFF;
static volatile uint16_t maxBadBits = 0;


// ======================================================
// TASK STATE
// ======================================================

static TaskHandle_t rxTaskHandle = nullptr;

static volatile bool rxTaskRunning = false;


// ======================================================
// FRAME COPY PROTECTION
// ======================================================

static portMUX_TYPE frameMux =
  portMUX_INITIALIZER_UNLOCKED;


// ======================================================
// DEBUG
// ======================================================

static uint32_t lastDebugMs = 0;
static uint32_t lastFramePrintMs = 0;


// ======================================================
// RESET CURRENT FRAME
// ======================================================

static void resetFrame()
{
  memset(
    workingFrame,
    0,
    sizeof(workingFrame)
  );

  frameBitCount = 0;
}


// ======================================================
// ADD ONE BIT
// ======================================================

static inline void addBit(
  bool bit
)
{
  if (!synced)
  {
    return;
  }


  if (
    frameBitCount >= FRAME_BITS
  )
  {
    return;
  }


  uint16_t byteIndex =
    frameBitCount >> 3;


  uint8_t bitPosition =
    7 - (frameBitCount & 0x07);


  if (bit)
  {
    workingFrame[byteIndex] |=
      (1U << bitPosition);
  }


  frameBitCount++;
}


// ======================================================
// DECODE HIGH PULSE
// ======================================================

static inline void decodeHigh(
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
// SAVE GOOD FRAME
// ======================================================

static void saveGoodFrame()
{
  portENTER_CRITICAL(
    &frameMux
  );


  memcpy(
    lastGoodFrame,
    workingFrame,
    FRAME_BYTES
  );


  framesGood++;


  portEXIT_CRITICAL(
    &frameMux
  );
}


// ======================================================
// REGISTER BAD FRAME
// ======================================================

static void registerBadFrame(
  uint16_t bits
)
{
  lastBadBits = bits;


  if (
    bits < minBadBits
  )
  {
    minBadBits = bits;
  }


  if (
    bits > maxBadBits
  )
  {
    maxBadBits = bits;
  }


  if (
    bits == FRAME_BITS - 1
  )
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
// FRAME RESET GAP
// ======================================================

static inline void onResetGap()
{
  resetsSeen++;


  // ----------------------------------------------------
  // Decode the last HIGH pulse before closing frame
  // ----------------------------------------------------

  if (havePendingHigh)
  {
    decodeHigh(
      pendingHighLength
    );


    havePendingHigh = false;

    pendingHighLength = 0;
  }


  // ----------------------------------------------------
  // Validate completed frame
  // ----------------------------------------------------

  if (synced)
  {
    uint16_t completedBits =
      frameBitCount;


    if (
      completedBits == FRAME_BITS
    )
    {
      saveGoodFrame();
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


  // ----------------------------------------------------
  // Start next frame
  // ----------------------------------------------------

  resetFrame();

  synced = true;
}


// ======================================================
// PROCESS DIGITAL SAMPLE
// ======================================================

static inline void processSample(
  bool level
)
{
  // Initial sample
  if (
    runLength == 0
  )
  {
    currentLevel = level;

    runLength = 1;

    return;
  }


  // Same level continues
  if (
    level == currentLevel
  )
  {
    runLength++;

    return;
  }


  // ====================================================
  // EDGE
  // ====================================================

  uint32_t completedRun =
    runLength;


  bool completedLevel =
    currentLevel;


  currentLevel = level;

  runLength = 1;


  // ====================================================
  // HIGH -> LOW
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


  // ----------------------------------------------------
  // Frame boundary
  // ----------------------------------------------------

  if (
    lowSamples >=
    RESET_LOW_SAMPLES
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
// PROCESS ONE 16-BIT I2S WORD
// ======================================================

static inline void processWord(
  uint16_t word
)
{
  // Keep same order as previous successful tests.

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
// I2S RX TASK
// ======================================================
//
// Dedicated task:
// blocks on i2s_read() and processes data immediately.
// ======================================================

static void i2sRxTask(
  void *parameter
)
{
  rxTaskRunning = true;


  while (true)
  {
    size_t bytesRead = 0;


    esp_err_t err =
      i2s_read(
        RX_I2S_PORT,
        dmaBuffer,
        sizeof(dmaBuffer),
        &bytesRead,

        // Important:
        // block until DMA data is available
        portMAX_DELAY
      );


    if (
      err != ESP_OK
    )
    {
      readErrors++;

      continue;
    }


    if (
      bytesRead == 0
    )
    {
      continue;
    }


    dmaReads++;


    size_t words =
      bytesRead /
      sizeof(uint16_t);


    // --------------------------------------------------
    // Decode immediately
    // --------------------------------------------------

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


  // ----------------------------------------------------
  // Install I2S
  // ----------------------------------------------------

  err =
    i2s_driver_install(
      RX_I2S_PORT,
      &config,
      0,
      nullptr
    );


  if (
    err != ESP_OK
  )
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
  // PINS
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


  if (
    err != ESP_OK
  )
  {
    Serial.print(
      "I2S PIN ERROR: "
    );

    Serial.println(
      esp_err_to_name(err)
    );

    return false;
  }


  // ----------------------------------------------------
  // Clear old DMA contents before starting task
  // ----------------------------------------------------

  i2s_zero_dma_buffer(
    RX_I2S_PORT
  );


  return true;
}


// ======================================================
// START RX TASK
// ======================================================

static bool startRxTask()
{
  /*
   * Put RX on Core 0.
   *
   * WLED's Arduino loop normally runs on the other core,
   * so RX decoding gets its own execution path.
   *
   * Priority 3 is intentionally above ordinary
   * background work but not extremely high.
   */

  BaseType_t result =
    xTaskCreatePinnedToCore(
      i2sRxTask,

      "WS2811_I2S_RX",

      8192,

      nullptr,

      3,

      &rxTaskHandle,

      0
    );


  return (
    result == pdPASS
  );
}


// ======================================================
// COPY LAST GOOD FRAME FOR PRINTING
// ======================================================

static void copyLastGoodFrame(
  uint8_t *destination
)
{
  portENTER_CRITICAL(
    &frameMux
  );


  memcpy(
    destination,
    lastGoodFrame,
    FRAME_BYTES
  );


  portEXIT_CRITICAL(
    &frameMux
  );
}


// ======================================================
// PRINT LAST GOOD FRAME
// ======================================================

static void printLastGoodFrame()
{
  uint8_t localFrame[
    FRAME_BYTES
  ];


  copyLastGoodFrame(
    localFrame
  );


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


    Serial.print(
      "IC"
    );

    Serial.print(
      i + 1
    );


    Serial.print(
      ": G="
    );

    Serial.print(
      localFrame[p + 0]
    );


    Serial.print(
      " R="
    );

    Serial.print(
      localFrame[p + 1]
    );


    Serial.print(
      " B="
    );

    Serial.println(
      localFrame[p + 2]
    );
  }


  Serial.println(
    "=================================="
  );
}


// ======================================================
// USERMOD
// ======================================================

class I2SStreamingRxTaskUsermod :
  public Usermod
{

public:

  void setup() override
  {
    memset(
      workingFrame,
      0,
      sizeof(workingFrame)
    );


    memset(
      lastGoodFrame,
      0,
      sizeof(lastGoodFrame)
    );


    Serial.println();

    Serial.println(
      "================================"
    );


    Serial.println(
      "WS2811 I2S RX STREAMING TEST V5"
    );


    Serial.println(
      "Dedicated FreeRTOS RX Task"
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
      "RMT: NOT USED"
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
      "Reset: "
    );

    Serial.print(
      RESET_LOW_SAMPLES
    );

    Serial.println(
      " samples (~100 us)"
    );


    Serial.print(
      "DMA: "
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
      "RX task core: 0"
    );


    Serial.println(
      "RX task priority: 3"
    );


    Serial.println(
      "Blocking i2s_read: ENABLED"
    );


    Serial.println(
      "================================"
    );


    // ==================================================
    // I2S SETUP
    // ==================================================

    if (
      !setupI2S()
    )
    {
      Serial.println(
        "I2S RX FAILED"
      );

      return;
    }


    Serial.println(
      "I2S RX READY"
    );


    // ==================================================
    // START TASK
    // ==================================================

    if (
      !startRxTask()
    )
    {
      Serial.println(
        "RX TASK CREATE FAILED"
      );

      return;
    }


    Serial.println(
      "RX TASK STARTED"
    );


    Serial.println(
      "================================"
    );
  }


  // ====================================================
  // NORMAL WLED LOOP
  //
  // No I2S receiving happens here anymore.
  // Only lightweight diagnostics.
  // ====================================================

  void loop() override
  {
    uint32_t now =
      millis();


    // --------------------------------------------------
    // Print frame once per second
    // --------------------------------------------------

    if (
      now - lastFramePrintMs >= 1000
    )
    {
      lastFramePrintMs =
        now;


      if (
        framesGood > 0
      )
      {
        printLastGoodFrame();
      }
    }


    // --------------------------------------------------
    // Statistics every 2 seconds
    // --------------------------------------------------

    if (
      now - lastDebugMs >= 2000
    )
    {
      lastDebugMs =
        now;


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


      if (
        minBadBits == 0xFFFF
      )
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

      Serial.print(
        dmaReads
      );


      Serial.print(
        "  ReadErrors: "
      );

      Serial.print(
        readErrors
      );


      Serial.print(
        "  Task: "
      );

      Serial.println(
        rxTaskRunning ?
        "RUNNING" :
        "STOPPED"
      );
    }
  }


  uint16_t getId() override
  {
    return 0x5051;
  }
};


// ======================================================
// REGISTER
// ======================================================

static I2SStreamingRxTaskUsermod
i2sStreamingRxTaskUsermod;


REGISTER_USERMOD(
  i2sStreamingRxTaskUsermod
);
