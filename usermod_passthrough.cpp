#include "wled.h"
#include "driver/i2s.h"
#include <WiFi.h>

// ======================================================
// WS2811 I2S RX STREAMING TEST V6
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
// V6:
// - RX Task on Core 1
// - Low task priority
// - Smaller DMA allocation
// - Periodic taskYIELD()
// - Wi-Fi diagnostics
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
// approx. 100 ns per serial sample
// ======================================================

#define I2S_SAMPLE_RATE 312500


// ======================================================
// TEST FRAME
//
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
// Previously confirmed:
//
// 0 HIGH ≈ 3-4 samples
// 1 HIGH ≈ 12-13 samples
// ======================================================

#define ONE_HIGH_SAMPLES 8

#define MIN_HIGH_SAMPLES 2
#define MAX_HIGH_SAMPLES 20


// ======================================================
// RESET
//
// ~100 us at our effective sample clock
// ======================================================

#define RESET_LOW_SAMPLES 1000


// ======================================================
// DMA
//
// Reduced from V5:
//
// V5 = 16 x 1024
// V6 = 8 x 512
//
// This reduces DMA-capable RAM consumption.
// ======================================================

#define DMA_BUFFER_COUNT  8
#define DMA_BUFFER_LENGTH 512

#define READ_WORDS 512

static uint16_t dmaBuffer[READ_WORDS];


// ======================================================
// FRAME BUFFERS
// ======================================================

static uint8_t workingFrame[FRAME_BYTES];

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
// TASK
// ======================================================

static TaskHandle_t rxTaskHandle = nullptr;

static volatile bool rxTaskRunning = false;


// ======================================================
// FRAME LOCK
// ======================================================

static portMUX_TYPE frameMux =
  portMUX_INITIALIZER_UNLOCKED;


// ======================================================
// DEBUG
// ======================================================

static uint32_t lastDebugMs = 0;
static uint32_t lastFramePrintMs = 0;


// ======================================================
// RESET WORKING FRAME
// ======================================================

static inline void resetFrame()
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

static inline void addBit(bool bit)
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
// DECODE HIGH
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
// BAD FRAME
// ======================================================

static inline void registerBadFrame(
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
// RESET GAP
// ======================================================

static inline void onResetGap()
{
  resetsSeen++;


  // Decode final pending HIGH before closing frame.
  if (havePendingHigh)
  {
    decodeHigh(
      pendingHighLength
    );


    havePendingHigh = false;

    pendingHighLength = 0;
  }


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
  if (runLength == 0)
  {
    currentLevel = level;

    runLength = 1;

    return;
  }


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


  // Frame boundary
  if (
    lowSamples >= RESET_LOW_SAMPLES
  )
  {
    onResetGap();

    return;
  }


  // Normal data bit
  if (havePendingHigh)
  {
    decodeHigh(
      pendingHighLength
    );


    havePendingHigh = false;

    pendingHighLength = 0;
  }
}


// ======================================================
// PROCESS 16-BIT I2S WORD
// ======================================================

static inline void processWord(
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
// I2S RX TASK
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

        // Block until DMA data exists.
        portMAX_DELAY
      );


    if (
      err != ESP_OK
    )
    {
      readErrors++;

      taskYIELD();

      continue;
    }


    if (
      bytesRead == 0
    )
    {
      taskYIELD();

      continue;
    }


    dmaReads++;


    size_t words =
      bytesRead /
      sizeof(uint16_t);


    // ==================================================
    // PROCESS DMA DATA
    // ==================================================

    for (
      size_t i = 0;
      i < words;
      i++
    )
    {
      processWord(
        dmaBuffer[i]
      );


      // ------------------------------------------------
      // Give WLED / Arduino loop CPU time regularly.
      // ------------------------------------------------

      if (
        (i & 0x3F) == 0x3F
      )
      {
        taskYIELD();
      }
    }


    // Give equal/lower priority tasks a scheduling point.
    taskYIELD();
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


  return true;
}


// ======================================================
// START RX TASK
// ======================================================

static bool startRxTask()
{
  // ====================================================
  // IMPORTANT V6 CHANGE
  //
  // Core 1 instead of Core 0.
  //
  // Priority 1 instead of 3.
  // ====================================================

  BaseType_t result =
    xTaskCreatePinnedToCore(
      i2sRxTask,

      "WS2811_I2S_RX",

      6144,

      nullptr,

      1,

      &rxTaskHandle,

      1
    );


  return (
    result == pdPASS
  );
}


// ======================================================
// COPY GOOD FRAME
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
// PRINT GOOD FRAME
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


    Serial.print("IC");
    Serial.print(i + 1);


    Serial.print(": G=");
    Serial.print(
      localFrame[p + 0]
    );


    Serial.print(" R=");
    Serial.print(
      localFrame[p + 1]
    );


    Serial.print(" B=");
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

class I2SStreamingRxV6Usermod :
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
      "WS2811 I2S RX STREAMING TEST V6"
    );


    Serial.println(
      "Wi-Fi friendly RX task"
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
      "RX task Core: 1"
    );


    Serial.println(
      "RX task Priority: 1"
    );


    Serial.println(
      "Periodic Yield: ENABLED"
    );


    Serial.println(
      "================================"
    );


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
  // ====================================================

  void loop() override
  {
    uint32_t now =
      millis();


    // ==================================================
    // PRINT FRAME ONCE PER SECOND
    // ==================================================

    if (
      now - lastFramePrintMs >= 1000
    )
    {
      lastFramePrintMs = now;


      if (
        framesGood > 0
      )
      {
        printLastGoodFrame();
      }
    }


    // ==================================================
    // STATUS EVERY 2 SECONDS
    // ==================================================

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
        "  BadPartial: "
      );

      Serial.print(
        badPartial
      );


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

      Serial.print(
        rxTaskRunning ?
        "RUNNING" :
        "STOPPED"
      );


      Serial.print(
        "  WiFi: "
      );


      wl_status_t wifiStatus =
        WiFi.status();


      if (
        wifiStatus == WL_CONNECTED
      )
      {
        Serial.print(
          "CONNECTED "
        );


        Serial.print(
          WiFi.localIP()
        );
      }
      else
      {
        Serial.print(
          "NOT_CONNECTED("
        );


        Serial.print(
          (int)wifiStatus
        );


        Serial.print(
          ")"
        );
      }


      Serial.println();
    }
  }


  uint16_t getId() override
  {
    return 0x5052;
  }
};


// ======================================================
// REGISTER
// ======================================================

static I2SStreamingRxV6Usermod
i2sStreamingRxV6Usermod;


REGISTER_USERMOD(
  i2sStreamingRxV6Usermod
);
