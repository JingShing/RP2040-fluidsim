#include <Wire.h>

#include "FluidRenderer.hpp"  // 新渲染库
#include "ParticleSimulation.hpp"
#include "lgfx_gc9a01.hpp"
#include "qmi8658c.hpp"

// ────────────────── 全局硬件 & 逻辑对象 ──────────────────
static LGFX_GC9A01 display;
static QMI8658C imu;
static ParticleSimulation sim;
static FluidRenderer renderer(&display, &sim);  // <<<

// ─────────────── 帧率 / 物理步长设定 ───────────────
static unsigned long lastFrameTime = 0;
static constexpr float TARGET_FPS = 10.0f;
static constexpr float FRAME_TIME_MS = 1000.0f / TARGET_FPS;
static constexpr float FIXED_DT = 1.0f / TARGET_FPS;

// ─────────────── 统计计时器 ───────────────
static unsigned long physAccumUs = 0;
static unsigned long rendAccumUs = 0;
static unsigned int frameCounter = 0;
static unsigned long lastPrintMs = 0;

// ────────────────── 初始化 ──────────────────
void setup() {
  Serial.begin(115200);
  Serial.println("Fluid Simulation Starting...");

  // --- LCD ---
  display.begin(PIN_LCD_SCLK, PIN_LCD_MOSI, PIN_LCD_DC, PIN_LCD_CS, PIN_LCD_RST,
                PIN_LCD_BL);
  display.setRotation(0);
  display.fillScreen(TFT_BLACK);

  // --- IMU ---
  Wire1.setSDA(PIN_IMU_SDA);
  Wire1.setSCL(PIN_IMU_SCL);
  Wire1.setClock(400000);
  Wire1.begin();
  if (!imu.begin(&Wire1, QMI8658C_I2C_ADDRESS_PULLUP))
    Serial.println("IMU init failed (simulation will still run)");
  else {
    imu.configureAcc(QMI8658C::AccScale::ACC_SCALE_4G,
                     QMI8658C::AccODR::ACC_ODR_250HZ,
                     QMI8658C::AccLPF::ACC_LPF_5_32PCT);
  }

  // --- Simulation ---
  sim.begin(&imu);

  // --- Renderer palette (可调) ---
  renderer.setGridSolidColor(TFT_DARKGREY);
  renderer.setGridFluidColor(TFT_BLUE);

  lastFrameTime = millis();
}

// ────────────────── 主循环 ──────────────────
void loop() {
  unsigned long nowMs = millis();
  float deltaTime = (nowMs - lastFrameTime) * 0.001f;

  // ----------- 固定步长物理 -----------
  static float acc = 0.0f;
  acc += deltaTime;
  acc = min(acc, 0.1f);

  while (acc >= FIXED_DT) {
    unsigned long t0 = micros();
    sim.simulate(FIXED_DT);
    physAccumUs += micros() - t0;
    acc -= FIXED_DT;
  }

  // ----------- 渲染（格子模式） -----------
  unsigned long t0 = micros();
  renderer.render(FluidRenderer::PARTIAL_GRID);
  rendAccumUs += micros() - t0;

  // --- 叠加 FPS 文本 ---
  static int fpsFrames = 0;
  static unsigned long fpsTimer = 0;
  static float fps = 0.0f;

  ++fpsFrames;
  if (nowMs - fpsTimer >= 1000) {
    fps = fpsFrames * 1000.0f / (nowMs - fpsTimer);
    fpsFrames = 0;
    fpsTimer = nowMs;
  }
  display.startWrite();
  display.setTextColor(TFT_WHITE, display.color565(0, 0, 16));
  display.setCursor(5, 5);
  display.printf("FPS: %.1f", fps);
  display.endWrite();

  lastFrameTime = nowMs;
  ++frameCounter;

  // ----------- 每秒串口打印统计 -----------
  if (nowMs - lastPrintMs >= 1000) {
    Serial.printf("[FPS %3u] Physics: %.2f ms  |  Render: %.2f ms\r\n",
                  frameCounter, physAccumUs * 0.001f, rendAccumUs * 0.001f);
    physAccumUs = rendAccumUs = 0;
    frameCounter = 0;
    lastPrintMs = nowMs;
  }
}
