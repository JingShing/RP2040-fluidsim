/******************************************************************
 *  main.cpp  ――  Fluid-SIM + 低功耗状态机 (RP2040)
 ******************************************************************/
#include <Wire.h>
#include "FluidRenderer.hpp"
#include "LowPowerRP2040.h"  // ★ 低功耗库
#include "ParticleSimulation.hpp"
#include "lgfx_gc9a01.hpp"
#include "qmi8658c.hpp"

using namespace low_power;

/* ────── 硬件/模块 ─────────────────────────── */
static LGFX_GC9A01 display;
static QMI8658C imu;
static ParticleSimulation sim;
static FluidRenderer renderer(&display, &sim);
static ArduinoLowPowerRP2040 lp;  // ★ 低功耗对象

/* ────── 运行参数 ──────────────────────────── */
static constexpr float TARGET_FPS = 30.f;
static constexpr float FIXED_DT = 1.f / TARGET_FPS;

/* ────── 运动检测参数 ──────────────────────── */
static constexpr float GYRO_EPS = 10.0f;     // Δ阈值
static constexpr uint32_t STILL_MS = 30000;   // 判静止时间
static constexpr uint32_t DETECT_MS = 1000;  // 判静止时间

/* ────── 状态机枚举 ────────────────────────── */
enum class AppState { RUNNING, GO_SLEEP, SLEEP_POLL };
static AppState state = AppState::RUNNING;

/* ────── 陀螺仪监视变量 ────────────────────── */
static float prevGx = 0, prevGy = 0;
static uint32_t stillTimer = 0;

/* ────── 辅助：读取陀螺仪 Δ ─────────────────── */
static bool gyroMoving(float& dxdy) {
  float gx, gy, gz;
  if (!imu.readGyroscope(&gx, &gy, &gz))
    return true;  // 读失败视为运动

  dxdy = hypotf(gx - prevGx, gy - prevGy);  // √Δx²+Δy²
  prevGx = gx;
  prevGy = gy;
  return dxdy > GYRO_EPS;
}

/* ────── 初始化 ────────────────────────────── */
void setup() {
  Serial.begin(115200);
  display.begin(PIN_LCD_SCLK, PIN_LCD_MOSI, PIN_LCD_DC, PIN_LCD_CS, PIN_LCD_RST,
                PIN_LCD_BL);
  display.setRotation(0);
  display.fillScreen(TFT_BLACK);

  Wire1.setSDA(PIN_IMU_SDA);
  Wire1.setSCL(PIN_IMU_SCL);
  Wire1.setClock(400000);
  Wire1.begin();
  imu.begin(&Wire1, QMI8658C_I2C_ADDRESS_PULLUP);
  imu.configureGyro(QMI8658C::GyroScale::GYRO_SCALE_256DPS,
                    QMI8658C::GyroODR::GYRO_ODR_250HZ);

  sim.begin(&imu);
  renderer.setGridSolidColor(TFT_DARKGREY);
  renderer.setGridFluidColor(TFT_BLUE);

  lp.setSleepMode(sleep_mode_enum_t::lightSleep);
  lp.setRestart(false);  // 醒来不复位
}

/* ────── 主循环 ────────────────────────────── */
void loop() {
  float dG = 0.f;  // 本帧 gyro Δ

  switch (state) {
    /* ――― 正常运行 ――― */
    case AppState::RUNNING: {
      /* 物理 & 渲染 */
      sim.simulate(FIXED_DT);
      renderer.render(FluidRenderer::PARTIAL_GRID);

      /* 运动检测 */
      bool moving = gyroMoving(dG);
      if (moving) {
        stillTimer = millis();  // 重置
      } else if (millis() - stillTimer > STILL_MS) {
        state = AppState::GO_SLEEP;
      }
      break;
    }

    /* ――― 进入休眠前的一次性收尾 ――― */
    case AppState::GO_SLEEP: {
      // display.fillScreen(TFT_BLACK);  // 可选：关闭背光
      display.setBrightness(0);  // 若驱动支持
      state = AppState::SLEEP_POLL;
      break;
    }

    /* ――― Deep-sleep 轮询 ――― */
    case AppState::SLEEP_POLL: {
      /* 进入 deep-sleep（约 1 s） */
      lp.sleepFor(DETECT_MS, time_unit_t::ms);

      /* 醒来后只读 gyro 做判断 */
      bool moving = gyroMoving(dG);
      if (moving) {
        stillTimer = millis();
        state = AppState::RUNNING;   // 恢复正常
        display.setBrightness(255);  // 恢复背光（如有）
      }
      /* 若仍静止则下一轮 loop() 会再次 lp.sleep() */
      break;
    }
  }
}
