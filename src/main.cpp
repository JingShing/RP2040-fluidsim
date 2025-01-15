#include <Arduino.h>
#include <LovyanGFX.h>
#include <Wire.h>
#include <pico/mutex.h>
#include "lgfx_gc9a01.hpp"
#include "qmi8658c.hpp"

// 屏幕分辨率
#define SCREEN_WIDTH 240
#define SCREEN_HEIGHT 240
#define NUM_PARTICLES 200

// 圆形屏幕的半径
#define SCREEN_RADIUS (SCREEN_WIDTH / 2)

// 粒子结构
struct Particle {
  float x, y;
  float vx, vy;
  uint16_t color;
};

// 全局变量
static LGFX_GC9A01 display;
static QMI8658C imu;
static Particle particles[NUM_PARTICLES];

// 初始化粒子
void initializeParticles() {
  for (int i = 0; i < NUM_PARTICLES; ++i) {
    particles[i].x = random(SCREEN_RADIUS - 5, SCREEN_RADIUS + 5);
    particles[i].y = random(SCREEN_RADIUS - 5, SCREEN_RADIUS + 5);
    particles[i].vx = random(-50, 50) / 100.0f;
    particles[i].vy = random(-50, 50) / 100.0f;
    particles[i].color = random(0xFFFF);
  }
}

// 更新粒子位置
void updateParticles(float ax, float ay) {
  for (int i = 0; i < NUM_PARTICLES; ++i) {
    particles[i].vx += ax * 0.1f;
    particles[i].vy += ay * 0.1f;

    particles[i].x += particles[i].vx;
    particles[i].y += particles[i].vy;

    // 边界反射
    float dx = particles[i].x - SCREEN_RADIUS;
    float dy = particles[i].y - SCREEN_RADIUS;
    if (sqrt(dx * dx + dy * dy) >= SCREEN_RADIUS - 5) {
      particles[i].vx = -particles[i].vx * 0.9f;
      particles[i].vy = -particles[i].vy * 0.9f;
      particles[i].x -= particles[i].vx;
      particles[i].y -= particles[i].vy;
    }
  }
}

// 绘制粒子
void drawParticles() {
  display.startWrite();
  display.fillScreen(TFT_BLACK);
  for (int i = 0; i < NUM_PARTICLES; ++i) {
    display.fillCircle(particles[i].x, particles[i].y, 2, particles[i].color);
  }
  display.endWrite();
}

void setup() {
  Serial.begin(115200);

  // 初始化显示屏
  display.begin(PIN_LCD_SCLK, PIN_LCD_MOSI, PIN_LCD_DC, PIN_LCD_CS, PIN_LCD_RST,
                PIN_LCD_BL);

  // 初始化IMU
  Wire1.setSDA(PIN_IMU_SDA);
  Wire1.setSCL(PIN_IMU_SCL);
  Wire1.setClock(400000);
  Wire1.begin();
  imu.begin(&Wire1, QMI8658C_I2C_ADDRESS_PULLUP);
  imu.configureAcc(QMI8658C::AccScale::ACC_SCALE_4G,
                   QMI8658C::AccODR::ACC_ODR_250HZ,
                   QMI8658C::AccLPF::ACC_LPF_5_32PCT);

  // 初始化粒子
  initializeParticles();
}

void loop() {
  // 读取IMU数据
  float ax = 0.0f, ay = 0.0f;
  imu.readAccelerometer(&ax, &ay, nullptr);

  // 更新粒子状态
  updateParticles(ax, ay);

  // 绘制粒子
  drawParticles();

  // 控制帧率
  delay(40);
}
