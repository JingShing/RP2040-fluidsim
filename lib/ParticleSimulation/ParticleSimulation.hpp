#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <stdint.h>
#include <cstring>
#include "qmi8658c.hpp"

// ─── 宏与常量 ─────────────────────────────────────
#define LOGICAL_GRID_SIZE 10
#define SCREEN_WIDTH 240
#define SCREEN_HEIGHT 240
#define PIXEL_PER_CELL (SCREEN_WIDTH / LOGICAL_GRID_SIZE)

#define FLUID_PARTICLE_THRESHOLD 1
#define FOAM_SPEED_THRESHOLD 5.0f

#define NUM_PARTICLES 30
#define PARTICLE_RADIUS 0.45f
#define FLUID_DENSITY 1.0f
#define SOLVER_ITERS_P 4
#define SEPARATE_ITERS_P 2
#define FLIP_RATIO 0.45f

// ─── 枚举 ─────────────────────────────────────────
enum CellType : uint8_t { FLUID_CELL, AIR_CELL, SOLID_CELL };
enum FluidType : uint8_t { FLUID_EMPTY, FLUID_LIQUID, FLUID_FOAM };

// ─── 结构 ─────────────────────────────────────────
struct Particle {
  float x, y;     // 位置 (逻辑)
  float vx, vy;   // 速度
  float r, g, b;  // 颜色 (调试/可视)
};

// ─── 主类 ─────────────────────────────────────────
class ParticleSimulation {
 public:
  // 供外部调用
  void begin(QMI8658C* imu);
  void simulate(float dt);

  // 渲染层只读接口
  const Particle* data() const { return m_particles; }
  int particleCount() const { return m_numParticles; }
  bool isSolid(int gx, int gy) const {
    return m_cellType[gx * GS + gy] == SOLID_CELL;
  }

  // 变化单元索引表
  inline const int* changedIndices() const { return m_changedIdx; }
  inline int changedCount() const { return m_changedCnt; }

  // 常量别名
  static constexpr int GS = LOGICAL_GRID_SIZE;
  static constexpr int GC = GS * GS;
  static constexpr int PC_MAX = NUM_PARTICLES;
  static constexpr float H = 1.0f;

  // 流体状态面板 (public 便于渲染)
  FluidType m_currFluid[GC]{};
  FluidType m_prevFluid[GC]{};

 private:
  // ──────── 网格字段 ──────────────────────
  float m_u[GC]{}, m_v[GC]{};
  float m_prevU[GC]{}, m_prevV[GC]{};
  float m_du[GC]{}, m_dv[GC]{};
  float m_pressure[GC]{};
  float m_s[GC]{};
  CellType m_cellType[GC]{};

  // ──────── 粒子字段 ──────────────────────
  Particle m_particles[PC_MAX];
  int m_numParticles{0};

  // 空间哈希
  static constexpr float P_INV_SP = 1.0f / (2.2f * PARTICLE_RADIUS);
  static constexpr int PNX = int(GS / (H * 2.2f * PARTICLE_RADIUS)) + 2;
  static constexpr int PNY = PNX;
  static constexpr int PNC = PNX * PNY;
  int m_numPartCell[PNC]{};
  int m_firstPart[PNC + 1]{};
  int m_cellPartIds[PC_MAX]{};

  // 变化列表
  int m_changedIdx[GC]{};
  int m_changedCnt{0};

  // 传感器
  QMI8658C* m_imu{nullptr};
  float m_ax{0.f}, m_ay{0.f};

  // ──────── 内部算法 ──────────────────────
  void seedParticles();
  void initGrid();
  void updateIMU();

  void integrateParticles(float dt);
  void pushParticlesApart(int iters);
  void transferVelocities(bool toGrid, float flipRatio);
  void solveIncompressibility(int iters, float dt);
  void updateFluidCells();

  // 工具
  inline int idx(int x, int y) const { return x * GS + y; }
  static inline float clampF(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
  }
};
