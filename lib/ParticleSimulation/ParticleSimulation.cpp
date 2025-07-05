/**************************************************************************
 *  ParticleSimulation.cpp  ――  2-D FLIP Demo (ESP32 + LovyanGFX)        *
 *                                                                       *
 *  - 圆形 SOLID_CELL 容器                                               *
 *  - 粒子 ↔ 网格 (简化 FLIP / PIC)                                       *
 *  - m_changedIdx / m_changedCnt 记录本帧状态变更单元                    *
 *  - classifyCell() 依据粒子计数 & 平均速度判定 EMPTY / LIQUID / FOAM    *
 **************************************************************************/

#include "ParticleSimulation.hpp"
#include <math.h>
#include <string.h>

// ────────────────────────────────────────────────────
//  工具：统计 → 流体类型
// ────────────────────────────────────────────────────
static inline FluidType classifyCell(int count, float avgSpeed) {
  if (count < FLUID_PARTICLE_THRESHOLD)
    return FLUID_EMPTY;
  if (avgSpeed > FOAM_SPEED_THRESHOLD)
    return FLUID_FOAM;
  return FLUID_LIQUID;
}

// ────────────────────────────────────────────────────
//  初始化
// ────────────────────────────────────────────────────
void ParticleSimulation::begin(QMI8658C* imu) {
  m_imu = imu;
  m_numParticles = PC_MAX;
  seedParticles();
  initGrid();
}

void ParticleSimulation::seedParticles() {
  for (int i = 0; i < m_numParticles; ++i) {
    m_particles[i].x = random(2, GS - 2);
    m_particles[i].y = random(2, GS - 2);
    m_particles[i].vx = random(-50, 50) / 100.0f;
    m_particles[i].vy = random(-50, 50) / 100.0f;
    m_particles[i].r = 0.2f;
    m_particles[i].g = 0.4f;
    m_particles[i].b = 1.0f;
  }
}

void ParticleSimulation::initGrid() {
  for (int i = 0; i < GC; ++i) {
    int gx = i / GS, gy = i % GS;
    float cx = gx - GS * 0.5f + 0.5f;
    float cy = gy - GS * 0.5f + 0.5f;
    float rad = GS * 0.5f - 1.0f;  // 圆半径（减 1 做 margin）
    m_cellType[i] = (cx * cx + cy * cy <= rad * rad) ? FLUID_CELL : SOLID_CELL;
    m_s[i] = 1.0f;
    m_u[i] = m_v[i] = m_prevU[i] = m_prevV[i] = 0.f;
    m_du[i] = m_dv[i] = m_pressure[i] = 0.f;
  }
}

// ────────────────────────────────────────────────────
//  对外 simulate(dt)
// ────────────────────────────────────────────────────
void ParticleSimulation::simulate(float dt) {
  updateIMU();
  integrateParticles(dt);
  pushParticlesApart(SEPARATE_ITERS_P);

  transferVelocities(true, 0.0f);  // 粒子 → 网格 (纯 PIC)
  solveIncompressibility(SOLVER_ITERS_P, dt);
  transferVelocities(false, FLIP_RATIO);  // 网格 → 粒子 (FLIP/PIC 混合)

  updateFluidCells();  // 统计流体状态 & 变化表
}

// ────────────────────────────────────────────────────
//  IMU
// ────────────────────────────────────────────────────
void ParticleSimulation::updateIMU() {
  if (!m_imu)
    return;
  float ax, ay, az;
  if (m_imu->readAccelerometer(&ax, &ay, &az)) {
    m_ax = ay * 10.f;  // 根据实际感度缩放
    m_ay = -ax * 10.f;
  }
}

// ────────────────────────────────────────────────────
//  粒子积分 & 圆形碰撞
// ────────────────────────────────────────────────────
void ParticleSimulation::integrateParticles(float dt) {
  constexpr float CX = GS * 0.5f - 0.5f;  // 圆心
  constexpr float CY = GS * 0.5f - 0.5f;
  constexpr float R = GS * 0.5f - 1.0f - PARTICLE_RADIUS;  // 有效半径

  for (int i = 0; i < m_numParticles; ++i) {
    Particle& p = m_particles[i];

    // ── 外力积分 ──
    p.vx += m_ax * dt;
    p.vy += m_ay * dt;
    p.x += p.vx * dt;
    p.y += p.vy * dt;

    // ── 边界盒 (确保坐标合法) ──
    if (p.x < 0) {
      p.x = 0;
      p.vx = 0;
    }
    if (p.x > GS - 1) {
      p.x = GS - 1;
      p.vx = 0;
    }
    if (p.y < 0) {
      p.y = 0;
      p.vy = 0;
    }
    if (p.y > GS - 1) {
      p.y = GS - 1;
      p.vy = 0;
    }

    // ── 圆形 SOLID_CELL 碰撞 ──
    float dx = p.x - CX;
    float dy = p.y - CY;
    float d2 = dx * dx + dy * dy;
    if (d2 > R * R) {
      float d = sqrtf(d2);
      float inv = 1.0f / d;
      float s = R * inv;  // 投回系数
      p.x = CX + dx * s;
      p.y = CY + dy * s;

      // 速度反射 (完全弹性)
      float vn = (p.vx * dx + p.vy * dy) * inv;
      p.vx -= 2.0f * vn * dx * inv;
      p.vy -= 2.0f * vn * dy * inv;
    }
  }
}

// ────────────────────────────────────────────────────
//  Push-Apart（空间哈希）
// ────────────────────────────────────────────────────
void ParticleSimulation::pushParticlesApart(int iters) {
  const float minDist2 = (2 * PARTICLE_RADIUS) * (2 * PARTICLE_RADIUS);

  memset(m_numPartCell, 0, sizeof(m_numPartCell));
  for (int i = 0; i < m_numParticles; ++i) {
    int xi = (int)clampF(m_particles[i].x * P_INV_SP, 0, PNX - 1);
    int yi = (int)clampF(m_particles[i].y * P_INV_SP, 0, PNY - 1);
    ++m_numPartCell[xi * PNY + yi];
  }
  int prefix = 0;
  for (int i = 0; i < PNC; ++i) {
    int n = m_numPartCell[i];
    m_firstPart[i] = prefix;
    prefix += n;
  }
  m_firstPart[PNC] = prefix;

  for (int i = 0; i < m_numParticles; ++i) {
    int xi = (int)clampF(m_particles[i].x * P_INV_SP, 0, PNX - 1);
    int yi = (int)clampF(m_particles[i].y * P_INV_SP, 0, PNY - 1);
    int cell = xi * PNY + yi;
    m_cellPartIds[--m_firstPart[cell]] = i;
  }

  for (int it = 0; it < iters; ++it) {
    for (int i = 0; i < m_numParticles; ++i) {
      Particle& a = m_particles[i];
      int cx = (int)(a.x * P_INV_SP);
      int cy = (int)(a.y * P_INV_SP);

      for (int xi = max(cx - 1, 0); xi <= min(cx + 1, PNX - 1); ++xi)
        for (int yi = max(cy - 1, 0); yi <= min(cy + 1, PNY - 1); ++yi) {
          int cell = xi * PNY + yi;
          for (int k = m_firstPart[cell]; k < m_firstPart[cell + 1]; ++k) {
            int j = m_cellPartIds[k];
            if (j <= i)
              continue;
            Particle& b = m_particles[j];
            float dx = b.x - a.x, dy = b.y - a.y;
            float d2 = dx * dx + dy * dy;
            if (d2 > minDist2 || d2 == 0.f)
              continue;
            float d = sqrtf(d2);
            float s = 0.5f * (2 * PARTICLE_RADIUS - d) / d;
            dx *= s;
            dy *= s;
            a.x -= dx;
            a.y -= dy;
            b.x += dx;
            b.y += dy;
          }
        }
    }
  }
}

// ────────────────────────────────────────────────────
//  粒子 ↔ 网格 速度搬运 (简化版)
// ────────────────────────────────────────────────────
void ParticleSimulation::transferVelocities(bool toGrid, float flipRatio) {
  const float hInv = 1.0f / H;

  if (toGrid) {
    memcpy(m_prevU, m_u, sizeof(m_u));
    memcpy(m_prevV, m_v, sizeof(m_v));
    memset(m_u, 0, sizeof(m_u));
    memset(m_v, 0, sizeof(m_v));
    memset(m_du, 0, sizeof(m_du));
    memset(m_dv, 0, sizeof(m_dv));
  }

  for (int comp = 0; comp < 2; ++comp) {
    float dx = comp == 0 ? 0.f : 0.5f;
    float dy = comp == 0 ? 0.5f : 0.f;
    float* f = comp == 0 ? m_u : m_v;
    float* f_prev = comp == 0 ? m_prevU : m_prevV;
    float* dwei = comp == 0 ? m_du : m_dv;

    for (int p = 0; p < m_numParticles; ++p) {
      Particle& pr = m_particles[p];
      float x = clampF(pr.x, 1, GS - 2);
      float y = clampF(pr.y, 1, GS - 2);

      float fx = (x - dx) * hInv;
      float fy = (y - dy) * hInv;
      int x0 = (int)fx, y0 = (int)fy;
      float tx = fx - x0, ty = fy - y0;
      int x1 = x0 + 1, y1 = y0 + 1;
      float sx = 1.f - tx, sy = 1.f - ty;

      float w0 = sx * sy, w1 = tx * sy, w2 = tx * ty, w3 = sx * ty;
      int n0 = idx(x0, y0), n1 = idx(x1, y0), n2 = idx(x1, y1),
          n3 = idx(x0, y1);

      if (toGrid) {
        float pv = (comp == 0) ? pr.vx : pr.vy;
        f[n0] += pv * w0;
        dwei[n0] += w0;
        f[n1] += pv * w1;
        dwei[n1] += w1;
        f[n2] += pv * w2;
        dwei[n2] += w2;
        f[n3] += pv * w3;
        dwei[n3] += w3;
      } else {
        float pic = w0 * f[n0] + w1 * f[n1] + w2 * f[n2] + w3 * f[n3];
        float corr = w0 * (f[n0] - f_prev[n0]) + w1 * (f[n1] - f_prev[n1]) +
                     w2 * (f[n2] - f_prev[n2]) + w3 * (f[n3] - f_prev[n3]);
        float flip = (comp == 0 ? pr.vx : pr.vy) + corr;
        float blend = (1.f - flipRatio) * pic + flipRatio * flip;
        if (comp == 0)
          pr.vx = blend;
        else
          pr.vy = blend;
      }
    }

    if (toGrid) {
      for (int i = 0; i < GC; ++i)
        if (dwei[i] > 0.f)
          f[i] /= dwei[i];
    }
  }
}

// ────────────────────────────────────────────────────
//  压力求解 (简化 Jacobi / Gauss-Seidel)
// ────────────────────────────────────────────────────
void ParticleSimulation::solveIncompressibility(int iters, float dt) {
  const float cp = FLUID_DENSITY * H / dt;

  for (int it = 0; it < iters; ++it) {
    for (int gx = 1; gx < GS - 1; ++gx)
      for (int gy = 1; gy < GS - 1; ++gy) {
        int c = idx(gx, gy);
        if (m_cellType[c] != FLUID_CELL)
          continue;

        int l = idx(gx - 1, gy), r = idx(gx + 1, gy), b = idx(gx, gy - 1),
            t = idx(gx, gy + 1);

        float div = m_u[r] - m_u[c] + m_v[t] - m_v[c];
        float p = -div / 4.f;  // 邻居权重均为 1
        p *= 1.9f;             // 过松弛
        m_pressure[c] += cp * p;

        m_u[c] -= p;
        m_u[r] += p;
        m_v[c] -= p;
        m_v[t] += p;
      }
  }
}

// ────────────────────────────────────────────────────
//  更新流体状态 & 变化表
// ────────────────────────────────────────────────────
void ParticleSimulation::updateFluidCells() {
  memcpy(m_prevFluid, m_currFluid, sizeof(m_currFluid));

  static uint16_t cnt[GC];
  static float acc[GC];
  memset(cnt, 0, sizeof(cnt));
  memset(acc, 0, sizeof(acc));

  for (int i = 0; i < m_numParticles; ++i) {
    int gx = (int)clampF(floorf(m_particles[i].x), 0, GS - 1);
    int gy = (int)clampF(floorf(m_particles[i].y), 0, GS - 1);
    int id = idx(gx, gy);
    float speed = sqrtf(m_particles[i].vx * m_particles[i].vx +
                        m_particles[i].vy * m_particles[i].vy);
    ++cnt[id];
    acc[id] += speed;
  }

  m_changedCnt = 0;
  for (int i = 0; i < GC; ++i) {
    float avg = cnt[i] ? acc[i] / cnt[i] : 0.f;
    m_currFluid[i] = classifyCell(cnt[i], avg);
    if (m_currFluid[i] != m_prevFluid[i])
      m_changedIdx[m_changedCnt++] = i;
  }
}
