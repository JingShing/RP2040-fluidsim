#pragma once
#include <LovyanGFX.h>  // LGFX_GC9A01 依赖
#include "ParticleSimulation.hpp"

// ────────────────────────────────────────────────
class FluidRenderer {
 public:
  enum Mode { BALLS, GRID };

  FluidRenderer(lgfx::LGFX_Device* disp, const ParticleSimulation* sim)
      : m_disp(disp), m_sim(sim) {}

  /** 渲染一次，按指定模式 */
  void render(Mode mode);

  /** 可选：单独调用 */
  void renderBalls();
  void renderGrid();

  /** 颜色参数，可自行修改 */
  void setBallBaseColor(uint16_t c) { m_ballBase = c; }
  void setGridSolidColor(uint16_t c) { m_gridSolid = c; }
  void setGridFluidColor(uint16_t c) { m_gridFluid = c; }

 private:
  lgfx::LGFX_Device* m_disp;
  const ParticleSimulation* m_sim;

  // 调色板
  uint16_t m_ballBase = TFT_CYAN;
  uint16_t m_gridSolid = TFT_DARKGREY;
  uint16_t m_gridFluid = TFT_BLUE;

  // 工具
  inline uint16_t lerp565(uint16_t c1, uint16_t c2, float t) const;
};
