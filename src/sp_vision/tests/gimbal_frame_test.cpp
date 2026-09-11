// 帧字节布局测试（ITL 协议，对照仓库根目录《摆臂部署与协议迁移计划.md》第三节规格表）
// 不依赖串口/相机/OpenVINO：纯编译期 static_assert + 逐字节校验。
#include <cstdint>
#include <cstring>
#include <iostream>
#include <type_traits>

#include "io/gimbal/gimbal.hpp"

static_assert(sizeof(io::VisionToGimbal) == 28, "VisionToGimbal must be 28 bytes");
static_assert(sizeof(io::GimbalToVision) == 42, "GimbalToVision must be 42 bytes");

namespace
{
int failures = 0;

void check(bool ok, const char * what)
{
  if (!ok) {
    std::cerr << "[FAIL] " << what << std::endl;
    failures++;
  } else {
    std::cout << "[PASS] " << what << std::endl;
  }
}

template <typename T>
void check_field(const T & got, const T & expect, const char * what)
{
  check(got == expect, what);  // 直接赋值后读回，位级相等
}
}  // namespace

int main()
{
  // ---- VisionToGimbal 28B（类内默认成员初始化给出 head/tail）----
  io::VisionToGimbal tx;
  tx.mode = 2;
  tx.yaw = 1.5f;
  tx.yaw_vel = 0.1f;
  tx.yaw_acc = 0.2f;
  tx.pitch = -0.3f;
  tx.pitch_vel = 0.4f;
  tx.pitch_acc = 0.5f;

  const auto * p = reinterpret_cast<const uint8_t *>(&tx);
  check(p[0] == 'V' && p[1] == 'G', "VisionToGimbal head = {'V','G'}");
  check(p[2] == 2, "VisionToGimbal mode @ offset 2");
  check(p[27] == 'V', "VisionToGimbal tail 'V' @ offset 27");

  float f;
  std::memcpy(&f, p + 3, 4);
  check_field(f, 1.5f, "VisionToGimbal yaw float32 LE @ 3");
  std::memcpy(&f, p + 7, 4);
  check_field(f, 0.1f, "VisionToGimbal yaw_vel @ 7");
  std::memcpy(&f, p + 11, 4);
  check_field(f, 0.2f, "VisionToGimbal yaw_acc @ 11");
  std::memcpy(&f, p + 15, 4);
  check_field(f, -0.3f, "VisionToGimbal pitch @ 15");
  std::memcpy(&f, p + 19, 4);
  check_field(f, 0.4f, "VisionToGimbal pitch_vel @ 19");
  std::memcpy(&f, p + 23, 4);
  check_field(f, 0.5f, "VisionToGimbal pitch_acc @ 23");

  // ---- GimbalToVision 42B ----
  io::GimbalToVision rx;
  rx.mode = 1;
  rx.q[0] = 0.98f;
  rx.q[1] = 0.1f;
  rx.q[2] = -0.02f;
  rx.q[3] = 0.15f;
  rx.yaw = 0.5f;
  rx.yaw_vel = -1.2f;
  rx.pitch = 0.3f;
  rx.pitch_vel = 0.7f;
  rx.bullet_speed = 18.5f;
  rx.bullet_count = 1234;

  const auto * r = reinterpret_cast<const uint8_t *>(&rx);
  check(r[0] == 'G' && r[1] == 'V', "GimbalToVision head = {'G','V'}");
  check(r[2] == 1, "GimbalToVision mode @ offset 2");
  std::memcpy(&f, r + 3, 4);
  check_field(f, 0.98f, "GimbalToVision q[0]=w @ 3");
  std::memcpy(&f, r + 7, 4);
  check_field(f, 0.1f, "GimbalToVision q[1]=x @ 7");
  std::memcpy(&f, r + 11, 4);
  check_field(f, -0.02f, "GimbalToVision q[2]=y @ 11");
  std::memcpy(&f, r + 15, 4);
  check_field(f, 0.15f, "GimbalToVision q[3]=z @ 15");
  std::memcpy(&f, r + 19, 4);
  check_field(f, 0.5f, "GimbalToVision yaw @ 19");
  std::memcpy(&f, r + 23, 4);
  check_field(f, -1.2f, "GimbalToVision yaw_vel @ 23");
  std::memcpy(&f, r + 27, 4);
  check_field(f, 0.3f, "GimbalToVision pitch @ 27");
  std::memcpy(&f, r + 31, 4);
  check_field(f, 0.7f, "GimbalToVision pitch_vel @ 31");
  std::memcpy(&f, r + 35, 4);
  check_field(f, 18.5f, "GimbalToVision bullet_speed @ 35");
  uint16_t bc;
  std::memcpy(&bc, r + 39, 2);
  check_field(bc, uint16_t(1234), "GimbalToVision bullet_count u16 LE @ 39");
  check(r[41] == 'G', "GimbalToVision tail 'G' @ offset 41");

  if (failures == 0) {
    std::cout << "ALL PASS" << std::endl;
    return 0;
  }
  std::cerr << failures << " FAILURE(S)" << std::endl;
  return 1;
}
