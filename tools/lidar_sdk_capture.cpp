// T-mini Plus diagnostic capture using the existing, unmodified YDLidar-SDK.
// No chassis commands. stdout includes SDK logs; consumers accept only SCAN_JSON.
#include "CYdLidar.h"
#include "core/common/ydlidar_help.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <unistd.h>
using namespace ydlidar;
using namespace ydlidar::core::common;

int main(int argc, char **argv) {
  if ((argc != 2 && argc != 3) || std::string(argv[1]).find("/dev/serial/by-id/") != 0) {
    fprintf(stderr, "Usage: lidar_sdk_capture /dev/serial/by-id/EXPLICIT_DEVICE [6..10 Hz]\n");
    return 2;
  }
  char *end = nullptr;
  float frequency = argc == 3 ? std::strtof(argv[2], &end) : 10.f;
  if (!std::isfinite(frequency) || frequency < 6 || frequency > 10 ||
      (argc == 3 && (end == argv[2] || *end != '\0'))) return 2;
  ydlidar::os_init();
  CYdLidar laser;
  std::string port(argv[1]);
  laser.setlidaropt(LidarPropSerialPort, port.c_str(), port.size());
  auto integer = [&](int key, int value) { laser.setlidaropt(key, &value, sizeof(value)); };
  auto boolean = [&](int key, bool value) { laser.setlidaropt(key, &value, sizeof(value)); };
  auto floating = [&](int key, float value) { laser.setlidaropt(key, &value, sizeof(value)); };
  integer(LidarPropSerialBaudrate, 230400);
  integer(LidarPropLidarType, TYPE_TRIANGLE);
  integer(LidarPropDeviceType, YDLIDAR_TYPE_SERIAL);
  integer(LidarPropSampleRate, 4);
  integer(LidarPropAbnormalCheckCount, 4);
  integer(LidarPropIntenstiyBit, 8);
  // Preserve raw angular samples rather than padded fixed-resolution bins.
  boolean(LidarPropFixedResolution, false);
  boolean(LidarPropReversion, false);
  boolean(LidarPropInverted, false);
  boolean(LidarPropAutoReconnect, false);
  boolean(LidarPropSingleChannel, false);
  boolean(LidarPropIntenstiy, true);
  boolean(LidarPropSupportMotorDtrCtrl, false);
  boolean(LidarPropSupportHeartBeat, false);
  floating(LidarPropMaxAngle, 180.f);
  floating(LidarPropMinAngle, -180.f);
  // Diagnostic clipping bounds, not a verified sensor accuracy specification.
  floating(LidarPropMaxRange, 12.f);
  floating(LidarPropMinRange, 0.05f);
  floating(LidarPropScanFrequency, frequency);
  laser.enableGlassNoise(false);
  laser.enableSunNoise(false);
  laser.setEnableDebug(false);
  if (!laser.initialize() || !laser.turnOn()) {
    fprintf(stderr, "LIDAR_START_FAILED: %s\n", laser.DescribeError());
    laser.turnOff();
    laser.disconnecting();
    return 1;
  }
  LaserScan scan;
  unsigned long seq = 0;
  int failures = 0;
  while (ydlidar::os_isOk()) {
    if (!laser.doProcessSimple(scan)) {
      fprintf(stderr, "SCAN_FAILED\n"); fflush(stderr);
      if (++failures >= 3) break;
      usleep(10000);
      continue;
    }
    failures = 0;
    printf("SCAN_JSON {\"seq\":%lu,\"frequency_hz\":%.3f,\"points\":[", ++seq,
           std::isfinite(scan.scanFreq) ? scan.scanFreq : 0.0);
    bool first = true;
    for (const auto &p : scan.points) {
      if (!std::isfinite(p.angle) || !std::isfinite(p.range) || p.range < .05f || p.range > 12.f) continue;
      printf("%s[%.6f,%.4f,%.0f]", first ? "" : ",", p.angle, p.range,
             std::isfinite(p.intensity) ? p.intensity : 0.0);
      first = false;
    }
    puts("]}"); fflush(stdout);
  }
  laser.turnOff();
  laser.disconnecting();
  return failures >= 3 ? 1 : 0;
}
