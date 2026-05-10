#pragma once

#include <Arduino.h>
#include <Preferences.h>

namespace CalibrationStorage {

static constexpr uint32_t kMagic = 0x58445243;  // "XDRC"
static constexpr uint32_t kVersion = 1;
static constexpr const char* kNamespace = "xiaodrone";
static constexpr const char* kKey = "calib";

struct Data {
  uint32_t magic;
  uint32_t version;
  float gyro_x_offset;
  float gyro_y_offset;
  float gyro_z_offset;
  float pitch_offset;
  float roll_offset;
  float referencePressure;
};

inline Data makeDefault() {
  Data data{};
  data.magic = kMagic;
  data.version = kVersion;
  return data;
}

inline bool save(const Data& data) {
  Preferences prefs;
  if (!prefs.begin(kNamespace, false)) {
    return false;
  }

  const size_t written = prefs.putBytes(kKey, &data, sizeof(data));
  prefs.end();
  return written == sizeof(data);
}

inline bool load(Data& data) {
  Preferences prefs;
  if (!prefs.begin(kNamespace, true)) {
    return false;
  }

  const size_t storedSize = prefs.getBytesLength(kKey);
  if (storedSize != sizeof(data)) {
    prefs.end();
    return false;
  }

  const size_t readSize = prefs.getBytes(kKey, &data, sizeof(data));
  prefs.end();

  return readSize == sizeof(data) && data.magic == kMagic && data.version == kVersion;
}

}  // namespace CalibrationStorage