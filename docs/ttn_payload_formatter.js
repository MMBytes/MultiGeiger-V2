// TTN payload formatter for MultiGeiger V2 LoRaWAN uplinks (V2.6.23+).
// Byte layouts are FROZEN and byte-identical to Multigeiger V1.9 /
// ttn2luft (ports 1 and 2), so existing community decoders keep working.
function decodeUplink(input) {
  var b = input.bytes;
  if (input.fPort === 1 && b.length === 10) {
    var counts = (b[0] << 24 >>> 0) + (b[1] << 16) + (b[2] << 8) + b[3];
    var dt_ms  = (b[4] << 16) + (b[5] << 8) + b[6];
    var ver    = (b[7] << 8) + b[8];
    return { data: {
      gm_counts: counts,
      interval_ms: dt_ms,
      cpm: dt_ms > 0 ? Math.round(counts * 60000 / dt_ms) : 0,
      fw_major: (ver >> 12) & 0xF, fw_minor: (ver >> 4) & 0xFF,
      fw_patch_clamped: ver & 0xF,           // 4-bit field, clamped at 15
      tube: ["unknown", "SBM-20", "SBM-19", "Si22G"][b[9]] || ("#" + b[9])
    }};
  }
  if (input.fPort === 2 && b.length === 5) {
    var t = (b[0] << 8) + b[1]; if (t & 0x8000) t -= 0x10000; // s16
    return { data: {
      temperature_c: t / 10.0,
      humidity_pct: b[2] / 2.0,
      pressure_hpa: ((b[3] << 8) + b[4]) / 10.0
    }};
  }
  return { errors: ["unknown port/length: " + input.fPort + "/" + b.length] };
}
