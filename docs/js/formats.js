const SYNC = 0xCAFE;
const RECORD_SIZE = 80;

function parseRecord(dv, offset) {
    let o = offset;
    const r = {};
    r.Sync = dv.getUint16(o, true); o += 2;
    r.Tick = dv.getUint32(o, true); o += 4;
    r.AccelX = dv.getFloat32(o, true); o += 4;
    r.AccelY = dv.getFloat32(o, true); o += 4;
    r.AccelZ = dv.getFloat32(o, true); o += 4;
    r.GyroX = dv.getFloat32(o, true); o += 4;
    r.GyroY = dv.getFloat32(o, true); o += 4;
    r.GyroZ = dv.getFloat32(o, true); o += 4;
    r.MagX = dv.getFloat32(o, true); o += 4;
    r.MagY = dv.getFloat32(o, true); o += 4;
    r.MagZ = dv.getFloat32(o, true); o += 4;
    r.PressurePa = dv.getFloat32(o, true); o += 4;
    r.TemperatureC = dv.getFloat32(o, true); o += 4;
    r.Latitude = dv.getInt32(o, true); o += 4;
    r.Longitude = dv.getInt32(o, true); o += 4;
    r.GPSAltitude = dv.getFloat32(o, true); o += 4;
    r.UnixTime = dv.getUint32(o, true); o += 4;
    r.Milliseconds = dv.getUint16(o, true); o += 2;
    r.Satellites = dv.getUint8(o); o += 1;
    r.Flags = dv.getUint32(o, true); o += 4;
    r.BatteryVoltage = dv.getFloat32(o, true); o += 4;
    r.State = dv.getUint8(o); o += 1;
    r.RelayState = dv.getUint8(o); o += 1;
    r.SyncEnd = dv.getUint8(o); o += 1;
    return { record: r, size: o - offset };
}
