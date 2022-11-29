import prometheus_client as prom
from prometheus_client import Gauge, Counter, Histogram

# set up objects

# altitudes
gps_altitude_gauge         = Gauge("gps_altitude_gague", "GPS Altitude Gague")
baro_altitude_gague        = Gauge("baro_altitude_gauge", "Barometric Altitude Gauge")
gps_altitude_histo         = Histogram("gps_altitude_histo", "GPS Altitude Histogram")
baro_altitude_histo        = Histogram("baro_altitude_histo", "Barometric Altitude Histogram")
altitude_discrepancy_histo = Histogram("altitude_discrepancy_histo", "Altitude Discrepancy")

# guidance
horizontal_velocity_gauge = Gauge("horizontal_velocity_gauge", "Horizontal Velocity")
vertical_velocity_gauge   = Gauge("vertical_velocity_gauge", "Vertical Velocity")
horizontal_velocity_histo = Histogram("horizontal_velocity_histo", "Horizontal Velocity")
vertical_velocity_histo   = Histogram("vertical_velocity_histo", "Vertical Velocity")

longitude = Histogram("longitude", "Longitude")
latitude  = Histogram("latitude", "Latitude")

heading   = Gauge("course", "Course")

# weather
internal_temperature_histo = Histogram("internal_temp", "Internal Temperature")
external_temperature_histo = Histogram("external_temp", "External Temperature")
pressure_histo             = Histogram("pressure", "Pressure")
humidity_histo             = Histogram("humidity", "Humidity")

# power
main_voltage_histo = Histogram("main_voltage", "Main Voltage")
main_current_histo = Histogram("main_current", "Main Current")
main_power_histo   = Histogram("main_power", "Main Power")

tx_voltage_histo = Histogram("tx_voltage", "TX Voltage")
tx_current_histo = Histogram("tx_current", "TX Current")
tx_power_histo   = Histogram("tx_power", "TX Power")

# radio
rssi_local_histo  = Histogram("rssi_local", "Local Signal Strength")
rssi_remote_histo = Histogram("rssi_remote", "Remote Signal Strength")
satellite_count   = Gauge("satellite_count", "Satellites")



# will take formatted payload and observe
def observe_payload(d):
    # altitudes
    gps_altitude_gauge.set(d["gps_altitude"])
    gps_altitude_histo.observe(d["gps_altitude"])

    baro_altitude_gague.set(d["barometric_altitude"])
    baro_altitude_histo.observe(d["barometric_altitude"])

    altitude_discrepancy_histo.observe(abs(d["barometric_altitude"] - d["barometric_altitude"]))

    # guidance
    h_velocity = d["speed"]
    v_velocity = 0

    horizontal_velocity_gauge.set(h_velocity)
    horizontal_velocity_histo.observe(h_velocity)
    vertical_velocity_gauge.set(v_velocity)
    vertical_velocity_histo.observe(v_velocity)

    latitude.observe(d["latitude"])
    longitude.observe(d["longitude"])

    heading.set(d["course"])

    # weather
    internal_temperature_histo.observe(d["internal_temperature"])
    external_temperature_histo.observe(d["external_temperature"])

    pressure_histo.observe(d["pressure"])
    humidity_histo.observe(d["humidity"])

    # power
    main_voltage_histo.observe(d["main_voltage"])
    main_current_histo.observe(d["main_current"])
    main_current_histo.observe(d["main_current"])

    tx_voltage_histo.observe(d["tx_voltage"])
    tx_current_histo.observe(d["tx_current"])
    tx_power_histo.observe(d["tx_power"])

    # radio
    rssi_local_histo.observe(0)
    rssi_remote_histo.observe(0)

    satellite_count.set(d["satellite_count"])


def start_server(port):
    prom.start_http_server(port)




