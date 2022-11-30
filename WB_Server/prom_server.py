import prometheus_client as prom
from prometheus_client import Gauge, Counter, Histogram, Info

# set up objects

# altitudes
gps_altitude_gauge         = Gauge("gps_altitude_gague", "GPS Altitude Gague")
baro_altitude_gague        = Gauge("baro_altitude_gauge", "Barometric Altitude Gauge")
#gps_altitude_histo         = Histogram("gps_altitude_histo", "GPS Altitude Histogram")
#baro_altitude_histo        = Histogram("baro_altitude_histo", "Barometric Altitude Histogram")
altitude_discrepancy_gauge = Gauge("altitude_discrepancy_gauge", "Altitude Discrepancy")

# guidance
horizontal_velocity_gauge = Gauge("horizontal_velocity_gauge", "Horizontal Velocity")
vertical_velocity_gauge   = Gauge("vertical_velocity_gauge", "Vertical Velocity")
#horizontal_velocity_histo = Histogram("horizontal_velocity_histo", "Horizontal Velocity")
#vertical_velocity_histo   = Histogram("vertical_velocity_histo", "Vertical Velocity")

longitude = Gauge("longitude", "Longitude")
latitude  = Gauge("latitude", "Latitude")

heading   = Gauge("course", "Course")

# weather
internal_temperature_gauge = Gauge("internal_temp", "Internal Temperature")
external_temperature_gauge = Gauge("external_temp", "External Temperature")
pressure_gauge             = Gauge("pressure", "Pressure")
humidity_gauge             = Gauge("humidity", "Humidity")

# power
main_voltage_gauge = Gauge("main_voltage", "Main Voltage")
main_current_gauge = Gauge("main_current", "Main Current")
main_power_gauge   = Gauge("main_power", "Main Power")

tx_voltage_gauge = Gauge("tx_voltage", "TX Voltage")
tx_current_gauge = Gauge("tx_current", "TX Current")
tx_power_gauge   = Gauge("tx_power", "TX Power")

# radio
rssi_local_gauge  = Gauge("rssi_local", "Local Signal Strength")
rssi_remote_gauge = Gauge("rssi_remote", "Remote Signal Strength")
satellite_count   = Gauge("satellite_count", "Satellites")



# will take formatted payload and observe
def observe_payload(d):
    # altitudes
    gps_altitude_gauge.set(d["gps_altitude"])
    #gps_altitude_histo.observe(d["gps_altitude"])

    baro_altitude_gague.set(d["barometric_altitude"])
    #baro_altitude_histo.observe(d["barometric_altitude"])

    altitude_discrepancy_gauge.set(abs(d["barometric_altitude"] - d["gps_altitude"]))

    # guidance
    h_velocity = d["speed"]
    v_velocity = d["v_speed"]

    horizontal_velocity_gauge.set(h_velocity)
    #horizontal_velocity_histo.observe(h_velocity)
    vertical_velocity_gauge.set(v_velocity)
    #vertical_velocity_histo.observe(v_velocity)

    latitude.set(d["latitude"])
    longitude.set(d["longitude"])

    heading.set(d["course"])

    # weather
    internal_temperature_gauge.set(d["internal_temperature"])
    external_temperature_gauge.set(d["external_temperature"])

    pressure_gauge.set(d["pressure"])
    humidity_gauge.set(d["humidity"])

    # power
    main_voltage_gauge.set(d["main_voltage"])
    main_current_gauge.set(d["main_current"])
    main_current_gauge.set(d["main_current"])

    tx_voltage_gauge.set(d["tx_voltage"])
    tx_current_gauge.set(d["tx_current"])
    tx_power_gauge.set(d["tx_power"])

    # radio
    satellite_count.set(d["satellite_count"])


def observe_rssi(local, remote):
    rssi_local_gauge.set(local)
    rssi_remote_gauge.set(remote)



def start_server(port):
    prom.start_http_server(port)




