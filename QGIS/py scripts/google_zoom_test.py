import math

lat = 40.743181607
lon = 30.331304792

def latlon_to_tile(lat, lon, z):
    lat_rad = math.radians(lat)
    n = 2 ** z
    x = int((lon + 180.0) / 360.0 * n)
    y = int((1.0 - math.log(math.tan(lat_rad) + 1 / math.cos(lat_rad)) / math.pi) / 2.0 * n)
    return x, y

for z in range(18, 23):
    x, y = latlon_to_tile(lat, lon, z)
    url = f"https://mt1.google.com/vt/lyrs=s&x={x}&y={y}&z={z}"
    print(f"z{z}: {url}")