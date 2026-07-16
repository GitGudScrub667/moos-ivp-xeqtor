#!/usr/bin/env python3
# Fetch Esri World Imagery tiles covering the Skaramangas cove, stitch into one
# raster, and emit a pMarineViewer .tif + .info (exact georef from tile bounds).
import math, io, os, sys, time, urllib.request

# --- area of interest (padded around the 3 control points) ---
LAT_S, LAT_N = 37.99620, 37.99920
LON_W, LON_E = 23.58040, 23.58650
ZOOM = 18

# --- datum (lighthouse = local 0,0) ---
DATUM_LAT, DATUM_LON = 37.998009, 23.581591

OUTDIR = "/mnt/c/Users/georg/AppData/Local/Temp/claude/C--Users-georg-Desktop-SESH/2b2e5fb8-5ac2-4911-b8a0-4538f7461d24/scratchpad"
BASENAME = "skaramangas"
TILE_URL = "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}"

def lon2tx(lon, z): return (lon + 180.0) / 360.0 * (2**z)
def lat2ty(lat, z):
    r = math.radians(lat)
    return (1.0 - math.log(math.tan(r) + 1.0/math.cos(r))/math.pi) / 2.0 * (2**z)
def tx2lon(x, z): return x / (2**z) * 360.0 - 180.0
def ty2lat(y, z):
    return math.degrees(math.atan(math.sinh(math.pi * (1.0 - 2.0*y/(2**z)))))

from PIL import Image

xt0 = int(math.floor(lon2tx(LON_W, ZOOM)))
xt1 = int(math.ceil (lon2tx(LON_E, ZOOM)))
yt0 = int(math.floor(lat2ty(LAT_N, ZOOM)))   # north = smaller y
yt1 = int(math.ceil (lat2ty(LAT_S, ZOOM)))
nx, ny = xt1 - xt0, yt1 - yt0
print(f"zoom={ZOOM} tiles x[{xt0},{xt1}) y[{yt0},{yt1}) -> {nx}x{ny} = {nx*ny} tiles")

mosaic = Image.new("RGB", (nx*256, ny*256))
hdr = {"User-Agent": "Mozilla/5.0 (moos-ivp chart builder)"}
ok = 0
for xt in range(xt0, xt1):
    for yt in range(yt0, yt1):
        url = TILE_URL.format(z=ZOOM, x=xt, y=yt)
        for attempt in range(3):
            try:
                req = urllib.request.Request(url, headers=hdr)
                data = urllib.request.urlopen(req, timeout=20).read()
                tile = Image.open(io.BytesIO(data)).convert("RGB")
                mosaic.paste(tile, ((xt-xt0)*256, (yt-yt0)*256))
                ok += 1
                break
            except Exception as e:
                if attempt == 2:
                    print(f"  FAILED tile {xt},{yt}: {e}")
                time.sleep(0.5)
        time.sleep(0.02)
print(f"downloaded {ok}/{nx*ny} tiles, mosaic {mosaic.size}")

# exact geographic bounds of the stitched mosaic (tile edges)
m_lon_w = tx2lon(xt0, ZOOM); m_lon_e = tx2lon(xt1, ZOOM)
m_lat_n = ty2lat(yt0, ZOOM); m_lat_s = ty2lat(yt1, ZOOM)

os.makedirs(OUTDIR, exist_ok=True)
tif = os.path.join(OUTDIR, BASENAME + ".tif")
png = os.path.join(OUTDIR, BASENAME + ".png")
info = os.path.join(OUTDIR, BASENAME + ".info")
mosaic.save(tif); mosaic.save(png)
with open(info, "w") as f:
    f.write(f"// pMarineViewer chart placement for {BASENAME}.tif (Esri World Imagery, zoom {ZOOM})\n\n")
    f.write(f"lat_north = {m_lat_n:.9f}\n")
    f.write(f"lat_south = {m_lat_s:.9f}\n")
    f.write(f"lon_east  = {m_lon_e:.9f}\n")
    f.write(f"lon_west  = {m_lon_w:.9f}\n\n")
    f.write(f"datum_lat = {DATUM_LAT}\n")
    f.write(f"datum_lon = {DATUM_LON}\n")

# span in meters for a sanity readout
mlat = 110996.0; mlon = 87831.0
print("---- RESULT ----")
print(f"tif: {tif}")
print(f"bounds N/S/E/W = {m_lat_n:.6f} / {m_lat_s:.6f} / {m_lon_e:.6f} / {m_lon_w:.6f}")
print(f"image span ~ {(m_lon_e-m_lon_w)*mlon:.0f} m E-W  x  {(m_lat_n-m_lat_s)*mlat:.0f} m N-S")
print(open(info).read())
