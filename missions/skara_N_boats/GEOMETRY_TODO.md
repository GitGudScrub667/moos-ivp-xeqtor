# skara_N_boats — geometry re-skin checklist

`skara_N_boats` is `pireas_N_boat` with **identical logic/behaviors/scripts**, moved
onto the **Skaramangas** chart. Everything below is still carrying **pireas-frame
coordinates** and must be re-placed on the skara chart before any real run.

## DONE (foundation, this scaffold)
- [x] Chart: `skaramangas.tif` + `.info` (from skara3_2), `tiff_file` updated.
- [x] Datum: `plugs.moos` → `LatOrigin 37.998009 / LongOrigin 23.581591` (= skara3_2 frame).
- [x] Mission naming (`pireas_N_boat` → `skara_N_boats`, area name in headers).
- [x] 3 buoy no-go behaviors REMOVED (per decision; add skara obstacles later if any).
- [x] pan/zoom set to skara3_2 defaults (0,0 / 0.85) as a starting point.

## PRIMARY ANCHORS — decide these on the skara chart (everything else derives)
1. **Ring** — center `(circle_x, circle_y)` + `circle_rad` (maybe rescale).
   - Set in TWO places, must match exactly:
     - `meta_shoreside.moos`  (pArrivalSync `circle_x/circle_y/circle_rad`)
     - `launch_vehicle.sh`    (`CIRCLE_X/CIRCLE_Y/CIRCLE_RAD` — the drawn ring)
   - Pireas: `-144,-43`, r `21.5`.
2. **Orientation** — the whole formation can be rotated to fit the skara water/backdrop
   (slot base angle, home-cluster bearing, MIO offset direction). N=1/2/3/4 spacing is
   auto (360/N) but the base angle / which way "east" points is a placement choice.
3. **MIO station** — `mio_x/mio_y` (`meta_shoreside.moos`). Pireas: `-90.43,-52.25`.
4. **Home cluster + headings** — 5 rows in `init_field.sh` (asha/bama/chip/flex/ewan).
   Cluster near the skara launch/recovery point; each heading aimed at the new ring.
5. **Op-region** — `core_poly` in `meta_vehicle.bhv` (BHV_OpRegionV24). DRAW a new one
   that contains ring + square + MIO + all run-in paths, with margin for save/halt dist.

## DERIVED — recompute from the primaries (don't hand-place)
- **DISPERSE square** (`meta_shoreside.moos square=`): axis-aligned, centred on ring
  center, half-side ≈ 1.15 × ring_radius (pireas used 28.52 for r=21.5).
- **show_vessel no-go** (`meta_vehicle.bhv`): 8 m octagon centred on the RING center.
- **mio_nogo no-go** (`meta_vehicle.bhv`): 4 m octagon centred on the MIO station.

## RETUNE AFTER PLACEMENT
- pMarineViewer `set_pan_x/set_pan_y/zoom` so the formation sits centred in view.
- Speeds/`mio_radius`/`square_radius` if the ring is rescaled.
- FIELD_OPS.txt quick-card coordinates + any lat/lon comments (currently stale/pireas).

## HELPER
Every spot to change is tagged `TODO(skara` in the source:
```
grep -rn "TODO(skara" .
```
Reference: skara3_2 is in this SAME datum frame — open it in the viewer to see where the
water is and sanity-check candidate coordinates.
