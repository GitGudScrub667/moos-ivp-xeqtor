# skara_N_boats — geometry status

`skara_N_boats` = `pireas_N_boat` logic, re-skinned onto the Skaramangas chart.

## DONE — foundation + geometry placed (2026-07-29)
- [x] Chart `skaramangas.tif`/`.info`; datum `37.998009 / 23.581591` (skara3_2 frame).
- [x] Naming; 3 pireas buoys removed.
- [x] **Ring** center **(55,-50)**, radius **33** — `meta_shoreside.moos` + `launch_vehicle.sh` (in sync). *(moved from 85 on 2026-08-20)*
- [x] **DISPERSE square** — `55,3.67 : 1.33,-50 : 55,-103.67 : 108.67,-50` (half-side 37.95 = 1.15·r, **rotated 45°** so corners sit N/W/S/E at circumradius 53.67). *(rotated 2026-08-20)*
- [x] **show_vessel** no-go — **11 m** octagon at ring center (meta_vehicle.bhv). *(grown from 9 m on 2026-08-20; envelope dists 5/7/9/9 unchanged — they are edge-relative)*
- [x] **MIO station** — **(25,-100)**, guard loiter **7.7**, no-go **4.4 m** octagon. *(moved from 175,-75 on 2026-08-20)*
- [x] **Op-region** — `-5,-15 : 70,25 : 190,12 : 230,-70 : 150,-155 : 10,-168 : -25,-50` (west vertex added 2026-08-20 for the rotated square; **must stay convex**).
- [x] **Homes** — asha(47,-150)h5 bama(57,-150)h359 chip(67,-150)h353 flex(52,-158)h2 ewan(62,-158)h356, aimed at the ring. *(re-aimed at 55,-50 on 2026-08-20)*
- [x] **Speeds** ×1.15 (max 2.42, orbit 1.61, min 0.32, orbit_min 0.81, orbit_max 2.82, disperse/mio 1.61, ORBIT_SPD 1.61, MAX_SPD 3.22).
- [x] Orientation unchanged (`slot_base_deg = 0`).

## REMAINING
- [ ] **pMarineViewer `set_pan_x/y/zoom`** — still skara3_2 defaults (0,0 / 0.85, shows the whole cove). Fine-tune in the GUI to centre the formation.
- [ ] **Obstacles** — none placed (buoys were dropped). Add BHV_AvoidObstacleV24 blocks only if the show area has real fixed hazards.
- [ ] **Sim-validate** the full mission (deploy / encircle / DISPERSE / MIO / RETURN), then **water-validate** at Skaramangas.

`grep -rn "TODO(skara"` for the two remaining in-source notes (pan/zoom, optional obstacles).
