# skara_N_boats — geometry status

`skara_N_boats` = `pireas_N_boat` logic, re-skinned onto the Skaramangas chart.

## DONE — foundation + geometry placed (2026-07-29)
- [x] Chart `skaramangas.tif`/`.info`; datum `37.998009 / 23.581591` (skara3_2 frame).
- [x] Naming; 3 pireas buoys removed.
- [x] **Ring** center **(85,-50)**, radius **33** — `meta_shoreside.moos` + `launch_vehicle.sh` (in sync).
- [x] **DISPERSE square** — `122.95,-12.05 : 47.05,-12.05 : 47.05,-87.95 : 122.95,-87.95` (half-side 37.95 = 1.15·r).
- [x] **show_vessel** no-go — 9 m octagon at ring center (meta_vehicle.bhv).
- [x] **MIO station** — **(175,-75)**, guard loiter **7.7**, no-go **4.4 m** octagon.
- [x] **Op-region** — `-5,-15 : 70,25 : 190,12 : 230,-70 : 150,-165 : 10,-168`.
- [x] **Homes** — asha(47,-150)h21 bama(57,-150)h16 chip(67,-150)h10 flex(52,-158)h17 ewan(62,-158)h12, aimed at the ring.
- [x] **Speeds** ×1.15 (max 2.42, orbit 1.61, min 0.32, orbit_min 0.81, orbit_max 2.82, disperse/mio 1.61, ORBIT_SPD 1.61, MAX_SPD 3.22).
- [x] Orientation unchanged (`slot_base_deg = 0`).

## REMAINING
- [ ] **pMarineViewer `set_pan_x/y/zoom`** — still skara3_2 defaults (0,0 / 0.85, shows the whole cove). Fine-tune in the GUI to centre the formation.
- [ ] **Obstacles** — none placed (buoys were dropped). Add BHV_AvoidObstacleV24 blocks only if the show area has real fixed hazards.
- [ ] **Sim-validate** the full mission (deploy / encircle / DISPERSE / MIO / RETURN), then **water-validate** at Skaramangas.

`grep -rn "TODO(skara"` for the two remaining in-source notes (pan/zoom, optional obstacles).
