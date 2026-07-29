# Local patches to core moos-ivp (~/moos-ivp)

These modify the **upstream** moos-ivp checkout (remote = moos-ivp/moos-ivp,
which we cannot push to). They are kept here, in a repo we DO own, so they
survive and can be re-applied after any moos-ivp update or fresh clone.

## pmarineviewer_ooo_guard.patch
**What:** pMarineViewer (ContactLedger.cpp) drops stale / out-of-order
NODE_REPORTs, keeping only the newest per vehicle.
**Why:** the greece field radio delivers NODE_REPORTs in ~0.5 s bursts that are
often newest-first (~2/3 of arrivals are older than the previous one). Without
this the drawn icon lurches backward every burst = the pMarineViewer "jitter".
Pairs with `extrap_policy = mode=cog, decay=2:5` in the mission shoreside .moos.

### Apply (after a moos-ivp update / on a new machine)
```
cd ~/moos-ivp
git apply ~/moos-ivp-xeqtor/patches/pmarineviewer_ooo_guard.patch
# rebuild just the viewer:
cd ~/moos-ivp/build/ivp/lib_geodaid && make
cd ~/moos-ivp/build/ivp/pMarineViewer && make
```
### Check whether it's already applied
```
grep -q "Reject stale / out-of-order" ~/moos-ivp/ivp/src/lib_geodaid/ContactLedger.cpp \
  && echo APPLIED || echo MISSING
```
