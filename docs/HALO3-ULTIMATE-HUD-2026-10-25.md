# Ultimate Firefight HUD layout candidate

User log source 673c120a: Halo3 scan repeatedly sees a 1280x720 anchor with
safe-frame pair 0/0, rejects it, then reports zero accepted blocks. Size,
aspect and curvature share this locator; hide and height use other paths.

Read-only HTTP ranges from supplied firefight_shrine.map: v13 header,
metadata file offset 2087653376, length123863040, virtual base7660765184.
chgd tag index20810 has three skins. Metadata offsets (relative to section):
- 0x6de21b8: canvas1280/720, sensor55/661/58/4, safe frames0/0.
- 0x6de221c: canvas1600/580, sensor55/520/58/4, safe frames.87/.87.
- 0x6de2280: canvas960/580, sensor55/520/58/4, safe frames.87/.87.
All three have destination Z -.95 at anchor-4. Other canvas records in this
block have zero safe frames and remain rejected. These records are 0x64 apart.

Candidate adds two exact 24-byte anchors using existing alternate-resolution
support, accepts 2..3 validated blocks, retains private-RW scan, finite/payload
checks, generation ownership and per-write identity checks. Stock prefix and
field offsets unchanged. No broad wildcard matching. This supports these
layouts in other mods, not arbitrary future CHUD definitions. Runtime selection
and visible response require headset confirmation; map evidence alone does
not prove the candidate fixes rendering. Universal tag-owned discovery remains
future work requiring stronger native ownership evidence.
