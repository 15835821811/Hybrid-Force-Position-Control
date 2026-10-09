# N209 diagnostic visualization quality review

Paper architect/reviewer, paper-figure Step 7; 2026-10-09. Scope is the new
display-only diagnostic refresh, not the five existing paper figures or the
canonical manuscript. No source, trace, metric, manifest, or controller was
changed by this review, and no physical experiment was run.

**Verdict: the selected figures and video previews are usable diagnostics;
correct the three display semantics below before treating the refresh as final.**

|Priority|Finding|Action|
|---|---|---|
|P2|`visualization.py:24,90`: reference-progress captions say "Normalized path progress", but the plotted `progress_s` is raw virtual time. R03 ends at 7.800, not a fraction in [0,1]; selected nominal ends at 7.952.|Either divide only the displayed values by the archived template duration and label dimensionless progress, or preserve values and label "Virtual path time (s)" with a matching caption. Do not change saved data.|
|P2|`visualization.py:151-152`: zero singular values are drawn at an unlabelled 1e-12 floor. Every saved D02 singular value is exactly zero, yet its plot appears to contain a small positive spectrum.|Annotate the log display floor and state that values at/below it are censored for display. For D02 explicitly identify the all-zero spectrum/no informative identification update. Retain its rank-zero and prior parameter values.|
|P2|`visualization.py:169,183`: failed-run captions and per-run gallery notices use "complete final window False" / a raw Boolean. The overview table already uses NOT_EVALUATED.|Use NOT_EVALUATED consistently for an absent post-latch window, including standalone captions and preferably the failed detumbling panels. Do not convert the missing window into a failed detumbling result.|

Two lower-priority improvements:

- Identification truth components all use indistinguishable gray dashed lines
  (`visualization.py:145`). In D02, three distinct COM/inertia truth levels
  cannot be assigned to x/y/z or tensor entries without reading raw data. Add
  component labels to the truth lines or a small truth legend/table; preserve
  the existing estimate line styles for grayscale reading.
- At the gallery's approximately 1240 px width, the 2880 px six-panel video's
  18 px telemetry text becomes about 8 px. Keep the separate 960 x 600 views and
  a clear fullscreen/native-resolution instruction. The composite remains
  useful for qualitative geometry; it should not be the only telemetry view.

Checks and positive findings:

- Inspected color PNGs for R01 nominal trajectory, tracking, capture/load and
  identification; R03 noisy tracking, state estimation and reference progress;
  D02 tracking, detumbling and identification. Also inspected grayscale PDF
  renders of nominal tracking and D02 identification. Units, line styles,
  legends and sampled layouts are readable, with no clipped labels observed.
  The tall six-panel figures are diagnostic pages, not ready-to-shrink
  single-column paper figures.
- All 88 diagnostic PDFs are one page, have extractable text and contain no
  raster images. This structural check covered every diagnostic PDF; visual
  inspection was representative rather than an assertion that all 88 pages
  were individually inspected.
- The nominal tracking curve ends at 7.990 s, immediately before the 7.992 s
  latch; it does not extend the inactive reference through the 20 s post-latch
  phase. R03 and D02 stop at their actual 7.840 s and 0.040 s endpoints. The
  geometry projections use equal X/Y or X/Z scales within each panel.
- Estimation plots select both valid and current estimator-output ticks. The
  three inspected runs have no internal gaps in this valid selection. Their
  covariance caption correctly describes maximum marginal 3-sigma shading,
  without a joint probability or safety-coverage claim.
- Failed detumbling plots do not shade an invented final window. Capture/load
  panels retain measured quantities and physical thresholds. Prior-valued
  D02 parameter curves are retained, and its rank is correctly zero; the
  remaining concern is the unlabelled singular-value display floor.

Video review boundary and evidence:

- Inspected six-panel previews at the start and near latch, body-side previews
  at the start, near latch and final sample, and the interface closeup near
  latch. Individual-view overlays identify run, absolute time, latch-relative
  time, phase and simulation scope. The final body view correctly switches to
  COMPLETED at physical time 27.992 s. The interface closeup is qualitative;
  some geometry/witness segments are occluded and the numerical gap overlay
  supplies the quantitative value.
- `render.py:79-98` restores recorded integration states and computes camera
  forward/up vectors using the complete base rotation matrix, including roll.
  The five world-view cameras use fixed framing; the body-side camera uses
  scene-center framing without an Euler-angle switch. This is supported by
  code and camera samples, not inferred from still images alone.
- The video manifest records eight H.264 videos at 30 fps with 841 frames each.
  The 841 selected state indices increase strictly from the initial to final
  trace sample; physical display times span 0 to 27.992 s, with maximum spacing
  0.034 s. Maximum adjacent body-camera forward and up changes are respectively
  0.0874 and 0.0818 degrees; maximum forward/up orthogonality error is 1.11e-16.
  These checks support smooth sampled camera orientation. This review did not
  watch every encoded frame and does not claim continuous-time visual proof.
- Encoded duration is 28.033 s because the final recorded state is included in
  a constant-frame-rate stream. Keep the physical endpoint distinct from file
  duration. The code performs state restoration/forward evaluation for display
  without `mj_step`; manifests report zero new physical attempts and steps.

All runs remain seen development/regression evidence. Videos, tracking plots
and the shadow identification views add explanatory detail; they do not supply
independent holdouts, component ablations, parameter accuracy certification, or
a new successful physical trajectory. Recheck the affected generated captions
and figures after the display corrections, then refresh their hashes using the
normal generator. No commit or push was performed by the reviewer.

## Final correction verification

Follow-up review on 2026-10-09: **all three P2 display findings above are
resolved, and the two lower-priority improvements have been implemented. No
remaining display blocker was found in this focused recheck.** This conclusion
supersedes the initial conditional verdict, within the same diagnostic scope.

- The regenerated nominal progress plot and all eight manifest captions now
  identify the unchanged raw quantity as virtual path time in seconds.
- The singular-value floor is disclosed in identification captions. D02 has
  the explicit all-zero-spectrum statement inside the panel. Its text and
  right-side legend are separated and readable, with no clipping observed.
  Prior values and rank zero remain intact.
- Identification plots now have matching component colors and explicit truth
  entries. Rechecked D02 and R01 legends and labels. An intermediate generic
  floor note overlapped R01's spectrum; it was subsequently removed from the
  nonzero-spectrum panel while remaining in the caption. After the final log
  reported R01 regeneration, its PNG was inspected again: the spectrum and
  threshold are unobstructed, and its PNG matches the manifest hash.
- Missing-window captions and D02's generated gallery notice explicitly say
  NOT_EVALUATED. The unshaded stopped detumbling trace remains descriptive
  prefix data, with no fabricated post-latch outcome.
- The generated overview page includes the fullscreen/original-size advice
  and retains links to individual camera views.

The completed correction pass was checked across eight manifests, each with
11 figures: the revised captions were present and all 88 PNG hashes matched at
that check. The final layout-only regeneration was then spot-checked on R01 as
requested. Full encoded-video decoding and final publication/push verification
remain the executor's separate checks; the reviewer's earlier limits on visual
sampling, scientific evidence and video continuity claims remain unchanged.
