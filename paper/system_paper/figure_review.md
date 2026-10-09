# Scientific figure quality review

Reviewer role: paper architect/reviewer, paper-figure Step 7. Reviewed 2026-10-09.
Scope: partial simulation systems V1; no target venue selected. The review covers
F1-F5, their source scripts and the captions in `figure_plan.md`. It does not
constitute a controller, claim, or citation audit.

**Verdict: acceptable for the explicitly limited development report after the
corrections below; insufficient evidence for independent robustness or component
benefit claims.** No measurements, metrics, gates, ledgers, controllers, or
experiments were changed.

|Figure|Assessment and corrections|
|---|---|
|F1: historical feasibility|The three line panels preserve all required historical failures. Added the dimensionless unit to z and corrected the caption's nonexistent shaded endpoint to the actual dashed endpoint. The caption now discloses separate vertical scales and explains that a frozen linear-set diagnostic is not penetration, a new trajectory, or repaired success. The original row-scale normalization is stated. Keep physical row contradictions in the accompanying diagnostic table; do not relabel them as z.|
|F2: same-packet estimator|A paired time trace is appropriate for this comparison. Moved the common legend above the panels to avoid obscuring spikes. The caption defines geometric-origin velocity error norms, evaluation-only truth, the 1-7 s preterminal display interval, and offline reuse of the original packets/trajectory. It does not turn lower errors into a closed-loop or covariance-calibration claim.|
|F3: nominal development trace|The four panels separate angular speed, actual load utilization and the two momentum units. The caption now identifies latch, fixed evaluation window, logarithmic speed axis, both speed thresholds, rho limit, and the initial-sample reference for momentum drift. Plot thresholds and window now read the archived run config/metrics instead of duplicate literals. The trace remains the first development nominal, irrespective of later versions.|
|F4: all attempts|Fixed a material omission: the old script skipped ledger entries without metrics. Every attempt is now retained; unavailable evaluations and missing full windows are NE. Added recorded status, algorithm version and actual stop time, with NE for unavailable stop times. Renamed the safety column to observed safety violation and clarified that No covers only the observed prefix. The matrix is now vector cells, and Yes/No remain literal outcomes rather than a common pass/fail color code.|
|F5: historical identification|Separate COM error and rank panels are appropriate. The caption defines COM and the ten possible parameter directions, keeps the unchanged 5 mm threshold, and makes H1's accuracy failure explicit despite rank 10. Torque identity refers to the complete paired records, separate from the two plotted prior runs. Neither identification control benefit nor independent accuracy validation is claimed.|

Visual/export checks completed:

- Regenerated all five PDF/PNG pairs with the specified Python interpreter and
  `OPENBLAS_NUM_THREADS=1`; all exports are nonempty.
- Inspected every color PNG and a Poppler grayscale rendering of every PDF.
  No clipped labels, overlapping legends, internal titles, or missing units
  remain. Curves use solid/dashed/dotted styles; F4 also prints Yes/No/NE.
- All five PDFs have one page, extractable text, embedded Type0 fonts, and no
  raster images. Shared style uses serif text and STIX math. PNG output is 300 dpi.
- Use these multi-panel figures at full manuscript width. Final printed font
  size and caption placement require one more layout check after a venue and
  actual inclusion width are selected; F1 and F4 should not be reduced to a
  single column.

Remaining evidence and delivery requirements:

1. None of F1-F5 supplies an independent holdout. F1/F2/F5 are historical or
   same-data reanalyses; F3 is one seen development trajectory; F4 retains all
   seen attempts. Do not report a population success rate or confidence interval
   from these rows.
2. Isolating controller/reference/estimator contributions requires prospective
   component ablations and matched closed-loop baselines. F2 only isolates
   estimator output on a fixed archived stream. These missing experiments remain
   planned, not fabricated plot entries.
3. F5 supports the distinction between numerical rank and parameter accuracy.
   Independent excitation/identification validation and an actual action-change
   comparison are still needed for stronger parameter or control claims.
4. F4 is a live ledger snapshot. At the reviewed render it contains D01-D04 and
   R01, with R01 still RUNNING and its metrics NE. Regenerate it after the final
   ledger state, retain every row, and recheck row-label legibility if it grows.
   Keep absent validation categories explicitly NOT_EVALUATED in the companion
   evidence table rather than plotting them as zero successes.

The revised standalone captions in `figure_plan.md` are the reviewed versions
to carry into the manuscript. Temporary grayscale PDF renders are under
`tmp/pdfs/n209_figure_review/` for traceable visual QA.
# Final ledger refresh

After the four seen regressions completed, the parent reviewer regenerated and
visually inspected F4 with all eight attempts. Labels and stop times are legible,
no row is clipped, all five missing post-capture windows remain NE, and the three
completed prefixes are distinct from observed safety status. Replay status is
reported in the generated tables, not encoded as task success in this figure.
