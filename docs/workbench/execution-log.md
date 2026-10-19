# SDD ledger — plan: docs/superpowers/plans/2026-10-19-piper-workbench-plan.md

Spec: docs/superpowers/specs/2026-10-19-piper-workbench-design.md
Base: a2d65384
User: execute all tasks inline; after compaction re-read design and this ledger; compare running UI screenshots with approved previews.

Pre-flight: T2/T6 share RViz goal state; T3/T5/T10 share persistent service/control/recovery events; T4/T5 share camera/session/track identity; T7/T8 share active calibration and actual restart status; T9/T10 consume service events. Interfaces consistent.
Ruling: Chinese task headings require local brief extraction instead of skill English Task parser — preserve user-facing plan — no product cost.
Ruling: apply targeted tests only and no full-suite gate — explicit user instructions override skill default — risk: unrelated existing failures not exhaustively checked.

Task 1: in progress.

Task 1: Ruling: onsite tests retained obsolete 1mm/40px rejection fixtures; updated invalid samples to 16mm/24px, without changing production 15mm/25px policy.
Task 1: complete — baseline before sync 128 passed/2 skipped; after sync calibration logic/TCP/approval relevant tests 133 passed (1.95s). No main directory edits.
Task 2: in progress — native Qt/RViz skeleton, then offline rendering. UI layout verified through build/screenshots rather than redundant label tests.

Task 2: native model launch exit=0; real Piper/conveyor/bins rendered in offline domain231. Custom RenderPanel segfault traced to Ogre resize; using native VisualizationFrame initialization fixes lifecycle. QScreen capture required for native GL. TRAC-IK independent source copied from existing read-only external asset; shared environment unchanged. Full shell/visual comparison still pending.

Tasks 3–5,7–10: backend integration in progress (not complete UI). Added persistent Unix socket server, existing process manager adapter, forced recovery, dual camera cache, placement snapshots, T5 direct cycle pause/cancel-only recovery, PTY calibration/application, raw capture and logs. New critical-path tests RED→GREEN: components3, data3, control2, calibration2; with geometry16 passed. TRAC-IK local plugin build passed (14.6s). Ruling: backend interfaces implemented together before final shell polish because pages consume shared operations; final completion remains per task requirements. No real devices started.

UI first six-page build passed; first real screenshot reviewed against reference — visual polish required, user explicitly reiterated beauty/animations. Backend socket status/parameters/calibration list all returned true in domain231. Connected-window exit revealed heap corruption; gdb identified destruction path, investigation focuses on QLocalSocket disconnected callback accessing already-destroyed members. Added explicit disconnection destructor plus dedicated 100x lifecycle regression. ASAN executable has excessive startup overhead in this graphics runtime; normal debugger evidence retained. No real hardware used.

Tasks 2–10: software implementation checkpoint. Targeted Python checks 255 passed (7.15s), domain223 ROS IO included; Qt build passed. Exact code/calibration hardware paths remain field validation, not hardware acceptance. Updated obsolete runtime-config fixtures to current onsite transforms/Dabai link6; restored graceful handeye Ctrl-C shutdown after its existing regression failed. Ruling: prevent hidden stock RViz frame from constructing its redundant MoveGroup client during model initialization — native markers/model remain, workbench manual controller owns planning — cost: hidden stock panel is intentionally not an alternate planner UI.
Task 8: RED→GREEN coordinate/unit mismatch + pending/effective evidence test. Selected values never imply effectiveness; freshness/numeric K/D/TF and SDK-start/parameter/MoveIt/UI model checked. Generic imports classified by camera directory/metadata, backup persists across UI restart. TCP selection normalized to one public TCP.
Task 9: raw PNG records now include configuration/calibration source snapshots. No labels/training generated.
Task 2: final lifecycle diagnosis confirmed by gdb: installed MoveIt hidden-frame constructor/shutdown race. Narrow subclass suppresses redundant hidden-frame group initialization; offline screenshot/close now process exit0, no extra planner gates. Six-page visual pass running.

Final: fresh read-only branch review a2d65384..189af816: 1 Critical, 8 Important, 1 original Minor promoted Important for visible capture failure. Fixed in one pass; no second reviewer. Details + all exclusions/rulings preserved docs/workbench/review.md.
Final: fixed emergency order + owned TCP cancel, result/ack/session identity, Dabai actual TCP parent, real pregrasp setting, per-kind restore, capture errors, manual goal equality, recovery epoch — new regressions RED→GREEN; relevant suite 268/268; Qt build16.3s; C++ state/lifecycle exit0.
Task 2: complete (software implementation and scoped offline checks; hardware/field acceptance explicitly pending in docs/workbench/validation.md).
Task 3: complete (software implementation and scoped offline checks; hardware/field acceptance explicitly pending in docs/workbench/validation.md).
Task 4: complete (software implementation and scoped offline checks; hardware/field acceptance explicitly pending in docs/workbench/validation.md).
Task 5: complete (software implementation and scoped offline checks; hardware/field acceptance explicitly pending in docs/workbench/validation.md).
Task 6: complete (software implementation and scoped offline checks; hardware/field acceptance explicitly pending in docs/workbench/validation.md).
Task 7: complete (software implementation and scoped offline checks; hardware/field acceptance explicitly pending in docs/workbench/validation.md).
Task 8: complete (software implementation and scoped offline checks; hardware/field acceptance explicitly pending in docs/workbench/validation.md).
Task 9: complete (software implementation and scoped offline checks; hardware/field acceptance explicitly pending in docs/workbench/validation.md).
Task 10: complete (software implementation and scoped offline checks; hardware/field acceptance explicitly pending in docs/workbench/validation.md).
Task 11: complete (software implementation and scoped offline checks; hardware/field acceptance explicitly pending in docs/workbench/validation.md).
Final: Ruling: keep existing failure-protection algorithm and external-process ownership; no automatic unknown T1 kill — preserve original operation + other workspaces — cost: externally owned T1 requires stopping original terminal.
Final: Ruling: capture error feedback graded Important and fixed — missing-camera/write failures must be visible — no remaining deferred minor.


## 2026-10-19 现场反馈修复

修复 RViz 高分屏闪烁、sudo 标准输入凭据、YOLO 本地权重准备、恢复串口耦合及残留驱动，并补全连接关闭和曝光原图预览。用户确认 RViz 已正常，第二次授权恢复确认机械臂使能与实际反馈。具体证据、测试及 USB 链路边界见 [现场修复记录](field-fixes-20261019.md)。
