# Operator UI sources

`web/lightnav-base.css` is copied unchanged from
`AlanZhu2006/lightnav_realworld@ddfd37059c49e8090217e11c4ffa6f6b3ff9d554`,
`robot_deploy/src/vln_web/web/styles.css`. Its Apache-2.0 license is retained
as `LICENSE.LightNav`. The ImageGoal HTML, JavaScript and additional stylesheet
are deployment-specific adaptations. They use the mature stack's visual
structure, task/control separation and observation layout; they do not expose
its manual drive, posture or SDK actions.

The UI sends only the task APIs documented by `deployment/imagegoal/CONTRACT.md`.
Runtime state, sensor provenance and arrival confirmation remain owned by the
runtime. A UI refresh cannot re-enable a stopped task.
