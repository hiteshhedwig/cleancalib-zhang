# Examples and visual demos

- [`demos/README.md`](demos/README.md) — the lightweight visible-output roadmap
- `images/` — small tracked inputs for image I/O and real-data examples
- `synthetic/` — small tracked synthetic inputs or reference outputs
- `output/` — generated images and reports; ignored by Git

Demo source files should be tracked; generated artifacts should go under
`examples/output/`. Demos are deliberately separate from unit tests: tests
answer “is it correct?”, while demos answer “what does it do?”.
