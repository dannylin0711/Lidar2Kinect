# Working in this repository

Two agents work here: one on macOS (shared library, iOS app), one on Windows (`windows/`).

- `shared/` is the contract. A change to a packet layout, a convention in `docs/03`, or a public
  header affects the other side: update the code, the tests, the golden packets
  (`test_l2k tests/vectors --write`) and the document in the same commit, and add a line to
  `docs/04-status.md`. Run `make -C shared test` and `swift test` (macOS) or the CMake tests
  (Windows) before committing.
- Stay in your folder otherwise: the Windows agent does not edit `ios/`, the macOS agent does not
  edit `windows/` apart from `AGENT_PROMPT.md`.
- Facts about the game come from the game's DLL (`docs/01` gives addresses). Do not guess a
  behaviour: read it, or mark it as open.
- Never commit game files, decompiled game code, or Microsoft SDK headers / binaries.
- Keep `docs/04-status.md` (and `windows/STATUS.md`) current: what was done, what was verified and
  how, what was not.
