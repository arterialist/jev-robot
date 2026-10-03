## Change

Describe the problem and the resulting behavior. Link the relevant issue if there is one.

## Verification

Record the commands you ran and their results. For a browser or robot behavior change, include a screenshot, short recording, or reproducible scenario with the control mode and model mode.

- [ ] `uv run --frozen python -m unittest discover -s tests -v`
- [ ] Relevant simulator or browser check
- [ ] `uv build --no-sources` if packaging changed
- [ ] Documentation reflects any new behavior or limitation
- [ ] No credentials, local `.env`, downloaded Unitree assets, or generated logs are included

## Limits or follow-up

Describe known limitations or measurements that still need a live model run. Report JEV or Gemini performance only if you measured it with live API calls.
