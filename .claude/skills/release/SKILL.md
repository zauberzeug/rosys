---
name: release
description: Release the next RoSys version for a milestone — draft the release notes and a short summary to tmp.md, then, after confirmation, tag and push `v<milestone>` and set the GitHub release body. Use when asked to "release", "create a release for 0.37.0" or "write the release notes".
---

Release the milestone passed as an argument (e.g. `0.37.0`).
Without an argument, take the oldest open milestone: `gh api 'repos/zauberzeug/rosys/milestones?state=open' --jq '.[].title'`.

Milestones are named `0.37.0`, tags `v0.37.0`.
Pushing the tag starts `.github/workflows/publish.yml`: pytest, PyPI, a GitHub release entry (created by `github-actions[bot]`, whose body is the message of the tagged commit until you replace it), and the Docker image.
So the only things to do by hand are the notes, the tag and the release body.

## 1. Check the milestone

- Every PR in the milestone is merged and no issue is still open:
  `gh issue list --milestone "<m>" --state open` and `gh pr list --search 'milestone:"<m>"' --state open` are empty.
- `main` is clean and fast-forwarded to `origin/main` (`git pull --ff-only`), so the tag lands on the latest commit.

If something is still open, stop and report it.

## 2. Gather the data

For every ticket in the milestone:

```bash
gh issue list --milestone "<m>" --state all --json number,title,author,labels
gh pr list --search 'milestone:"<m>"' --state all --json number,title,author,labels
gh pr view <n> --json body,comments,reviews,commits
gh api repos/zauberzeug/rosys/pulls/<n>/comments --jq '[.[].user.login] | unique'
```

Read the PR body for motivation, breaking changes and referenced issues or discussions.
The NiceGUI release-notes skill has a helper that collects all of this into one dossier if a NiceGUI checkout is around:
`RELEASE_NOTES_REPO=zauberzeug/rosys <nicegui>/.claude/skills/release-notes/release-notes.sh dossier "<m>"` (ignore its sponsors part).

## 3. Write the release notes

Follow the format of the previous release (`gh release view v<previous> --json body --jq .body`).

Sections by label, in this order, empty ones left out:

| Label           | Section                       |
| --------------- | ----------------------------- |
| `enhancement`   | New features and enhancements |
| `bug`           | Bugfixes                      |
| `documentation` | Documentation                 |

Dependabot fixes and other maintenance commits outside the milestone are not listed.

Item format, one item per story (an issue and its PRs, plus follow-ups):

```
- Short user-visible effect in one line, about 15–25 words (#issue, #pr by @author, @reviewer, @committer)
```

- Credit the issue author, the PR author, committers (not `claude` or other bots), reviewers with real feedback (formal reviews and inline comments) and substantive commenters, also from referenced discussions.
  Not the merger, not plain approvals without comments.

- A PR labeled `breaking change` gets an indented block below its item, one or two sentences with the old and new behavior and how to migrate:

  ```
    ⚠️ **Breaking change:** ...
  ```

  A deprecation uses `⚠️ **Deprecation:** ...` the same way.

Write the notes to `tmp.md` in the repo root, each item on a single line.

## 4. Summary

Below the notes, separated by `---`, add a short summary for chat.
Write it as plain Markdown, not Slack syntax: the user copies it from the rendered preview, which pastes better than raw Slack markup.

```
**[RoSys <m>](https://github.com/zauberzeug/rosys/releases/tag/v<m>)**

Two German sentences with only the headline changes, breaking changes mentioned briefly. <colophon sigil>
```

The first line is the bold title, linked to the GitHub release, nothing else.
Build the sigil with the `colophon` skill for `--platform github` (plain Markdown link), e.g. with the text "Von Claude aus den Release Notes zusammengefasst.".

## 5. Verify

Compare the milestone numbers with the `#`s in `tmp.md`; every milestone ticket must appear:

```bash
comm -23 <({ gh issue list --milestone "<m>" --state all --json number --jq '.[].number'; gh pr list --search 'milestone:"<m>"' --state all --json number --jq '.[].number'; } | sort -u) \
         <(grep -oE '#[0-9]+' tmp.md | tr -d '#' | sort -u)
```

## 6. Confirm, tag, push

Ask with `AskUserQuestion` whether to release: name the milestone, the number of items per section, the breaking changes and the commit the tag will point to, and point at `tmp.md`.
Only after an explicit yes:

```bash
git tag v<m> && git push origin v<m>
```

## 7. Set the release body

The release appears after pytest and the PyPI upload, a few minutes after the push, with the last commit message as its body.
Replace it right away instead of waiting for the whole workflow (the Docker job takes several more minutes): poll `gh release view v<m>` with the `Monitor` tool, then write the notes without the summary to a scratch file and set them:

```bash
gh release edit v<m> --notes-file <notes.md>
```

Afterwards, watch the rest of the run (`gh run list --branch v<m>`, `gh run watch <id> --exit-status`).
If the workflow fails, report the failing job and do not retry on your own.
Finish with the release URL and remind the user that the summary is ready in `tmp.md`.
