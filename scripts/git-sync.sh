#!/usr/bin/env bash
set -euo pipefail

if [ "$#" -lt 1 ]; then
  echo "Usage: ./scripts/git-sync.sh \"Describe the change\""
  exit 1
fi

git status --short
git pull --ff-only
git add .

if [ -z "$(git status --short)" ]; then
  echo "No changes to commit."
  exit 0
fi

git commit -m "$1"
git push
