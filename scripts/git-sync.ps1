param(
    [Parameter(Mandatory=$true)]
    [string]$Message
)

$ErrorActionPreference = "Stop"

git status --short
git pull --ff-only
git add .

$changes = git status --short
if (-not $changes) {
    Write-Host "No changes to commit."
    exit 0
}

git commit -m $Message
git push
