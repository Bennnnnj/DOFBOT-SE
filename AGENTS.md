# DOFBOT-SE Project Notes

This repository is the working copy for Bennnnnj's DOFBOT-SE learning code.

## GitHub Remote

- Owner: `Bennnnnj`
- Repository: `DOFBOT-SE`
- URL: `https://github.com/Bennnnnj/DOFBOT-SE.git`
- Default branch: `main`

Do not guess or search for the GitHub repository when working in this folder. Use the configured `origin` remote unless the user explicitly provides a different repository.

## Sync Workflow

Before editing:

```powershell
git status --short
git pull --ff-only
```

After editing:

```powershell
git status --short
git add .
git commit -m "Describe the change"
git push
```

On macOS:

```bash
git clone https://github.com/Bennnnnj/DOFBOT-SE.git
cd DOFBOT-SE
git pull --ff-only
```

## Repository Rules

- Keep source, notebooks, manuals, models, and robot assets in Git so the project can be studied from multiple devices.
- Do not commit generated ROS/catkin outputs such as `build/`, `devel/`, `install/`, or `logs/`.
- Do not commit Python cache files, `.pyc` files, or Jupyter checkpoint folders.
- Avoid rewriting history or force pushing unless the user explicitly asks for it.
