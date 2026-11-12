"""Export a source release without local calibration, bindings or runtime data."""

import argparse
import hashlib
import io
import json
import re
import tarfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
ROOT_FILES = {".gitignore", "pyproject.toml", "README.md", "README_STANDALONE.md", "VALIDATION.md"}
SOURCE_DIRS = {"src", "tests", "docs", "scripts", "tools"}
SUFFIXES = {".py", ".cpp", ".md", ".txt", ".sh", ".html", ".css", ".js", ".urdf", ".stl", ".json"}


def source_files():
    for path in sorted(ROOT.rglob("*")):
        relative = path.relative_to(ROOT)
        if (
            not path.is_file()
            or path.is_symlink()
            or any(
                part.startswith(".") or part == "__pycache__" or part.endswith((".egg-info", ".dist-info"))
                for part in relative.parts[:-1]
            )
            or path.name.endswith((".local.json", ".binding.json", ".caps.json"))
            or path.name in {"calibration.json", "lumos_joint_limits.json"}
        ):
            continue
        if relative.as_posix() in ROOT_FILES or relative.as_posix() == "configs/example.json":
            yield path, relative
        elif relative.parts[0] in SOURCE_DIRS and path.suffix.lower() in SUFFIXES:
            yield path, relative


def export(output):
    version = re.search(r'^version = "([^"]+)"', (ROOT / "pyproject.toml").read_text(), re.M)[1]
    prefix = f"stararm-lumos-feedback-{version}"
    files = list(source_files())
    manifest = dict(
        version=version,
        format="source",
        hardware_validated=False,
        files=[dict(path=r.as_posix(), sha256=hashlib.sha256(p.read_bytes()).hexdigest()) for p, r in files],
    )
    # Exclusive creation keeps earlier delivery artifacts intact.
    with output.open("xb") as stream, tarfile.open(fileobj=stream, mode="w:gz") as archive:
        for path, relative in files:
            archive.add(path, arcname=f"{prefix}/{relative.as_posix()}", recursive=False)
        data = (json.dumps(manifest, indent=2, ensure_ascii=False) + "\n").encode()
        entry = tarfile.TarInfo(f"{prefix}/RELEASE.json")
        entry.size, entry.mode = len(data), 0o644
        archive.addfile(entry, io.BytesIO(data))
    print(f"Source release: {output} ({len(files)} files; no local setup or runtime data)")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    export(parser.parse_args().output)
