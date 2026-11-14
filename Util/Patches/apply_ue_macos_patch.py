#!/usr/bin/env python3
"""Apply, verify, or revert CARLA's macOS compatibility edits on Unreal Engine 5.8.

Replaces the monolithic diff with surgical, verified in-place edits against the
pinned CarlaUnreal/UnrealEngine (ue58-dev-carla) commit without storing large
engine code excerpts in the repository.

Every transform validates uniqueness across all target files in memory before
writing anything to disk, preventing partial updates.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
import sys
from typing import Callable


@dataclass(frozen=True)
class PatchRule:
    """Single target file transform with forward, reverse, and state checks."""

    rel_path: str
    forward: Callable[[str], str]
    backward: Callable[[str], str]


def _replace_unique(text: str, old: str, new: str, label: str) -> str:
    """Replace an anchor occurring exactly once, raising ValueError otherwise."""
    count = text.count(old)
    if count != 1:
        raise ValueError(f"{label}: expected 1 anchor match, found {count}")
    return text.replace(old, new, 1)


def _patch_apple_sdk(text: str) -> str:
    text = _replace_unique(
        text, '"MainVersion": "26.1.1",', '"MainVersion": "27.0.0",', "Apple_SDK MainVersion"
    )
    text = _replace_unique(
        text, '"MaxVersion": "26.9.0",', '"MaxVersion": "30.0.0",', "Apple_SDK MaxVersion"
    )
    text = _replace_unique(
        text,
        '\t\t\t"26.4.0-21.1.6"\n\t\t]',
        '\t\t\t"26.4.0-21.1.6",\n\t\t\t"27.0.0-21.1.6"\n\t\t]',
        "Apple_SDK AppleVersionToLLVMVersion",
    )
    return text


def _unpatch_apple_sdk(text: str) -> str:
    text = _replace_unique(
        text, '"MainVersion": "27.0.0",', '"MainVersion": "26.1.1",', "Apple_SDK MainVersion (rev)"
    )
    text = _replace_unique(
        text, '"MaxVersion": "30.0.0",', '"MaxVersion": "26.9.0",', "Apple_SDK MaxVersion (rev)"
    )
    text = _replace_unique(
        text,
        '\t\t\t"26.4.0-21.1.6",\n\t\t\t"27.0.0-21.1.6"\n\t\t]',
        '\t\t\t"26.4.0-21.1.6"\n\t\t]',
        "Apple_SDK AppleVersionToLLVMVersion (rev)",
    )
    return text


def _transform_blend_profile(text: str, *, reverse: bool) -> str:
    if not reverse:
        open_anchor = "\t\t.SupportsMaximize(false)\n\t\t[\n"
        close_anchor = "\n\t\t];\n\n\n\tGEditor->EditorAddModalWindow(Window);"
        new_open = "\t\t.SupportsMaximize(false);\n\n\tWindow->SetContent(\n"
        new_close = "\n\t);\n\n\n\tGEditor->EditorAddModalWindow(Window);"
    else:
        open_anchor = "\t\t.SupportsMaximize(false);\n\n\tWindow->SetContent(\n"
        close_anchor = "\n\t);\n\n\n\tGEditor->EditorAddModalWindow(Window);"
        new_open = "\t\t.SupportsMaximize(false)\n\t\t[\n"
        new_close = "\n\t\t];\n\n\n\tGEditor->EditorAddModalWindow(Window);"

    if text.count(open_anchor) != 1:
        raise ValueError(
            f"BlendProfileStandaloneFactory opening anchor matched {text.count(open_anchor)} times"
        )
    if text.count(close_anchor) != 1:
        raise ValueError(
            f"BlendProfileStandaloneFactory closing anchor matched {text.count(close_anchor)} times"
        )

    start = text.index(open_anchor)
    body_start = start + len(open_anchor)
    body_end = text.index(close_anchor)
    if body_end <= body_start:
        raise ValueError("BlendProfileStandaloneFactory anchors out of order")

    body_lines = text[body_start:body_end].split("\n")
    transformed_lines: list[str] = []
    for idx, line in enumerate(body_lines):
        if not reverse:
            if line and not line.startswith("\t"):
                raise ValueError(
                    f"BlendProfileStandaloneFactory body line {idx} missing expected leading tab"
                )
            transformed_lines.append(line[1:] if line.startswith("\t") else line)
        else:
            transformed_lines.append(f"\t{line}" if line else line)

    return text[:start] + new_open + "\n".join(transformed_lines) + new_close + text[body_end + len(close_anchor) :]


def _patch_metal_viewport(text: str) -> str:
    return _replace_unique(
        text,
        "\t\t\t.Drawable = nullptr,\n\t\t\t.BackBuffer = {nullptr, nullptr},",
        "\t\t\t.BackBuffer = {nullptr, nullptr},\n\t\t\t.Drawable = nullptr,",
        "MetalViewport DrawLayer order",
    )


def _unpatch_metal_viewport(text: str) -> str:
    return _replace_unique(
        text,
        "\t\t\t.BackBuffer = {nullptr, nullptr},\n\t\t\t.Drawable = nullptr,",
        "\t\t\t.Drawable = nullptr,\n\t\t\t.BackBuffer = {nullptr, nullptr},",
        "MetalViewport DrawLayer order (rev)",
    )


def _patch_launch_build(text: str) -> str:
    return _replace_unique(
        text,
        "protected virtual bool bAllowSwiftMain => Target.IsInPlatformGroup(UnrealPlatformGroup.Apple) && Target.Type != TargetType.Editor;",
        "protected virtual bool bAllowSwiftMain => Target.IsInPlatformGroup(UnrealPlatformGroup.Apple) && Target.Platform != UnrealTargetPlatform.Mac && Target.Type != TargetType.Editor;",
        "Launch.Build.cs bAllowSwiftMain",
    )


def _unpatch_launch_build(text: str) -> str:
    return _replace_unique(
        text,
        "protected virtual bool bAllowSwiftMain => Target.IsInPlatformGroup(UnrealPlatformGroup.Apple) && Target.Platform != UnrealTargetPlatform.Mac && Target.Type != TargetType.Editor;",
        "protected virtual bool bAllowSwiftMain => Target.IsInPlatformGroup(UnrealPlatformGroup.Apple) && Target.Type != TargetType.Editor;",
        "Launch.Build.cs bAllowSwiftMain (rev)",
    )


RULES: tuple[PatchRule, ...] = (
    PatchRule(
        rel_path="Engine/Config/Apple/Apple_SDK.json",
        forward=_patch_apple_sdk,
        backward=_unpatch_apple_sdk,
    ),
    PatchRule(
        rel_path="Engine/Plugins/Experimental/Animation/HierarchyTableAnimation/Source/Editor/Private/BlendProfileStandaloneFactory.cpp",
        forward=lambda text: _transform_blend_profile(text, reverse=False),
        backward=lambda text: _transform_blend_profile(text, reverse=True),
    ),
    PatchRule(
        rel_path="Engine/Source/Runtime/Apple/MetalRHI/Private/MetalViewport.cpp",
        forward=_patch_metal_viewport,
        backward=_unpatch_metal_viewport,
    ),
    PatchRule(
        rel_path="Engine/Source/Runtime/Launch/Launch.Build.cs",
        forward=_patch_launch_build,
        backward=_unpatch_launch_build,
    ),
)


def _inspect_state(ue_root: Path) -> tuple[list[tuple[Path, str, str]], bool, bool]:
    """Validate every rule and classify whether the tree is clean-unpatched or clean-patched."""
    unpatched_ready = True
    already_patched = True
    planned_writes: list[tuple[Path, str, str]] = []

    for rule in RULES:
        target = ue_root / rule.rel_path
        if not target.is_file():
            raise FileNotFoundError(f"Missing target Unreal Engine file: {target}")
        raw = target.read_text(encoding="utf-8")

        can_forward = False
        can_reverse = False
        fwd_out = ""
        rev_out = ""

        try:
            fwd_out = rule.forward(raw)
            can_forward = True
        except ValueError:
            unpatched_ready = False

        try:
            rev_out = rule.backward(raw)
            can_reverse = True
        except ValueError:
            already_patched = False

        if not can_forward and not can_reverse:
            # Re-run forward to surface its diagnostic message.
            rule.forward(raw)

        planned_writes.append((target, fwd_out if can_forward else raw, rev_out if can_reverse else raw))

    return planned_writes, unpatched_ready, already_patched


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "ue_root",
        nargs="?",
        default=".",
        help="Path to the UnrealEngine5_carla checkout (defaults to current working directory).",
    )
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--check", action="store_true", help="Exit 0 if all edits can apply cleanly.")
    mode.add_argument("--check-applied", action="store_true", help="Exit 0 if already applied.")
    mode.add_argument("--reverse", action="store_true", help="Revert the macOS compatibility edits.")
    args = parser.parse_args(argv)

    ue_root = Path(args.ue_root).resolve()
    try:
        planned, unpatched_ready, already_patched = _inspect_state(ue_root)
    except Exception as exc:  # noqa: BLE001
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    if args.check:
        if unpatched_ready:
            return 0
        print("Patch cannot apply cleanly (already applied or unexpected file state).", file=sys.stderr)
        return 1

    if args.check_applied:
        if already_patched:
            return 0
        print("Patch is not currently applied cleanly across all target files.", file=sys.stderr)
        return 1

    if args.reverse:
        if not already_patched:
            print("ERROR: Cannot revert; target files are not all in patched state.", file=sys.stderr)
            return 1
        for path, _, rev_content in planned:
            path.write_text(rev_content, encoding="utf-8", newline="")
        print("Reverted macOS compatibility edits from Unreal Engine.")
        return 0

    if already_patched:
        print("Unreal Engine macOS compatibility edits are already applied.")
        return 0

    if not unpatched_ready:
        print("ERROR: Unreal Engine tree is in a mixed/unexpected patch state.", file=sys.stderr)
        return 1

    for path, fwd_content, _ in planned:
        path.write_text(fwd_content, encoding="utf-8", newline="")
    print("Applied macOS compatibility edits to Unreal Engine successfully.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
