#!/usr/bin/env python3
"""
统一修改所有包中敌方颜色配置的脚本。

用法:
    python3 change_enemy_color.py red      # 切换为红方
    python3 change_enemy_color.py blue     # 切换为蓝方
    python3 change_enemy_color.py --dry-run red   # 预览修改（不实际写入）
"""

import re
import sys
import os
from pathlib import Path

# workspace 根目录（脚本所在目录）
WORKSPACE_ROOT = Path(__file__).resolve().parent

# ============================================================
# 配置文件定义
# 每个条目: (相对路径, 正则匹配行, 替换模板)
# 替换模板中使用 {color} 作为新颜色的占位符
# ============================================================

SIMPLE_FILES = [
    # 1. decision/config/decision.yaml
    {
        "path": "src/main/decision/config/decision.yaml",
        "pattern": r'^(enemy:\s*)"(red|blue)"',
        "replacement": r'\1"{color}"',
    },
    # 2. filter/config/filter.yaml
    {
        "path": "src/main/filter/config/filter.yaml",
        "pattern": r'^(enemy_color:\s*)"(red|blue)"',
        "replacement": r'\1"{color}"',
    },
    # 3. debug_toolbox/minimap/config/minimap_drawer.yaml
    {
        "path": "src/debug_toolbox/minimap/config/minimap_drawer.yaml",
        "pattern": r'^(enemy_color:\s*)"(red|blue)"',
        "replacement": r'\1"{color}"',
    },
    # 4. locate/configs/locate.yaml
    {
        "path": "src/main/locate/configs/locate.yaml",
        "pattern": r'^(enemy:\s*)"(red|blue)"',
        "replacement": r'\1"{color}"',
    },
]

# relocalization config 特殊处理：切换 init_params 行
# 红方和蓝方对应不同的 init_params，通过注释/取消注释来切换
RELOCALIZATION_FILE = "src/main/relocalization/config/config.yaml"

# 匹配 init_params 行（带注释中 enemy is red/blue）
RELOC_PATTERN_ACTIVE = re.compile(
    r'^init_params:\s*\[([^\]]+)\]\s*#\s*enemy is (red|blue)'
)
RELOC_PATTERN_COMMENTED = re.compile(
    r'^#\s*init_params:\s*\[([^\]]+)\]\s*#\s*enemy is (red|blue)'
)

# RPS 变体
RELOC_PATTERN_RPS_ACTIVE = re.compile(
    r'^init_params:\s*\[([^\]]+)\]\s*#\s*RPS.*enemy is (red|blue)'
)
RELOC_PATTERN_RPS_COMMENTED = re.compile(
    r'^#\s*init_params:\s*\[([^\]]+)\]\s*#\s*RPS.*enemy is (red|blue)'
)


def modify_simple_file(file_path: Path, pattern: str, replacement: str, color: str, dry_run: bool):
    """修改简单的 enemy/enemy_color 键值对文件。"""
    if not file_path.exists():
        print(f"  [警告] 文件不存在，跳过: {file_path}")
        return False

    content = file_path.read_text(encoding="utf-8")
    regex = re.compile(pattern, re.MULTILINE)
    new_content, count = regex.subn(replacement.replace("{color}", color), content)

    if count == 0:
        print(f"  [跳过] 未找到匹配行: {file_path}")
        return False

    if new_content == content:
        print(f"  [跳过] 已是目标颜色: {file_path}")
        return False

    if dry_run:
        # 显示差异
        old_lines = content.splitlines()
        new_lines = new_content.splitlines()
        for i, (old, new) in enumerate(zip(old_lines, new_lines)):
            if old != new:
                print(f"  - 第{i+1}行: {old.strip()}")
                print(f"  + 第{i+1}行: {new.strip()}")
        return True

    file_path.write_text(new_content, encoding="utf-8")
    print(f"  [已修改] {file_path} ({count} 处)")
    return True


def modify_relocalization(file_path: Path, color: str, dry_run: bool):
    """修改 relocalization config 中的 init_params（切换红蓝方参数）。"""
    if not file_path.exists():
        print(f"  [警告] 文件不存在，跳过: {file_path}")
        return False

    content = file_path.read_text(encoding="utf-8")
    new_lines = []
    modified = False

    for line in content.splitlines(keepends=True):
        new_line = line

        # 检查非 RPS 的 init_params 行
        m_active = RELOC_PATTERN_ACTIVE.match(line)
        m_commented = RELOC_PATTERN_COMMENTED.match(line)
        m_rps_active = RELOC_PATTERN_RPS_ACTIVE.match(line)
        m_rps_commented = RELOC_PATTERN_RPS_COMMENTED.match(line)

        if m_active and "RPS" not in line:
            current_color = m_active.group(2).strip()
            if current_color != color:
                # 注释掉当前行
                new_line = "# " + line
                modified = True
        elif m_commented and "RPS" not in line:
            target_color = m_commented.group(2).strip()
            if target_color == color:
                # 取消注释
                new_line = line[2:]  # 去掉 "# "
                modified = True
        elif m_rps_active:
            # RPS 行始终注释掉
            new_line = "# " + line
            modified = True
        elif m_rps_commented:
            # 保持注释
            pass

        new_lines.append(new_line)

    if not modified:
        print(f"  [跳过] 无需修改: {file_path}")
        return False

    if dry_run:
        old_lines = content.splitlines()
        new_lines_stripped = [l.rstrip("\n\r") for l in new_lines]
        for i, (old, new) in enumerate(zip(old_lines, new_lines_stripped)):
            if old != new:
                print(f"  - 第{i+1}行: {old.strip()}")
                print(f"  + 第{i+1}行: {new.strip()}")
        return True

    file_path.write_text("".join(new_lines), encoding="utf-8")
    print(f"  [已修改] {file_path}")
    return True


def main():
    dry_run = "--dry-run" in sys.argv
    args = [a for a in sys.argv[1:] if a != "--dry-run"]

    if len(args) != 1 or args[0] not in ("red", "blue"):
        print("用法: python3 change_enemy_color.py [--dry-run] <red|blue>")
        print("示例:")
        print("  python3 change_enemy_color.py red        # 切换为红方")
        print("  python3 change_enemy_color.py blue       # 切换为蓝方")
        print("  python3 change_enemy_color.py --dry-run red  # 预览修改")
        sys.exit(1)

    color = args[0]
    mode = "[预览模式] " if dry_run else ""

    print(f"{'='*60}")
    print(f"  敌方颜色统一修改工具 {mode}")
    print(f"  目标颜色: {color}")
    print(f"{'='*60}")
    print()

    any_modified = False

    # 1. 修改简单键值对文件
    print("[1/2] 修改 enemy/enemy_color 键值对...")
    for cfg in SIMPLE_FILES:
        full_path = WORKSPACE_ROOT / cfg["path"]
        rel_path = cfg["path"]
        print(f"  处理: {rel_path}")
        if modify_simple_file(full_path, cfg["pattern"], cfg["replacement"], color, dry_run):
            any_modified = True

    # 2. 修改 relocalization config
    print()
    print("[2/2] 修改 relocalization init_params...")
    full_path = WORKSPACE_ROOT / RELOCALIZATION_FILE
    print(f"  处理: {RELOCALIZATION_FILE}")
    if modify_relocalization(full_path, color, dry_run):
        any_modified = True

    print()
    print(f"{'='*60}")
    if dry_run:
        print("  预览完成。去掉 --dry-run 参数以实际写入。")
    elif any_modified:
        print(f"  所有配置已切换为 {color} 方。")
    else:
        print(f"  所有配置已经是 {color} 方，无需修改。")
    print(f"{'='*60}")


if __name__ == "__main__":
    main()
