#!/usr/bin/env python3
import os
import re

# Determine repo root path
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
AGENTS_PATH = os.path.join(REPO_ROOT, "AGENTS.md")

def check_file_exists(rel_path):
    path = os.path.join(REPO_ROOT, rel_path)
    return os.path.isfile(path)

def check_param_defined(param_name):
    params_file = os.path.join(REPO_ROOT, "src/modules/antenna_tracker/tracker_params.c")
    if not os.path.isfile(params_file):
        return False
    with open(params_file, 'r', encoding='utf-8') as f:
        content = f.read()
    return param_name in content

def check_function_defined(func_name):
    # Check in tracker_geo.hpp or tracker_geo.cpp
    geo_h = os.path.join(REPO_ROOT, "src/modules/antenna_tracker/tracker_geo.hpp")
    geo_cpp = os.path.join(REPO_ROOT, "src/modules/antenna_tracker/tracker_geo.cpp")
    
    found = False
    for path in [geo_h, geo_cpp]:
        if os.path.isfile(path):
            with open(path, 'r', encoding='utf-8') as f:
                if func_name in f.read():
                    found = True
                    break
    return found

def check_command_implemented(cmd):
    main_cpp = os.path.join(REPO_ROOT, "src/modules/antenna_tracker/antenna_tracker_main.cpp")
    if not os.path.isfile(main_cpp):
        return False
    with open(main_cpp, 'r', encoding='utf-8') as f:
        content = f.read()
    # Check if command name is mentioned in the main C++ file
    cmd_word = cmd.split()[-1] # e.g. start, stop, status
    return cmd_word in content

def check_actuator_servos_published():
    main_cpp = os.path.join(REPO_ROOT, "src/modules/antenna_tracker/antenna_tracker_main.cpp")
    main_hpp = os.path.join(REPO_ROOT, "src/modules/antenna_tracker/antenna_tracker.hpp")
    found = False
    for path in [main_cpp, main_hpp]:
        if os.path.isfile(path):
            with open(path, 'r', encoding='utf-8') as f:
                content = f.read()
                if "actuator_servos" in content:
                    found = True
                    break
    return found

def evaluate_checkbox(text):
    # Match files (e.g. `src/modules/...`)
    file_match = re.search(r'`([^`]+)`', text)
    if file_match:
        val = file_match.group(1)
        # 1. Check if it's a file path
        if "/" in val:
            return check_file_exists(val)
        # 2. Check if it's a parameter
        if val.startswith("TRK_"):
            return check_param_defined(val)
        # 3. Check if it's a function signature
        func_name_match = re.match(r'^([a-zA-Z0-9_]+)\b', val)
        if func_name_match:
            func_name = func_name_match.group(1)
            # Make sure it isn't one of the other items
            if func_name not in ["antenna_tracker", "actuator_servos"]:
                return check_function_defined(func_name)

    # Match shell commands: `antenna_tracker start` etc.
    cmd_match = re.search(r'`antenna_tracker (start|status|stop)`', text)
    if cmd_match:
        return check_command_implemented(cmd_match.group(0).replace('`', ''))

    # Match actuator_servos publishing statement
    if "actuator_servos.control" in text or "actuator_servos" in text:
        return check_actuator_servos_published()

    return None

def main():
    if not os.path.isfile(AGENTS_PATH):
        print(f"Error: AGENTS.md not found at {AGENTS_PATH}")
        return

    with open(AGENTS_PATH, 'r', encoding='utf-8') as f:
        lines = f.readlines()

    updated_lines = []
    changes_made = 0
    total_checkboxes = 0
    checked_checkboxes = 0

    for line in lines:
        # Match markdown checkbox line: - [ ] or - [x]
        match = re.match(r'^(\s*-\s*\[)([ xX])(\]\s+)(.+)$', line)
        if match:
            total_checkboxes += 1
            prefix, current_state, suffix, text = match.groups()
            detected_state = evaluate_checkbox(text)
            
            if detected_state is not None:
                new_state = "x" if detected_state else " "
                if new_state != current_state:
                    line = f"{prefix}{new_state}{suffix}{text}\n"
                    changes_made += 1
                if new_state == "x":
                    checked_checkboxes += 1
            else:
                # If we cannot evaluate, keep the current state
                if current_state.lower() == "x":
                    checked_checkboxes += 1
                    
        updated_lines.append(line)

    if changes_made > 0:
        with open(AGENTS_PATH, 'w', encoding='utf-8') as f:
            f.writelines(updated_lines)
        print(f"Success: Updated AGENTS.md. Made {changes_made} changes.")
    else:
        print("No changes needed. AGENTS.md is already up to date.")

    print(f"Progress: {checked_checkboxes}/{total_checkboxes} tasks completed ({checked_checkboxes/total_checkboxes:.1%}).")

if __name__ == "__main__":
    main()
