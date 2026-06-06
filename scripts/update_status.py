#!/usr/bin/env python3
import os
import re

# Determine repo root path
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
AGENTS_PATH = os.path.join(REPO_ROOT, "AGENTS.md")

# Level 2 configuration: modified files and their content markers
LEVEL2_MODIFIED_FILES = {
    "src/modules/mavlink/mavlink_receiver.cpp": ["TrackerTargetPosition", "tracker_target_position", "TRK_SYSID_TARGET"],
    "src/modules/mavlink/mavlink_receiver.h": ["TrackerTargetPosition", "tracker_target_position", "TRK_SYSID_TARGET"]
}

def check_file_level(rel_path):
    path = os.path.join(REPO_ROOT, rel_path)
    if not os.path.isfile(path):
        return False
        
    # Level 2 check: if file is an existing PX4 file, check for custom content markers
    if rel_path in LEVEL2_MODIFIED_FILES:
        with open(path, 'r', encoding='utf-8') as f:
            content = f.read()
        markers = LEVEL2_MODIFIED_FILES[rel_path]
        # Must contain at least one of our custom keywords/integration markers
        return any(marker in content for marker in markers)
        
    # Level 1 check: new file existence
    return True

def check_param_defined(param_name):
    params_file = os.path.join(REPO_ROOT, "src/modules/antenna_tracker/tracker_params.c")
    if not os.path.isfile(params_file):
        return False
    with open(params_file, 'r', encoding='utf-8') as f:
        content = f.read()
    return param_name in content

def check_function_defined(func_name):
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

def check_verification_logs(target_command):
    """
    Level 3 check: Search for verification of a command in logs/verification/
    We look for the pattern "pxh> <target_command>" in the logs.
    """
    verification_dir = os.path.join(REPO_ROOT, "logs", "verification")
    if not os.path.isdir(verification_dir):
        return False
        
    pattern = f"pxh> {target_command}"
    
    for filename in os.listdir(verification_dir):
        filepath = os.path.join(verification_dir, filename)
        if os.path.isfile(filepath):
            try:
                with open(filepath, 'r', encoding='utf-8') as f:
                    if pattern in f.read():
                        return True
            except Exception:
                pass
    return False

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
    # Match backticked content
    code_match = re.search(r'`([^`]+)`', text)
    if code_match:
        val = code_match.group(1)
        
        # 1. Check if it looks like a file path
        if "/" in val:
            return check_file_level(val)
            
        # 2. Check if it's a parameter (starts with TRK_)
        if val.startswith("TRK_"):
            return check_param_defined(val)
            
        # 3. Check if it's a verification command (Level 3)
        # e.g., `antenna_tracker start`, `antenna_tracker status`, `antenna_tracker stop`, `listener actuator_servos`
        if val.startswith("antenna_tracker ") or val.startswith("listener "):
            return check_verification_logs(val)
            
        # 4. Check if it's a function signature
        func_name_match = re.match(r'^([a-zA-Z0-9_]+)\b', val)
        if func_name_match:
            func_name = func_name_match.group(1)
            if func_name not in ["antenna_tracker", "actuator_servos", "listener"]:
                return check_function_defined(func_name)

    # Fallback to check if actuator_servos integration is mentioned
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
