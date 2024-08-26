import os
import sys
import argparse
import json
import matplotlib.pyplot as plt
import numpy as np

def parse_arguments():
    parser = argparse.ArgumentParser(description="SPEC06 Result Analysis")
    parser.add_argument('-r', '--result', default="SPEC06_EmuTasks",
                        help="Specify the directory name containing SPEC06 results (Default: 'SPEC06_EmuTasks')")
    parser.add_argument('-c', '--cov_json', help="Specify the absolute path of the JSON file used to process SPEC06 subfolders (Default: 'simpoint_coverage0.3_test.json')")
    parser.add_argument('-s', '--search', help="Specify the name of the filter counter")
    return parser.parse_args()

def parse_cov_json(designated_json):
    if designated_json:
        return designated_json
    predefined_paths = [
        "/nfs/share/checkpoints_profiles/spec06_rv64gcb_o2_20m/json/simpoint_coverage0.3_test.json",
        "/nfs-nvme/home/share/checkpoints_profiles/spec06_rv64gcb_o2_20m/json/simpoint_coverage0.3_test.json"
    ]
    for path in predefined_paths:
        if os.path.exists(path):
            return path
    return None

def generate_folder_names(json_file_path):
    with open(json_file_path, 'r') as file:
        data = json.load(file)
    folder_names = []
    for benchmark, entries in data.items():
        for data, cov in entries.items():
            folder_name = f"{benchmark}_{data}_{cov}"
            folder_names.append(folder_name)
    return folder_names

def extract_result(work_directory, folder_names):
    raw_data = {}
    success_list = []
    failure_list = []
    for entry in sorted(os.listdir(work_directory)):
        subfolder_path = os.path.join(work_directory, entry)
        if os.path.isdir(subfolder_path):
            if entry in folder_names:
                simulator_err_path = os.path.join(subfolder_path, 'simulator_err.txt')
                if os.path.exists(simulator_err_path):
                    with open(simulator_err_path, 'r') as file:
                        raw_data[entry] = file.read()
                    success_list.append(entry)
                else:
                    failure_list.append(entry)
    if success_list:
        print("Already read data in sub-folder:")
        for folder in success_list:
            print(f" \t {folder}")
    if failure_list:
        print("Can't find simulator_err in sub-folder:")
        for folder in failure_list:
            print(f" \t {folder}")
            print("It may because Json file and SPEC06 result not match ?")
    return raw_data

def process_data(raw_data):
    processed_data = {}
    for folder_name, data in raw_data.items():
        # split lines
        lines = data.strip().split("\n")
        # strip warmup data
        total_lines = len(lines)
        half_lines = total_lines // 2
        relevant_lines = lines[half_lines:]
        
        # prepare storage for processed lines
        processed_lines = []
        
        for line in relevant_lines:
            parts = line.split("]", 2)
            if len(parts) < 3:
                continue
            useful_data = parts[2].strip()
            hierarchy, rest = useful_data.split(":", 1)
            name, count = rest.rsplit(",", 1)
            processed_lines.append({
                'hierarchy': hierarchy.strip(),
                'name': name.strip(),
                'count': count.strip()
            })
        
        processed_data[folder_name] = processed_lines
    
    return processed_data

def main():
    # set NOOP_HOME as Nanhu-v3 directory
    noop_home = os.getenv('NOOP_HOME')
    if noop_home is None:
        print("Error: Unable to get $NOOP_HOME, set $NOOP_HOME first")
        return

    # args parser
    args = parse_arguments()

    # set spec06 result directory
    spec06_dir = args.result

    work_directory = os.path.join(noop_home, spec06_dir)
    if not os.path.exists(work_directory):
        print(f"Error: There is no SPEC06 result at:\n \t{work_directory}")
        return
    else:
        print(f"Processing SPEC06 result in:\n \t{work_directory}")

    # set covrage json file location
    cov_json_home = parse_cov_json(args.cov_json)
    if cov_json_home:
        print(f"Parse SPEC06 sub-folder by json file:\n \t{cov_json_home}")
    else:
        print("Error: There is no avaliable json")
        return

    # parse folder name by json file
    folder_names = generate_folder_names(cov_json_home)

    # extract result
    raw_data = extract_result(work_directory, folder_names)
    if not raw_data:
        print("Error: empty data")

    # process data
    processed_data = process_data(raw_data)

    serach_name = args.search

    topdown_entries = {}
    cycle_entries = {}
    commit_entries = {}
    for folder_name, entries in processed_data.items():
        topdown_entries[folder_name] = [
            entry for entry in entries if "TopDown".lower() in entry['name'].lower()
        ]
        cycle_entries[folder_name] = [
            entry for entry in entries if "clock_cycle".lower() in entry['name'].lower()
        ]
        commit_entries[folder_name] = [
            entry for entry in entries if "commitInstr".lower() in entry['name'].lower()
        ]

    # plot topdown-ARM
    plot_foldername = []
    plot_clock = []
    plot_backend_stall = []
    plot_frontend_stall = []
    plot_op_retired = []
    plot_op_spec = []
    plot_stall = []
    plot_mis_pred = []
    for folder_name, entries in topdown_entries.items():
        plot_foldername.append(folder_name)
        plot_clock.append(cycle_entries[folder_name][0]['count'])
        plot_op_retired.append(commit_entries[folder_name][0]['count'])
        for entry in entries:
            if entry['name'] == 'Topdown_Backend_Stall':
                plot_backend_stall.append(entry['count'])
            if entry['name'] == 'Topdown_Frontend_Stall':
                plot_frontend_stall.append(entry['count'])
            if entry['name'] == 'Topdown_Stall':
                plot_stall.append(entry['count'])
            if entry['name'] == 'Topdown_Op_spec':
                plot_op_spec.append(entry['count'])
            if entry['name'] == 'Topdown_Mispredict':
                plot_mis_pred.append(entry['count'])
            # print(f"Hierarchy: {entry['hierarchy']}, Name: {entry['name']}, Count: {entry['count']}")

    backend_bound = []
    bad_speculation = []
    frontend_bound = []
    retire = []

    for i in range(len(plot_foldername)):
        cpu_cycle      = int(plot_clock[i])
        backend_stall  = int(plot_backend_stall[i])
        frontend_stall = int(plot_frontend_stall[i])
        stall          = int(plot_stall[i])
        op_spec        = int(plot_op_spec[i])
        op_retired     = int(plot_op_retired[i])
        mis_pred       = int(plot_mis_pred[i])
        backend_bound_value = 100 * backend_stall / (cpu_cycle * 4)
        bad_speculation_value = 100 * ((1 - op_retired / op_spec) * (1 - stall / (cpu_cycle * 4)) + ((mis_pred * 4) / cpu_cycle))
        frontend_bound_value = 100 * (frontend_stall / (cpu_cycle * 4) - (mis_pred * 4) / cpu_cycle)
        retire_value = 100 * (op_retired / op_spec) * (1 - stall / (cpu_cycle * 4))
        
        backend_bound.append(backend_bound_value)
        bad_speculation.append(bad_speculation_value)
        frontend_bound.append(frontend_bound_value)
        retire.append(retire_value)

    fig, ax = plt.subplots(figsize=(10, 7))
    ax.bar(plot_foldername, retire, label='Retire')
    ax.bar(plot_foldername, backend_bound, bottom=retire, label='Backend Bound')
    ax.bar(plot_foldername, bad_speculation, bottom=[i+j for i,j in zip(retire, backend_bound)], label='Bad Speculation')
    ax.bar(plot_foldername, frontend_bound, bottom=[i+j+k for i,j,k in zip(retire, backend_bound, bad_speculation)], label='Frontend Bound')
    ax.set_xlabel('checkpoints')
    ax.set_ylabel('Percentage')
    ax.set_title('spec06')
    x = np.arange(len(plot_foldername)) 
    ax.set_xticks(x)
    ax.set_xticklabels(plot_foldername, rotation=45, ha='right', fontsize=8)

    ax.legend()
    plt.xticks(rotation=45)
    plt.tight_layout()
    plt.savefig('topdown_level1.png')

if __name__ == "__main__":
    main()