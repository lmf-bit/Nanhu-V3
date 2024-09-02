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
    parser.add_argument('-s', '--search', default="L1",
                        help="Specify the hierarchy of the counter \
                            --L1 \
                            --L2_backend")
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

    serach_type = args.search

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
    plot_backend_robstall = []
    plot_backend_walkstall = []
    plot_backend_intflstall = []
    plot_backend_fpflstall = []
    plot_backend_intdqstall = []
    plot_backend_fpdqstall = []
    plot_backend_lsdqstall = []
    plot_backend_intflandrobstall = []
    plot_backend_fpflandrobstall = []
    plot_backend_intdqandintflstall = []
    plot_backend_lsdqandintflstall = []
    plot_backend_intandlsdqstall = []
    plot_backend_robandintdqstall = []
    plot_backend_robandfpdqstall = []
    plot_backend_robandlsdqstall = []
    plot_backend_robandintlsdqstall = []
    for folder_name, entries in topdown_entries.items():
        plot_foldername.append(folder_name)
        plot_clock.append(cycle_entries[folder_name][0]['count'])
        plot_op_retired.append(commit_entries[folder_name][0]['count'])
        for entry in entries:
            #L1
            if serach_type == 'L1':
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
            #L2
            elif serach_type == 'L2_backend':
                if entry['name'] == 'Topdown_Backend_Stall':
                    plot_backend_stall.append(entry['count'])
                if entry['name'] == 'TopdownL2Backend_RobStall':
                    plot_backend_robstall.append(entry['count'])
                if entry['name'] == 'TopdownL2Backend_WalkStall':
                    plot_backend_walkstall.append(entry['count'])
                if entry['name'] == 'TopdownL2Backend_IntFlStall':
                    plot_backend_intflstall.append(entry['count'])
                if entry['name'] == 'TopdownL2Backend_FpFlStall':
                    plot_backend_fpflstall.append(entry['count'])
                if entry['name'] == 'TopdownL2Backend_IntDqStall':
                    plot_backend_intdqstall.append(entry['count'])
                if entry['name'] == 'TopdownL2Backend_FpDqStall':
                    plot_backend_fpdqstall.append(entry['count'])
                if entry['name'] == 'TopdownL2Backend_LsDqStall':
                    plot_backend_lsdqstall.append(entry['count'])
                if entry['name'] == 'TopdownL2Backend_IntFlAndRobStall':
                    plot_backend_intflandrobstall.append(entry['count'])
                if entry['name'] == 'TopdownL2Backend_FpFlAndRobStall':
                    plot_backend_fpflandrobstall.append(entry['count'])
                if entry['name'] == 'TopdownL2Backend_IntDqAndintFlStall':
                    plot_backend_intdqandintflstall.append(entry['count'])
                if entry['name'] == 'TopdownL2Backend_LsDqAndintFlStall':
                    plot_backend_lsdqandintflstall.append(entry['count'])
                if entry['name'] == 'TopdownL2Backend_IntAndlsDqStall':
                    plot_backend_intandlsdqstall.append(entry['count'])
                if entry['name'] == 'TopdownL2Backend_RobAndintDqStall':
                    plot_backend_robandintdqstall.append(entry['count'])
                if entry['name'] == 'TopdownL2Backend_RobAndlsDqStall':
                    plot_backend_robandlsdqstall.append(entry['count'])
                if entry['name'] == 'TopdownL2Backend_RobAndfpDqStall':
                    plot_backend_robandfpdqstall.append(entry['count'])
                if entry['name'] == 'TopdownL2Backend_RobAndintlsDqStall':
                    plot_backend_robandintlsdqstall.append(entry['count'])
            # print(f"Hierarchy: {entry['hierarchy']}, Name: {entry['name']}, Count: {entry['count']}")

    backend_bound = []
    bad_speculation = []
    frontend_bound = []
    retire = []

    rob_bound = []
    walk_bound = []
    intFl_bound = []
    fpFl_bound = []
    intDq_bound = []
    fpDq_bound = []
    lsDq_bound = []
    intflandrob_bound = []
    fpflandrob_bound = []
    robandintdq_bound = []
    robandfpdq_bound = []
    robandlsdq_bound = []
    robandintlsdq_bound = []
    intdqandintfl_bound = []
    lsdqandintfl_bound = []
    robandother_stall_bound = []
    intandlsdq_bound = []
    other_bound = []

    if serach_type == 'L1':
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
    elif serach_type == 'L2_backend':
        for i in range(len(plot_foldername)):
            cpu_cycle      = int(plot_clock[i])
            backend_stall  = int(plot_backend_stall[i])
            rob_stall       = int(plot_backend_robstall[i])
            walk_stall      = int(plot_backend_walkstall[i])
            intFl_stall     = int(plot_backend_intflstall[i])
            fpFl_stall      = int(plot_backend_fpflstall[i])
            intDq_stall     = int(plot_backend_intdqstall[i])
            fpDq_stall      = int(plot_backend_fpdqstall[i])
            lsDq_stall      = int(plot_backend_lsdqstall[i])
            intflandrob_stall  = int(plot_backend_intflandrobstall[i])
            fpflandrob_stall  = int(plot_backend_fpflandrobstall[i])
            robandintdq_stall  = int(plot_backend_robandintdqstall[i])
            robandfpdq_stall  = int(plot_backend_robandfpdqstall[i])
            robandlsdq_stall  = int(plot_backend_robandlsdqstall[i])
            robandintlsdq_stall  = int(plot_backend_robandintlsdqstall[i])
            intdqandintfl_stall   = int(plot_backend_intdqandintflstall[i])
            lsdqandintfl_stall   = int(plot_backend_lsdqandintflstall[i])
            intandlsdq_stall = int(plot_backend_intandlsdqstall[i])

            rob_bound_value = 100 * rob_stall / backend_stall
            walk_bound_value = 100 * walk_stall / backend_stall
            intFl_bound_value = 100 * intFl_stall / backend_stall
            fpFl_bound_value = 100 * fpFl_stall / backend_stall
            intDq_bound_value = 100 * intDq_stall / backend_stall
            fpDq_bound_value = 100 * fpDq_stall / backend_stall
            lsDq_bound_value = 100 * lsDq_stall / backend_stall
            intflandrob_bound_value = 100 * intflandrob_stall / backend_stall
            fpflandrob_bound_value = 100 * fpflandrob_stall / backend_stall
            robandother_stall = intflandrob_stall + fpflandrob_stall + robandintdq_stall + robandfpdq_stall + robandlsdq_stall + robandintlsdq_stall
            robandother_stall_bound_value = 100 * robandother_stall / backend_stall
            robandintdq_bound_value = 100 * robandintdq_stall / backend_stall
            robandfpdq_bound_value = 100 * robandfpdq_stall / backend_stall
            robandlsdq_bound_value = 100 * robandlsdq_stall / backend_stall
            robandintlsdq_bound_value = 100 * robandintlsdq_stall / backend_stall
            intdqandintfl_bound_value = 100 * intdqandintfl_stall / backend_stall
            lsdqandintfl_bound_value = 100 * lsdqandintfl_stall / backend_stall
            intandlsdq_bound_value = 100 * intandlsdq_stall / backend_stall
            single_bound_value = rob_bound_value + walk_bound_value + intFl_bound_value + fpFl_bound_value + intDq_bound_value + fpDq_bound_value + lsDq_bound_value
            conmbine_bound_value = intflandrob_bound_value + fpflandrob_bound_value + robandintdq_bound_value + robandfpdq_bound_value + robandlsdq_bound_value + robandintlsdq_bound_value + intdqandintfl_bound_value + lsdqandintfl_bound_value + intandlsdq_bound_value
            other_bound_value = 100 - (single_bound_value + conmbine_bound_value)
            
            rob_bound.append(rob_bound_value)
            walk_bound.append(walk_bound_value)
            intFl_bound.append(intFl_bound_value)
            fpFl_bound.append(fpFl_bound_value)
            intDq_bound.append(intDq_bound_value)
            fpDq_bound.append(fpDq_bound_value)
            lsDq_bound.append(lsDq_bound_value)
            intflandrob_bound.append(intflandrob_bound_value)
            fpflandrob_bound.append(fpflandrob_bound_value)
            robandintdq_bound.append(robandintdq_bound_value)
            robandfpdq_bound.append(robandfpdq_bound_value)
            robandlsdq_bound.append(robandlsdq_bound_value)
            robandintlsdq_bound.append(robandintlsdq_bound_value)
            intdqandintfl_bound.append(intdqandintfl_bound_value)
            lsdqandintfl_bound.append(lsdqandintfl_bound_value)
            robandother_stall_bound.append(robandother_stall_bound_value)
            intandlsdq_bound.append(intandlsdq_bound_value)
            other_bound.append(other_bound_value)

        fig, ax = plt.subplots(figsize=(10, 7))
        ax.bar(plot_foldername, rob_bound, label='rob Stall')
        ax.bar(plot_foldername, walk_bound, bottom=rob_bound, label='walk Stall')
        ax.bar(plot_foldername, intFl_bound, bottom=[i+j for i,j in zip(rob_bound, walk_bound)], label='intFl Stall')
        # ax.bar(plot_foldername, fpFl_bound, bottom=[i+j+k for i,j,k in zip(rob_bound, walk_bound, intFl_bound)], label='fpFl Stall')
        ax.bar(plot_foldername, intDq_bound, bottom=[i+j+k for i,j,k in zip(rob_bound, walk_bound, intFl_bound)], label='intDq Stall')
        # ax.bar(plot_foldername, fpDq_bound, bottom=[i+j+k+l+m for i,j,k,l,m in zip(rob_bound, walk_bound, intFl_bound, fpFl_bound, intDq_bound)], label='fpDq Stall')
        ax.bar(plot_foldername, lsDq_bound, bottom=[i+j+k+l for i,j,k,l in zip(rob_bound, walk_bound, intFl_bound, intDq_bound)], label='lsDq Stall')
        # ax.bar(plot_foldername, intflandrob_bound, bottom=[i+j+k+l for i,j,k,l in zip(rob_bound, walk_bound, intFl_bound, intDq_bound, lsDq_bound)], label='rob and intfl Stall')
        # ax.bar(plot_foldername, fpflandrob_bound, bottom=[i+j+k+l+m+n+o+p for i,j,k,l,m,n,o,p in zip(rob_bound, walk_bound, intFl_bound, fpFl_bound, intDq_bound, fpDq_bound, lsDq_bound, intflandrob_bound)], label='rob and fpfl Stall')
        # ax.bar(plot_foldername, robandintdq_bound, bottom=[i+j+k+l+m for i,j,k,l,m in zip(rob_bound, walk_bound, intFl_bound, intDq_bound, lsDq_bound, intflandrob_bound)], label='rob and intdq Stall')
        # ax.bar(plot_foldername, robandfpdq_bound, bottom=[i+j+k+l+m+n+o+p+q+r for i,j,k,l,m,n,o,p,q,r in zip(rob_bound, walk_bound, intFl_bound, fpFl_bound, intDq_bound, fpDq_bound, lsDq_bound, intflandrob_bound, fpflandrob_bound, robandintdq_bound)], label='rob and fpdq Stall')
        # ax.bar(plot_foldername, robandlsdq_bound, bottom=[i+j+k+l+m+n for i,j,k,l,m,n in zip(rob_bound, walk_bound, intFl_bound, intDq_bound, lsDq_bound, intflandrob_bound, robandintdq_bound)], label='rob and lsdq Stall')
        # ax.bar(plot_foldername, robandintlsdq_bound, bottom=[i+j+k+l+m+n+o for i,j,k,l,m,n,o in zip(rob_bound, walk_bound, intFl_bound, intDq_bound, lsDq_bound, intflandrob_bound, robandintdq_bound, robandlsdq_bound)], label='rob and intlsdq Stall')
        ax.bar(plot_foldername, robandother_stall_bound, bottom=[i+j+k+l+m for i,j,k,l,m in zip(rob_bound, walk_bound, intFl_bound, intDq_bound, lsDq_bound)], label='rob and other stall')
        ax.bar(plot_foldername, intdqandintfl_bound, bottom=[i+j+k+l+m+n for i,j,k,l,m,n in zip(rob_bound, walk_bound, intFl_bound, intDq_bound, lsDq_bound, robandother_stall_bound)], label='intdq and intfl Stall')
        ax.bar(plot_foldername, lsdqandintfl_bound, bottom=[i+j+k+l+m+n+o for i,j,k,l,m,n,o in zip(rob_bound, walk_bound, intFl_bound, intDq_bound, lsDq_bound, robandother_stall_bound, intdqandintfl_bound)], label='lsdq and intfl Stall')
        ax.bar(plot_foldername, intandlsdq_bound, bottom=[i+j+k+l+m+n+o+p for i,j,k,l,m,n,o,p in zip(rob_bound, walk_bound, intFl_bound, intDq_bound, lsDq_bound, robandother_stall_bound, intdqandintfl_bound, lsdqandintfl_bound)], label='int and ls dq Stall')
        ax.bar(plot_foldername, other_bound, bottom=[i+j+k+l+m+n+o+p+q for i,j,k,l,m,n,o,p,q in zip(rob_bound, walk_bound, intFl_bound, intDq_bound, lsDq_bound, robandother_stall_bound, intdqandintfl_bound, lsdqandintfl_bound, intandlsdq_bound)], label='other Stall')
        ax.set_xlabel('checkpoints')
        ax.set_ylabel('Percentage')
        ax.set_title('spec06')
        x = np.arange(len(plot_foldername)) 
        ax.set_xticks(x)
        ax.set_xticklabels(plot_foldername, rotation=45, ha='right', fontsize=8)

        ax.legend(bbox_to_anchor=(1,0))
        plt.xticks(rotation=45)
        plt.tight_layout()
        plt.savefig('topdown_level2_backend.png')

if __name__ == "__main__":
    main()