# Usage: This runner is designed to execute ChampSim trace files. To run the GShare or UTAGE Python model, 
# update the model name in the import statement below and specify the corresponding trace file path in the main function.

import struct
import sys
import lzma
from u_tage_lp_py_model import GShareBPU

# ChampSim input_instr struct: ip(Q), is_branch(B), branch_taken(B), 
# dest_reg(2B), src_reg(4B), dest_mem(2Q), src_mem(4Q) = 64 bytes
CHAMPSIM_STRUCT_FORMAT = "QBB2B4B2Q4Q"
CHAMPSIM_STRUCT_SIZE = struct.calcsize(CHAMPSIM_STRUCT_FORMAT)

def get_champsim_instr(file_handle):
    """Helper to read one struct from the binary trace."""
    buf = file_handle.read(CHAMPSIM_STRUCT_SIZE)
    if not buf or len(buf) < CHAMPSIM_STRUCT_SIZE:
        return None
    data = struct.unpack(CHAMPSIM_STRUCT_FORMAT, buf)
    return {
        "ip": data[0],
        "is_branch": data[1],
        "branch_taken": data[2]
    }

def main(trace_path, instr_limit=None):
    bpu = GShareBPU(debug=False, btb_depth=512, bht_depth=1024)
    debug = False

    total_branches = 0
    total_conditional = 0
    mispred_cond = 0
    mispred_jump = 0
    mispred_btbmiss = 0
    instr_count = 0
    
    # Determine if file is xz compressed
    if trace_path.endswith('.xz'):
        file_handle = lzma.open(trace_path, "rb")
    else:
        file_handle = open(trace_path, "rb")
    
    with file_handle:
        # We need two instructions (current and next) to simulate your loop logic
        curr_instr = get_champsim_instr(file_handle)
        if not curr_instr:
            return

        while True:
            next_instr = get_champsim_instr(file_handle)
            instr_count += 1
            if instr_limit is not None and instr_count > instr_limit:
                print(f"Reached instruction limit of {instr_limit}. Stopping simulation.")
                break
            if not next_instr:
                break

            # Mapping ChampSim fields to your existing variable names
            pc = curr_instr["ip"]
            next_pc = next_instr["ip"]
            
            # ChampSim provides pre-decoded branch info
            # Mapping is_branch to your 'ci' (category info)
            if curr_instr["is_branch"]:
                # Note: ChampSim traces typically treat all control flow as 'is_branch'
                # We default to "Branch" to utilize your BHT logic. 
                ci = "Branch" 
                actual_taken = bool(curr_instr["branch_taken"])
                ilen = (next_pc - pc) if not actual_taken else 4
                is_rvc = False # ChampSim traces are usually decoded x86
            else:
                ci = None

            if pc is None or ci is None: 
                curr_instr = next_instr
                continue
            
            if ci == "Branch":
                total_branches += 1
                total_conditional += 1
                

                if debug:
                    print("\n", total_branches)
                pred = bpu.predict(pc, ilen)

                target_from_instr = next_pc 

                bpu.train(
                    pc=pc,
                    target=target_from_instr,
                    ci="Branch",
                    history=bpu.ghr[0],
                    actual_taken=actual_taken,
                    btbhit=pred["btbhit"],
                    instr16=is_rvc,
                    mispred=(pred["nextpc"] != next_pc),
                    lp_hist=pred["lp_hist"]
                )

                predicted_taken = (pred["prediction"] >> 1) & 1

                if predicted_taken != actual_taken:
                    mispred_cond += 1
                    if not pred["btbhit"]:
                        mispred_btbmiss += 1
                    if debug:
                        print(">>> MISPRED_C: ", hex(pc))
                    bpu.mispredict(pred["btbhit"], bpu.ghr[0])


            else:
                # Handle non-conditional branches/jumps if 'ci' was set to something else
                total_branches += 1

                if debug:
                    print("\n", total_branches)
                pred = bpu.predict(pc, ilen)

                if pred["nextpc"] != next_pc:
                    mispred_jump += 1
                    if debug:
                        print(">>> MISPRED_UC: ", hex(pc))
                    if not pred["btbhit"]:
                        mispred_btbmiss += 1

                bpu.train(
                    pc=pc,
                    target=next_pc,
                    ci=ci, 
                    history=bpu.ghr[0],
                    actual_taken=True,
                    btbhit=pred["btbhit"],
                    instr16=is_rvc,
                    mispred=(pred["nextpc"] != next_pc),
                    lp_hist=pred["lp_hist"]

                )



            curr_instr = next_instr
    
    print("\n========= FINAL =========")
    print(f"Total Mispredictions    : {mispred_cond + mispred_jump}")
    print(f"Mispred from jumps      : {mispred_jump}")
    print(f"Mispred from BTB miss   : {mispred_btbmiss}")
    print(f"Conditional branches    : {total_conditional}")
    print(f"Jumps                   : {total_branches - total_conditional}")
    print(f"Total branches          : {total_branches}")

    if total_branches > 0:
        mispred_rate = ((mispred_cond + mispred_jump) / total_branches) * 100
    else:
        mispred_rate = 0.0

    print(f"Misprediction rate (%)  : {mispred_rate:.2f}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python trace_runner_champsim.py <trace_file_path> [instr_limit]")
        sys.exit(1)
    
    trace_path = sys.argv[1]
    instr_limit = None
    if len(sys.argv) > 2:
        try:
            instr_limit = int(sys.argv[2])
        except ValueError:
            print(f"Error: instr_limit must be an integer, got '{sys.argv[2]}'")
            sys.exit(1)
    
    main(trace_path, instr_limit)
