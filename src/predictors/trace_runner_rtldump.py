# Usage: This runner is designed to execute rtldump trace files. To run the GShare or UTAGE Python model, 
# update the model name in the import statement below and keep the rtldump files in the same path as these scripts.

import re
from u_tage_py_model import GShareBPU

def parse_line(line):
    match = re.search(r'0x([0-9a-fA-F]+) \((0x[0-9a-fA-F]+)\)', line)
    return (int(match.group(1), 16), int(match.group(2), 16)) if match else (None, None)

def load_trace():
    with open("rtl.dump") as f1:
        lines1 = f1.readlines()
    with open("rtl1.dump") as f2:
        lines2 = f2.readlines()
    return lines1 + lines2

def get_instr_info(instr, pc):
    is_rvc = (instr & 0x3) != 0x3
    ilen = 2 if is_rvc else 4
    target, ci = pc + ilen, None

    if not is_rvc:
        opcode = instr & 0x7F
        rd, rs1 = (instr >> 7) & 0x1F, (instr >> 15) & 0x1F
        if opcode == 0x63: # Branch
            ci = "Branch"
            imm = (((instr >> 31) & 0x1) << 12) | (((instr >> 7) & 0x1) << 11) | \
                  (((instr >> 25) & 0x3F) << 5) | (((instr >> 8) & 0xF) << 1)
            if imm & (1 << 12): imm -= (1 << 13)
            target = pc + imm
        elif opcode == 0x6F: # JAL
            ci = "Call" if rd in [1, 5] else "JAL"
            imm = (((instr >> 31) & 0x1) << 20) | (((instr >> 12) & 0xFF) << 12) | \
                  (((instr >> 20) & 0x1) << 11) | (((instr >> 21) & 0x3FF) << 1)
            if imm & (1 << 20): imm -= (1 << 21)
            target = pc + imm
        elif opcode == 0x67: # JALR
            ci = "Ret" if (rd == 0 and rs1 in [1, 5]) else "JAL"
            target = None # Target is unknown until execution
    else:
        f3, op = (instr >> 13) & 0x7, instr & 0x3
        if op == 1 and f3 in [6, 7]: # C.BEQZ/C.BNEZ
            ci = "Branch"
            imm = (((instr >> 12) & 0x1) << 8) | (((instr >> 5) & 0x3) << 6) | \
                  (((instr >> 2) & 0x1) << 5) | (((instr >> 10) & 0x3) << 3) | (((instr >> 3) & 0x3) << 1)
            if imm & (1 << 8): imm -= (1 << 9)
            target = pc + imm
        elif op == 1 and f3 == 5: # C.J
            ci = "JAL"
            imm = (((instr >> 12) & 0x1) << 11) | (((instr >> 8) & 0x1) << 10) | \
                  (((instr >> 9) & 0x3) << 8) | (((instr >> 6) & 0x1) << 7) | \
                  (((instr >> 7) & 0x1) << 6) | (((instr >> 2) & 0x1) << 5) | \
                  (((instr >> 11) & 0x1) << 4) | (((instr >> 3) & 0x5) << 1)
            if imm & (1 << 11): imm -= (1 << 12)
            target = pc + imm
        elif op == 2 and f3 == 4: # C.JR/C.JALR/C.MV
            rs1, rs2 = (instr >> 7) & 0x1F, (instr >> 2) & 0x1F
            if rs1 != 0 and rs2 == 0: # C.JR / C.JALR
                ci = "Ret" if rs1 in [1, 5] else "JAL"
                target = None
    return is_rvc, ilen, target, ci

def main():

    bpu = GShareBPU(debug=False, btb_depth=64, bht_depth=1024)
    debug = False
    lines = load_trace()

    total_branches = 0
    total_conditional = 0
    mispred_cond = 0
    mispred_jump = 0
    mispred_btbmiss = 0

    for i in range(len(lines) - 1):

        pc, instr = parse_line(lines[i])
        next_pc, _ = parse_line(lines[i + 1])

        is_rvc, ilen, calc_target, ci = get_instr_info(instr, pc)

        if pc is None or ci is None: continue
        
        if ci == "Branch":

            total_branches += 1
            total_conditional += 1

            if debug:
                print("\n",total_branches)
            pred = bpu.predict(pc, ilen)

            predicted_taken = (pred["prediction"] >> 1) & 1
            actual_taken = (next_pc != pc + ilen)

            target_from_instr = next_pc if calc_target is None else calc_target

            bpu.train(
                pc=pc,
                target=target_from_instr,
                ci="Branch",
                history=bpu.ghr[0],
                actual_taken=actual_taken,
                btbhit=pred["btbhit"],
                instr16=is_rvc
            )

            if pred["nextpc"] != next_pc:
                mispred_cond += 1
                if (pred["btbhit"] == False):
                    mispred_btbmiss += 1
                if debug:
                    print(">>> MISPRED_C: ",hex(pc))
                bpu.mispredict(pred["btbhit"], bpu.ghr[0])

        else:

            total_branches += 1

            if debug:
                print("\n",total_branches)
            pred = bpu.predict(pc, ilen)

            if pred["nextpc"] != next_pc:
                mispred_jump += 1
                if debug:
                    print(">>> MISPRED_UC: ",hex(pc))
                if (pred["btbhit"] == False):
                    mispred_btbmiss += 1

            bpu.train(
                pc=pc,
                target=next_pc,
                ci=ci,
                history=bpu.ghr[0],
                actual_taken=True,
                btbhit=pred["btbhit"],
                instr16=is_rvc
            )
      
    
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
    main()