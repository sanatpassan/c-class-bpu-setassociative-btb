# Shakti C-Class Branch Predictor Configuration Guide

This guide details how to configure, switch between, and test different Branch Prediction Unit (BPU) architectures in the C-Class core. 

By default, the core uses a **Fully Associative Gshare** predictor with a BTB size of 32 and a BHT size of 512.

---

## 1. General Configuration (Sizing & Parameters)

Before generating the SoC or changing predictor types, you can adjust the structural capacities of the BPU (BTB depth, BHT depth, and History Length).

1. Open the core configuration file:
   `c-class/sample_config/c64/core64.yaml`
2. Modify the respective BPU parameters (e.g., BTB size, BHT size, history length).
3. Apply the changes by running the `soc_config` generator from the root directory:

```bash
soc_config -ispec sample_config/c64/rv64i_isa.yaml -customspec sample_config/c64/rv64i_custom.yaml -cspec sample_config/c64/core64.yaml -gspec sample_config/c64/csr_grouping64.yaml -dspec sample_config/c64/rv64i_debug.yaml --verbose info
```

---

## 2. Predictor Architectures

### A. Gshare (Fully Associative)
This is the default configuration. 
* **Setup:** No macro definitions are required.
* **Default Sizes:** BTB = 32, BHT = 512.

### B. Gshare (Set Associative)
To scale up to larger BTB capacities efficiently, you should switch to the Set Associative Gshare model.
* **Recommended Sizes:** BTB = 64, BHT = 128, History Length = 8
* **Setup:** 1. Open `c-class/makefile.inc`.
  2. Locate the `BSC_DEFINES` variable.
  3. Add the `btb_set_assoc` macro:
     ```makefile
     BSC_DEFINES += -D btb_set_assoc
     ```

### C. µ-TAGE (`u_tage`)
To use the TAGE predictor for complex workloads, you need to swap the module import in the pipeline frontend and increase the history tracking length.
* **Recommended Sizes:** BTB = 64, BHT = 1024, History Length = 40
* **Setup:**
  1. Open `c-class/src/stage0.bsv`.
  2. Change the BPU import statement to `u_tage`:
     ```verilog
     `ifdef bpu
       import u_tage :: * ;
     `endif
     ```

### D. µ-TAGE with Loop Predictor (`u_tage_lp`)
This configuration pairs the TAGE predictor with a dedicated Loop Predictor to prevent capacity pollution caused by deterministic loops. By default, it features a 16-entry loop table. You can customize the table size and maximum iteration count by modifying the defines `LOOP_ENTRIES` and `ITER_WIDTH` macros in both `u_tage_lp.bsv` and `c-class/src/ccore_params.defines`.
* **Recommended Sizes:** BTB = 64, BHT = 1024, History Length = 40 (Loop table is 16 entries by default).
* **Setup:**
  1. Open `c-class/src/stage0.bsv` and change the import to `u_tage_lp`:
     ```verilog
     `ifdef bpu
       import u_tage_lp :: * ;
     `endif
     ```
  2. Open `c-class/makefile.inc`.
  3. Add the `bpu_lp` macro to your build definitions:
     ```makefile
     BSC_DEFINES += -D bpu_lp
     ```

---

## 3. Running the Python Models

Standalone Python models are provided to verify predictor logic and simulate algorithmic efficiency without requiring a full RTL compilation. These are evaluated using trace files and the provided trace runner scripts (`trace_runner_rtldump.py` or `trace_runner_champsim.py`).

**Steps to Run:**
1. Ensure your desired trace files are located in the same directory as the Python scripts (`c-class/src/predictors/`).
2. Open the specific trace runner you intend to use (e.g., `trace_runner_rtldump.py`).
3. At the top of the file, modify the import statement to pull in the Python model you want to test. For example, to run the **µ-TAGE** model, set it to:
   ```python
   import re
   from u_tage_py_model import GShareBPU
   ```
   *(To run the Set Associative Gshare model instead, change the import to `from gshare_sa_py_model import GShareBPU`)*.
4. Execute the trace runner using Python 3:
   ```bash
   python3 trace_runner_rtldump.py
   ```
