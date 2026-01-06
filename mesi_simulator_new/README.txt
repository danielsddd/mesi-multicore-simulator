# MESI Simulator Test Suite

## Directory Structure
.
├── sim.c                    # Simulator source code
├── test.py                  # Test suite
├── README.md                # This file
├── example_061225_win/      # Teacher-provided example (from Moodle, as-is)
│   ├── imem0.txt, imem1.txt, imem2.txt, imem3.txt
│   ├── memin.txt
│   └── [all expected output files]
├── counter/                 # Counter test (4 cores × 128 increments)
│   ├── imem0.txt, imem1.txt, imem2.txt, imem3.txt
│   └── memin.txt
├── mulserial/               # Serial matrix multiplication (core 0 only)
│   ├── imem0.txt, imem1.txt, imem2.txt, imem3.txt
│   └── memin.txt
└── mulparallel/             # Parallel matrix multiplication (all 4 cores)
    ├── imem0.txt, imem1.txt, imem2.txt, imem3.txt
    └── memin.txt
```
### Commands

python test.py -e example_061225_win -x

python test.py -t .

python test.py -e example_061225_win -t .

python test.py -e example_061225_win -t .
