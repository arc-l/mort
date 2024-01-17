# [Multi-Layer Object Rearrangement](https://arxiv.org/pdf/2306.14251.pdf)

## Dependencies
* [Python](https://www.python.org/)
* [Gurobi](https://www.gurobi.com/documentation/quickstart.html) ([Python package](https://pypi.org/project/gurobipy/))
* [Timeout decorator](https://pypi.org/project/wrapt-timeout-decorator/)

## Setup Instruction
1. It is recommended to use a virtual environment with Python 3 for this project, e.g., `python -m venv /path/to/new/virtual/environment`.
2. Make sure you are in your virtual environment.
3. Run `pip install gurobipy` to install the gurobi Python package.
4. Run `pip install wrapt-timeout-decorator` to install the timeout Python package.
5. For solving larger cases, install the full version of Gurobi on your computer [here](https://www.gurobi.com/documentation/quickstart.html).

## Run
* Open a new terminal tab from the folder with the testcases to be ran, e.g., `pre-constructed_testcases`
* Execute: `python ../simulation/main.py`

## Introduction
Object rearrangement is a fundamental sub-task in accomplishing a great many physical tasks.
As such, effectively executing rearrangement is an important skill for intelligent robots to master.
In this study, we conduct the first algorithmic study on optimally solving the problem of Multilayer Object Rearrangement on a Tabletop (MORT), in which one object may be relocated at a time, and an object can only be moved if other objects do not block its top surface.
In addition, any intermediate structure during the reconfiguration process must be physically stable, i.e., it should stand without external support.
To tackle the dual challenges of untangling the dependencies between objects and ensuring structural stability, we develop an algorithm that interleaves the computation of the optimal rearrangement plan and structural stability checking.
Using a carefully constructed integer linear programming (ILP) model, our algorithm, Stability-Aware Rearrangement Programming (SARP), readily scales to optimally solve complex rearrangement problems of 3D structures with over 60 building blocks, with solution quality significantly outperforming natural greedy best-first approaches.

In the evaluation, we used three types of simulation-based evaluation setups.
From left to right: 2D Pyramid, 3D Pyramid, and Random Pile.
The top row shows the start configurations and the bottom row shows the corresponding goal configurations.
![simulation](https://github.com/arc-l/mort/assets/77806295/707dfa99-447a-4f5b-baa9-dde4f1256923)
