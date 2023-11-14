# SpRegret: Optimal Distributed Control with Regret Minimization

## Overview

This repository contains the MATLAB code for the implementation of the controllers presented in the paper titled "Closing the Gap to Quadratic Invariance: a Regret Minimization Approach to Optimal Distributed Control" by Daniele Martinelli et al. The paper is available on [arXiv](https://arxiv.org/abs/2311.02068) (arXiv:2311.02068, 2023).

The primary goal of this code is to synthesize four controllers: H2, Hinf, SpRegret with a QI benchmark $\mathbf{K}_{QI}$, and SpRegret with a Centralized benchmark $\mathbf{K}_{C}$. Additionally, the code includes scripts to generate Figure 1 and Figure 2 from the paper.

The repository also aims to provide a Python version soon.

## Getting Started

### Prerequisites

- MATLAB R2019a or later
- Parallel Computing Toolbox (optional, for faster execution of certain scripts)

### Installation

1. Clone the repository to your local machine:

   ```bash
   git clone https://github.com/DecodEPFL/SpRegret.git
   cd SpRegret
   ```

2. Open MATLAB and navigate to the repository folder.

## Usage

### 1. Main Script (`main.m`)

The `main.m` script is the main entry point for running the synthesis of the controllers. In this script, you can set the linear system to control and the sparsity matrix $S$. Execute the script to obtain the controllers H2, Hinf, SpRegret with QI benchmark $\mathbf{K}_{QI}$, and SpRegret with centralized benchmark $\mathbf{K}_{C}$.

### 2. Script for Figure 1 (`Script_for_Fig_1.m`)

This script generates Figure 1 from the paper. It utilizes the Parallel Computing Toolbox for faster execution. If the Parallel Computing Toolbox is not available, replace the "parfor" in line 51 with a regular "for."

### 3. Script for Figure 2 (`Script_for_Fig_2.m`)

Similar to Figure 1, this script generates Figure 2 from the paper. If the Parallel Computing Toolbox is not available, replace the "parfor" in line 43 with a regular "for."

## License
This project is licensed under the Apache License 2.0.

## Acknowledgments

- The code in this repository is based on the work presented in the paper "Closing the Gap to Quadratic Invariance: a Regret Minimization Approach to Optimal Distributed Control" by Daniele Martinelli et al.

Please refer to the paper for more details on the methodology and algorithms used in this code.

Feel free to contribute, report issues, or contact the authors for further information.
