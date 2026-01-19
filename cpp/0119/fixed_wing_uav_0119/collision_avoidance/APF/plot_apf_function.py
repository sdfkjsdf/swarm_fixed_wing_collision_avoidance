import numpy as np
import matplotlib.pyplot as plt
import os
import re

def load_apf_parameters():
    """Load parameters from APFParameter.c"""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    c_file = os.path.join(script_dir, 'APFParameter.c')
    
    with open(c_file, 'r') as f:
        content = f.read()
    
    # Parse parameters using regex
    alpha = float(re.search(r'\.m_alpha\s*=\s*([\d.]+)', content).group(1))
    a = float(re.search(r'\.m_a\s*=\s*([\d.]+)', content).group(1))
    b = float(re.search(r'\.m_b\s*=\s*([\d.]+)', content).group(1))
    
    return alpha, a, b

# Load parameters from C file
alpha, a, b = load_apf_parameters()
print(f"Loaded from APFParameter.c: alpha={alpha}, a={a}, b={b}")

# Distance range
norm_position = np.linspace(0, 100, 1000)

# APF gain function
APF_gain = alpha / (1.0 + np.power(norm_position / a, 2.0 * b))

# Plot
plt.figure(figsize=(10, 6))
plt.plot(norm_position, APF_gain, 'b-', linewidth=2)
plt.xlabel('Distance (norm_position) [m]', fontsize=12)
plt.ylabel('APF Gain', fontsize=12)
plt.title(f'APF Function: $-\\alpha / (1 + (d/a)^{{2b}})$\n'
          f'alpha={alpha}, a={a}, b={b}', fontsize=14)
plt.grid(True, alpha=0.3)
plt.axhline(y=0, color='k', linestyle='-', linewidth=0.5)
plt.axvline(x=a, color='r', linestyle='--', linewidth=1, label=f'a = {a}m')
plt.legend()
plt.xlim([0, 100])
plt.tight_layout()
plt.savefig('apf_function.png', dpi=150)
plt.show()

# Print key values
print("\n========== APF Function Analysis ==========")
print(f"At d=0:  APF_gain = {-alpha / (1.0 + 0):.4f}")
print(f"At d=a:  APF_gain = {-alpha / (1.0 + 1):.4f}")
print(f"At d=2a: APF_gain = {-alpha / (1.0 + np.power(2, 2*b)):.10f}")