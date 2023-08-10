import numpy as np
from PIL import Image
import matplotlib.pyplot as plt

# Function to convert a binary number to gray code
def binary_to_gray(bin_str):
    n = int(bin_str, 2)
    n ^= (n >> 1)
    return format(n, '0{}b'.format(len(bin_str)))

# Encoder resolution
resolution = 16

# Number of positions (2^resolution)
num_positions = 2**resolution

# Number of pixels for each bit
pixels_per_bit_y = 20
pixels_per_bit_x = 2


# Initialize an array to hold the pattern
pattern = np.zeros((num_positions, resolution))

# Generate the gray code pattern
for i in range(num_positions):
    binary = format(i, '0{}b'.format(resolution))
    gray = binary_to_gray(binary)
    for j in range(resolution):
        pattern[i, j] = int(gray[j])

# Expand the pattern to have 20 pixels per bit
expanded_pattern = np.repeat(np.repeat(pattern, pixels_per_bit_x, axis=0), pixels_per_bit_y, axis=1)

# Convert numpy array to PIL image
pattern_img = Image.fromarray(np.uint8(expanded_pattern * 255), 'L')

# Save as PNG
pattern_img.save('pattern.png')

# Plot the pattern
fig, ax = plt.subplots()
ax.imshow(pattern_img, cmap='gray', aspect='auto')
ax.set_xlabel('Bit position')
ax.set_ylabel('Encoder position')
ax.set_title('Gray Code Pattern for a {}-bit Absolute Optical Encoder'.format(resolution))

# Save as SVG
fig.savefig('encoder_pattern.svg', format='svg')