import pandas as pd

# Read data from file
file_path = 'scatter.txt'  # Replace with the actual file path

with open(file_path, 'r') as file:
    lines = file.readlines()

# Remove duplicate rows while preserving order
unique_lines = []
seen_lines = set()

for line in lines:
    if line not in seen_lines:
        unique_lines.append(line)
        seen_lines.add(line)

# Save the cleaned data back to the same file
with open(file_path, 'w') as file:
    file.writelines(unique_lines)

print("Duplicate rows removed without changing the order, and saved to the same file.")

