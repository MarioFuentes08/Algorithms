import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.cm as cm

# ---  Load Data (Coordinates and Solution) ---
try:
    # Read node coordinates from node_data.csv file 
    df_nodes = pd.read_csv("node_data.csv")
    df_vehicles = pd.read_csv("vehicle_routes.csv")
    df_drones = pd.read_csv("drone_missions.csv")
    df_summary = pd.read_csv("solution_summary.csv")
except FileNotFoundError:
    print("Error: Files not found. Run the C++ program first.")
    print("Make sure 'node_data.csv', 'vehicle_routes.csv',  'drone_missions.csv' and 'solution_summary' exist.")
    exit()

# --- Get the dynamic objective value
optimal_value = df_summary['objective_value'].iloc[0]

# --- node_coords from the file ---
node_coords = {}
depot_nodes = []
customer_nodes = []
depot_end_id = df_nodes['node_id'].max() # Get the ID of the end depot 

for _, row in df_nodes.iterrows():
    node_id = int(row['node_id'])
    # Store coordinates
    node_coords[node_id] = (row['x_coord'], row['y_coord'])
    
    # Check if it's a depot (demand == 0) or customer
    if row['demand'] == 0:
        depot_nodes.append(node_id)
    else:
        customer_nodes.append(node_id)

# --- Create plot ---
fig, ax = plt.subplots(figsize=(12, 10))

# --- Plot nodes ---
# Plot nodes first
depot_label_added = False
customer_label_added = False

# Plot depots (0 and N+1)
for node_id in depot_nodes:
    x, y = node_coords[node_id]
    
    # Only label the depot once
    label = f'Depot (Node 0 & {depot_end_id})'
    if depot_label_added:
        label = None
    depot_label_added = True
    
    # Plot start and end depot as green square
    ax.scatter(x, y, s=400, color='green', marker='s', zorder=5, 
               edgecolors='black', label=label)
    
    # Only plot text for the first depot (node 0)
    if node_id == 0:
        ax.text(x + 2, y + 2, f"0 / {depot_end_id}", ha='left', va='bottom', 
                fontweight='bold', fontsize=12)

# Plot customers
for node_id in customer_nodes:
    x, y = node_coords[node_id]
    
    # Only label 'Customers' once
    label = 'Customers'
    if customer_label_added:
        label = None
    customer_label_added = True

    # Customers as sky blue
    ax.scatter(x, y, s=300, color='skyblue', zorder=5, 
               edgecolors='black', alpha=0.9, label=label)
    ax.text(x, y-2, str(node_id), ha='center', va='top', # Text below node
            fontweight='bold', fontsize=12)

# --- Color generation---
# Get all unique vehicle/drone IDs that were actually used
used_v_ids = df_vehicles['vehicle_id'].unique()
used_d_ids = df_drones['drone_id'].unique()

# Create color maps using standard matplotlib colormaps

v_cmap = cm.get_cmap('tab20')
# 'Set1' has 9 distinct color
d_cmap = cm.get_cmap('Set1')

# Build the color dictionaries dynamically
vehicle_colors = {v_id: v_cmap(v_id % 20) for v_id in used_v_ids}
drone_colors = {d_id: d_cmap(d_id % 9) for d_id in used_d_ids}

# --- Plot vehicle missions ---

for _, row in df_vehicles.iterrows():
    v_id = int(row['vehicle_id'])
    i = int(row['from_node'])
    j = int(row['to_node'])
    
    # get start and final coordinates
    x_coords = [node_coords[i][0], node_coords[j][0]]
    y_coords = [node_coords[i][1], node_coords[j][1]]
    
    color = vehicle_colors.get(v_id, 'gray') # Default colors
    label = f'Vehicle {v_id}'
    
    # Plot vehicle line
    ax.plot(x_coords, y_coords,
            color=color,
            linestyle='-',    
            linewidth=4,      
            alpha=0.6,
            label=label if label not in ax.get_legend_handles_labels()[1] else "",
            marker='>',              # Add arrow marker
            markersize=15,           
            markevery=[1],           # Only at the end of the line
            markerfacecolor=color    
           )

# --- Plot Drone missions ---

for _, row in df_drones.iterrows():
    d_id = int(row['drone_id'])
    i = int(row['from_node'])       # Launch
    j = int(row['served_node'])    # Customer
    k = int(row['to_node'])          # Meet
    
    color = drone_colors.get(d_id, 'pink') # Default color
    label = f'Drone {d_id}'

    # Launch (i -> j)
    x1 = [node_coords[i][0], node_coords[j][0]]
    y1 = [node_coords[i][1], node_coords[j][1]]
    ax.plot(x1, y1, color=color, linestyle='--', linewidth=2,
            label=label if label not in ax.get_legend_handles_labels()[1] else "",
            marker='>',
            markersize=10,
            markevery=[1], 
            markerfacecolor=color            
           )

    # Meeting (j -> k)
    x2 = [node_coords[j][0], node_coords[k][0]]
    y2 = [node_coords[j][1], node_coords[k][1]]
    ax.plot(x2, y2, color=color, linestyle='--', linewidth=2,
            marker='>',
            markersize=10,
            markevery=[1], 
            markerfacecolor=color            
           )

# --- Plot details ---
# Read the objective value from the log 
# 
ax.set_title(f'Optimal Solution CCVRDP ({optimal_value})', fontsize=18)
ax.set_xlabel('X Coordinate', fontsize=12)
ax.set_ylabel('Y Coordinate', fontsize=12)

# ---  Legend ---
# This method won't crash if a vehicle/drone is not used
handles, labels = ax.get_legend_handles_labels()
# Use a dictionary to remove duplicate labels
legend_dict = dict(zip(labels, handles))

# Dynamically find all used labels in a sorted order
depot_labels = [l for l in legend_dict if l.startswith('Depot')]
cust_labels = [l for l in legend_dict if l.startswith('Customers')]
v_labels = sorted([l for l in legend_dict if l.startswith('Vehicle')])
d_labels = sorted([l for l in legend_dict if l.startswith('Drone')])

# Create the final ordered list
final_labels = depot_labels + cust_labels + v_labels + d_labels
final_handles = [legend_dict[label] for label in final_labels]

ax.legend(final_handles, final_labels, fontsize=10)

ax.grid(True, linestyle=':', alpha=0.6) # Add grid
plt.axis('equal') # Same X and Y scale
plt.show()