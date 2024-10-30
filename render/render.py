import json
import sys
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os
from PIL import Image  # Importing Pillow for opening images

def visualize_brep(json_filename, output_image=None):
    # Read BREP data from JSON file
    with open(json_filename, 'r') as f:
        data = json.load(f)

    vertices = data.get('vertices', [])
    edges = data.get('edges', [])

    if not vertices:
        print("No vertices found in the JSON file.")
        return

    # Extract vertex coordinates from the dictionary format
    x_coords = [v['x'] for v in vertices]
    y_coords = [v['y'] for v in vertices]
    z_coords = [v['z'] for v in vertices]

    # Create a 3D plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot vertices
    ax.scatter(x_coords, y_coords, z_coords, c='blue', marker='o', label='Vertices')

    # Plot edges
    for edge in edges:
        start_idx = edge['start']
        end_idx = edge['end']
        x_edge = [vertices[start_idx]['x'], vertices[end_idx]['x']]
        y_edge = [vertices[start_idx]['y'], vertices[end_idx]['y']]
        z_edge = [vertices[start_idx]['z'], vertices[end_idx]['z']]
        ax.plot(x_edge, y_edge, z_edge, c='red', linewidth=1)

    # Set labels and title
    ax.set_xlabel('X-axis')
    ax.set_ylabel('Y-axis')
    ax.set_zlabel('Z-axis')
    ax.set_title('BREP Model Visualization')

    # Show legend
    ax.legend()

    # Adjust the view angle if desired
    ax.view_init(elev=30, azim=30)

    # Show or save the plot
    if output_image:
        plt.savefig(output_image)
        print(f"Visualization saved to {output_image}.")
        # Try to open the file after saving using Pillow
        try:
            img = Image.open(output_image)
            img.show()  # Open the image using the default image viewer
        except Exception as e:
            print(f"Error opening the image: {e}")
    else:
        plt.show()

if __name__ == '__main__':
    if len(sys.argv) != 3:
        print("Usage: python render.py <json_file> <output_image>")
        sys.exit(1)
    
    json_file = sys.argv[1]
    output_image = sys.argv[2]
    visualize_brep(json_file, output_image)
