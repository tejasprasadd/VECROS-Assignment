import matplotlib
matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def plot_paths(paths):
    """3D visualization of paths with different colors"""
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    colors = ['red', 'blue', 'green', 'purple', 'orange', 'pink', 'yellow', 'black']
    for i, path in enumerate(paths):
        if not path: continue
        x, y, z = zip(*path)
        ax.plot(x, y, z, color=colors[i%len(colors)], marker='o', 
                markersize=2, label=f'Path {i+1}')
    
    ax.set_xlabel('X Axis')
    ax.set_ylabel('Y Axis')
    ax.set_zlabel('Z Axis')
    ax.legend()
    plt.show()