import matplotlib.pyplot as plt

def plot_mission_2d(original_wps, modified_wps, actual_path=None):
    """Plot 2D mission path"""
    plt.figure(figsize=(12, 8))
    plt.plot([wp['lon'] for wp in original_wps], [wp['lat'] for wp in original_wps], 'b--', label='Original Plan')
    if len(modified_wps) > len(original_wps):
        plt.plot([wp['lon'] for wp in modified_wps], [wp['lat'] for wp in modified_wps], 'r-', label='Modified Path')
    if actual_path:
        plt.plot([p[1] for p in actual_path], [p[0] for p in actual_path], 'g-', label='Actual Path')
    plt.xlabel('Longitude')
    plt.ylabel('Latitude')
    plt.legend()
    plt.grid(True)
    plt.show()