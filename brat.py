import sqlite3
import time
import matplotlib.pyplot as plt
import numpy as np

# Database connection
DATABASE = "C:\Users\Dani\Desktop\Licenta\BratRobotic.sqlite"  

def fetch_seed_data():
    """Fetch seed data from the database."""
    connection = sqlite3.connect(DATABASE)
    cursor = connection.cursor()
    cursor.execute("SELECT seed_id, trench_length_cm, seed_distance_cm FROM Seeds LIMIT 1;")
    seed_data = cursor.fetchone()
    connection.close()
    return seed_data  # Ex: (1, 100.0, 10.0)

def simulate_trench_and_planting(trench_length, seed_distance):
    """Simulate digging a trench and planting seeds along the line."""
    print(f"Simulating trench digging for length {trench_length} cm and planting seeds every {seed_distance} cm.")

    # Visualize trench digging and seed planting
    fig, ax = plt.subplots()
    ax.set_xlim(0, trench_length)
    ax.set_ylim(-5, 5)
    ax.axhline(y=0, color='brown', linestyle='--', linewidth=2, label='Trench Line')
    
    # Simulate planting seeds along the trench
    seed_positions = np.arange(0, trench_length + 1, seed_distance)
    for pos in seed_positions:
        ax.scatter(pos, 0, color='green', label='Seed' if pos == 0 else "")
        print(f"Planting seed at position {pos} cm...")
        time.sleep(0.5)  # Simulate delay for planting each seed

    ax.legend()
    plt.title("Trench Digging and Seed Planting Simulation")
    plt.xlabel("Trench Length (cm)")
    plt.ylabel("Trench Depth")
    plt.show()

def main():
    print("Connecting to the database...")
    seed_data = fetch_seed_data()

    if not seed_data:
        print("No seed data found in the database.")
        return

    seed_id, trench_length, seed_distance = seed_data
    print(f"Fetched Seed Data - ID: {seed_id}, Trench Length: {trench_length} cm, Seed Distance: {seed_distance} cm")

    # Simulate trench digging and seed planting
    simulate_trench_and_planting(trench_length, seed_distance)
    print("Simulation complete.")

if __name__ == "__main__":
    main()
