import numpy as np
import math
import matplotlib.pyplot as plt

class particle:
    epsilon = 1 / (4 * math.pi)
    mu = 4 * math.pi

    def __init__(self, mass, electricCharge, magneticCharge, position, velocity, electricField=np.array([0, 0, 0]), magneticField=np.array([0, 0, 0])):
        self.mass = mass
        self.electricCharge = electricCharge
        self.magneticCharge = magneticCharge
        self.position = position
        self.velocity = velocity
        self.electricField = electricField
        self.magneticField = magneticField

    def acceleration(self):
        force = self.electricCharge * (self.electricField + np.cross(self.velocity, self.magneticField)) + \
                self.magneticCharge * (self.magneticField - np.cross(self.velocity, self.electricField) / (particle.epsilon * particle.mu))
        acceleration = force / self.mass
        return acceleration

def normalize(vec):
    norm = np.linalg.norm(vec)
    if norm == 0:
        return vec
    return vec / norm

tim = 0.001
particleList = []

# Adding two particles
particleList.append(particle(math.inf, 1, 0, np.array([0, 0, 0]), np.array([0, 0, 0])))
particleList.append(particle(1, 0, 1, np.array([1, 0, 0]), np.array([0, 1, 0])))

desired_distance = np.linalg.norm(particleList[1].position - particleList[0].position)

# Matplotlib setup for live graph
plt.figure()
distances = []
times = []
plt.xlabel('Time (s)')
plt.ylabel('Distance Between Particles')
plt.title('Real-Time Distance Between Particles')
plt.ion()
plt.show()

time_counter = 0
plot_update_interval = 100  # Update plot every 100 iterations
for t in range(0, 1000000):
    # Field Updating
    for particle1 in particleList:
        particle1.electricField = np.array([0, 0, 0])
        particle1.magneticField = np.array([0, 0, 0])
        for particle2 in particleList:
            if particle2 is not particle1:
                displacement = particle1.position - particle2.position
                if np.dot(displacement, displacement) == 0:
                    continue
                eField = 1 / (4 * math.pi * particle.epsilon) * (particle2.electricCharge) / np.dot(displacement, displacement) * normalize(displacement)
                mField = particle.mu / (4 * math.pi) * (particle2.magneticCharge) / np.dot(displacement, displacement) * normalize(displacement)
                particle1.electricField = particle1.electricField + eField
                particle1.magneticField = particle1.magneticField + mField

    # Update positions while keeping distance the same
    for i, particle1 in enumerate(particleList):
        a = particle1.acceleration()
        particle1.position = particle1.position + tim * particle1.velocity + tim ** 2 * a
        particle1.velocity = particle1.velocity + tim * a

    # Enforce distance constraint between particle 0 and particle 1
    current_distance = np.linalg.norm(particleList[1].position - particleList[0].position)
    correction_vector = particleList[1].position - particleList[0].position
    correction_vector = normalize(correction_vector) * (desired_distance - current_distance)
    particleList[1].position += correction_vector

    # Output non-zero values
    print(f"Time: {t * tim:.3f}s, Particle 1 Position: {particleList[1].position}, Distance: {np.linalg.norm(particleList[1].position - particleList[0].position):.3f}, Speed: {np.linalg.norm(particleList[1].velocity):.3f}")

    # Update graph after calculations every plot_update_interval iterations
    if t % plot_update_interval == 0:
        time_counter += tim * plot_update_interval
        times.append(time_counter)
        distances.append(current_distance)
        plt.cla()  # Clear the previous plot
        plt.xlabel('Time (s)')
        plt.ylabel('Distance Between Particles')
        plt.title('Real-Time Distance Between Particles')
        plt.plot(times, distances, 'b-')
        plt.pause(0.01)

plt.ioff()
plt.show()
