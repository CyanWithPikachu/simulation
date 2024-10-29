import numpy as np
import math
import pygame
import time
class particle:
    epsilon=1/(4*math.pi)
    mu=4*math.pi
    def __init__(self,mass,electricCharge,magneticCharge,position,velocity,electricField=np.array([0.0,0.0,0.0]),magneticField=np.array([0.0,0.0,0.0])):
        self.mass=mass
        self.electricCharge=electricCharge
        self.magneticCharge=magneticCharge
        self.position=position
        self.velocity=velocity
        self.electricField=electricField
        self.magneticField=magneticField
    def acceleration(self):
        force=self.electricCharge*(self.electricField+np.cross(self.velocity,self.magneticField))+self.magneticCharge*(self.magneticField-np.cross(self.velocity,self.electricField)/(particle.epsilon*particle.mu))
        acceleration=force/self.mass
        return acceleration
tim=0.001
running=True
frameSkip=True
particleList=[]
particleList.append(particle(math.inf,1,0,np.array([0,0,0]),np.array([0,0,0])))
particleList.append(particle(1,0,1,np.array([1,0,0]),np.array([0,1,0])))

with open("particle_positions.txt", "w") as file:
    for t in range(0, 100000):
        if running:
            #Field Updating
            for particle1 in particleList:
                particle1.electricField=np.array([0,0,0])
                particle1.magneticField=np.array([0,0,0])
                for particle2 in particleList:
                    if particle2 is not particle1:
                        displacement=particle1.position-particle2.position
                        if np.dot(displacement,displacement) == 0:
                            continue
                        eField=1/(4*math.pi*particle.epsilon)*(particle2.electricCharge)/np.dot(displacement,displacement)*displacement/np.linalg.norm(displacement)
                        mField=particle.mu/(4*math.pi)*(particle2.magneticCharge)/np.dot(displacement,displacement)*displacement/np.linalg.norm(displacement)
                        particle1.electricField=particle1.electricField+eField
                        particle1.magneticField=particle1.magneticField+mField
            for particle1 in particleList:
                a=particle1.acceleration()
                particle1.position = particle1.position + tim * particle1.velocity + tim ** 2 * a
                particle1.velocity= particle1.velocity + tim * a
            print(str(t) + "Particle 1 Position: " + np.array2string(particleList[0].position) + "      Particle 2 Position: " + np.array2string(particleList[1].position) + "       Distance: " + str(np.linalg.norm(particleList[1].position)) + "        Speed: " + str(np.linalg.norm(particleList[1].velocity)))

            position_string = f" {np.array2string(particleList[1].position, formatter={'float_kind': lambda x: '%.8f' % x})} \n"
            file.write(position_string)


