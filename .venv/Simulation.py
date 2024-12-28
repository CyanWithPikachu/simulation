import numpy as np
import math
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
class particle:
    epsilon=1/(4*math.pi)
    mu=4*math.pi
    def __init__(self,mass,electricCharge,magneticCharge,position,velocity,electricField=np.array([0,0,0]),magneticField=np.array([0,0,0])):
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
tim=0.01
particleList=[]

#List Of Particles (Properties in order are mass, electric charge, magnetic charge, position, velocity within the particle function)
particleList.append(particle(1,1,0,np.array([0,0,0]),np.array([0,0,0])))
particleList.append(particle(1,0,1,np.array([1,0,0]),np.array([0,1,0])))
particleList.append(particle(1,-1,-1,np.array([0,1,0]),np.array([0.5,0,0])))

fig = plt.figure(num="Electric and Magnetic Charge Simulation")
ax = fig.add_subplot(111,projection='3d')
ax.set_xlim([-2,2])
ax.set_ylim([-2,2])
ax.set_zlim([-2,2])
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
#Field Updating
def animate(i):
    ax.clear()
    ax.set_xlim([-2, 2])
    ax.set_ylim([-2, 2])
    ax.set_zlim([-2, 2])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
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
    for particle1 in particleList:
        i=particleList.index(particle1)
        ax.scatter(particle1.position[0],particle1.position[1],particle1.position[2], s=20)
        ax.text(particle1.position[0]+0.05,particle1.position[1]+0.05,particle1.position[2]+0.05, "Particle " + str(i+1) + "\n m: " + str(particle1.mass) + " q: " + str(particle1.electricCharge) + " b: " + str(particle1.magneticCharge), fontsize =7)

ani = animation.FuncAnimation(fig, animate, frames=400, interval=1, blit=False)
#For saving the animation as gif, uncomment two lines below and change the directory
#writer = animation.PillowWriter(fps=30, metadata=dict(artist='Cyan Gupta'), bitrate=1800)
#ani.save('../../../Downloads/Animation.gif', writer=writer)
plt.show()
