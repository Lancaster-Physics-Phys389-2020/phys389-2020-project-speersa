import numpy as np
import scipy.constants
import copy

class Particle:
    """Class to create a massive particle with charge. Data for position, velocity and acceleration are stored in NumPy arrays
    Position, velocity and acceleration are in metres and seconds. Charge is in units of the elementary charge. Mass is in kg."""

    def __init__(self, position, velocity, acceleration, mass, charge, name):
        self.name=name
        self.position=np.array(position,dtype=float)
        self.velocity=np.array(velocity)
        self.acceleration=np.array(acceleration)
        self.mass=mass
        self.charge=charge
    
    def __repr__(self):
        return 'Particle: {0}, Mass: {1}, Position: {2}, Velocity: {3}, Acceleration: {4}'.format(self.name,self.mass,self.position, self.velocity,self.acceleration)

    def momentum(self):
        return self.mass*self.velocity

    def kineticenergy(self):
        return 0.5*self.mass*np.vdot(self.velocity, self.velocity)

    def updates(self, deltaT):
        self.velocity = self.velocity + self.acceleration * deltaT
        self.position = self.position + self.velocity * deltaT


class cyclotron:
    """Class that actually runs the thing. Calculates the force on a particle from the fields and updates the position, velocity and acceleration
    
    """
    def __init__(self, deltaT, particleList, magField):
        self.deltaT=deltaT
        self.particleList=particleList
        self.magField = np.array(magField)
    
    def __repr__(self):
        return '{0}'.format(self.particleList)

    def simulation(self):

        for p in self.particleList:
            force = p.charge * scipy.constants.e * np.cross(p.velocity, self.magField)
            p.acceleration = force/p.mass
            p.updates(0.00001)
            #print(p)
            return p


            
