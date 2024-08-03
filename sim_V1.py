import matplotlib.pyplot as plt
import numpy as np
import time as tm

# vehicle constants and variables
vehicleMass = 1.5 #Kg
propellantMass = 4 # Kg
dragCoefficient = 0.4 # adimensional
crossSectionalArea = 0.07854 #m^2

# engine constants
massFlow = 0.8 # Kg/s
exahustVelocity = 343 #m/s half of speed of sound 

#physical constants
gravity = 9.80665 #kgm/s^2 gravity acceleration
R = 287.05  # J/kgK, specific gas constant of air
tempAtSeaLevel = 288.15  # K, temperature at sea level
tempLapseRate = 0.0065  # K/m, temperature lapse rate
pressureSeaLevel = 101325  # Pa, pressure at sea level
atmDensitySeaLevel = 1.225  # kg/m^3, density at sea level
    

# time tracker
time = 0.0 #s
dt = 0.001 #s

#initial values
altitude = 0
speed = 0
acceleration = 0
thrust = 0

# data storage arrays 
altitudes = []
speeds = []
accelerations = []
thrusts = []
dragForces = []
netForces = []
times = []
masses = []

# states tracker
startupMode = False 
readyForLaunch = False 
launch = False
ascentPhase = False
apogee = False
descentPhase = False
landing = False 

#targets
# targetAltitude = 100 #m

def flightComputerOn(startupMode): 
    startupMode = True
    return startupMode

def atmospheric_effects(altitude, speed):

    # Calculate temperature and pressure based on altitude
    temperature = tempAtSeaLevel - tempLapseRate * altitude
    pressure = pressureSeaLevel * (temperature/ tempAtSeaLevel) ** (-gravity / (tempLapseRate * R))
    atmDensitySeaLevel = pressure / (R * temperature)

    dragForce = 0.5 * atmDensitySeaLevel * speed**2 * dragCoefficient * crossSectionalArea
    
    return dragForce

def appendValues(time, currentMass, altitude, speed, acceleration, thrust, netForce, dragForce): 
    # Store the position, velocity, and acceleration of the rocket at this time step
    times.append(time)
    masses.append(currentMass)
    altitudes.append(altitude)
    speeds.append(speed)
    accelerations.append(acceleration)
    thrusts.append(thrust)
    netForces.append(netForce)
    dragForces.append(dragForce)

def graphics(): 
    # # Create a new figure with 4 subplots
    fig, ax = plt.subplots(2, 4)

    # Plot the position of the rocket in the top left subplot
    ax[0, 0].plot(altitudes, label='Position (m)')
    # ax[0, 0].set_ylabel('Position (m)')
    ax[0, 0].legend()

    # Plot the velocity of the rocket in the top right subplot
    ax[0, 1].plot(speeds, label='Velocity (m/s)')
    # ax[0, 1].set_xlabel('time s')
    ax[0, 1].legend()

    # Plot the acceleration of the rocket in the bottom left subplot
    ax[0, 2].plot(accelerations, label='Acceleration (m/s^2)')
    # ax[1, 0].set_ylabel('Acceleration (m/s^2)')
    ax[0, 2].legend()

    # Plot the thrust of the rocket in the bottom right subplot
    ax[1, 0].plot(thrusts, label='Thrust (kg m/s^2)')
    # ax[1, 1].set_ylabel('Thrust (kg m/s^2)')
    ax[1, 0].legend()

    # Plot the position of the rocket in the top left subplot
    ax[0, 3].plot(masses, label='mass (kg)')
    # ax[0, 2].set_ylabel('mass (kg)')
    # ax[0, 2].set_xlabel('time')
    ax[0, 3].legend()

    # Plot the position of the rocket in the top left subplot
    ax[1, 2].plot(netForces, label='netForce (N)')
    # ax[1, 2].set_ylabel('time (s)')
    ax[1, 2].legend()

        # Plot the position of the rocket in the top left subplot
    ax[1, 3].plot(times, label='time (s)')
    # ax[1, 2].set_ylabel('time (s)')
    ax[1, 3].legend()
        
    # Plot the position of the rocket in the top left subplot
    ax[1, 1].plot(dragForces, label='dragForce (N)')
    # ax[0, 3].set_ylabel('time (s)')
    ax[1, 1].legend()

    # Show the figure
    plt.show()

startupMode = flightComputerOn(startupMode)

if startupMode == True:
    def calibrateSensors():
        print('SENSORS CHECK')

    def checkTvc():
        print('TVC CHECK')

    # calibrateSensors()
    # checkTvc()

    readyForLaunch = True

if readyForLaunch == True:
    def initCountdown():
        countdown = 11
        i = 1
        
        print('INIT COUNTDOWN')

        while countdown > 0:
            countdown -= i
            print(countdown) 
            tm.sleep(0.01)

    # initCountdown()
    launch = True

if launch == True:
    print('IGNITION')
    print('LIFTOFF')

    while propellantMass >= 0:
       
        # print(dragForce)
        
        time += dt
        
        propellantMass -= massFlow * dt
        currentMass = vehicleMass + propellantMass
 
        vehicleWeight = - currentMass * gravity
        dragForce = - atmospheric_effects(altitude, speed)
        thrust = massFlow * exahustVelocity
        netForce = thrust + vehicleWeight + dragForce

        acceleration = netForce / currentMass
        speed += acceleration * dt
        altitude += speed * dt


        if propellantMass <= 0:
            print('BURNTIME COMPLETE: ', time , ' seconds at ', altitude, ' meters' )
            print('THRUST: ',thrusts[-1])
            ascentPhase = True

        appendValues(time, currentMass, altitude, speed, acceleration, thrust, netForce, dragForce)

if ascentPhase == True:
    while speed >=0:

        time += dt 

        thrust = 0
        dragForce = - atmospheric_effects(altitude, speed)
        netForce = dragForce + vehicleWeight

        acceleration = -netForce / vehicleWeight 
        speed += acceleration * dt
        altitude += speed * dt

        if speed <= 0:
            print('APOGEE: ', altitude, 'meters at ',time, ' seconds')
            apogee = True

        appendValues(time, currentMass, altitude, speed, acceleration, thrust, netForce, dragForce)

if apogee == True:
    print('PARACHUTE DEPLOY ')
    descentPhase = True

if descentPhase == True:
    
    while altitude >=0:
        time += dt 

        thrust = 0
        dragForce = atmospheric_effects(altitude, speed)
        netForce = dragForce + vehicleWeight

        acceleration = -netForce / vehicleWeight
        speed += acceleration * dt
        altitude += speed * dt

        if altitude <= 0:
            print('LANDING: ', time)
            landing = True

        appendValues(time, currentMass, altitude, speed, acceleration, thrust, netForce, dragForce)

if landing == True:
    print('fast data logging')
    graphics()  

