import matplotlib.pyplot as plt
import numpy as np
import time as tm

# vehicle constants and variables
vehicleMass = 4.5 #Kg
propellantMass = 1.5 # Kg
dragCoefficient = 0.4 # adimensional
crossSectionalArea = 0.07854 #m^2

# engine constants
massFlow = 0.3 # Kg/s
exahustVelocity = 343*0.9 #m/s speed of sound 

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
    # Set the default style of the plots
    plt.style.use('dark_background')

    # Create a new figure with 4 subplots
    fig, ax = plt.subplots(2, 4, figsize=(18, 10))

    # Customize each subplot
    for a in ax.flatten():
        a.set_facecolor('black')
        a.tick_params(colors='white')
        for spine in a.spines.values():
            spine.set_edgecolor('white')

    # Plot the position of the rocket
    ax[0, 0].plot(altitudes, color='white', label='Position (m)')
    ax[0, 0].legend()

    # Plot the velocity of the rocket
    ax[0, 1].plot(speeds, color='white', label='Velocity (m/s)')
    ax[0, 1].legend()

    # Plot the acceleration of the rocket
    ax[0, 2].plot(accelerations, color='white', label='Acceleration (m/s^2)')
    ax[0, 2].legend()

    # Plot the thrust of the rocket
    ax[1, 0].plot(thrusts, color='white', label='Thrust (kg m/s^2)')
    ax[1, 0].legend()

    # Plot the mass of the rocket
    ax[0, 3].plot(masses, color='white', label='Mass (kg)')
    ax[0, 3].legend()

    # Plot the net force on the rocket
    ax[1, 2].plot(netForces, color='white', label='Net Force (N)')
    ax[1, 2].legend()

    # Plot the time
    ax[1, 3].plot(times, color='white', label='Time (s)')
    ax[1, 3].legend()

    # Plot the drag force on the rocket
    ax[1, 1].plot(dragForces, color='white', label='Drag Force (N)')
    ax[1, 1].legend()

    # Adjust layout
    plt.tight_layout()

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

