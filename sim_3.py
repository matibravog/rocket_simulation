import matplotlib.pyplot as plt
import numpy as np
import time as tm

# vehicle constants and variables
vehicleMass = 4.5  # Kg
propellantMass = 1.5  # Kg
dragCoefficient = 0.4  # adimensional
crossSectionalArea = 0.07854  # m^2

# engine constants
massFlow = 0.3  # Kg/s
exhaustVelocity = 343 * 0.9  # m/s speed of sound

# physical constants
gravity = 9.80665  # m/s^2 gravity acceleration
R = 287.05  # J/kgK, specific gas constant of air
tempAtSeaLevel = 288.15  # K, temperature at sea level
tempLapseRate = 0.0065  # K/m, temperature lapse rate
pressureSeaLevel = 101325  # Pa, pressure at sea level
atmDensitySeaLevel = 1.225  # kg/m^3, density at sea level

# time tracker
time = 0.0  # s
dt = 0.001  # s

# initial values
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

# functions
def flightComputerOn(startupMode):
    startupMode = True
    return startupMode

def atmospheric_effects(altitude, speed):
    # Calculate temperature and pressure based on altitude
    temperature = tempAtSeaLevel - tempLapseRate * altitude
    pressure = pressureSeaLevel * (temperature / tempAtSeaLevel) ** (-gravity / (tempLapseRate * R))
    atmDensity = pressure / (R * temperature)
    dragForce = 0.5 * atmDensity * speed**2 * dragCoefficient * crossSectionalArea
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
    ax[0, 0].plot(times, altitudes, color='cyan', label='Altitude (m)')
    ax[0, 0].set_xlabel('Time (s)')
    ax[0, 0].set_ylabel('Altitude (m)')
    ax[0, 0].legend()

    # Plot the velocity of the rocket
    ax[0, 1].plot(times, speeds, color='cyan', label='Velocity (m/s)')
    ax[0, 1].set_xlabel('Time (s)')
    ax[0, 1].set_ylabel('Velocity (m/s)')
    ax[0, 1].legend()

    # Plot the acceleration of the rocket
    ax[0, 2].plot(times, accelerations, color='cyan', label='Acceleration (m/s^2)')
    ax[0, 2].set_xlabel('Time (s)')
    ax[0, 2].set_ylabel('Acceleration (m/s^2)')
    ax[0, 2].legend()

    # Plot the thrust of the rocket
    ax[1, 0].plot(times, thrusts, color='cyan', label='Thrust (N)')
    ax[1, 0].set_xlabel('Time (s)')
    ax[1, 0].set_ylabel('Thrust (N)')
    ax[1, 0].legend()

    # Plot the mass of the rocket
    ax[0, 3].plot(times, masses, color='cyan', label='Mass (kg)')
    ax[0, 3].set_xlabel('Time (s)')
    ax[0, 3].set_ylabel('Mass (kg)')
    ax[0, 3].legend()

    # Plot the net force on the rocket
    ax[1, 2].plot(times, netForces, color='cyan', label='Net Force (N)')
    ax[1, 2].set_xlabel('Time (s)')
    ax[1, 2].set_ylabel('Net Force (N)')
    ax[1, 2].legend()

    # Plot the drag force on the rocket
    ax[1, 1].plot(times, dragForces, color='cyan', label='Drag Force (N)')
    ax[1, 1].set_xlabel('Time (s)')
    ax[1, 1].set_ylabel('Drag Force (N)')
    ax[1, 1].legend()

    # Adjust layout
    plt.tight_layout()
    # Show the figure
    plt.show()

# Execution
startupMode = flightComputerOn(startupMode)

if startupMode:
    def calibrateSensors():
        print('SENSORS CHECK')

    def checkTvc():
        print('TVC CHECK')

    readyForLaunch = True

if readyForLaunch:
    def initCountdown():
        countdown = 11
        i = 1
        print('INIT COUNTDOWN')
        while countdown > 0:
            countdown -= i
            print(countdown)
            tm.sleep(1)  # Ajustar tiempo de sueño para la cuenta regresiva en segundos

    initCountdown()
    launch = True

if launch:
    print('IGNITION')
    print('LIFTOFF')

    while propellantMass > 0:
        time += dt
        propellantMass -= massFlow * dt
        currentMass = vehicleMass + propellantMass
        vehicleWeight = -currentMass * gravity
        dragForce = atmospheric_effects(altitude, speed)
        thrust = massFlow * exhaustVelocity
        netForce = thrust + vehicleWeight - dragForce

        acceleration = netForce / currentMass
        speed += acceleration * dt
        altitude += speed * dt

        if propellantMass <= 0:
            print(f'BURNTIME COMPLETE: {time:.2f} seconds at {altitude:.2f} meters')
            print(f'THRUST: {thrusts[-1]}')
            ascentPhase = True

        appendValues(time, currentMass, altitude, speed, acceleration, thrust, netForce, dragForce)

if ascentPhase:
    while speed >= 0:
        time += dt
        thrust = 0
        dragForce = atmospheric_effects(altitude, speed)
        netForce = -vehicleWeight - dragForce

        acceleration = netForce / currentMass
        speed += acceleration * dt
        altitude += speed * dt

        if speed <= 0:
            print(f'APOGEE: {altitude:.2f} meters at {time:.2f} seconds')
            apogee = True

        appendValues(time, currentMass, altitude, speed, acceleration, thrust, netForce, dragForce)

if apogee:
    print('PARACHUTE DEPLOY')
    descentPhase = True

if descentPhase:
    while altitude >= 0:
        time += dt
        thrust = 0
        dragForce = atmospheric_effects(altitude, speed)
        netForce = -vehicleWeight + dragForce

        acceleration = netForce / currentMass
        speed += acceleration * dt
        altitude += speed * dt

        if altitude <= 0:
            print(f'LANDING: {time:.2f} seconds')
            landing = True

        appendValues(time, currentMass, altitude, speed, acceleration, thrust, netForce, dragForce)

if landing:
    print('Data logging complete')
    graphics()
