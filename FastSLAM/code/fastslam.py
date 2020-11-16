from read_data import read_world, read_sensor_data
from misc_tools import *
from scipy.stats import multivariate_normal
import numpy as np
import math
from celluloid import Camera

# plot preferences, interactive plotting mode
plt.axis([-1, 12, 0, 10])
plt.ion()
plt.show()


def initialize_particles(num_particles, num_landmarks):
    # initialize particle at pose [0,0,0] with an empty map

    particles = []

    for i in range(num_particles):
        particle = dict()

        # initialize pose: at the beginning, robot is certain it is at [0,0,0]
        particle['x'] = 0
        particle['y'] = 0
        particle['theta'] = 0

        # initial weight
        particle['weight'] = 1.0 / num_particles

        # particle history aka all visited poses
        particle['history'] = []

        # initialize landmarks of the particle
        landmarks = dict()

        for i in range(num_landmarks):
            landmark = dict()

            # initialize the landmark mean and covariance
            landmark['mu'] = np.array([0, 0])
            landmark['sigma'] = np.zeros([2, 2])
            landmark['observed'] = False

            landmarks[i+1] = landmark

        # add landmarks to particle
        particle['landmarks'] = landmarks

        # add particle to set
        particles.append(particle)

    return particles


def sample_motion_model(odometry, particles):
    # Updates the particle positions, based on old positions, the odometry
    # measurements and the motion noise

    delta_rot1 = odometry['r1']
    delta_trans = odometry['t']
    delta_rot2 = odometry['r2']

    # the motion noise parameters: [alpha1, alpha2, alpha3, alpha4]
    noise = [0.1, 0.1, 0.05, 0.05]

    # compute the standard deviate of the uncertain motion
    sigma_rot1 = noise[0] * abs(delta_rot1) + noise[1] * delta_trans
    sigma_trans = noise[2] * delta_trans + \
        noise[3] * (abs(delta_rot1) + abs(delta_rot2))
    sigma_rot2 = noise[0] * abs(delta_rot2) + noise[1] * delta_trans

    # update postion of each particle based on the motion model
    for particle in particles:
        delta_rot1_hat = delta_rot1 + np.random.normal(0, sigma_rot1)
        delta_trans_hat = delta_trans + np.random.normal(0, sigma_trans)
        delta_rot2_hat = delta_rot2 + np.random.normal(0, sigma_rot2)

        particle['history'].append([particle['x'], particle['y']])
        particle['x'] = particle['x'] + \
            delta_trans_hat * np.cos(particle['theta'] + delta_rot1_hat)
        particle['y'] = particle['y'] + \
            delta_trans_hat * np.sin(particle['theta'] + delta_rot1_hat)
        particle['theta'] = particle['theta'] + delta_rot1_hat + delta_rot2_hat

    return


def measurement_model(particle, landmark):
    # Compute the expected measurement for a landmark
    # and the Jacobian with respect to the landmark.

    px = particle['x']
    py = particle['y']
    ptheta = particle['theta']

    lx = landmark['mu'][0]
    ly = landmark['mu'][1]

    # calculate expected range measurement
    meas_range_exp = np.sqrt((lx - px)**2 + (ly - py)**2)
    meas_bearing_exp = math.atan2(ly - py, lx - px) - ptheta

    h = np.array([meas_range_exp, meas_bearing_exp])

    # Compute the Jacobian H of the measurement function h
    # wrt the landmark location (different with EKF)

    H = np.zeros((2, 2))
    H[0, 0] = (lx - px) / h[0]
    H[0, 1] = (ly - py) / h[0]
    H[1, 0] = (py - ly) / (h[0]**2)
    H[1, 1] = (lx - px) / (h[0]**2)

    return h, H


def eval_sensor_model(sensor_data, particles):
    # Correct landmark poses with a measurement and
    # calculate particle weight

    # sensor noise
    Q_t = np.array([[1.0, 0],
                    [0, 0.1]])

    # measured landmark ids and ranges
    ids = sensor_data['id']
    ranges = sensor_data['range']
    bearings = sensor_data['bearing']

    # update landmarks and calculate weight for each particle
    for particle in particles:

        landmarks = particle['landmarks']

        px = particle['x']
        py = particle['y']
        ptheta = particle['theta']

        particle['weight'] = 1.0

        # loop over observed landmarks
        for i, lm_id in enumerate(ids):

            # current landmark
            landmark = landmarks[lm_id]

            # measured range and bearing to current landmark
            meas_range = ranges[i]
            meas_bearing = bearings[i]

            if not landmark['observed']:
                # landmark is observed for the first time

                # initialize landmark mean and covariance.
                landmark['mu'][0] = px + meas_range * \
                    np.cos(meas_bearing + ptheta)
                landmark['mu'][1] = py + meas_range * \
                    np.sin(meas_bearing + ptheta)

                _, H = measurement_model(particle, landmark)
                H_inv = np.linalg.inv(H)
                landmark['sigma'] = H_inv.dot(Q_t).dot(H_inv.T)

                landmark['observed'] = True

            else:
                # landmark was observed before

                # update landmark mean and covariance.
                z = np.array([meas_range, meas_bearing])
                z_hat, H = measurement_model(particle, landmark)
                Q = H.dot(landmark['sigma']).dot(H.T) + Q_t
                K = landmark['sigma'].dot(H.T).dot(np.linalg.inv(Q))
                delta = np.array([z[0] - z_hat[0], angle_diff(z[1], z_hat[1])])
                landmark['mu'] = landmark['mu'] + K.dot(delta)
                landmark['sigma'] = (np.eye(2) - K.dot(H)
                                     ).dot(landmark['sigma'])
                weight = multivariate_normal.pdf(delta, np.zeros(2), Q)
                particle['weight'] = particle['weight'] * weight

    # normalize weights
    normalizer = sum([p['weight'] for p in particles])

    for particle in particles:
        particle['weight'] = particle['weight'] / normalizer

    return


def resample_particles(particles):
    # Returns a new set of particles obtained by performing
    # stochastic universal sampling, according to the particle
    # weights.

    new_particles = []
    M = len(particles)
    r = np.random.uniform(0, 1 / M)
    c = particles[0]['weight']
    i = 0

    for m in range(M):
        u = r + m / M
        while u > c:
            i += 1
            c = c + particles[i]['weight']
        new_particle = particles[i].copy()
        new_particles.append(new_particle)

    return new_particles


def main():

    # plot preferences, interactive plotting mode
    # fig = plt.figure()
    # camera = Camera(fig)

    print("Reading landmark positions")
    landmarks = read_world("../data/world.dat")

    print("Reading sensor data")
    sensor_readings = read_sensor_data("../data/sensor_data.dat")

    num_particles = 100
    num_landmarks = len(landmarks)

    # create particle set
    particles = initialize_particles(num_particles, num_landmarks)

    # run FastSLAM
    for timestep in range(int(len(sensor_readings)/2)):

        # predict particles by sampling from motion model with odometry info
        sample_motion_model(sensor_readings[timestep, 'odometry'], particles)

        # evaluate sensor model to update landmarks and calculate particle weights
        eval_sensor_model(sensor_readings[timestep, 'sensor'], particles)

        # plot filter state
        plot_state(particles, landmarks)
        # camera.snap()

        # calculate new set of equally weighted particles
        particles = resample_particles(particles)

    # save animation as .mp4
    # animation = camera.animate()
    # animation.save('animation.mp4')

    plt.show(block=True)


if __name__ == "__main__":
    main()
