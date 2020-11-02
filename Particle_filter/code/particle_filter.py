import numpy as np
import scipy.stats
import matplotlib.pyplot as plt
from celluloid import Camera
from read_data import read_world, read_sensor_data


def plot_state(particles, landmarks, map_limits):
    # Visualizes the state of the particle filter.
    #
    # Displays the particle cloud, mean position and landmarks.

    xs = []
    ys = []

    for particle in particles:
        xs.append(particle['x'])
        ys.append(particle['y'])

    # landmark positions
    lx = []
    ly = []

    for i in range(len(landmarks)):
        lx.append(landmarks[i+1][0])
        ly.append(landmarks[i+1][1])

    # mean pose as current estimate
    estimated_pose = mean_pose(particles)

    # plot filter state
    plt.clf()
    plt.plot(xs, ys, 'r.')
    plt.plot(lx, ly, 'bo', markersize=10)
    name = list(range(1, 10))
    for i, label in enumerate(name):
        plt.annotate(label, (lx[i]-0.1, ly[i]-0.6))
    plt.quiver(estimated_pose[0], estimated_pose[1], np.cos(
        estimated_pose[2]), np.sin(estimated_pose[2]), angles='xy', scale_units='xy')

    plt.axis(map_limits)

    plt.pause(0.01)


def initialize_particles(num_particles, map_limits):
    # randomly initialize the particles inside the map limits

    particles = []

    for _ in range(num_particles):
        particle = dict()

        # draw x,y and theta coordinate from uniform distribution
        # inside map limits
        particle['x'] = np.random.uniform(map_limits[0], map_limits[1])
        particle['y'] = np.random.uniform(map_limits[2], map_limits[3])
        particle['theta'] = np.random.uniform(-np.pi, np.pi)

        particles.append(particle)

    return particles


def mean_pose(particles):
    # calculate the mean pose of a particle set.
    #
    # for x and y, the mean position is the mean of the particle coordinates
    #
    # for theta, we cannot simply average the angles because of the wraparound
    # (jump from -pi to pi). Therefore, we generate unit vectors from the
    # angles and calculate the angle of their average

    # save x and y coordinates of particles
    xs = []
    ys = []

    # save unit vectors corresponding to particle orientations
    vxs_theta = []
    vys_theta = []

    for particle in particles:
        xs.append(particle['x'])
        ys.append(particle['y'])

        # make unit vector from particle orientation
        vxs_theta.append(np.cos(particle['theta']))
        vys_theta.append(np.sin(particle['theta']))

    # calculate average coordinates
    mean_x = np.mean(xs)
    mean_y = np.mean(ys)
    mean_theta = np.arctan2(np.mean(vys_theta), np.mean(vxs_theta))

    return [mean_x, mean_y, mean_theta]


def sample_motion_model(odometry, particles):
    # Samples new particle positions, based on old positions, the odometry
    # measurements and the motion noise

    delta_rot1 = odometry['r1']
    delta_trans = odometry['t']
    delta_rot2 = odometry['r2']

    # the motion noise parameters: [alpha1, alpha2, alpha3, alpha4]
    noise = [0.1, 0.1, 0.05, 0.05]

    # generate new particle set after motion update
    sigma_rot1 = noise[0] * abs(delta_rot1) + noise[1] * delta_trans
    sigma_trans = noise[2] * delta_trans + \
        noise[3] * (abs(delta_rot1) + abs(delta_rot2))
    sigma_rot2 = noise[0] * abs(delta_rot2) + noise[1] * delta_trans

    new_particles = []

    for particle in particles:
        delta_rot1_bar = delta_rot1 + np.random.normal(0, sigma_rot1)
        delta_trans_bar = delta_trans + np.random.normal(0, sigma_trans)
        delta_rot2_bar = delta_rot2 + np.random.normal(0, sigma_rot2)

        x_prime = particle['x'] + \
            delta_trans_bar * np.cos(particle['theta'] + delta_rot1_bar)
        y_prime = particle['y'] + \
            delta_trans_bar * np.sin(particle['theta'] + delta_rot1_bar)
        theta_prime = particle['theta'] + delta_rot1_bar + delta_rot2_bar

        new_particle = dict()
        new_particle['x'] = x_prime
        new_particle['y'] = y_prime
        new_particle['theta'] = theta_prime
        new_particles.append(new_particle)

    return new_particles


def eval_sensor_model(sensor_data, particles, landmarks):
    # Computes the observation likelihood of all particles, given the
    # particle and landmark positions and sensor measurements
    # (probabilistic sensor models slide 33)
    #
    # The employed sensor model is range only.

    sigma_r = 0.2

    # measured landmark ids and ranges
    ids = sensor_data['id']
    ranges = sensor_data['range']

    weights = []

    for particle in particles:

        prob = 1
        for id, range_i in zip(ids, ranges):
            range_expi = np.sqrt(
                (particle['x'] - landmarks[id][0])**2 + (particle['y'] - landmarks[id][1])**2)
            prob_i = scipy.stats.norm.pdf(range_i, range_expi, sigma_r)
            prob = prob * prob_i
        weights.append(prob)

    # normalize weights
    normalizer = sum(weights)
    weights = weights / normalizer

    return weights


def resample_particles(particles, weights):
    # Returns a new set of particles obtained by performing
    # stochastic universal sampling, according to the particle weights.

    new_particles = []
    mm = len(particles)
    r = np.random.uniform(0, 1/mm)
    c = weights[0]
    i = 0

    for m in range(mm):
        u = r + m / mm
        while u > c:
            i = i + 1
            c = c + weights[i]
        new_particles.append(particles[i])

    return new_particles


def main():

    # add random seed for generating comparable pseudo random numbers
    np.random.seed(123)

    # plot preferences, interactive plotting mode
    # fig = plt.figure()
    plt.title('Particle Filter')
    plt.xlabel('x')
    plt.ylabel('y')
    # camera = Camera(fig)
    plt.axis([-1, 12, 0, 11])
    plt.ion()
    plt.show()

    # implementation of a particle filter for robot pose estimation

    print("Reading landmark positions")
    landmarks = read_world("../data/world.dat")

    print("Reading sensor data")
    sensor_readings = read_sensor_data("../data/sensor_data.dat")

    # initialize the particles
    map_limits = [-1, 12, 0, 11]
    particles = initialize_particles(1000, map_limits)

    # run particle filter
    for timestep in range(int(len(sensor_readings)/2)):

        # plot the current state
        plot_state(particles, landmarks, map_limits)
        # camera.snap()

        # predict particles by sampling from motion model with odometry info
        new_particles = sample_motion_model(
            sensor_readings[timestep, 'odometry'], particles)

        # calculate importance weights according to sensor model
        weights = eval_sensor_model(
            sensor_readings[timestep, 'sensor'], new_particles, landmarks)

        # resample new particle set according to their importance weights
        particles = resample_particles(new_particles, weights)
    # save animation as .mp4
    # animation = camera.animate()
    # animation.save('animation.mp4')
    # plt.show('hold')


if __name__ == "__main__":
    main()
